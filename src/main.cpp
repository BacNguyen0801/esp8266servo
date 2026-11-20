#include <Arduino.h>
#include <Servo.h>
#include <Esp.h>

// ============================================================
// Configuration
// ============================================================

constexpr uint8_t SERVO_PIN = 4;

constexpr uint32_t RX_TASK_INTERVAL_MS    = 10;
constexpr uint32_t MOTOR_TASK_INTERVAL_MS = 20;
constexpr uint32_t MOTOR_MONITOR_TASK_INTERVAL_MS = 10;
constexpr uint32_t ALIVE_TASK_INTERVAL_MS = 10000;

constexpr uint8_t MAX_FRAME_LENGTH = 8;
constexpr uint8_t COMMAND_LENGTH_BYTE = 0x05;
constexpr uint8_t RESET_COMMAND_LENGTH_BYTE = 0x02;
constexpr uint8_t COMMAND_FRAME_LENGTH = 5; // [SID, CH, 0xF0, 0x01, DIRECTION]
constexpr uint8_t COMMAND_FRAME_WITH_LENGTH_LENGTH = 6; // [LEN, SID, CH, 0xF0, 0x01, DIRECTION]
constexpr uint8_t RESET_COMMAND_FRAME_LENGTH = 2; // [SID, SUB_FUNCTION]
constexpr uint8_t RESET_COMMAND_FRAME_WITH_LENGTH_LENGTH = 3; // [LEN, SID, SUB_FUNCTION]

constexpr uint16_t SERVO_STOP_US = 1500;
constexpr uint16_t SERVO_CLOCKWISE_US = 1550;
constexpr uint16_t SERVO_ANTICLOCKWISE_US = 1450;
constexpr uint32_t SERVO_RUN_DURATION_MS = 50;
constexpr uint8_t ADC_INPUT_PIN = A0;
constexpr uint8_t ADC_VOLTAGE_MIN_TARGET_TENTHS = 3;  // 0.3V in 0.1V units
constexpr uint8_t ADC_VOLTAGE_MAX_TARGET_TENTHS = 24; // 2.4V in 0.1V units
constexpr uint8_t ADC_VOLTAGE_TARGET_OFFSET_VOLT = 1; // 1.0 V in 0.1 V units
constexpr uint16_t ADC_MAX_RAW = 1023;
constexpr uint16_t ADC_FULL_SCALE_VOLTAGE_MV = 33; // ADC sees 0..3.3V
constexpr uint8_t ADC_WINDOW_SIZE = 3U; // number of samples for moving average

// ============================================================
// UDS-style Service IDs
// ============================================================

constexpr uint8_t SID_SET_SERVO_ANGLE = 0x31;
constexpr uint8_t SID_SET_SERVO_ANGLE_RESPONSE = 0x71;
constexpr uint8_t CMD_SET_SERVO_DIRECTION_HI = 0xF0;
constexpr uint8_t CMD_SET_SERVO_DIRECTION_LOW = 0x01;
constexpr uint8_t CMD_SET_VOLTAGE_TARGET_HI = 0xC0;
constexpr uint8_t CMD_SET_VOLTAGE_TARGET_LOW = 0x01;
constexpr uint8_t UDS_POSITIVE_RESPONSE_LENGTH = 4; // [SID, CH, CMD_HI, CMD_LOW]

constexpr uint8_t SID_ECU_RESET = 0x11;
constexpr uint8_t ECU_RESET_SUB_FUNCTION = 0x01;
constexpr uint8_t SID_ECU_RESET_RESPONSE = 0x51;

constexpr uint32_t ADC_MONITOR_DELAY_MS = 100;

// Pulse offset (us) from neutral, indexed by absolute voltage gap in 0.1V units.
constexpr uint16_t g_pulseMotorTableReference[24] = {
    0, 35, 49, 65, 85,
    109, 136, 166, 199, 233, 269,
    307, 500, 500, 500, 500, 500,
    500, 500, 500, 500, 500, 500, 500
};
// ============================================================
// Shared data
// ============================================================

volatile uint16_t g_targetPulseUs = SERVO_STOP_US;
volatile bool g_newCommand = false;
volatile uint32_t g_commandStartMs = 0;
volatile bool g_voltageControlActive = false;
volatile bool g_stopMotorRequested = false;
volatile bool g_voltageIncreaseTarget = false;
volatile uint8_t g_voltageTargetV = 0;
volatile uint16_t g_lastMeasuredVoltageV = 0;
volatile uint8_t g_voltageUnchangedCount = 0;
volatile uint32_t g_lastVoltageMonitorMs = 0;

uint32_t g_aliveCounter = 0;

Servo g_servo;

// ============================================================
// Command processing
// ============================================================

uint16_t ReadMeasuredVoltageV()
{
    static uint16_t samples[ADC_WINDOW_SIZE] = {};
    static uint8_t sampleCount = 0;
    static uint8_t nextSampleIndex = 0;
    static uint32_t rawSum = 0;

    if (sampleCount == ADC_WINDOW_SIZE)
    {
        rawSum -= samples[nextSampleIndex];
    }

    samples[nextSampleIndex] = analogRead(ADC_INPUT_PIN);
    rawSum += samples[nextSampleIndex];

    if (sampleCount < ADC_WINDOW_SIZE)
        ++sampleCount;

    nextSampleIndex = (nextSampleIndex + 1) % ADC_WINDOW_SIZE;

    const uint16_t averagedRaw = static_cast<uint16_t>(
        (rawSum + (sampleCount / 2)) / sampleCount);

    const uint32_t physicalVoltage = (static_cast<uint32_t>(averagedRaw) * ADC_FULL_SCALE_VOLTAGE_MV)>>10U;
    return physicalVoltage; // convert 0..3.3V ADC value to 0..33V real scale, then keep in 0.1V units
}

void SendPositiveUdsResponse(uint8_t cmdHi, uint8_t cmdLow)
{
    static const uint8_t response[UDS_POSITIVE_RESPONSE_LENGTH] =
    {
        SID_SET_SERVO_ANGLE_RESPONSE,
        0x01,
        cmdHi,
        cmdLow
    };

    Serial.write(response, sizeof(response));
    Serial.println();

    Serial.printf("[RX] UDSResp(%02X%02X%02X%02X)\n",
                  response[0], response[1], response[2], response[3]);
}

void SendResetPositiveResponse()
{
    Serial.write(SID_ECU_RESET_RESPONSE);
    Serial.write(ECU_RESET_SUB_FUNCTION);
    Serial.println();
    Serial.printf("[RX] UDSResp(%02X%02X)\n", SID_ECU_RESET_RESPONSE, ECU_RESET_SUB_FUNCTION);
    Serial.flush();
}

bool IsVoltageTargetReached(uint16_t currentVoltageV, bool increaseTarget, uint8_t targetVoltageV)
{
    if (increaseTarget)
    {
        return currentVoltageV >= targetVoltageV;
    }

    return currentVoltageV <= targetVoltageV;
}

uint16_t GetVoltageControlPulseUs(uint16_t currentVoltageV, uint16_t targetVoltageV, bool increaseTarget)
{
    const uint16_t voltageGap = currentVoltageV > targetVoltageV
        ? currentVoltageV - targetVoltageV
        : targetVoltageV - currentVoltageV;
    const uint16_t maxGapIndex = (sizeof(g_pulseMotorTableReference) / sizeof(g_pulseMotorTableReference[0])) - 1;
    const uint16_t gapIndex = voltageGap < maxGapIndex ? voltageGap : maxGapIndex;
    const uint16_t pulseOffsetUs = g_pulseMotorTableReference[gapIndex];

    return increaseTarget
        ? SERVO_STOP_US + pulseOffsetUs
        : SERVO_STOP_US - pulseOffsetUs;
}

bool IsVoltageUnchanged(uint16_t currentVoltageV)
{
    if (currentVoltageV == g_lastMeasuredVoltageV)
    {
        if (g_voltageUnchangedCount < 255)
            ++g_voltageUnchangedCount;
    }
    else
    {
        g_lastMeasuredVoltageV = currentVoltageV;
        g_voltageUnchangedCount = 0;
    }

    return g_voltageUnchangedCount >= 100;
}

void RequestMotorStop()
{
    g_stopMotorRequested = true;
}

void CompleteVoltageAdjustment()
{
    g_servo.writeMicroseconds(SERVO_STOP_US);
    g_newCommand = false;
    g_stopMotorRequested = true;
    g_voltageControlActive = false;
    g_voltageIncreaseTarget = false;
    g_voltageTargetV = 0;
    g_lastMeasuredVoltageV = 0;
    g_voltageUnchangedCount = 0;

    SendPositiveUdsResponse(CMD_SET_VOLTAGE_TARGET_HI, CMD_SET_VOLTAGE_TARGET_LOW);
}

void ResetMcu()
{
    SendResetPositiveResponse();
    delay(100);
    ESP.restart();
}

void ProcessCommand(const uint8_t *data, uint8_t length)
{
    if (length == 0)
        return;

    const uint8_t *frame = data;
    uint8_t frameLength = length;

    if (length == RESET_COMMAND_FRAME_WITH_LENGTH_LENGTH)
    {
        if (data[0] != RESET_COMMAND_LENGTH_BYTE)
        {
            Serial.printf("[RX] Invalid reset frame length byte: 0x%02X\n", data[0]);
            return;
        }

        frame = data + 1;
        frameLength = RESET_COMMAND_FRAME_LENGTH;
    }
    else if (length == RESET_COMMAND_FRAME_LENGTH)
    {
        frameLength = RESET_COMMAND_FRAME_LENGTH;
    }
    else if (length == COMMAND_FRAME_WITH_LENGTH_LENGTH)
    {
        if (data[0] != COMMAND_LENGTH_BYTE)
        {
            Serial.printf("[RX] Invalid frame length byte: 0x%02X\n", data[0]);
            return;
        }

        frame = data + 1;
        frameLength = COMMAND_FRAME_LENGTH;
    }
    else if (length != COMMAND_FRAME_LENGTH)
    {
        Serial.printf("[RX] Invalid command length: %u\n", length);
        return;
    }

    uint8_t sid = frame[0];

    switch (sid)
    {
        case SID_ECU_RESET:
        {
            if (frameLength != RESET_COMMAND_FRAME_LENGTH || frame[1] != ECU_RESET_SUB_FUNCTION)
            {
                Serial.println("[RX] Invalid ECU reset subfunction");
                return;
            }

            Serial.println("[RX] ECU reset requested");
            ResetMcu();
            break;
        }

        case SID_SET_SERVO_ANGLE:
        {
            if (frameLength != COMMAND_FRAME_LENGTH)
            {
                Serial.println("[RX] Invalid command length");
                return;
            }

            uint8_t channel = frame[1];
            uint8_t cmdHi = frame[2];
            uint8_t cmdLow = frame[3];
            uint8_t value = frame[4];

            if (channel != 0x01)
            {
                Serial.printf("[RX] Unsupported channel: %u\n", channel);
                return;
            }

            if (cmdHi == CMD_SET_SERVO_DIRECTION_HI && cmdLow == CMD_SET_SERVO_DIRECTION_LOW)
            {
                uint16_t pulseUs;
                switch (value)
                {
                    case 0x00:
                        pulseUs = SERVO_CLOCKWISE_US;
                        break;

                    case 0x01:
                        pulseUs = SERVO_ANTICLOCKWISE_US;
                        break;

                    default:
                        Serial.printf("[RX] Invalid direction: 0x%02X\n", value);
                        return;
                }

                g_targetPulseUs = pulseUs;
                g_commandStartMs = millis();
                g_voltageControlActive = false;
                g_stopMotorRequested = false;
                g_voltageIncreaseTarget = false;
                g_voltageTargetV = 0;
                g_newCommand = true;

                Serial.printf("[RX] Set servo direction: %s -> %u us\n",
                              (value == 0x00) ? "clockwise" : "anticlockwise",
                              pulseUs);
                break;
            }

            if (cmdHi == CMD_SET_VOLTAGE_TARGET_HI && cmdLow == CMD_SET_VOLTAGE_TARGET_LOW)
            {
                if (value < ADC_VOLTAGE_MIN_TARGET_TENTHS || value > ADC_VOLTAGE_MAX_TARGET_TENTHS)
                {
                    Serial.printf("[RX] Invalid target voltage: %u (valid 3..24 in 0.1V units = 0.3..2.4V)\n", value);
                    return;
                }

                const uint16_t currentVoltageV = ReadMeasuredVoltageV();
                const bool shouldIncrease = currentVoltageV < value;

                if (IsVoltageTargetReached(currentVoltageV, shouldIncrease, value))
                {
                    Serial.printf("[RX] Voltage target already reached: %u V (target %u V)\n",
                                  currentVoltageV,
                                  value);
                    CompleteVoltageAdjustment();
                    break;
                }

                g_targetPulseUs = GetVoltageControlPulseUs(currentVoltageV, value, shouldIncrease);
                g_commandStartMs = millis();
                g_voltageControlActive = true;
                g_stopMotorRequested = false;
                g_voltageIncreaseTarget = shouldIncrease;
                g_voltageTargetV = value;
                g_lastMeasuredVoltageV = currentVoltageV;
                g_voltageUnchangedCount = 0;
                g_lastVoltageMonitorMs = g_commandStartMs;
                g_newCommand = true;

                Serial.printf("[RX] Set voltage target: %u V, current %u V -> %s\n",
                              value,
                              currentVoltageV,
                              shouldIncrease ? "clockwise" : "anticlockwise");
                break;
            }

            Serial.printf("[RX] Unsupported command: %02X %02X\n", cmdHi, cmdLow);
            break;
        }

        default:
            Serial.printf("[RX] Unsupported SID: 0x%02X\n", sid);
            break;
    }
}

// ============================================================
// Task 1: Serial RX
// Frame: LEN | SID | CH | 0xF0 | 0x01 | DIRECTION
// First byte must be 0x05 for the length-prefixed protocol.
// ============================================================

void SerialRxTask()
{
    static uint8_t rxBuffer[MAX_FRAME_LENGTH];
    static uint8_t receivedLength = 0;
    static uint8_t expectedLength = 0;

    while (Serial.available() > 0)
    {
        uint8_t rxByte = Serial.read();

        if (receivedLength == 0)
        {
            if (rxByte != SID_SET_SERVO_ANGLE &&
                rxByte != SID_ECU_RESET &&
                rxByte != COMMAND_LENGTH_BYTE &&
                rxByte != RESET_COMMAND_LENGTH_BYTE)
            {
                Serial.printf("[RX] Invalid header: 0x%02X\n", rxByte);
                continue;
            }

            rxBuffer[receivedLength++] = rxByte;
            if (rxByte == COMMAND_LENGTH_BYTE)
                expectedLength = COMMAND_FRAME_WITH_LENGTH_LENGTH;
            else if (rxByte == RESET_COMMAND_LENGTH_BYTE)
                expectedLength = RESET_COMMAND_FRAME_WITH_LENGTH_LENGTH;
            else if (rxByte == SID_ECU_RESET)
                expectedLength = RESET_COMMAND_FRAME_LENGTH;
            else
                expectedLength = COMMAND_FRAME_LENGTH;
            continue;
        }

        rxBuffer[receivedLength++] = rxByte;

        if (receivedLength == expectedLength)
        {
            Serial.print("[RX] Frame: ");

            for (uint8_t i = 0; i < receivedLength; i++)
                Serial.printf("%02X ", rxBuffer[i]);

            Serial.println();

            ProcessCommand(rxBuffer, receivedLength);

            receivedLength = 0;
            expectedLength = 0;
        }
    }
}

// ============================================================
// Task 2: Servo motor control
// ============================================================

void MotorControlTask()
{
    if (!g_newCommand)
    {
        g_servo.writeMicroseconds(SERVO_STOP_US);
        return;
    }

    if (g_stopMotorRequested)
    {
        const bool wasVoltageControlActive = g_voltageControlActive;
        g_servo.writeMicroseconds(SERVO_STOP_US);

        if (wasVoltageControlActive)
        {
            CompleteVoltageAdjustment();
        }
        else
        {
            g_newCommand = false;
            Serial.printf("[MOTOR] Stop after %lu ms -> %u us\n",
                          (unsigned long)(millis() - g_commandStartMs),
                          SERVO_STOP_US);
            SendPositiveUdsResponse(CMD_SET_SERVO_DIRECTION_HI, CMD_SET_SERVO_DIRECTION_LOW);
        }
        return;
    }

    g_servo.writeMicroseconds(g_targetPulseUs);
}

// ============================================================
// Task 3: Motor stop-condition monitor
// ============================================================

void MonitorConditionStopMotor()
{
    if (!g_newCommand || g_stopMotorRequested)
        return;

    const uint32_t now = millis();
    const uint32_t elapsedMs = now - g_commandStartMs;

    if (!g_voltageControlActive)
    {
        if (elapsedMs >= SERVO_RUN_DURATION_MS)
        {
            RequestMotorStop();
        }
        return;
    }

    g_lastVoltageMonitorMs = now;
    const uint16_t currentVoltageV = ReadMeasuredVoltageV();
    const uint16_t targetVoltageV = g_voltageTargetV;

    if (IsVoltageTargetReached(currentVoltageV, g_voltageIncreaseTarget, targetVoltageV))
    {
        Serial.printf("[MOTOR] Voltage target reached: %u V (target %u V, elapsed %lu ms)\n",
                      currentVoltageV,
                      targetVoltageV,
                      (unsigned long)(millis() - g_commandStartMs));
        RequestMotorStop();
        return;
    }

    g_targetPulseUs = GetVoltageControlPulseUs(
        currentVoltageV,
        targetVoltageV,
        g_voltageIncreaseTarget);

    if (IsVoltageUnchanged(currentVoltageV))
    {
        Serial.printf("[MOTOR] Voltage unchanged 5 times: stopped at %u V (target %u V, elapsed %lu ms)\n",
                      currentVoltageV,
                      targetVoltageV,
                      (unsigned long)(millis() - g_commandStartMs));
        RequestMotorStop();
    }
}

// ============================================================
// Task 4: MCU alive monitor
// ============================================================

void AliveMonitorTask()
{
    ++g_aliveCounter;

    Serial.printf(
        "[MCU] ESP8266 Alive! Counter: %lu\n",
        (unsigned long)g_aliveCounter
    );
}

// ============================================================
// Arduino setup
// ============================================================

void setup()
{
    Serial.begin(921600);
    delay(500);

    Serial.println();
    Serial.println("ESP8266 NodeMCU Started");

    pinMode(ADC_INPUT_PIN, INPUT);
    g_servo.attach(SERVO_PIN);
    g_servo.writeMicroseconds(SERVO_STOP_US); // stop at startup

    Serial.println("[INIT] Servo initialized on D2");
    Serial.println("[INIT] ADC voltage monitoring enabled on A0");
    Serial.println("[INIT] Binary RX protocol enabled");
}

// ============================================================
// Cooperative task scheduler
// ============================================================

void loop()
{
    static uint32_t lastRxTask = 0;
    static uint32_t lastMotorTask = 0;
    static uint32_t lastMotorMonitorTask = 0;
    static uint32_t lastAliveTask = 0;

    uint32_t now = millis();

    if (now - lastRxTask >= RX_TASK_INTERVAL_MS)
    {
        lastRxTask = now;
        SerialRxTask();
    }

    if (now - lastMotorTask >= MOTOR_TASK_INTERVAL_MS)
    {
        lastMotorTask = now;
        MotorControlTask();
    }

    if (now - lastMotorMonitorTask >= MOTOR_MONITOR_TASK_INTERVAL_MS)
    {
        lastMotorMonitorTask = now;
        MonitorConditionStopMotor();
    }

    if (now - lastAliveTask >= ALIVE_TASK_INTERVAL_MS)
    {
        lastAliveTask = now;
        AliveMonitorTask();
    }

    yield();
}
