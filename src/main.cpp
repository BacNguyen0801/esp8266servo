
#include <Arduino.h>
#include <Servo.h>

// ============================================================
// Configuration
// ============================================================

constexpr uint8_t SERVO_PIN = 4;

constexpr uint32_t RX_TASK_INTERVAL_MS    = 10;
constexpr uint32_t MOTOR_TASK_INTERVAL_MS = 20;
constexpr uint32_t ALIVE_TASK_INTERVAL_MS = 1000;

constexpr uint8_t MAX_FRAME_LENGTH = 8;
constexpr uint8_t COMMAND_LENGTH_BYTE = 0x05;
constexpr uint8_t COMMAND_FRAME_LENGTH = 5; // [SID, CH, 0xF0, 0x01, DIRECTION]
constexpr uint8_t COMMAND_FRAME_WITH_LENGTH_LENGTH = 6; // [LEN, SID, CH, 0xF0, 0x01, DIRECTION]

constexpr uint16_t SERVO_STOP_US = 1500;
constexpr uint16_t SERVO_CLOCKWISE_US = 1600;
constexpr uint16_t SERVO_ANTICLOCKWISE_US = 1400;
constexpr uint32_t SERVO_RUN_DURATION_MS = 100;
constexpr uint8_t ADC_INPUT_PIN = A0;
constexpr uint8_t ADC_VOLTAGE_MIN_TARGET_TENTHS = 3;  // 0.3V in 0.1V units
constexpr uint8_t ADC_VOLTAGE_MAX_TARGET_TENTHS = 24; // 2.4V in 0.1V units
constexpr uint16_t ADC_MAX_RAW = 1023;

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

// ============================================================
// Shared data
// ============================================================

volatile uint16_t g_targetPulseUs = SERVO_STOP_US;
volatile bool g_newCommand = false;
volatile uint32_t g_commandStartMs = 0;
volatile bool g_voltageControlActive = false;
volatile bool g_voltageIncreaseTarget = false;
volatile uint8_t g_voltageTargetV = 0;

uint16_t g_currentPulseUs = SERVO_STOP_US;
uint32_t g_aliveCounter = 0;

Servo g_servo;

// ============================================================
// Command processing
// ============================================================

uint16_t ReadMeasuredVoltageV()
{
    const uint16_t raw = analogRead(ADC_INPUT_PIN);
    return (static_cast<uint32_t>(raw) * ADC_VOLTAGE_MAX_TARGET_TENTHS + (ADC_MAX_RAW / 2)) / ADC_MAX_RAW;
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

bool IsVoltageTargetReached(uint16_t currentVoltageV, bool increaseTarget, uint8_t targetVoltageV)
{
    return increaseTarget
        ? (currentVoltageV >= static_cast<uint16_t>(targetVoltageV))
        : (currentVoltageV <= static_cast<uint16_t>(targetVoltageV));
}

void CompleteVoltageAdjustment()
{
    g_servo.writeMicroseconds(SERVO_STOP_US);
    g_newCommand = false;
    g_voltageControlActive = false;
    g_voltageIncreaseTarget = false;
    g_voltageTargetV = 0;
    g_currentPulseUs = SERVO_STOP_US;

    SendPositiveUdsResponse(CMD_SET_VOLTAGE_TARGET_HI, CMD_SET_VOLTAGE_TARGET_LOW);
}

void ProcessCommand(const uint8_t *data, uint8_t length)
{
    if (length == 0)
        return;

    const uint8_t *frame = data;
    uint8_t frameLength = length;

    if (length == COMMAND_FRAME_WITH_LENGTH_LENGTH)
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
                g_targetPulseUs = shouldIncrease ? SERVO_CLOCKWISE_US : SERVO_ANTICLOCKWISE_US;
                g_commandStartMs = millis();
                g_voltageControlActive = true;
                g_voltageIncreaseTarget = shouldIncrease;
                g_voltageTargetV = value;
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
            if (rxByte != SID_SET_SERVO_ANGLE && rxByte != COMMAND_LENGTH_BYTE)
            {
                Serial.printf("[RX] Invalid header: 0x%02X\n", rxByte);
                continue;
            }

            rxBuffer[receivedLength++] = rxByte;
            expectedLength = (rxByte == COMMAND_LENGTH_BYTE) ? COMMAND_FRAME_WITH_LENGTH_LENGTH : COMMAND_FRAME_LENGTH;
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
        return;

    const uint32_t elapsedMs = millis() - g_commandStartMs;

    if (g_voltageControlActive)
    {
        const uint16_t currentVoltageV = ReadMeasuredVoltageV();

        if (IsVoltageTargetReached(currentVoltageV, g_voltageIncreaseTarget, g_voltageTargetV))
        {
            CompleteVoltageAdjustment();
            Serial.printf("[MOTOR] Voltage target reached: %u V (target %u V, elapsed %lu ms)\n",
                          currentVoltageV,
                          g_voltageTargetV,
                          (unsigned long)elapsedMs);
            return;
        }

        if (elapsedMs >= SERVO_RUN_DURATION_MS)
        {
            g_servo.writeMicroseconds(SERVO_STOP_US);
            g_currentPulseUs = SERVO_STOP_US;

            const uint16_t measuredAfterStopV = ReadMeasuredVoltageV();
            const bool finishedAfterStop = IsVoltageTargetReached(
                measuredAfterStopV,
                g_voltageIncreaseTarget,
                g_voltageTargetV
            );

            Serial.printf("[MOTOR] Step complete: %u V / %u V -> %s\n",
                          measuredAfterStopV,
                          g_voltageTargetV,
                          finishedAfterStop ? "done" : "continue");

            if (finishedAfterStop)
            {
                CompleteVoltageAdjustment();
                return;
            }

            g_commandStartMs = millis();
            return;
        }

        const bool needIncrease = currentVoltageV < static_cast<uint16_t>(g_voltageTargetV);
        const uint16_t pulseUs = needIncrease ? SERVO_CLOCKWISE_US : SERVO_ANTICLOCKWISE_US;

        g_servo.writeMicroseconds(pulseUs);
        g_currentPulseUs = pulseUs;

        Serial.printf("[MOTOR] Voltage step: %u V / %u V -> %s, elapsed %lu ms\n",
                      currentVoltageV,
                      g_voltageTargetV,
                      needIncrease ? "clockwise" : "anticlockwise",
                      (unsigned long)elapsedMs);
        return;
    }

    if (elapsedMs >= SERVO_RUN_DURATION_MS)
    {
        g_servo.writeMicroseconds(SERVO_STOP_US);
        g_newCommand = false;
        g_currentPulseUs = SERVO_STOP_US;

        SendPositiveUdsResponse(CMD_SET_SERVO_DIRECTION_HI, CMD_SET_SERVO_DIRECTION_LOW);

        Serial.printf("[MOTOR] Stop after %lu ms -> %u us\n",
                      (unsigned long)SERVO_RUN_DURATION_MS,
                      SERVO_STOP_US);
        return;
    }

    const uint16_t pulseUs = g_targetPulseUs;
    g_servo.writeMicroseconds(pulseUs);
    g_currentPulseUs = pulseUs;

    Serial.printf("[MOTOR] Active pulse: %u us (elapsed %lu ms)\n",
                  pulseUs,
                  (unsigned long)elapsedMs);
}

// ============================================================
// Task 3: MCU alive monitor
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
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("ESP8266 NodeMCU Started");

    pinMode(ADC_INPUT_PIN, INPUT);
    g_servo.attach(SERVO_PIN);
    g_servo.writeMicroseconds(1500); // stop at startup

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

    if (now - lastAliveTask >= ALIVE_TASK_INTERVAL_MS)
    {
        lastAliveTask = now;
        AliveMonitorTask();
    }

    yield();
}
