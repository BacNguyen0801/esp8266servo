#include <Arduino.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif
#include <Esp.h>
#include "wifi_connection.h"

// ============================================================
// Configuration
// ============================================================

#if defined(ARDUINO_ARCH_ESP32)
constexpr uint8_t SERVO_PIN = 4; // ESP32 DevKit D2 GPIO
#else
constexpr uint8_t SERVO_PIN = D2; // ESP8266 NodeMCU D2 GPIO
#endif

constexpr uint32_t RX_TASK_INTERVAL_MS    = 10;
constexpr uint32_t MOTOR_TASK_INTERVAL_MS = 20;
constexpr uint32_t MOTOR_MONITOR_TASK_INTERVAL_MS = 10;
constexpr uint32_t ALIVE_TASK_INTERVAL_MS = 10000;
constexpr uint32_t WIFI_TASK_INTERVAL_MS = 100;
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 10000;

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
#if defined(ARDUINO_ARCH_ESP32)
constexpr uint8_t ADC_INPUT_PIN = 36; // ESP32 DevKit A0 / VP
constexpr uint8_t ADC_VOLTAGE_MIN_TARGET = 3;
constexpr uint8_t ADC_VOLTAGE_MAX_TARGET = 24;
constexpr uint8_t DAC_OUTPUT_PIN = 25;
#else
constexpr uint8_t ADC_INPUT_PIN = A0;
constexpr uint8_t ADC_VOLTAGE_MIN_TARGET = 3;
constexpr uint8_t ADC_VOLTAGE_MAX_TARGET = 24;
#endif
#if defined(ARDUINO_ARCH_ESP32)
struct AdcCalibrationPoint
{
    uint16_t raw;
    uint32_t physicalVoltageMv;
};

constexpr AdcCalibrationPoint ADC_CALIBRATION_TABLE[] = {
    {0, 0},
    {240, 3300},
    {409, 4700},
    {815, 8230},
    {1120, 10730},
    {1194, 11340},  
    {1412, 13180},
    {1530, 14150},
    {1709, 15600},
    {2014, 18220},
    {2336, 20000},
    {2813, 24800},
    {4095, 33000},
};
constexpr uint16_t ADC_FULL_SCALE_INPUT_MV = 4095;
constexpr uint16_t ADC_FULL_SCALE_PHYSICAL_MV = 33000;
#else
constexpr uint16_t ADC_FULL_SCALE_INPUT_MV = 1023;
constexpr uint16_t ADC_FULL_SCALE_PHYSICAL_MV = 33000;
#endif
constexpr uint8_t ADC_WINDOW_SIZE = 4U; // number of samples for moving average

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

constexpr uint32_t VOLTAGE_UNCHANGED_OFFSET_MV = 100;

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
volatile uint32_t g_voltageTargetMv = 0;
volatile uint32_t g_lastMeasuredVoltageMv = 0;
volatile uint8_t g_voltageUnchangedCount = 0;
volatile uint32_t g_lastVoltageMonitorMs = 0;

uint32_t g_aliveCounter = 0;

Servo g_servo;
WifiConnection g_wifiConnection;

// ============================================================
// Command processing
// ============================================================

uint32_t ReadMeasuredVoltageMv()
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
        (rawSum / sampleCount)
    );

    Serial.printf("[ADC] Raw: %u, Avg: %u, Samples: %u\n",
                  analogRead(ADC_INPUT_PIN),
                  averagedRaw,
                  sampleCount);

    if (averagedRaw >= ADC_FULL_SCALE_INPUT_MV)
        return ADC_FULL_SCALE_PHYSICAL_MV;

#if defined(ARDUINO_ARCH_ESP32)
    for (size_t i = 1; i < sizeof(ADC_CALIBRATION_TABLE) / sizeof(ADC_CALIBRATION_TABLE[0]); ++i)
    {
        const AdcCalibrationPoint &lower = ADC_CALIBRATION_TABLE[i - 1];
        const AdcCalibrationPoint &upper = ADC_CALIBRATION_TABLE[i];
        if (averagedRaw <= upper.raw)
        {
            return lower.physicalVoltageMv +
                (static_cast<uint32_t>(averagedRaw - lower.raw) *
                 (upper.physicalVoltageMv - lower.physicalVoltageMv)) /
                    (upper.raw - lower.raw);
        }
    }

    return ADC_CALIBRATION_TABLE[
        sizeof(ADC_CALIBRATION_TABLE) / sizeof(ADC_CALIBRATION_TABLE[0]) - 1
    ].physicalVoltageMv;
#else
    return (static_cast<uint32_t>(averagedRaw) * ADC_FULL_SCALE_PHYSICAL_MV) / ADC_FULL_SCALE_INPUT_MV;
#endif
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

bool IsVoltageTargetReached(uint32_t currentVoltageMv,
                            bool increaseTarget,
                            uint32_t targetVoltageMv)
{
    if (increaseTarget)
    {
        return currentVoltageMv >= targetVoltageMv;
    }

    return currentVoltageMv <= targetVoltageMv;
}

uint16_t GetVoltageControlPulseUs(uint32_t currentVoltageMv,
                                  uint32_t targetVoltageMv,
                                  bool increaseTarget)
{
    const uint32_t voltageGapMv = currentVoltageMv > targetVoltageMv
        ? currentVoltageMv - targetVoltageMv
        : targetVoltageMv - currentVoltageMv;
    const uint16_t voltageGapTenths =
        static_cast<uint16_t>((voltageGapMv + 50U) / 100U);
    const uint16_t maxGapIndex = (sizeof(g_pulseMotorTableReference) / sizeof(g_pulseMotorTableReference[0])) - 1;
    const uint16_t gapIndex = voltageGapTenths < maxGapIndex
        ? voltageGapTenths
        : maxGapIndex;
    const uint16_t pulseOffsetUs = g_pulseMotorTableReference[gapIndex];

    return increaseTarget
        ? SERVO_STOP_US + pulseOffsetUs
        : SERVO_STOP_US - pulseOffsetUs;
}

bool IsVoltageUnchanged(uint32_t currentVoltageMv)
{
    if (g_lastMeasuredVoltageMv == 0)
    {
        g_lastMeasuredVoltageMv = currentVoltageMv;
        g_voltageUnchangedCount = 0;
        return false;
    }

    const uint32_t voltageDifferenceMv =
        currentVoltageMv > g_lastMeasuredVoltageMv
            ? currentVoltageMv - g_lastMeasuredVoltageMv
            : g_lastMeasuredVoltageMv - currentVoltageMv;

    if (voltageDifferenceMv <= VOLTAGE_UNCHANGED_OFFSET_MV)
    {
        uint8_t unchangedCount = g_voltageUnchangedCount;
        if (unchangedCount < 255)
            g_voltageUnchangedCount = unchangedCount + 1;
    }
    else
    {
        g_lastMeasuredVoltageMv = currentVoltageMv;
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
    g_voltageTargetMv = 0;
    g_lastMeasuredVoltageMv = 0;
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
                g_voltageTargetMv = 0;
                g_newCommand = true;

                Serial.printf("[RX] Set servo direction: %s -> %u us\n",
                              (value == 0x00) ? "clockwise" : "anticlockwise",
                              pulseUs);
                break;
            }

            if (cmdHi == CMD_SET_VOLTAGE_TARGET_HI && cmdLow == CMD_SET_VOLTAGE_TARGET_LOW)
            {
                if (value < ADC_VOLTAGE_MIN_TARGET || value > ADC_VOLTAGE_MAX_TARGET)
                {
#if defined(ARDUINO_ARCH_ESP32)
                    Serial.printf("[RX] Invalid target voltage: %u (valid 3..24 V)\n", value);
#else
                    Serial.printf("[RX] Invalid target voltage: %u (valid 3..24 V)\n", value);
#endif
                    return;
                }

                const uint32_t currentVoltageMv = ReadMeasuredVoltageMv();
                const uint32_t targetVoltageMv =
                    static_cast<uint32_t>(value) * 1000U;
                const bool shouldIncrease = currentVoltageMv < targetVoltageMv;

                if (IsVoltageTargetReached(currentVoltageMv, shouldIncrease, targetVoltageMv))
                {
                    Serial.printf("[RX] Voltage target already reached: %lu mV (target %lu mV)\n",
                                  (unsigned long)currentVoltageMv,
                                  (unsigned long)targetVoltageMv);
                    CompleteVoltageAdjustment();
                    break;
                }

                g_targetPulseUs = GetVoltageControlPulseUs(
                    currentVoltageMv, targetVoltageMv, shouldIncrease);
                g_commandStartMs = millis();
                g_voltageControlActive = true;
                g_stopMotorRequested = false;
                g_voltageIncreaseTarget = shouldIncrease;
                g_voltageTargetMv = targetVoltageMv;
                g_lastMeasuredVoltageMv = currentVoltageMv;
                g_voltageUnchangedCount = 0;
                g_lastVoltageMonitorMs = g_commandStartMs;
                g_newCommand = true;

                Serial.printf("[RX] Set voltage target: %lu mV, current %lu mV -> %s\n",
                              (unsigned long)targetVoltageMv,
                              (unsigned long)currentVoltageMv,
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
    const uint32_t currentVoltageMv = ReadMeasuredVoltageMv();
    const uint32_t targetVoltageMv = g_voltageTargetMv;

    if (IsVoltageTargetReached(currentVoltageMv, g_voltageIncreaseTarget, targetVoltageMv))
    {
        Serial.printf("[MOTOR] Voltage target reached: %lu mV (target %lu mV, elapsed %lu ms)\n",
                      (unsigned long)currentVoltageMv,
                      (unsigned long)targetVoltageMv,
                      (unsigned long)(millis() - g_commandStartMs));
        RequestMotorStop();
        return;
    }

    g_targetPulseUs = GetVoltageControlPulseUs(
        currentVoltageMv,
        targetVoltageMv,
        g_voltageIncreaseTarget);

    if (IsVoltageUnchanged(currentVoltageMv))
    {
        Serial.printf("[MOTOR] Voltage unchanged 5 times: stopped at %lu mV (target %lu mV, elapsed %lu ms)\n",
                      (unsigned long)currentVoltageMv,
                      (unsigned long)targetVoltageMv,
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
        "[MCU] eMotor Alive! Counter: %lu\n",
        (unsigned long)g_aliveCounter
    );
}

void WifiConnectionTask()
{
    static bool connectionAttempted = false;
    static uint32_t lastConnectionAttemptMs = 0;

    const bool alreadyConnected = g_wifiConnection.isConnected();
    const bool credentialsChanged = g_wifiConnection.isChanged();
    if (g_wifiConnection.getSsid().length() == 0 ||
        (alreadyConnected && !credentialsChanged))
        return;

    const uint32_t now = millis();
    if (connectionAttempted && !credentialsChanged &&
        now - lastConnectionAttemptMs < WIFI_RETRY_INTERVAL_MS)
        return;

    const bool started = connectionAttempted || credentialsChanged
        ? g_wifiConnection.reconnect()
        : g_wifiConnection.connect();
    if (started)
    {
        connectionAttempted = true;
        lastConnectionAttemptMs = now;
    }
}

// ============================================================
// Arduino setup
// ============================================================

void setup()
{
    Serial.begin(921600);
    delay(500);
    pinMode(ADC_INPUT_PIN, INPUT);
#if defined(ARDUINO_ARCH_ESP32)
    dacWrite(DAC_OUTPUT_PIN, 85); // Set DAC output to mid-scale (1.65V) for reference
    analogReadResolution(12);
    analogSetPinAttenuation(ADC_INPUT_PIN, ADC_11db);
#endif
    g_servo.attach(SERVO_PIN);
    g_servo.writeMicroseconds(SERVO_STOP_US); // stop at startup

    Serial.print("[INIT] Servo initialized on ");
#if defined(ARDUINO_ARCH_ESP32)
    Serial.println("ESP32 GPIO 4");
    Serial.println("[INIT] ADC voltage monitoring enabled on ESP32 GPIO 36");
#else
    Serial.println("ESP8266 D2");
    Serial.println("[INIT] ADC voltage monitoring enabled on ESP8266 A0");
#endif
    Serial.println("[INIT] Binary RX protocol enabled");

    if (g_wifiConnection.begin())
    {
        Serial.println("[INIT] Wi-Fi storage initialized");
    }
    else
    {
        Serial.println("[INIT] Wi-Fi storage initialization failed");
    }
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
    static uint32_t lastWifiTask = 0;

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

    if (now - lastWifiTask >= WIFI_TASK_INTERVAL_MS)
    {
        lastWifiTask = now;
        WifiConnectionTask();
    }

    yield();
}
