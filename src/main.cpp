
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

// ============================================================
// UDS-style Service IDs
// ============================================================

constexpr uint8_t SID_SET_SERVO_ANGLE = 0x31;

// ============================================================
// Shared data
// ============================================================

volatile uint16_t g_targetPulseUs = SERVO_STOP_US;
volatile bool g_newCommand = false;
volatile uint32_t g_commandStartMs = 0;

uint16_t g_currentPulseUs = SERVO_STOP_US;
uint32_t g_aliveCounter = 0;

Servo g_servo;

// ============================================================
// Command processing
// ============================================================

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
            uint8_t direction = frame[4];

            if (channel != 0x01)
            {
                Serial.printf("[RX] Unsupported channel: %u\n", channel);
                return;
            }

            if (cmdHi != 0xF0 || cmdLow != 0x01)
            {
                Serial.printf("[RX] Unsupported command: %02X %02X\n", cmdHi, cmdLow);
                return;
            }

            uint16_t pulseUs;
            switch (direction)
            {
                case 0x00:
                    pulseUs = SERVO_CLOCKWISE_US;
                    break;

                case 0x01:
                    pulseUs = SERVO_ANTICLOCKWISE_US;
                    break;

                default:
                    Serial.printf("[RX] Invalid direction: 0x%02X\n", direction);
                    return;
            }

            g_targetPulseUs = pulseUs;
            g_commandStartMs = millis();
            g_newCommand = true;

            Serial.printf("[RX] Set servo direction: %s -> %u us\n",
                          (direction == 0x00) ? "clockwise" : "anticlockwise",
                          pulseUs);
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

    uint32_t elapsedMs = millis() - g_commandStartMs;

    if (elapsedMs >= SERVO_RUN_DURATION_MS)
    {
        g_servo.writeMicroseconds(SERVO_STOP_US);
        g_newCommand = false;
        g_currentPulseUs = SERVO_STOP_US;
        Serial.printf("[MOTOR] Stop after %lu ms -> %u us\n",
                      (unsigned long)SERVO_RUN_DURATION_MS,
                      SERVO_STOP_US);
        return;
    }

    uint16_t pulseUs = g_targetPulseUs;
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

    g_servo.attach(SERVO_PIN);
    g_servo.writeMicroseconds(1500); // stop at startup

    Serial.println("[INIT] Servo initialized on D2");
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
