
#include <Arduino.h>
#include <Servo.h>

// ============================================================
// Configuration
// ============================================================

constexpr uint8_t SERVO_PIN = D2;

constexpr uint32_t RX_TASK_INTERVAL_MS    = 10;
constexpr uint32_t MOTOR_TASK_INTERVAL_MS = 20;
constexpr uint32_t ALIVE_TASK_INTERVAL_MS = 1000;

constexpr uint8_t MAX_FRAME_LENGTH = 8;
constexpr uint8_t COMMAND_FRAME_LENGTH = 6; // [SID, CH, 0xF0, 0x01, VALUE_LOW, VALUE_HIGH]

// ============================================================
// UDS-style Service IDs
// ============================================================

constexpr uint8_t SID_SET_SERVO_ANGLE = 0x31;

// ============================================================
// Shared data
// ============================================================

volatile uint16_t g_targetAngle = 90;
volatile bool g_newCommand = false;

uint16_t g_currentAngle = 90;
uint32_t g_aliveCounter = 0;

Servo g_servo;

// ============================================================
// Command processing
// ============================================================

void ProcessCommand(const uint8_t *data, uint8_t length)
{
    if (length == 0)
        return;

    uint8_t sid = data[0];

    switch (sid)
    {
        case SID_SET_SERVO_ANGLE:
        {
            if (length != COMMAND_FRAME_LENGTH)
            {
                Serial.println("[RX] Invalid command length");
                return;
            }

            uint8_t channel = data[1];
            uint8_t cmdHi = data[2];
            uint8_t cmdLow = data[3];
            uint8_t valueHigh = data[4];
            uint8_t valueLow = data[5];
            uint16_t angle = ((uint16_t)valueHigh << 8) | valueLow;

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

                    if (angle > 360)
            {
                Serial.println("[RX] Invalid servo angle");
                return;
            }

            g_targetAngle = angle;
            g_newCommand = true;

            Serial.printf("[RX] Set servo value: %u\n", angle);
            break;
        }

        default:
            Serial.printf("[RX] Unsupported SID: 0x%02X\n", sid);
            break;
    }
}

// ============================================================
// Task 1: Serial RX
// Frame: LEN | SID | PARAM...
// ============================================================

void SerialRxTask()
{
    static uint8_t rxBuffer[MAX_FRAME_LENGTH];
    static uint8_t receivedLength = 0;

    while (Serial.available() > 0)
    {
        uint8_t rxByte = Serial.read();

        if (receivedLength == 0)
        {
            if (rxByte != SID_SET_SERVO_ANGLE)
            {
                Serial.printf("[RX] Invalid header: 0x%02X\n", rxByte);
                continue;
            }
        }

        rxBuffer[receivedLength++] = rxByte;

        if (receivedLength == COMMAND_FRAME_LENGTH)
        {
            Serial.print("[RX] Frame: ");

            for (uint8_t i = 0; i < receivedLength; i++)
                Serial.printf("%02X ", rxBuffer[i]);

            Serial.println();

            ProcessCommand(rxBuffer, receivedLength);

            receivedLength = 0;
        }
    }
}

// ============================================================
// Task 2: Servo motor control
// ============================================================

uint16_t ValueToMicroseconds(uint16_t value)
{
    // SG90 360° continuous rotation:
    // 0 -> 1000 (full counter-clockwise)
    // 180 -> 1500 (stop)
    // 360 -> 2000 (full clockwise)
    if (value > 360)
        value = 360;

    // map 0..360 to 1000..2000 with 180 => 1500
    int32_t us = 1000 + ((int32_t)value * 1000 / 360);
    return constrain((uint16_t)us, 1000U, 2000U);
}

void servoSetSpeed(uint16_t value)
{
    uint16_t us = ValueToMicroseconds(value);
    g_servo.writeMicroseconds(us);
}

void MotorControlTask()
{
    if (!g_newCommand)
        return;

    uint16_t targetAngle = g_targetAngle;
    g_newCommand = false;

    uint16_t us = ValueToMicroseconds(targetAngle);
    g_servo.writeMicroseconds(us);

    g_currentAngle = targetAngle;

    Serial.printf("[MOTOR] Raw value: %u -> us: %u\n", g_currentAngle, us);
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
