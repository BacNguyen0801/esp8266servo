
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

// ============================================================
// UDS-style Service IDs
// ============================================================

constexpr uint8_t SID_SET_SERVO_ANGLE = 0x31;

// ============================================================
// Shared data
// ============================================================

volatile uint8_t g_targetAngle = 90;
volatile bool g_newCommand = false;

uint8_t g_currentAngle = 90;
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
            if (length != 2)
            {
                Serial.println("[RX] Invalid command length");
                return;
            }

            uint8_t angle = data[1];

            if (angle > 180)
            {
                Serial.println("[RX] Invalid servo angle");
                return;
            }

            g_targetAngle = angle;
            g_newCommand = true;

            Serial.printf("[RX] Set servo angle: %u\n", angle);
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
    static uint8_t expectedLength = 0;
    static uint8_t receivedLength = 0;

    while (Serial.available() > 0)
    {
        uint8_t rxByte = Serial.read();

        if (expectedLength == 0)
        {
            if (rxByte == 0 || rxByte > MAX_FRAME_LENGTH)
            {
                Serial.printf("[RX] Invalid length: %u\n", rxByte);
                continue;
            }

            expectedLength = rxByte;
            receivedLength = 0;
            continue;
        }

        rxBuffer[receivedLength++] = rxByte;

        if (receivedLength == expectedLength)
        {
            Serial.print("[RX] Frame: ");

            for (uint8_t i = 0; i < expectedLength; i++)
                Serial.printf("%02X ", rxBuffer[i]);

            Serial.println();

            ProcessCommand(rxBuffer, expectedLength);

            expectedLength = 0;
            receivedLength = 0;
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

    uint8_t targetAngle = g_targetAngle;
    g_newCommand = false;

    g_servo.write(targetAngle);
    g_currentAngle = targetAngle;

    Serial.printf("[MOTOR] Angle: %u degrees\n", g_currentAngle);
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
    g_servo.write(g_currentAngle);

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
