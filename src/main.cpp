#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <ESP8266WiFi.h>
#include <espnow.h>

MPU6050 mpu;

const int INTERRUPT_PIN = D8; // MPU6050 interrupt pin
volatile bool MPUInterrupt = false;

// MPU6050 control/status variables
bool DMPReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

// Orientation/Motion variables
Quaternion q;
VectorFloat gravity;
float ypr[3];

// ESP-NOW communication
uint8_t receiverMAC[] = {0x74, 0x4D, 0xBD, 0xAA, 0x7A, 0x98};

struct Message {
    float yaw, pitch, roll;
};
Message myData;

// Interrupt handler for MPU6050 data ready
void IRAM_ATTR DMPDataReady() {
    MPUInterrupt = true;
}

bool blinkState = false;

// ESP-NOW send callback
void onSent(uint8_t *macAddr, uint8_t status) {
    Serial.printf("ESP-NOW Send Status: %s\n", status == 0 ? "Success" : "Fail");
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock
    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialize ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != 0) {
        Serial.println("ESP-NOW initialization failed!");
        return;
    }
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_register_send_cb(onSent);
    esp_now_add_peer(receiverMAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    Serial.println("ESP-NOW initialized.");

    // Initialize MPU6050
    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (true);
    }
    Serial.println("MPU6050 connected.");

    // Initialize DMP
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println("MPU6050 Calibration Complete.");
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println("DMP initialized and ready.");
    } else {
        Serial.printf("DMP Initialization failed (code %d)\n", devStatus);
    }
}

void loop() {
    if (!DMPReady) return;

    // Read data from MPU6050 if available
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        myData.yaw = ypr[0] * 180 / M_PI;
        myData.pitch = ypr[1] * 180 / M_PI;
        myData.roll = ypr[2] * 180 / M_PI;

        // Send data via ESP-NOW
        esp_now_send(receiverMAC, (uint8_t *)&myData, sizeof(myData));
        Serial.printf("Yaw: %.2f, Pitch: %.2f, Roll: %.2f\n", myData.yaw, myData.pitch, myData.roll);
    }
    delay(20);
}
