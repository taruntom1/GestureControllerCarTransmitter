#include <Arduino.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <ESP8266WiFi.h>
#include <espnow.h>

MPU6050 mpu;

int const INTERRUPT_PIN = D8; // Define the interruption #0 pin
bool blinkState;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;        // [w, x, y, z]         Quaternion container
VectorFloat gravity; // [x, y, z]            Gravity vector
float ypr[3];        // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false; // Indicates whether MPU6050 interrupt pin has gone high

void IRAM_ATTR DMPDataReady()
{
    MPUInterrupt = true;
}
/////////////////////////////////////////////////////////////////
// Define receiver MAC address (ESP32 MAC address)
uint8_t receiverMAC[] = {0x74, 0x4D, 0xBD, 0xAA, 0x7A, 0x98};

// Define message structure
typedef struct Message
{
    float yaw, pitch, roll;
} Message;

Message myData;

void onSent(uint8_t *macAddr, uint8_t status)
{
    Serial.print("Send Status: ");
    Serial.println(status == 0 ? "Success" : "Fail");
}
///////////////////////////////////////////////////////////
void setup()
{
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties

    Serial.begin(115200); // 115200 is required for Teapot Demo output
                          ////////////////////////////////////////////////////
    WiFi.mode(WIFI_STA);  // Set ESP8266 as Wi-Fi Station

    if (esp_now_init() != 0)
    {
        Serial.println("ESP-NOW Init Failed");
        return;
    }

    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_register_send_cb(onSent);

    esp_now_add_peer(receiverMAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    //////////////////////////////////////////////////////////

    /*Initialize device*/
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    /*Verify connection*/
    Serial.println(F("Testing MPU6050 connection..."));
    if (mpu.testConnection() == false)
    {
        Serial.println("MPU6050 connection failed");
        while (true)
            ;
    }
    else
    {
        Serial.println("MPU6050 connection successful");
    }

    /* Initializate and configure the DMP*/
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    /* Supply your gyro offsets here, scaled for min sensitivity */
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    /* Making sure it worked (returns 0 if so) */
    if (devStatus == 0)
    {
        mpu.CalibrateAccel(6); // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateGyro(6);
        Serial.println("These are the Active offsets: ");
        mpu.PrintActiveOffsets();
        Serial.println(F("Enabling DMP...")); // Turning ON DMP
        mpu.setDMPEnabled(true);

        /*Enable Arduino interrupt detection*/
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
        MPUIntStatus = mpu.getIntStatus();

        /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        DMPReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size for later comparison
    }
    else
    {
        Serial.print(F("DMP Initialization failed (code ")); // Print the error code
        Serial.print(devStatus);
        Serial.println(F(")"));
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
    }
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    if (!DMPReady)
        return; // Stop the program if DMP programming fails.

    /* Read a packet from FIFO */
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer))
    { // Get the Latest packet

        /* Display Euler angles in degrees */
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        /* Blink LED to indicate activity */
        blinkState = !blinkState;
        digitalWrite(LED_BUILTIN, blinkState);

        myData.yaw = ypr[0] * 180 / M_PI;
        myData.pitch = ypr[1] * 180 / M_PI;
        myData.roll = ypr[2] * 180 / M_PI;

        esp_now_send(receiverMAC, (uint8_t *)&myData, sizeof(myData));
    }
    delay(20);
}
