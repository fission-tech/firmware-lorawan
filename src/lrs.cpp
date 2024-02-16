/*
 *  lora_device.cpp
 *  Sensible LoRa Remote Switcher - Combined Transmitter and Receiver
 *
 * This code initializes the LoRa module and the switch in the setup() function.
 * If the device is configured as a transmitter, in the loop() function, it reads the state of the dry contact and sends it as a LoRa packet every second.
 * If the device is configured as a receiver, it listens for incoming LoRa packets and toggles the relay accordingly.
 */

// Include the SPI and LoRa libraries
#include <SPI.h>        // SPI library which is required by LoRa
#include <LoRa.h>       // LoRa library which is required for LoRa communication
#include <ArduinoOTA.h> // ArduinoOTA library which is required for OTA updates
#if defined(ESP8266)
/* ESP8266 Dependencies */
#include <ESP8266WiFi.h> // WiFi library which is required for WiFi communication
#include <ESP8266mDNS.h> // ESP8266mDNS library which is required for mDNS
#elif defined(ESP32)
/* ESP32 Dependencies */
#include <WiFi.h>    // WiFi library which is required for WiFi communication
#include <ESPmDNS.h> // ESP8266mDNS library which is required for mDNS
#endif

// #include <EEPROM.h>
// #include <EEPROMWearLevel.h>   // EEPROMWearLevel library which is required for wear leveling of EEPROM
#include "commonFunctions.h"

// Set to true for transmitter, false for receiver
bool isTransmitter = true;
//bool isTransmitter = false;

byte txAddress = 0xFF; // address of this device
byte rxAddress = 0x07; // destination to send to

// Set to true to enable WiFi on the ESP8266
bool enableWiFi = false;

String defaultAPssid = "Sensible IOT"; // default AP SSID
String defaultAPpassword = "13371337"; // default AP password

uint16_t relayStateControlId; // Declare the control ID globally

// Define the pins for the LoRa module
#define nss 15 // LoRa chip select
#define rst 16 // LoRa reset
#define dio0 0 // LoRa DIO0

// Define the pin for the dry contact input
const int sw1 = 4;

// Define the pin for the relay
const int RLY1 = 5; // replace with your relay pin

// Declare a variable to store the previous state of the dry contact
int previousState = -1;

// Define the debounce delay (in milliseconds)
const unsigned long debounceDelay = 50; // adjust as needed

// Declare variables to store the cry contact state and the last debounce time
int sw1State;
int lastSw1State = LOW;
unsigned long lastDebounceTime = 0;

// Declare a variable to store the previous time a heartbeat was sent
unsigned long previousMillisHeartbeat = 0;

// The setup function runs once when you press reset or power the board
void setup()
{
    // Begin serial communication at 115200 baud
    Serial.begin(115200);

    // Wait for serial port to connect
    while (!Serial)
        ;

    if (isTransmitter)
    {
        // Transmitter-specific setup code...
        // Set the dry contact pin as an input
        pinMode(sw1, INPUT);
    }
    /* else
    {
        // Receiver-specific setup code...
    } */
    // Set the LED pin as an output
    pinMode(2, OUTPUT);

    // Set the relay pin as an output - on tx only if local echo is enabled
    pinMode(RLY1, OUTPUT);

    // Set the LoRa module pins
    LoRa.setPins(nss, rst, dio0);

    // Wait
    delay(100);

    // Initialize the LoRa module at 433 MHz
    LoRa.begin(433E6) ? Serial.println("LoRa started successfully!") : Serial.println("LoRa startup failed!");

    // Announce startup
    isTransmitter ? Serial.println("TX - Transmitter started!") : Serial.println("RX - Receiver started!");

    if (enableWiFi)
    {
        /* Connect WiFi */
        WiFi.mode(WIFI_STA);
        WiFi.begin(defaultAPssid, defaultAPpassword);
        if (WiFi.waitForConnectResult() != WL_CONNECTED)
        {
            Serial.printf("WiFi Failed!\n");
            return;
        }
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());

        // Initialise OTA update service
        ArduinoOTA.onStart([]()
                           {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type); });

        ArduinoOTA.onEnd([]()
                         { Serial.println("\nEnd"); });

        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                              { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });

        ArduinoOTA.onError([](ota_error_t error)
                           {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

        // Start OTA update service
        ArduinoOTA.begin();

        // Update mDNS
        MDNS.update();

        // Start mDNS
        if (MDNS.begin("lora"))
        {
            Serial.println("mDNS responder started. Connect to http://lora.local to view the Dashboard.");
        }
        else
        {
            Serial.println("Error setting up MDNS responder!");
        }
    }
}

// The loop function runs over and over again forever
void loop()
{
    // Update the LED
    updateLED();

    if (enableWiFi)
    {
        // handle OTA update requests
        ArduinoOTA.handle();
    }

    if (isTransmitter)
    {
        // Transmitter-specific loop code...
        // Get the current time
        unsigned long currentMillis = millis();

        if (currentMillis - previousMillisHeartbeat >= 60000)
        {

            // Update the previous time a heartbeat was sent
            previousMillisHeartbeat = currentMillis;
        }
        else
        {
            // Read the state of the switch
            int reading = digitalRead(sw1);

            // If the switch state has changed, reset the debounce timer
            if (reading != lastSw1State)
            {
                lastDebounceTime = millis();
            }

            // If more than debounceDelay milliseconds have passed since the switch state last changed,
            // the switch is considered to be in a stable state
            if ((millis() - lastDebounceTime) > debounceDelay)
            {
                // If the switch state has changed, update the switch state
                if (reading != sw1State)
                {
                    sw1State = reading;

                    // If the state has changed
                    if (sw1State != previousState)
                    {
                        sendRelayState(rxAddress, txAddress, sw1State);

                        // Update the previous state with the current state
                        previousState = sw1State;

                        // Debug
                        Serial.println("TX - Saved changed relay state: " + String(sw1State));
                        // Set the state of the relay according to the received value
                        digitalWrite(RLY1, sw1State);

                        // Debug
                        Serial.println("TX - Set relay state: " + String(sw1State));

                        // Debug
                        Serial.println("TX - RSSI: " + String(LoRa.packetRssi()));
                    }
                }
            }
            // Save the reading for the next loop iteration
            lastSw1State = reading;
        }
    }
    else
    {
        // Receiver-specific loop code...
        // Check if a LoRa packet is available
        int packetSize = LoRa.parsePacket();
        if (packetSize)
        {
            // If a packet is available, read the destination address
            byte destinationAddress = LoRa.read();

            // Read the sender address
            byte senderAddress = LoRa.read();

            // Read the message type
            char messageType = LoRa.read();

            // If the destination address matches the local address
            if (destinationAddress == rxAddress && senderAddress == txAddress)
            {
                // If the address matches, read the rest of the message into a string
                String received = "";
                while (LoRa.available())
                {
                    received += (char)LoRa.read();
                }

                if (messageType == 'R')
                {
                    // Convert the received string to an integer
                    int relayState = received.toInt();

                    // Set the state of the relay according to the received value
                    digitalWrite(RLY1, relayState);

                    // Debug
                    Serial.println("RX - RSSI: " + String(LoRa.packetRssi()));

                    // Debug
                    Serial.println("RX - Set relay state: " + String(relayState));
                }
            }
        }
    }
}