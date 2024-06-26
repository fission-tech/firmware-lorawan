/*
 *  main.cpp
 *  Sensible LoRa Remote Switcher - Combined Transmitter and Receiver
 *
 * This code initializes the LoRa module and the switch in the setup() function.
 * If the device is configured as a transmitter, it reads the state of the dry contact and transmit it to the specified receiver.
 * If the device is configured as a receiver, it listens for incoming LoRa packets and toggles the relay accordingly.
 * The code includes a heartbeat, ackknowledgement and timeout mechanism to ensure the reliability of the communication.
 */

#include "functions.h"

// The setup function runs once when you press reset or power the board
void setup()
{
    // Begin serial communication at 115200 baud
    Serial.begin(115200);

    // Wait for serial port to connect
    while (!Serial)
        ;

    // Wait
    delay(1000);

    if (isTransmitter)
    {
        // TODO: Write this to EEPROM and provide web interface to read it on TX and write it on RX
        // Generate they encryption key
        // generateKeyFromChipId();

        // Set the dry contact pin as an input
        pinMode(INP1, INPUT);
    }

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

    // Print the encryption key
    printKey();

    // Announce startup
    isTransmitter ? Serial.println("TX - Transmitter started!") : Serial.println("RX - Receiver started!");

    client.setServer(mqtt_server, Port);


}


// The loop function runs over and over again forever
void loop()
{
    if (enableWiFi)
    {
        // handle OTA update requests
        ArduinoOTA.handle();
    }

    client.loop();
    handleTransceiver();
    // Blink the LED
    updateLED();
}