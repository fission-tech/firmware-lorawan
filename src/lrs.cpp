/*
 *  lora_device.cpp
 *  Sensible LoRa Remote Switcher - Combined Transmitter and Receiver
 *
 * This code initializes the LoRa module and the switch in the setup() function.
 * If the device is configured as a transmitter, in the loop() function, it reads the state of the dry contact and sends it as a LoRa packet every second.
 * If the device is configured as a receiver, it listens for incoming LoRa packets and toggles the relay accordingly.
 */

// Include the SPI and LoRa libraries
#include <SPI.h>
#include <LoRa.h>
#include "commonFunctions.h"

// Set to true for transmitter, false for receiver
bool isTransmitter = true;

// Define the pins for the LoRa module
#define nss 15 // LoRa chip select
#define rst 16 // LoRa reset
#define dio0 0 // LoRa DIO0

// Define the pin for the dry contact input
const int sw1 = 4;

// Define the pin for the relay
const int RLY1 = 5; // replace with your relay pin

byte localAddress = 0x04; // address of this device
byte destination = 0x03;  // destination to send to

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

    // Initialize the LoRa module at 433 MHz
    LoRa.begin(433E6) ? Serial.println("Starting LoRa started!") : Serial.println("Starting LoRa failed!");
}

// The loop function runs over and over again forever
void loop()
{
    // Update the LED
    updateLED();

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
                        sendRelayState(destination, localAddress, sw1State);

                        // Update the previous state with the current state
                        previousState = sw1State;

                        // Debug
                        Serial.println("Wrote changed relay state: " + String(sw1State));
                        // Set the state of the relay according to the received value
                        digitalWrite(RLY1, sw1State);

                        // Debug
                        Serial.println("WroteSet TX relay state: " + String(sw1State));

                        // Debug
                        Serial.println("RSSI: " + String(LoRa.packetRssi()));
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
            if (destinationAddress == localAddress && senderAddress == destination)
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
                    Serial.println("RSSI: " + String(LoRa.packetRssi()));

                    // Debug
                    Serial.println("Wrote changed relay state: " + String(relayState));
                }
            }
        }
    }
}