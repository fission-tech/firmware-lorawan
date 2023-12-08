/*
 *  rx.cpp
 *  Sensible LoRa Remote Switcher - Receiver
 *
 * This code initializes the LoRa module and the relay in the setup() function. In the loop() function,
 * it checks if a LoRa packet is received. If a packet is received, it reads the packet, converts it to
 * an integer, and sets the state of the relay accordingly.
 *
 * This code will retain the last known state of the relay even if the connection is lost, as it only
 * changes the state of the relay when a new packet is received.
 */

// Include the SPI and LoRa libraries
#include <SPI.h>
#include <LoRa.h>
#include "commonFunctions.h"

// Define the pins for the LoRa module
#define nss 15 // LoRa chip select
#define rst 16 // LoRa reset
#define dio0 0 // LoRa DIO0

// Define the pin for the relay
const int RLY1 = 5; // replace with your relay pin

byte localAddress = 0x04; // address of this device
byte destination = 0x03;  // destination to send to

// The setup function runs once when you press reset or power the board
void setup()
{
    // Begin serial communication at 115200 baud
    Serial.begin(115200);
    // Wait for serial port to connect
    while (!Serial)
        ;

    // Set the relay pin as an output
    pinMode(RLY1, OUTPUT);

    // Set the LED pin as an output
    pinMode(2, OUTPUT);

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