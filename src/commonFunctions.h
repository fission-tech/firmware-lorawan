// commonFunctions.h

#ifndef COMMON_FUNCTIONS_H
#define COMMON_FUNCTIONS_H

// Declare a variable to store the previous time the LED blinked
unsigned long previousMillisLED = 0;

/**
 * @brief Sends the relay state over LoRa.
 *
 * This function begins a LoRa packet, writes the destination address, local address, message type ('R' for relay),
 * and relay state to the packet, ends the packet, and sends it.
 *
 * @param destination The destination address.
 * @param localAddress The local address.
 * @param state The relay state.
 */
void sendRelayState(byte destination, byte localAddress, int state)
{
    // Begin a LoRa packet
    LoRa.beginPacket();

    // Write the destination address to the packet
    LoRa.write(destination);

    // Write the local address to the packet
    LoRa.write(localAddress);

    // Write the message type to the packet
    LoRa.write('R'); // 'R' for relay

    // Write the relay state to the packet
    LoRa.print(state);

    // End the LoRa packet and send it
    LoRa.endPacket();

    // Debug
    Serial.println("TX - Sent relay state: " + String(state));
}

/**
 * @brief Sends a reply over LoRa.
 *
 * This function begins a LoRa packet, writes the destination address, local address, and message type to the packet,
 * ends the packet, sends it, and switches the LoRa module back into receive mode.
 *
 * @param destination The destination address.
 * @param localAddress The local address.
 * @param type The type of the message ('H' for heartbeat, 'A' for acknowledgment).
 */
void sendReply(byte destination, byte localAddress, byte type)
{
    // Switch the LoRa module into transmit mode
    LoRa.beginPacket();

    // Write the destination address to the packet
    LoRa.write(destination);

    // Write the local address to the packet
    LoRa.write(localAddress);

    // Write the type of the message ('H' for heartbeat, 'A' for acknowledgment)
    LoRa.write(type);

    // End the packet and send it
    LoRa.endPacket();

    // Switch the LoRa module back into receive mode
    LoRa.receive();

    // Debug
    Serial.println("Sent '" + String(type) + "'.");
}

/**
 * @brief Blinks the LED for a specified duration.
 *
 * This function uses non-blocking code to blink the LED. It checks if the specified duration has passed
 * since the last state change. If it has, it toggles the LED state. This way, the function is non-blocking
 * and won't interfere with other parts of the code.
 *
 * @param duration The duration for which the LED should blink.
 */
void blinkLED(int duration)
{
    // Get the current time
    unsigned long currentMillis = millis();

    // Check if the specified duration has passed since the last state change
    if (currentMillis - previousMillisLED >= static_cast<unsigned long>(duration))
    {
        // If it has, update the last state change time
        previousMillisLED = currentMillis;

        // Read the current state of the LED
        int ledState = digitalRead(2);

        // Toggle the LED state
        digitalWrite(2, !ledState);
    }
}

/**
 * @brief Updates the LED based on the received signal strength.
 *
 * This function is called when a LoRa packet is received. It checks the received signal strength and blinks the LED accordingly.
 */
void updateLED()
{
    // Check the received signal strength
    if (-40 <= LoRa.packetRssi() && LoRa.packetRssi() <= 0)
    {
        // If the signal strength is between -40 and 0, blink the LED every 1000ms
        blinkLED(2000);
    }
    else if (-80 <= LoRa.packetRssi() && LoRa.packetRssi() <= -40)
    {
        // If the signal strength is between -80 and -40, blink the LED every 500ms
        blinkLED(1000);
    }
    else if (-120 <= LoRa.packetRssi() && LoRa.packetRssi() <= -80)
    {
        // If the signal strength is between -80 and -40, blink the LED every 500ms
        blinkLED(100);
    }
    else
    {
        // Turn LED off
        digitalWrite(2, HIGH);
    }
}

#endif