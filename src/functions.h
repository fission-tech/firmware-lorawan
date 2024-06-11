// functions.h

// Include the SPI and LoRa libraries
#include <SPI.h>        // SPI library which is required by LoRa
#include <LoRa.h>       // LoRa library which is required for LoRa communication
#include <ArduinoOTA.h> // ArduinoOTA library which is required for OTA updates
#include <deque>
#include <string>

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

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#define __FUNC_NAME__ __PRETTY_FUNCTION__

// Set to true for transmitter, false for receiver
bool isTransmitter = true;
//bool isTransmitter = false;

byte txAddress = 0xFF; // address of this device
byte rxAddress = 0x13; // destination to send to

// Set to true to enable WiFi on the ESP8266
bool enableWiFi = false;
//bool enableWiFi = true;

String defaultAPssid = "Sensible IOT"; // default AP SSID
String defaultAPpassword = "13371337"; // default AP password

uint16_t relayStateControlId; // Declare the control ID globally

// Define the pins for the LoRa module
#define nss 15 // LoRa chip select
#define rst 16 // LoRa reset
#define dio0 0 // LoRa DIO0

// Define the pin for the dry contact input
const int INP1 = 4;

// Define the pin for the relay
const int RLY1 = 5; // replace with your relay pin

// Declare a variable to store the previous state of the dry contact
int previousState = -1;

// Define the debounce delay (in milliseconds)
const unsigned long debounceDelay = 50; // adjust as needed

// Declare variables to store the cry contact state and the last debounce time
int inpState;
int lastInpState = LOW;
unsigned long lastDebounceTime = 0;

// Declare a variable to store the previous time a heartbeat was sent
unsigned long previousMillisHeartbeat = 0;
unsigned long ackMillis = 0;
bool timeoutActive = false;

const unsigned long HEARTBEAT_INTERVAL = 60000;
const char NODE_TX = 'T';
const char NODE_RX = 'R';

const char T_CHANGE = 'C';
const char T_HEARTBEAT = 'H';
const char T_ACK = 'A';
const char T_LOCAL = 'L';
const char T_TIMEOUT = 'T';

#define __FUNC_NAME__ __PRETTY_FUNCTION__
//unsigned long lastPrintTime = 0;

// Declare a variable to store the previous time the LED blinked
unsigned long previousMillisLED = 0;

// log buffer
std::deque<std::string> logEntries;

void logEntry(byte localAddress, byte destination, byte type, byte state, byte nodeType, String funcSignature) {
    // Consider using local/remote instead of localAddress/destination
    // Consider using https://github.com/thijse/Arduino-Log
    // Consider sending the time from a gateway device or TX device in the heartbeat packet
    int firstSpacePos = funcSignature.indexOf(' ');
    int firstParenthesisPos = funcSignature.indexOf('(');
    String funcName = funcSignature.substring(firstSpacePos + 1, firstParenthesisPos);
    String rssi = String(LoRa.packetRssi());
    time_t timestamp = time(NULL);
    String logEntry = "";

    // Handle type ACK, HEARTHBEAT, RELAY CHANGE, LOCAL RELAY
    if ((type == 'A') || (type == 'H') || (type == 'C') || (type == 'L')) {
        String message = "";
        if (type == 'L') {
            message = "set local relay state";
        } else if (type == 'H') {
            message = "input state";
        } else {
            message = "relay state";
        }
        logEntry = String(timestamp) + " (" + String(char(nodeType)) + "X) " + funcName + " from 0x"
        + String(localAddress, HEX) + " type \'" + String(char(type)) + "\' " + message + " " 
        + String(state ? "ON" : "OFF") + " to 0x" + String(destination, HEX) + " (RSSI " + rssi + ")";
        Serial.println(logEntry);
        // Add the entry to the logEntries deque
        if (logEntries.size() == 100) { // limit to 100 entries
            logEntries.pop_front(); // remove the oldest entry
        }
        std::string stdStr = logEntry.c_str();
        logEntries.push_back(stdStr);
    }

    // Handle timeout
    if (type == 'T') {
        Serial.println(String(timestamp) + " (" + String(char(nodeType)) + "X) " + funcName + " 0x"
        + String(localAddress, HEX) + " type \'" + String(char(type)) + "\' timeout waiting for ACK from 0x" 
        + String(destination, HEX) + " - set local relay OFF (RSSI " + rssi + ")");
    }
}

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
void sendRelayState(byte localAddress, byte destination, byte type, byte state, byte nodeType)
// Consider using local/remote instead of localAddress/destination
{
    // Begin a LoRa packet
    LoRa.beginPacket();

    // Write the destination address to the packet
    LoRa.write(destination);

    // Write the local address to the packet
    LoRa.write(localAddress);

    // Write the message type to the packet
    LoRa.write(type);

    // Write the relay state to the packet
    LoRa.write(state);

    // End the LoRa packet and send it
    LoRa.endPacket();

    // Switch the LoRa module back into receive mode
    LoRa.receive();

    logEntry(localAddress, destination, type, state, nodeType, String(__FUNC_NAME__));
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

// TX - Transmitter
void handleSwitchStateChange(int state)
{
    inpState = state;
    // Transmit the new relay state
    sendRelayState(txAddress, rxAddress, T_CHANGE, inpState, (isTransmitter) ? 'T' : 'R');
}

// RX - Receiver (and local TX echo of confirmed relay state change)
void handleChangeRelayState(int relayState)
{
    digitalWrite(RLY1, relayState);
    if (!isTransmitter)
    {
        // on RX, log local relay change and send T_ACK to TX
        logEntry(txAddress, rxAddress, T_LOCAL , relayState, (isTransmitter) ? 'T' : 'R', String(__FUNC_NAME__));
        sendRelayState(rxAddress, txAddress, T_ACK, relayState, (isTransmitter) ? 'T' : 'R');
    }
    else
    {
        // on TX, log the local relay change
        logEntry(txAddress, rxAddress, T_LOCAL, relayState, (isTransmitter) ? 'T' : 'R', String(__FUNC_NAME__));
    }
}

// TX - Transmitter
void handleTransmitting() {
    
    // only applicable to TX units
    if (!isTransmitter) return;

    // fetch the current time, compare with the previous heartbeat time
    // if the interval has passed, send a heartbeat
    // and reset the previous heartbeat time to now
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisHeartbeat >= HEARTBEAT_INTERVAL) {
        previousMillisHeartbeat = currentMillis;
        sendRelayState(txAddress, rxAddress, T_HEARTBEAT, inpState, NODE_TX);
    }

    // read the state of the dry contact
    // if the state has changed, reset the debounce timer
    int state = digitalRead(INP1);
    if (state != lastInpState) {
        lastDebounceTime = millis();
    }

    // if the debounce timer has passed the debounce delay, update the switch state
    if ((millis() - lastDebounceTime) > debounceDelay && state != inpState) {
        handleSwitchStateChange(state);
    }

    // save the state of the dry contact for the next iteration
    lastInpState = state;
}

void handleHeartbeatOrAck(char messageType, int relayState, int localRlyState) {
    
    // if ACK - which should only be received by TX
    // reset ackMillis and timeoutActive

    if (messageType == T_ACK) {
        ackMillis = millis();
        timeoutActive = false;
    }

    // if heartbeat  - which should only be received by RX
    // and local relay state is different from received relay state
    // change the relay state to match the remote state and return
    // but if the messageType is ACK, delay before changing the relay state

    if (localRlyState != relayState) {
        if (messageType == T_ACK) {
            delay(500);
        }
        handleChangeRelayState(relayState);
        return;
    } else if (messageType == T_HEARTBEAT) {
        sendRelayState(rxAddress, txAddress, T_ACK, localRlyState, NODE_RX);
    }

}

void handleReceiving() {
    int packetSize = LoRa.parsePacket();
    if (packetSize == 0) return;

    byte destinationAddress = LoRa.read();
    byte senderAddress = LoRa.read();

    if ((isTransmitter && (destinationAddress != txAddress || senderAddress != rxAddress)) ||
        (!isTransmitter && (destinationAddress != rxAddress || senderAddress != txAddress))) {
        Serial.println(String(senderAddress) + " not permitted to send to " + String(destinationAddress));
        return;
    }

    char messageType = LoRa.read();
    int relayState = LoRa.read();

    while (LoRa.available()) {
        Serial.println(String(__FUNC_NAME__) + ": " + "Unexpected additional data, terminating...");
        return;
    }

    int localRlyState = digitalRead(RLY1);

    if (messageType == T_CHANGE && localRlyState != relayState) {
        handleChangeRelayState(relayState);
    } else if (messageType == T_HEARTBEAT || messageType == T_ACK) {
        handleHeartbeatOrAck(messageType, relayState, localRlyState);
    } else if (messageType == T_CHANGE && localRlyState == relayState) {
        logEntry(txAddress, rxAddress, T_ACK, relayState, (isTransmitter) ? 'T' : 'R', String(__FUNC_NAME__));
    }
}

void handleTimeout() {
    if (!isTransmitter || timeoutActive) return;

    unsigned long timeoutMillis = millis();
    if (timeoutMillis - ackMillis >= HEARTBEAT_INTERVAL * 2) {
        logEntry(txAddress, rxAddress, T_TIMEOUT, 0, (isTransmitter) ? 'T' : 'R', String(__FUNC_NAME__));
        handleChangeRelayState(0);
        timeoutActive = true;
    }
}

void handleTransceiver() {
    handleReceiving();
    handleTimeout();
    handleTransmitting();
}

#endif