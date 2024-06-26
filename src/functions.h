// functions.h

// Include the SPI and LoRa libraries
#include <SPI.h>        // SPI library which is required by LoRa
#include <LoRa.h>       // LoRa library which is required for LoRa communication
#include <ArduinoOTA.h> // ArduinoOTA library which is required for OTA updates
#include <EEPROM.h>     // EEPROM library which is required for EEPROM read/write
#include <AESLib.h>     // AES library which is required for encryption
#include <deque>        // deque library which is required for log buffer
#include <string>       // string library which is required for log buffer



#if defined(ESP8266)
/* ESP8266 Dependencies */
#include <ESP8266WiFi.h> // WiFi library which is required for WiFi communication
#include <ESP8266mDNS.h> // ESP8266mDNS library which is required for mDNS
#include "WiFiClientSecure.h"
#include <PubSubClient.h>


#elif defined(ESP32)
/* ESP32 Dependencies */
#include <WiFi.h>    // WiFi library which is required for WiFi communication
#include <ESPmDNS.h> // ESP8266mDNS library which is required for mDNS
#endif

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#define DEBUG true
// #define DEBUG false
#define DEBUG_VERBOSE false

#define KEY_LENGTH 16 // AES key length in bytes

byte txAddress = 0xFF; // address of this device
byte rxAddress = 0x13; // remoteAddress to send to

// Set to true for transmitter, false for receiver
bool isTransmitter = true;
//bool isTransmitter = false;

// Use the AES key from the TX device on the RX device to decrypt messages
byte aesKey[KEY_LENGTH] = {0x36, 0x6F, 0x88, 0x00, 0x37, 0x6F, 0x88, 0x00, 0x37, 0x6F, 0x88, 0x00, 0x37, 0x6F, 0x88, 0x00};
// byte aesKey[KEY_LENGTH]; // use this on a TX device to generate a new key
//byte aesIv[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F}; // 16 bytes IV
const char* ivString = "ThandaLoRaRange!";
byte aesIv[16];

// Set to true/false to enable/disable WiFi on the ESP8266
//bool enableWiFi = false;
bool enableWiFi = true;

String defaultAPssid = "Sensible IOT"; // default AP SSID
String defaultAPpassword = "13371337"; // default AP password

uint16_t relayStateControlId; // Declare the control ID globally


//Getting connected to MQTT Server
const char* mqtt_server = "YOUR_MQTT_BROKER_IP_ADDRESS"; // Declare your IP address here
const long int Port = 8883; // Declare your port here
const char* subscribedTopic = "esp32/output"; //Declare your topic here
WiFiClientSecure client;
PubSubClient mqtt_client(client); 
const char* CA_cert = \
"-----BEGIN CERTIFICATE-----\n" \
"################################################################\n" \
"################################################################\n" \
"################################################################\n" \
"-----END CERTIFICATE-----";

const char* ESP_CA_cert = \
"-----BEGIN CERTIFICATE-----\n" \
"################################################################\n" \
"################################################################\n" \
"################################################################\n" \
"-----END CERTIFICATE-----";

const char* ESP_RSA_key= \
"-----BEGIN RSA PRIVATE KEY-----\n" \
"################################################################\n" \
"################################################################\n" \
"################################################################\n" \
"-----END RSA PRIVATE KEY-----";

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
const char NODE_RX = 'R'; // HEX 0x52
const char NODE_TX = 'T'; // HEX 0x54

const char T_ACK = 'A';       // HEX 0x41
const char T_CHANGE = 'C';    // HEX 0x43
const char T_HEARTBEAT = 'H'; // HEX 0x48
const char T_LOCAL = 'L';     // HEX 0x4C
const char T_TIMEOUT = 'T';   // HEX 0x54

#define __FUNC_NAME__ __PRETTY_FUNCTION__
// unsigned long lastPrintTime = 0;

// Declare a variable to store the previous time the LED blinked
unsigned long previousMillisLED = 0;

// log buffer
std::deque<std::string> logEntries;

AESLib aesLib;

char getMessageType(byte typeByte)
{
    switch (typeByte)
    {
    case 0x41:
        return T_ACK;
    case 0x43:
        return T_CHANGE;
    case 0x48:
        return T_HEARTBEAT;
    case 0x4C:
        return T_LOCAL;
    case 0x54:
        return T_TIMEOUT;
    default:
        return '?'; // return a question mark if the byte doesn't match any known type
    }
}

void padPayload(byte *payload, int &length, int blockSize)
{
    int padLength = blockSize - (length % blockSize);
    for (int i = 0; i < padLength; i++)
    {
        payload[length + i] = padLength;
    }
    length += padLength;
}

int unpadPayload(byte *payload, int length)
{
    int padLength = payload[length - 1];
    return length - padLength;
}

void encrypt(byte *input, byte *output, int &length)
{
    if (DEBUG_VERBOSE)
    {
        // Print the input to the console
        Serial.print(String(__FUNC_NAME__) + "Raw data: ");
        for (uint16_t i = 0; i < sizeof(input); i++)
        {
            Serial.print(input[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    // Add padding
    padPayload(input, length, 16);

    // Encrypt
    memcpy(aesIv, ivString, 16);
    aesLib.encrypt(input, length, output, aesKey, 128, aesIv);

    if (DEBUG_VERBOSE)
    {
        // Print the output to the console
        Serial.print(String(__FUNC_NAME__) + "Encrypted data: ");
        for (uint16_t i = 0; i < sizeof(output); i++)
        {
            Serial.print(output[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}

void decrypt(byte *input, byte *output, int &length)
{
    if (DEBUG_VERBOSE)
    {
        // Print the input to the console
        Serial.print(String(__FUNC_NAME__) + "Encrypted data: ");
        for (uint16_t i = 0; i < sizeof(input); i++)
        {
            Serial.print(input[i], HEX);
            Serial.print(" ");
        }
    }

    // Decrypt
    memcpy(aesIv, ivString, 16);
    aesLib.decrypt(input, length, output, aesKey, 128, aesIv);

    // Remove padding
    length = unpadPayload(output, length);

    if (DEBUG_VERBOSE)
    {
        Serial.println();
        // Print the output to the console
        Serial.print(String(__FUNC_NAME__) + "Decrypted data: ");
        for (uint16_t i = 0; i < sizeof(output); i++)
        {
            Serial.print(output[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}

void generateKeyFromChipId()
{
    uint32_t chipId = ESP.getChipId();

    // Convert the chip ID to a byte array
    for (int i = 0; i < KEY_LENGTH; i++)
    {
        aesKey[i] = (chipId >> (i * 8)) & 0xFF;
    }
}

void printKey()
{
    // Print the key
    Serial.print("AES Key: {");
    for (int i = 0; i < KEY_LENGTH; i++)
    {
        if (i != 0)
        {
            Serial.print(", ");
        }
        Serial.print("0x");
        if (aesKey[i] < 16)
        {
            Serial.print("0");
        }
        Serial.print(aesKey[i], HEX);
    }
    Serial.println("}");
}
/*
void webUI()
{
    // Handle web UI
    // Consider using ESP.getFreeHeap() to check available memory
    // Consider using work time to calculate 'load' on the device

    unsigned long loopStartTime = 0;
    unsigned long workTime = 0;

    //    void loop()
    //    {
    unsigned long startTime = micros(); // record start time

    // ... your code here ...

    unsigned long endTime = micros(); // record end time
    workTime = endTime - startTime;
    unsigned long loopTime = endTime - loopStartTime;
    unsigned long idleTime = loopTime - workTime;

    float cpuLoad = (float)workTime / loopTime;

    loopStartTime = endTime; // reset for next loop
    //    }
}
*/
void logEntry(byte localAddress, byte remoteAddress, byte type, byte state, byte nodeType, String funcSignature)
{
    // Consider using local/remote instead of localAddress/remoteAddress
    // Consider using https://github.com/thijse/Arduino-Log
    // Consider sending the time from a gateway device or TX device in the heartbeat packet
    int firstSpacePos = funcSignature.indexOf(' ');
    int firstParenthesisPos = funcSignature.indexOf('(');
    String funcName = funcSignature.substring(firstSpacePos + 1, firstParenthesisPos);
    String rssi = String(LoRa.packetRssi());
    time_t timestamp = time(NULL);
    String logEntry = "";

    // Handle type ACK, HEARTHBEAT, RELAY CHANGE, LOCAL RELAY
    if ((type == 'A') || (type == 'H') || (type == 'C') || (type == 'L'))
    {
        String message = "";
        if (type == 'L')
        {
            message = "set local relay state";
        }
        else if (type == 'H')
        {
            message = "input state";
        }
        else
        {
            message = "relay state";
        }
        logEntry = String(timestamp) + " (" + String(char(nodeType)) + "X) " + funcName + " from 0x" + String(localAddress, HEX) + " type \'" + String(char(type)) + "\' " + message + " " + String(state ? "ON" : "OFF") + " to 0x" + String(remoteAddress, HEX) + " (RSSI " + rssi + ")";
        Serial.println(logEntry);
        // Add the entry to the logEntries deque
        if (logEntries.size() == 100)
        {                           // limit to 100 entries
            logEntries.pop_front(); // remove the oldest entry
        }
        std::string stdStr = logEntry.c_str();
        logEntries.push_back(stdStr);
    }

    // Handle timeout
    if (type == 'T')
    {
        Serial.println(String(timestamp) + " (" + String(char(nodeType)) + "X) " + funcName + " 0x" + String(localAddress, HEX) + " type \'" + String(char(type)) + "\' timeout waiting for ACK from 0x" + String(remoteAddress, HEX) + " - set local relay OFF (RSSI " + rssi + ")");
    }
}
/// For Clients to attempt to reconnect if the client is not being connected

/**
 * @brief Sends a relay state message to the specified remoteAddress.
 *
 * This function sends a relay state message to the specified remoteAddress. The message is encrypted using AES encryption.
 *
 * @param localAddress The address of the local device.
 * @param remoteAddress The address of the remoteAddress device.
 * @param type The type of the message.
 * @param state The relay state to send.
 * @param nodeType The type of the node (TX or RX).
 */
void sendRelayState(byte localAddress, byte remoteAddress, byte type, byte state, byte nodeType)
// Consider using local/remote instead of localAddress/remoteAddress
{

    // Create a buffer to hold the packet data
    byte packet[18]; // Adjust the size as needed
    unsigned int packetSize = 18;

    // Write the remoteAddress address to the packet
    packet[0] = remoteAddress;

    // Write the local address to the packet
    packet[1] = localAddress;

    // Bundle the messageType and relayState together
    byte payload[16] = {type, state};
    int payloadLength = 2; // Length of actual data
    if (DEBUG)
    {
        /*        Serial.print("Message type: ");
                Serial.print(payload[0], HEX);
                Serial.print(" | Relay state: ");
                Serial.println(payload[1], HEX);*/
    }

    // Encrypt the message
    byte encryptedPayload[16];
    encrypt(payload, encryptedPayload, payloadLength);

    // Copy encrypted payload to packet
    memcpy(&packet[2], encryptedPayload, 16);

    // Print the encrypted message to the Serial console
    if (DEBUG_VERBOSE)
    {
        Serial.print("Encrypted message: ");
        for (unsigned int i = 0; i < sizeof(encryptedPayload); i++)
        {
            Serial.print(encryptedPayload[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        // Print the packet data to the Serial console
        Serial.print("LoRa packet data: ");
        for (unsigned int i = 0; i < packetSize; i++)
        {
            Serial.print(packet[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    // Write the packet data to the LoRa module
    LoRa.beginPacket();
    LoRa.write(packet, packetSize);
    LoRa.endPacket();
..................
   if (mqtt_client.connect("ESP32")) {                       
    Serial.print("Connected, mqtt_client state: ");
    Serial.println(mqtt_client.state());
    //Publsih a demo message to topic with the payload data
    mqtt_client.publish(subscribedTopic, payload);
  }
  else {
    Serial.println("Connected failed!  mqtt_client state:");
    Serial.print(mqtt_client.state());
    Serial.println("WiFiClientSecure client state:");
    char lastError[100];
    client.lastError(lastError,100);  //Get the last error for WiFiClientSecure
    Serial.print(lastError);
  }
...................
    // Switch the LoRa module back into receive mode
    LoRa.receive();

    logEntry(localAddress, remoteAddress, payload[0], payload[1], nodeType, String(__FUNC_NAME__));
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
        logEntry(txAddress, rxAddress, T_LOCAL, relayState, (isTransmitter) ? 'T' : 'R', String(__FUNC_NAME__));
        sendRelayState(rxAddress, txAddress, T_ACK, relayState, (isTransmitter) ? 'T' : 'R');
    }
    else
    {
        // on TX, log the local relay change
        logEntry(txAddress, rxAddress, T_LOCAL, relayState, (isTransmitter) ? 'T' : 'R', String(__FUNC_NAME__));
    }
}

void handleHeartbeatOrAck(char messageType, int relayState, int localRlyState)
{

    // if ACK - which should only be received by TX
    // reset ackMillis and timeoutActive

    if (messageType == T_ACK)
    {
        ackMillis = millis();
        timeoutActive = false;
    }

    // if heartbeat  - which should only be received by RX
    // and local relay state is different from received relay state
    // change the relay state to match the remote state and return
    // but if the messageType is ACK, delay before changing the relay state

    if (localRlyState != relayState)
    {
        if (messageType == T_ACK)
        {
            delay(500);
        }
        handleChangeRelayState(relayState);
        return;
    }
    else if (messageType == T_HEARTBEAT)
    {
        sendRelayState(rxAddress, txAddress, T_ACK, localRlyState, NODE_RX);
    }
}

// TX - Transmitter
void handleTransmitting()
{

    // only applicable to TX units
    if (!isTransmitter)
        return;

    // fetch the current time, compare with the previous heartbeat time
    // if the interval has passed, send a heartbeat
    // and reset the previous heartbeat time to now
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisHeartbeat >= HEARTBEAT_INTERVAL)
    {
        previousMillisHeartbeat = currentMillis;
        sendRelayState(txAddress, rxAddress, T_HEARTBEAT, inpState, NODE_TX);
    }

    // read the state of the dry contact
    // if the state has changed, reset the debounce timer
    int state = digitalRead(INP1);
    if (state != lastInpState)
    {
        lastDebounceTime = millis();
    }

    // if the debounce timer has passed the debounce delay, update the switch state
    if ((millis() - lastDebounceTime) > debounceDelay && state != inpState)
    {
        handleSwitchStateChange(state);
    }

    // save the state of the dry contact for the next iteration
    lastInpState = state;
}

void handleReceiving()
{
    int packetSize = LoRa.parsePacket();
    if (packetSize == 0)
        return;

    byte packet[18];
    for (unsigned int i = 0; i < sizeof(packet); i++)
    {
        packet[i] = LoRa.read();
    }

    byte thisAddress = packet[0];
    byte remoteAddress = packet[1];

    if ((isTransmitter && (thisAddress != txAddress || remoteAddress != rxAddress)) ||
        (!isTransmitter && (thisAddress != rxAddress || remoteAddress != txAddress)))
    {
        if (DEBUG_VERBOSE)
        {
            Serial.println(String(__FUNC_NAME__) + (remoteAddress) + " not permitted to send to " + String(thisAddress));
        }
        return;
    }

    else if (packetSize != 18)
    {
        if (DEBUG)
        {
            Serial.println(String(__FUNC_NAME__) + ": " + "Unexpected packet size (" + packetSize + "), terminating...");
        }
        return;
    }

    // Decrypt the received message
    byte decryptedPayload[16];
    int payloadLength = 16;
    decrypt(&packet[2], decryptedPayload, payloadLength);

    byte messageType = decryptedPayload[0];
    byte relayState = decryptedPayload[1];

    if (DEBUG)
    {
        logEntry(thisAddress, remoteAddress, messageType, relayState, (isTransmitter) ? 'T' : 'R', String(__FUNC_NAME__));
    }

    while (LoRa.available())
    {
        if (DEBUG_VERBOSE)
        {
            Serial.println(String(__FUNC_NAME__) + ": " + "Unexpected additional data, terminating...");
        }
        return;
    }

    int localRlyState = digitalRead(RLY1);

    if (messageType == T_CHANGE && localRlyState != relayState)
    {
        handleChangeRelayState(relayState);
    }
    else if (messageType == T_HEARTBEAT || messageType == T_ACK)
    {
        handleHeartbeatOrAck(messageType, relayState, localRlyState);
    }
    else if (messageType == T_CHANGE && localRlyState == relayState)
    {
        logEntry(txAddress, rxAddress, T_ACK, relayState, (isTransmitter) ? 'T' : 'R', String(__FUNC_NAME__));
    }
}

void handleTimeout()
{
    if (!isTransmitter || timeoutActive)
        return;

    unsigned long timeoutMillis = millis();
    if (timeoutMillis - ackMillis >= HEARTBEAT_INTERVAL * 2)
    {
        logEntry(txAddress, rxAddress, T_TIMEOUT, 0, (isTransmitter) ? 'T' : 'R', String(__FUNC_NAME__));
        handleChangeRelayState(0);
        timeoutActive = true;
    }
}

void handleTransceiver()
{
    handleReceiving();
    handleTimeout();
    handleTransmitting();
}

#endif