#include "functions.h"

// Define states for the state machine
enum State {
    STATE_INIT,
    STATE_WAIT_FOR_COMMAND,
    STATE_TRANSMIT,
    STATE_RECEIVE,
    STATE_ERROR,
    STATE_OTA_UPDATE
};

// Define events
enum Event {
    EVENT_WIFI_CONNECTED,
    EVENT_LORA_PACKET_RECEIVED,
    EVENT_MQTT_CONNECTED,
    EVENT_OTA_UPDATE_REQUESTED,
    EVENT_ERROR
};

// Current state
State currentState = STATE_INIT;

// Function prototypes
void handleInit();
void handleWaitForCommand();
void handleTransmit();
void handleReceive();
void handleError();
void handleOTAUpdate();
void transitionToState(State newState);

// Setup function (runs once at startup)
void setup() {
    Serial.begin(115200);
    delay(1000);  // Wait for serial port to connect
    
    transitionToState(STATE_INIT);  // Start with the initialization state
}

// Main loop (runs continuously)
void loop() {
    switch (currentState) {
        case STATE_INIT:
            handleInit();
            break;
        case STATE_WAIT_FOR_COMMAND:
            handleWaitForCommand();
            break;
        case STATE_TRANSMIT:
            handleTransmit();
            break;
        case STATE_RECEIVE:
            handleReceive();
            break;
        case STATE_ERROR:
            handleError();
            break;
        case STATE_OTA_UPDATE:
            handleOTAUpdate();
            break;
    }
}

// State Handlers

void handleInit() {
    Serial.println("Initializing...");

    // Set up LoRa module
    LoRa.setPins(nss, rst, dio0);
    if (!LoRa.begin(433E6)) {
        Serial.println("LoRa startup failed!");
        transitionToState(STATE_ERROR);  // Transition to error state
        return;
    }
    Serial.println("LoRa started successfully!");

    // Set up WiFi if enabled
    if (enableWiFi) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(defaultAPssid, defaultAPpassword);
        if (WiFi.waitForConnectResult() != WL_CONNECTED) {
            Serial.println("WiFi Failed!");
            transitionToState(STATE_ERROR);  // Transition to error state
            return;
        }
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());

        // Set up OTA update
        ArduinoOTA.begin();
    }

    // Set up MQTT
    mqtt_client.setServer(mqtt_server, Port);
    reconnect();  // Attempt to reconnect to the MQTT broker

    // Transition to waiting for commands
    transitionToState(STATE_WAIT_FOR_COMMAND);
}

void handleWaitForCommand() {
    // Handle incoming packets or commands
    if (isTransmitter) {
        transitionToState(STATE_TRANSMIT);
    } else {
        if (LoRa.parsePacket()) {
            transitionToState(STATE_RECEIVE);  // Received a packet, transition to receive state
        }
    }

    if (enableWiFi) {
        ArduinoOTA.handle();  // Check for OTA updates
    }

    // Check for events (LoRa packets, MQTT commands, etc.)
}

void handleTransmit() {
    // Transmit the state of the dry contact
    int state = digitalRead(INP1);
    LoRa.beginPacket();
    LoRa.print(state);
    LoRa.endPacket();
    
    Serial.println("Data transmitted!");

    // After transmission, return to wait state
    transitionToState(STATE_WAIT_FOR_COMMAND);
}

void handleReceive() {
    // Receive data and toggle relay
    int incomingData = LoRa.read();
    digitalWrite(RLY1, incomingData);
    
    Serial.println("Data received and relay toggled!");

    // After receiving, return to wait state
    transitionToState(STATE_WAIT_FOR_COMMAND);
}

void handleError() {
    Serial.println("An error occurred. Reattempting initialization...");
    delay(5000);
    transitionToState(STATE_INIT);  // Retry initialization
}

void handleOTAUpdate() {
    // Handle OTA update requests
    ArduinoOTA.handle();
}

// Helper function to transition between states
void transitionToState(State newState) {
    currentState = newState;
}

// Reconnect function for MQTT
void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP8266Client")) {
            Serial.println("connected");
            client.subscribe(subscribedTopic);
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}
