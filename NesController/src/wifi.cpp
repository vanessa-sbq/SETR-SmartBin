#include "wifi.h"
#include <ESP8266WiFi.h>

#define SSID "TheBlutotDevice"
#define PSW "aaaaaaaa"

const char* hostIP = "192.168.4.2";
const uint16_t port = 6767;

WiFiEventHandler stationConnectedHandler;
WiFiEventHandler stationDisconnectedHandler;
WiFiClient client;

enum WIFI_STATUS {
    BOOT,
    BEGIN,
    CONNECTED, // Client just finished connecting
    DISCONNECTED, // Client got disconnected
    CLIENT_READY, // We have a socket connection
};

WIFI_STATUS conn_status = BOOT;

void wifiSetup() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PSW);

  // Register event handler for when a client connects
  stationConnectedHandler = WiFi.onSoftAPModeStationConnected([](const WiFiEventSoftAPModeStationConnected& event) {
    Serial.printf("Client connected! MAC: %02X:%02X:%02X:%02X:%02X:%02X AID: %d\n",
                  event.mac[0], event.mac[1], event.mac[2],
                  event.mac[3], event.mac[4], event.mac[5], event.aid);
    conn_status = CONNECTED;
  });

  // Register event handler for when a client disconnects
  stationDisconnectedHandler = WiFi.onSoftAPModeStationDisconnected([](const WiFiEventSoftAPModeStationDisconnected& event) {
    Serial.printf("Client disconnected! MAC: %02X:%02X:%02X:%02X:%02X:%02X AID: %d\n",
                  event.mac[0], event.mac[1], event.mac[2],
                  event.mac[3], event.mac[4], event.mac[5], event.aid);
    conn_status = DISCONNECTED;
  });

  IPAddress myIP = WiFi.softAPIP();     //IP Address of our Esp32 accesspoint(where we can host webpages, and see data)
  Serial.print("AP IP address: ");
  Serial.println(myIP);                 //Default IP is 192.168.4.1
}

void connectToSocket() {
    Serial.print("Connecting to ");
    Serial.print(hostIP);
    Serial.print(":");
    Serial.println(port);

    if (!client.connect(hostIP, port)) {
        Serial.println("Connection failed");
        return;
    }
    
    Serial.println("Connected to socket!");
    conn_status = CLIENT_READY;
}

void wifiLoop() {
    switch (conn_status) {
    case BOOT: // Do nothing. Still waiiting for a connection.
        break;
    case DISCONNECTED:
        if (!client.connected()) {
            conn_status = BOOT;
        }
        client.flush();
        break;
    case CONNECTED:
        Serial.println("Connected! Will now try to open the socket.");
        connectToSocket();
        break;
    case CLIENT_READY:
        // Example: Send some data once connected
        Serial.println("Sending data to client");
        client.println("Hello from ESP8266.");
        if (!client.connected()) {
            conn_status = CONNECTED;
        }
        client.flush();
        break;
    default:
        break;
    }
    
}
