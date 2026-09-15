#include "wifi.h"
#include <ESP8266WiFi.h>

#define SSID "TheBlutotDevice"
#define PSW "aaaaaaaa"

const char* hostIP = "192.168.4.2";
const uint16_t port = 6767;

static unsigned long lastSend = 0;

WiFiEventHandler stationConnectedHandler;
WiFiEventHandler stationDisconnectedHandler;
WiFiClient client;

std::stringstream ss;

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

void wifiLoop(std::map<std::string, ButtonState> message) {
    switch (conn_status) {
    case BOOT: // Do nothing. Still waiiting for a connection.
        break;
    case DISCONNECTED:
        Serial.println("Got Disconnected.");
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

        if (!client.connected()) {
            conn_status = CONNECTED;
        }
        
        /* if (millis() - lastSend < 200) break;   // throttle to ~30 ms
        lastSend = millis(); */

        Serial.println("Sending data to client");

        for (auto row : message) {

            std::string button = row.first;

            ButtonState state = row.second;

            if (state.pressed && !state.previous) {
                // If the stringstream already has elements inside then we should append a comma.
                if (ss.tellp() > 0) {
                    ss << ",";  
                }
                
                ss << button;
            }


        }

        client.println(ss.str().c_str());

        Serial.printf("Data: %s\n", ss.str().c_str());


        // Clear the stringstream for the next loop iteration
        ss.str("");
        ss.clear();

        client.flush();
        break;
    default:
        break;
    }
    
}
