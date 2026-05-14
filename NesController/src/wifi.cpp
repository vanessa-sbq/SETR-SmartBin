#include "wifi.h"
#include <ESP8266WiFi.h>

#define SSID "TheBlutotDevice"
#define PSW "aaaaaaaa"

WiFiEventHandler stationConnectedHandler;
WiFiEventHandler stationDisconnectedHandler;


void wifiSetup() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(SSID, PSW);

  // Register event handler for when a client connects
  stationConnectedHandler = WiFi.onSoftAPModeStationConnected([](const WiFiEventSoftAPModeStationConnected& event) {
    Serial.printf("Client connected! MAC: %02X:%02X:%02X:%02X:%02X:%02X AID: %d\n",
                  event.mac[0], event.mac[1], event.mac[2],
                  event.mac[3], event.mac[4], event.mac[5], event.aid);
  });

  // Register event handler for when a client disconnects
  stationDisconnectedHandler = WiFi.onSoftAPModeStationDisconnected([](const WiFiEventSoftAPModeStationDisconnected& event) {
    Serial.printf("Client disconnected! MAC: %02X:%02X:%02X:%02X:%02X:%02X AID: %d\n",
                  event.mac[0], event.mac[1], event.mac[2],
                  event.mac[3], event.mac[4], event.mac[5], event.aid);
  });

  IPAddress myIP = WiFi.softAPIP();     //IP Address of our Esp32 accesspoint(where we can host webpages, and see data)
  Serial.print("AP IP address: ");
  Serial.println(myIP);                 //Default IP is 192.168.4.1
}

void wifiLoop() {
    
}
