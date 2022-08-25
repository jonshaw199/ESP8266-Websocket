#ifndef EXAMPLES_WEBSOCKETCLIENT_DEMO_H_
#define EXAMPLES_WEBSOCKETCLIENT_DEMO_H_

#include <Arduino.h>
#include <WiFi.h>

#include "WebSocketClient.h"

class WebSocketClientDemo
{
public:
  static WebSocketClient webSocketClient;
  // Use WiFiClient class to create TCP connections
  static WiFiClient client;
  static void setup();
  static void loop();
};

#endif // EXAMPLES_WEBSOCKETCLIENT_DEMO_H_