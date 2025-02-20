
#include "WebSocketClient_Demo.h"
#include "WebSocketClient.h"

#include "pre.h"

const char *ssid = STRINGIFY(JSSSID);
const char *password = STRINGIFY(JSPASS);
char path[] = STRINGIFY(WS_PATH);
char host[] = STRINGIFY(WS_HOST);

WebSocketClient WebSocketClientDemo::webSocketClient;

// Use WiFiClient class to create TCP connections
WiFiClient WebSocketClientDemo::client;

void WebSocketClientDemo::setup()
{
  Serial.begin(9600);
  delay(10);

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(5000);

  // Connect to the websocket server
  if (client.connect(STRINGIFY(WS_HOST), WS_PORT))
  {
    Serial.println("Connected");
  }
  else
  {
    Serial.println("Connection failed.");
    while (1)
    {
      // Hang on failure
    }
  }

  // Handshake with the server
  webSocketClient.path = path;
  webSocketClient.host = host;
  if (webSocketClient.handshake(client))
  {
    Serial.println("Handshake successful");
  }
  else
  {
    Serial.println("Handshake failed.");
    while (1)
    {
      // Hang on failure
    }
  }
}

void WebSocketClientDemo::loop()
{
  String data;

  if (client.connected())
  {

    webSocketClient.getData(data);
    if (data.length() > 0)
    {
      Serial.print("Received data: ");
      Serial.println(data);
    }

    data = "Hello world!";
    Serial.print("Sending data: ");
    Serial.println(data);
    webSocketClient.sendData(data);
  }
  else
  {
    Serial.println("Client disconnected.");
    while (1)
    {
      // Hang on disconnect.
    }
  }

  // wait to fully let the client disconnect
  delay(3000);
}
