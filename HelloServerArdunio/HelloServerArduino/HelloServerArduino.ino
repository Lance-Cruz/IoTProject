// This code is derived from the HelloServer Example 
// in the (ESP32) WebServer library .
//
// It hosts a webpage which has one temperature reading to display.
// The webpage is always the same apart from the reading which would change.
// The getTemp() function simulates getting a temperature reading.
// homePage.h contains 2 constant string literals which is the two parts of the
// webpage that never change.
// handleRoot() builds up the webpage by adding as a C++ String:
// homePagePart1 + getTemp() +homePagePart2 
// It then serves the webpage with the command:  
// server.send(200, "text/html", message);
// Note the text is served as html.
//
// Replace the code in the homepage.h file with your own website HTML code.
// 
// This example requires only an ESP32 and download cable. No other hardware is reuired.
// A wifi SSID and password is required.
// Written by: Natasha Rohan  12/3/23
//
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "homepage.h"
#include "index.h"
#include "feature1.h"
#include "feature2.h"
#include "feature3.h"
#include "DFRobot_DHT11.h"

const char* ssid = "POCOM4Pro";
const char* password = "B714B435";

DFRobot_DHT11 dht11;

#define DHT11_PIN 4
#define LED_PIN 26

bool ledState = false;

WebServer server(80);

//temp function to simulate temp sensor
//String getTemp() {
//  float temp = random(200, 301) / 10.0;
//  return String(temp, 2);
//}

String getTemp() {
  dht11.read(DHT11_PIN);
  float temp = dht11.temperature; // Read temperature
  Serial.println("DHT11 Temperature: " + String(temp, 1) + " °C");
  
  // Return as a string with 1 decimal place
  return String(temp, 1);
}


// Toggle LED on/off from webpage
void toggleLED() {

  ledState = !ledState;  // Toggle LED state

  if (ledState) {     // If LED is 
    digitalWrite(LED_PIN, HIGH);
    Serial.println("LED Toggled: ON");
    server.send(200, "text/plain", "ON");
  } else {
    digitalWrite(LED_PIN, LOW);
    Serial.println("LED Toggled: OFF");
    server.send(200, "text/plain", "OFF");
  }

}

void handleTemperature() {
  server.send(200, "text/plain", getTemp());
}

/*void handleRoot() {
  server.send(200, "text/html", homePagePart1); // Send the HTML page
}*/

//Index page
void indexPage() {
  server.send(200, "text/html", indexPagepart1);
}

void feature1Page() {
  server.send(200, "text/html", feature1Pagepart1);
}

void feature2Page() {
  server.send(200, "text/html", feature2Pagepart1);
}

void feature3Page() {
  server.send(200, "text/html", feature3Pagepart1);
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void setup(void) {

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  //server.on("/", handleRoot);
  server.on("/", indexPage);
  server.on("/feature1.html", feature1Page);
  server.on("/feature2.html", feature2Page);
  server.on("/feature3.html", feature3Page);
  server.on("/temperature", handleTemperature);
  server.on("/toggleLED", toggleLED);
  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop(void) {
  server.handleClient();
  delay(100);//allow the cpu to switch to other tasks
}
