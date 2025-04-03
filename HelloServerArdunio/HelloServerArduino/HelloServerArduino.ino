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
#include "index.h"
#include "feature1.h"
#include "feature2.h"
#include "feature3.h"
#include "feature4.h"
#include "DFRobot_DHT11.h"

const char* ssid = "POCOM4Pro";
const char* password = "B714B435";

DFRobot_DHT11 dht11;

#define DHT11_PIN 4
#define LED_PIN 22
#define RELAY_PIN 16
#define MOTIONSENSOR_PIN 12
#define VOLTAGE_SENSOR 32     
#define CURRENT_SENSOR 34     

bool ledState = false;
bool fanState = false;

int pinStateCurrent = LOW;    
int pinStatePrevious = LOW;   

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

void toggleFan() {

  fanState = !fanState;

  if (fanState) {     // If LED is 
    digitalWrite(RELAY_PIN, HIGH);
    Serial.println("Fan Toggled: ON");
    server.send(200, "text/plain", "ON");
  } else {
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("Fan Toggled: OFF");
    server.send(200, "text/plain", "OFF");
  }
}

void getPIR() {
  pinStatePrevious = pinStateCurrent; // store old state
  pinStateCurrent = digitalRead(MOTIONSENSOR_PIN); // read new state

  if (pinStatePrevious == LOW && pinStateCurrent == HIGH) {
    Serial.println("Motion detected!");
    Serial.println("Check camera.");
    server.send(200, "text/plain", "Motion detected! Check camera.");
  } 
  else if (pinStatePrevious == HIGH && pinStateCurrent == LOW) {
    Serial.println("Motion stopped!");
    server.send(200, "text/plain", "Motion stopped");
  } 
  else {
    Serial.println("No motion change detected.");
    server.send(200, "text/plain", "No motion change detected."); 
  }
}

String getEnergyData(){
  int currentValue = analogRead(CURRENT_SENSOR);
  int voltageValue = analogRead(VOLTAGE_SENSOR);

  // Convert the analog value into current and voltage
  float currentReading = (currentValue * (0.22 / 4095)); // Current in milleamperes 
  float voltageReading = voltageValue * 3.3 / 4095; // Voltage in volts

  // Convert the current and voltage into power
  float power = voltageReading * currentReading; // Measured power in millewatts

  // Simulate appliance power with 1500W maximum scaling
  float maxVoltage = 3.3;     // Maximum measurable voltage (based on sensor)
  float maxCurrent = 0.22;    // Maximum measurable current in milleamperes
  float maxPower = maxVoltage * maxCurrent; // Maximum sensor power (mW)

  float scale = 1500 / maxPower;           // Scale to reach 1500W at max sensor output
  float simulatedPower = power * scale;   // Dynamically adjust power based on scale

  Serial.print("Simulated Appliance Power (W): " + String(simulatedPower, 3));

  return String(simulatedPower, 3);
}

void handleTemperature() {
  server.send(200, "text/plain", getTemp());
}

void handleEnergy() {
  server.send(200, "text/plain", getEnergyData());
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

void feature4Page() {
  server.send(200, "text/html", feature4Pagepart1);
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

  pinMode(MOTIONSENSOR_PIN, INPUT);

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
  server.on("/feature4.html", feature4Page);
  server.on("/temperature", handleTemperature);
  server.on("/toggleFan", toggleFan);
  server.on("/toggleLED", toggleLED);
  server.on("/getPIR", getPIR);
  server.on("/energyMeter", handleEnergy);
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
