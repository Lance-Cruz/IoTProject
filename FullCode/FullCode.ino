#include <DFRobot_DHT11.h>
DFRobot_DHT11 DHT;
#define DHT11_PIN 4           // DHT11 sensor pin
#define PIN_TO_SENSOR 12     // Motion sensor pin
#define LIGHT_SENSOR_PIN 25   // Light sensor pin
#define LED_PIN 22            // LED pin
#define ANALOG_THRESHOLD 500  // Light sensor threshold
#define VOLTAGE_SENSOR 32     // Voltage sensor pin
#define CURRENT_SENSOR 34     // Current sensor pin
#define RELAY_PIN 16          // Relay Pin

int pinStateCurrent = LOW;    
int pinStatePrevious = LOW;   

void setup() {
  Serial.begin(115200);

  // DHT11 setup
  DHT.read(DHT11_PIN); 

  // 2 channel relay setup
  pinMode(RELAY_PIN, OUTPUT);

  // Motion sensor setup
  pinMode(PIN_TO_SENSOR, INPUT); 

  // Light sensor and LED setup
  analogSetAttenuation(ADC_11db); 
  pinMode(LED_PIN, OUTPUT);       

  // Initial message
  Serial.println("Setup complete. Starting to monitor sensors...");
}

void loop() {
  // Temperature and Humidity
  DHT.read(DHT11_PIN); 
  Serial.println("Reading temperature and humidity...");
  Serial.print("Temp: ");
  Serial.print(DHT.temperature);
  Serial.print("°C  Humidity: ");
  Serial.println(DHT.humidity);

  if(DHT.temperature > 25){
    Serial.println("Temperature threshold reached... Turning on fan.");
    digitalWrite(RELAY_PIN, HIGH);
  }
  else{
    digitalWrite(RELAY_PIN, LOW);
  }
  
  delay(2000); 

  // Motion Sensor and Camera
  Serial.println("Checking motion sensor status...");
  pinStatePrevious = pinStateCurrent; 
  pinStateCurrent = digitalRead(PIN_TO_SENSOR);

  if (pinStatePrevious == LOW && pinStateCurrent == HIGH) {
    Serial.println("Motion detected!");
    Serial.println("Check Camera");
  } else if (pinStatePrevious == HIGH && pinStateCurrent == LOW) {
    Serial.println("Motion stopped!");
  } else {
    Serial.println("No motion change detected.");
  }
  
  delay(2000);

  // Light Sensor and LED
  Serial.println("Checking light sensor status...");
  int analogValue = analogRead(LIGHT_SENSOR_PIN); 
  Serial.print("Light sensor value: ");
  Serial.println(analogValue);

  if (analogValue < ANALOG_THRESHOLD) {
    digitalWrite(LED_PIN, HIGH); 
    Serial.println("Low light detected, LED is ON.");
  } else {
    digitalWrite(LED_PIN, LOW);  
    Serial.println("Sufficient light, LED is OFF.");
  }

  delay(2000);

  // Energy Meter 
  int currentValue = analogRead(CURRENT_SENSOR);
  int voltageValue = analogRead(VOLTAGE_SENSOR);

  // Get the analog value of the two pins
  Serial.print("The current analog value is: ");
  Serial.println(currentValue);
  Serial.print("The voltage analog value is: ");
  Serial.println(voltageValue);

  delay(1000);

  Serial.println("");
  Serial.println("--------");
  Serial.println("");

  // Convert the analog value into current and voltage
  float currentReading = (currentValue * (0.22 / 4095)); // Current in milleamperes 
  float voltageReading = voltageValue * 3.3 / 4095; // Voltage in volts

  Serial.print("Current Reading (mA): ");
  Serial.println(currentReading);
  Serial.print("Voltage Reading (V): ");
  Serial.println(voltageReading);

  Serial.println("");
  Serial.println("--------");
  Serial.println("");

  delay(1000);

  // Convert the current and voltage into power
  float power = voltageReading * currentReading; // Measured power in millewatts

  Serial.print("Measured Power (mW): ");
  Serial.println(power);

  Serial.println("");
  Serial.println("--------");
  Serial.println("");

  delay(1000);

  // Simulate appliance power with 1500W maximum scaling
  float maxVoltage = 3.3;     // Maximum measurable voltage (based on sensor)
  float maxCurrent = 0.22;    // Maximum measurable current in milleamperes
  float maxPower = maxVoltage * maxCurrent; // Maximum sensor power (mW)

  float scale = 1500 / maxPower;           // Scale to reach 1500W at max sensor output
  float simulatedPower = power * scale;   // Dynamically adjust power based on scale

  Serial.print("Simulated Appliance Power (W): ");
  Serial.println(simulatedPower);

  Serial.println("");
  Serial.println("--------");
  Serial.println("");

  delay(3000);
}
