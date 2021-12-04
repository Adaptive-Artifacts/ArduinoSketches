#include "WiFi.h"
WiFiClient espClient;

#include "PubSubClient.h"
const char* mqttServer = "66.94.100.229";
const int mqttPort = 1883;
const char* mqttClientId = "MQTTLoggerClient_ESP32";
const char* mqttClientName = "MQTTLoggerClient_ESP32Sensor";
PubSubClient client(espClient);

#include <ArduinoJson.h>
int maxLoopCount = 30;
int currCounter = 0;

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;

void setup()
{
    Serial.begin(115200);

    //WiFi.begin("ops","howstheweather");
    WiFi.begin("4ld3rs0n.3LL10t","Lorem1psum6464");
    for(int i = 0; i <= 3; i++)
    {
      if(WiFi.status() != WL_CONNECTED)
      {
        delay(1000);
        Serial.println("Could not connect  to Wifi!");
        Serial.println("Retrying... (" + String(i) + ")");
        delay(1500);
        //WiFi connecting
        //WiFi.begin("ops","howstheweather");
        WiFi.begin("4ld3rs0n.3LL10t","Lorem1psum6464");
      }
    }

    client.setServer(mqttServer,mqttPort);
    Serial.println("Connecting to MQTT...");
    client.connect(mqttClientName);
    for(int i = 0; i <= 3; i++)
    {
      if(!client.connected())
      {
        delay(1000);
        Serial.println("Could not connect to MQTT");
        Serial.println("Retrying... (" + String(i) + ")");
        delay(2500);
        client.connect(mqttClientName);
      }
    }
    Serial.println("Connected to MQTT Server");
    delay(500);
    //Identifying the sensor
    Serial.println("I am " + String(mqttClientId));
    delay(1500);

    Serial.println("Testing BME280 board");
    delay(200);

    bool statusOfBME;
    statusOfBME = bme.begin();
    if(!statusOfBME)
    {
      Serial.println("BME not found, check wiring!");
      while(1);
    }
    Serial.println("BME is connected!");
    delay(1000);
}

void loop()
{
  //Getting the value readings from the BME sensor
    float temperature = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100;

    if(client.connected() == false)
    {
        ReconnectWifi();
    }
    Serial.println(mqttClientId);
    Serial.println("Temperature: " + String(temperature));
    Serial.println("Humidity: " + String(humidity));
    Serial.println("Bar Pressure: " + String(pressure));
    delay(1000);
  
    if(currCounter == maxLoopCount && client.connected())
    {
      //reset the counter
      currCounter = 0;
      
      // Send a JSON token every 5 minutes
      DynamicJsonDocument JSONencoder(300);
      //Encoding the temperature to a JSON object
      JSONencoder["SensorName"] = mqttClientId;
      JSONencoder["Temperature"] = String(temperature);
      JSONencoder["Humidity"] = String(humidity);
      JSONencoder["Pressure"] = String(pressure);
  
      //Message buffer to hold JSON Info
      char JSONmessageBuffer[300];
      //Writing the JSON object to the buffer
      serializeJson(JSONencoder, JSONmessageBuffer);
      
      //Sending a message to the server
      client.publish("/sensors/values", JSONmessageBuffer);
    }
    if(client.connected())
    {
          client.publish("/sensors/general", mqttClientId);
          Serial.println("Sending ping to '/sensors/general'");
    }
    //repeat this loop every 3 seconds
    currCounter++;
    delay(10000);
}

// Method to reconnect to Wi-Fi
void ReconnectWifi()
{
  // Retry 5 times
  for (int i = 0; i <= 3; i++)
  {
    if(WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Could not connect to Wifi!");
        Serial.println("Retrying.. ("+String(i)+")");
        delay(2500);
        //WiFi.begin("ops","howstheweather");
        WiFi.begin("4ld3rs0n.3LL10t","Lorem1psum6464");
    }
  }
  // Connected to Wifi
  Serial.println("Connected to TF_Mobile!");
  delay(1500);

  //Connect to MQTT
  client.setServer(mqttServer,mqttPort);
  Serial.println("Connecting to MQTT...");
  client.connect(mqttClientName);
  for (int i = 0; i <= 3; i++)
  {
    if(!client.connected())
    {
        delay(1000);
        Serial.println("Could not connect to MQTT!");
        Serial.println("Retrying.. ("+String(i)+")");
        delay(500);
        client.connect(mqttClientName);
    }
  }
}
