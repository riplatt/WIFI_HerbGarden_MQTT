// Includes
#include "product.h" //Contains Product ID & Version
#include "SHT1x.h"
#include "PowerShield.h"
#include "MQTT.h"

void callback(char *topic, byte *payload, unsigned int length);

// System setting
SerialLogHandler logHandler(LOG_LEVEL_ALL);

// Specify data and clock connections
#define dataPin D5
#define clockPin D6
// Define Object
SHT1x sht1x(dataPin, clockPin);
PowerShield batteryMonitor;
byte server[] = {192, 168, 1, 5};         //MQTT Server Address
MQTT mqttClient(server, 1883, callback);
// Define Variables
unsigned short int _qos2messageid = 0;
long _sleepTime = 60; // 1 minute
long _ms = 0;
float _tempC;
float _humidity;
float _cellVoltage;
float _stateOfCharge;

// MQTT callback function
// recieve message
void callback(char *topic, byte *payload, unsigned int length)
{
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);
  Log.info("Incoming msg:" + message);
  delay(1000);
}

// QOS ack callback.
// MQTT server sendback message id ack, if use QOS1 or QOS2.
void qoscallback(unsigned int messageid)
{
  Log.info("Ack Message Id:%d", messageid);

  if (messageid == _qos2messageid)
  {
    Log.info("Release QoS2 Message");
    mqttClient.publishRelease(_qos2messageid);
  }
}

void setup()
{
  // Open serial for usb debug
  Serial.begin(115200);
  // Setup fuel gauge
  batteryMonitor.begin();
  batteryMonitor.quickStart();
  // short delay for battery monitor to settle
  delay(500);

  // Log some debug info
  Log.info("Starting up...");
  Log.info("System version: %s", System.version().c_str());
  Log.info("IP: %d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
  Log.info("Subnet: %d.%d.%d.%d", WiFi.subnetMask()[0], WiFi.subnetMask()[1], WiFi.subnetMask()[2], WiFi.subnetMask()[3]);
  Log.info("Gateway: %d.%d.%d.%d", WiFi.gatewayIP()[0], WiFi.gatewayIP()[1], WiFi.gatewayIP()[2], WiFi.gatewayIP()[3]);
  Log.info("SSID: %s", WiFi.SSID());
}

void loop()
{
  _ms = millis();
  // Connect to MQTT Broker
  while ((!mqttClient.isConnected()) && (millis() - _ms < 5000))
  {
    Log.info("Not Connected to MQTT Broker...");
    mqttClient.connect("herbgarden");
    delay(250);
  }

  // Just in case we didn't connect
  if (mqttClient.isConnected())
  {
    // Read values from the sensor
    _tempC = sht1x.readTemperatureC();
    _humidity = sht1x.readHumidity();

    // MQTT Publish
    mqttClient.loop();
    Log.info("Publishing - Temp:%.2fC, Humidity:%.4f%%, Volats:%.4fV, Charge:%.4f%%", _tempC, _humidity, batteryMonitor.getVCell(), batteryMonitor.getSoC());
    mqttClient.publish("herbgarden/probe/temperature/value", String::format("%.2f", _tempC));
    mqttClient.publish("herbgarden/probe/temperature/tag", "TI-0001");
    mqttClient.publish("herbgarden/probe/temperature/eu", "C");
    mqttClient.publish("herbgarden/probe/temperature/min", "-40.00");
    mqttClient.publish("herbgarden/probe/temperature/max", "123.80");

    mqttClient.publish("herbgarden/probe/humidity/value", String::format("%.4f", _humidity));
    mqttClient.publish("herbgarden/probe/humidity/tag", "AI-0001");
    mqttClient.publish("herbgarden/probe/humidity/eu", "%RH");
    mqttClient.publish("herbgarden/probe/humidity/min", "0.00");
    mqttClient.publish("herbgarden/probe/humidity/max", "100.00");

    mqttClient.publish("herbgarden/battery/voltage/value", String::format("%.4f", batteryMonitor.getVCell()));
    mqttClient.publish("herbgarden/battery/voltage/tag", "VI-0001");
    mqttClient.publish("herbgarden/battery/voltage/eu", "V");
    mqttClient.publish("herbgarden/battery/voltage/min", "0.00");
    mqttClient.publish("herbgarden/battery/voltage/max", "");

    mqttClient.publish("herbgarden/battery/charge/value", String::format("%.4f", batteryMonitor.getSoC()));
    mqttClient.publish("herbgarden/battery/charge/tag", "XI-0001");
    mqttClient.publish("herbgarden/battery/charge/eu", "%");
    mqttClient.publish("herbgarden/battery/charge/min", "0.00");
    mqttClient.publish("herbgarden/battery/charge/max", "100.00");

  }

  // Check for a update
  Log.info("Checking for update...");
  while ((!System.updatesPending()) && (millis() - _ms < 10000))
  {
    Particle.process();
  }

  if (System.updatesPending())
  { // if update is pending hang around for it a minute - will reboot after update
    Log.info("Update Avaliable, Doing update...");
    _ms = millis();
    while (millis() - _ms < 60000)
    {
      Particle.process();
    }
  }
  // Go to sleep and save some power
  System.sleep(SLEEP_MODE_DEEP, _sleepTime, SLEEP_NETWORK_STANDBY);
}
