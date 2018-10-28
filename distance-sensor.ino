/*
 * Distance Sensor
 * 
 * Still to do:
 * 
 * Change Hostname and OTA update host name to include last 6 characters of MAC address so that multiple sensors can be configured on the same network.
 * Change MQTT topic to include the last 6 characters of the device's MAC address,
 */

#include <FS.h>
#include <DHT.h>
#include <NewPingESP8266.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

#define DHTPIN 5       // Output pin from DHT-22 Temperature Sensor
#define DHTTYPE DHT22
#define TRIGGER_PIN  14  // Trigger pin for Ultrasonic Distance Sensor
#define ECHO_PIN     12  // Echo pin for Ultrasonic Distance Sensor
#define MAX_DISTANCE 400

#define PROPERTY_MQTT_SERVER_ADDRESS "mqtt_server_address"
#define PROPERTY_MQTT_SERVER_PORT "mqtt_server_port"

#define MQTT_TOPIC_TEMPERATURE "distancesensor/temperature"
#define MQTT_TOPIC_HUMIDITY "distancesensor/humidity"
#define MQTT_TOPIC_DISTANCE "distancesensor/distance"

#define iterations 5

#define mqttServerAddress "127.0.0.1"
#define mqttServerPort "1883"

bool shouldSaveConfig = false;
long lastReconnectAttempt = 0;
float lastTemperature = 0;
float lastHumidity = 0;
float lastDistance = 0;

NewPingESP8266 sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
DHT dht(DHTPIN, DHTTYPE);
WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup() {
  Serial.begin (9600);
  pinMode(LED_BUILTIN, OUTPUT);
  blink(100);
  readConfig();
  blink(100);
  wifi_station_set_hostname("DistanceSensor");
  setupWiFi();
  blink(100);
  ArduinoOTA.setHostname("DistanceSensor");
  ArduinoOTA.begin();
  dht.begin();
}

void readConfig() {
  if (SPIFFS.begin()) {
    trc("Mounted file system");
    if (SPIFFS.exists("/config.json")) {
      trc("Reading configuration file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        trc("Opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        if (json.success()) {
          strcpy(mqttServerAddress, json[PROPERTY_MQTT_SERVER_ADDRESS]);
          strcpy(mqttServerPort, json[PROPERTY_MQTT_SERVER_PORT]);
        } else {
          trc("Failed to read configuration");
        }
      }
    } else {
      trc("File /config.json doesn't exist");
    }
  } else {
    trc("Failed to mount FS");
  }
}

void setupWiFi() {
  WiFiManagerParameter mqttServer(PROPERTY_MQTT_SERVER_ADDRESS, "MQTT Address", mqttServerAddress, 40);
  WiFiManagerParameter mqttPort(PROPERTY_MQTT_SERVER_PORT, "MQTT Port", mqttServerPort, 6);

  WiFiManager wifiManager;

  if (mqttServerAddress == "" || mqttServerPort == "") {
    trc("Resetting WiFi Manager");
    WiFi.disconnect();
    wifiManager.resetSettings();
    ESP.reset();
    delay(1000);
  }

  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setConfigPortalTimeout(180);

  wifiManager.addParameter(&mqttServer);
  wifiManager.addParameter(&mqttPort);

  if (!wifiManager.autoConnect("DistanceSensor", "")) {
    trc("Failed to initialise onboard access point");
    delay(3000);
    ESP.reset();
    delay(5000);
  }

  trc("WiFi Connected");

  strcpy(mqttServerAddress, mqttServer.getValue());
  strcpy(mqttServerPort, mqttPort.getValue());

  if (shouldSaveConfig) {
    trc("Saving configuration");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json[PROPERTY_MQTT_SERVER_ADDRESS] = mqttServerAddress;
    json[PROPERTY_MQTT_SERVER_PORT] = mqttServerPort;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      trc("Failed to open configuration file");
    }

    json.printTo(configFile);
    configFile.close();
  }

  trc("Setting MQTT Server connection");
  unsigned int mqtt_port_x = atoi (mqttServerPort);
  client.setServer(mqttServerAddress, mqtt_port_x);
  reconnect();
}

void saveConfigCallback() {
  shouldSaveConfig = true;
}

boolean reconnect() {
  while (!client.connected()) {
    trc("Attempting to connect to MQTT Server");
    String mqname =  WiFi.macAddress();
    char charBuf[50];
    mqname.toCharArray(charBuf, 50) ;

    if (client.connect(charBuf)) {
      trc("Connected to MQTT Server");
    } else {
      trc("Failed to connect to MQTT Server");
      trc(String(client.state()));
      trc("Trying again in 5 seconds ...");
      delay(5000);
    }
  }
  return client.connected();
}

void trc(String msg) {
  Serial.println(msg);
}

void loop()
{
  ArduinoOTA.handle();
  client.loop();
  delay(2000);  // Delay so DHT-22 sensor can stabalize

  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  if (isnan(temp)) {
    temp = lastTemperature;
  }

  if (isnan(hum)) {
    hum = lastHumidity;
  }

  // Calculate the Speed of Sound in M/S using the current temperature and humidity
  float soundsp = 331.4 + (0.606 * temp) + (0.0124 * hum);
  // Convert to cm/ms
  float soundcm = soundsp / 10000;
  float duration = sonar.ping_median(iterations);

  // Calculate the distance
  int distance = round ((duration / 2) * soundcm);

  if (distance > MAX_DISTANCE) {
    distance = MAX_DISTANCE;
  }

  if (distance < 2) {
    distance = 2;
  }

  if (temp != lastTemperature) {
    sendMQTT (MQTT_TOPIC_TEMPERATURE, String (temp));
    lastTemperature = temp;
  }

  if (hum != lastHumidity) {
    sendMQTT (MQTT_TOPIC_HUMIDITY, String (hum));
    lastHumidity = hum;
  }

  if (distance != lastDistance) {
    sendMQTT (MQTT_TOPIC_DISTANCE, String (distance));
    lastDistance = distance;
  }
}

void sendMQTT(String topic, String payload) {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      trc("MQTT not connected.  Attemping to reconnect ...");
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    client.loop();
  }
  char topicChar[20];
  topic.toCharArray(topicChar, 50);
  char payloadChar[10];
  payload.toCharArray(payloadChar, 10);
  if (!client.publish(topicChar, payloadChar)) {
    trc("Message not published");
  }
}

void blink(int duration) {
  digitalWrite(LED_BUILTIN, LOW);
  delay(duration);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(duration);
}

