#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>

const int RecordTime = 3;  //Define Measuring Time (Seconds)
const int SensorPin = 2;   //Define Interrupt Pin (2 or 3 @ Arduino Uno)

unsigned long previousMillis = 0;
const long interval = 10000;

// const char* ssid = "LSKKWorks";
// const char* password = "1234567890";
const char* mqtt_server = "rmq2.pptik.id";
const char* mqtt_user = "/smarthome:smarthome";
const char* mqtt_pass = "Ssm4rt2!";
const char* CL = "LSKK-WINDTRACKER-01";  //ini diganti
const char* mqtt_pub_topic = "Sensor";
String input_guid = "b13905e3-5705-4b92-8jhgvb308-1f15661d6b8e";
String client_id = "WINDTRACKER-01";
String APName = "ESP8266-AP-" + client_id;
volatile int InterruptCounter = 0;
float WindSpeed;

/////////////////////////////////////////////////////////////////////////////////////////////////////////

WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wifiManager;

byte mac[6];
String MACAddress;

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup_wifi() {
  if (!wifiManager.autoConnect(APName.c_str(), "")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);

    ESP.reset();
    delay(5000);
  }
  Serial.println("");
  Serial.println("WiFi connected To:");
  Serial.println(WiFi.SSID());
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("ESP8266 MAC ADDRESS: ");
  Serial.println(WiFi.macAddress());
}

String mac2String(byte ar[]) {
  String s;
  for (byte i = 0; i < 6; ++i) {
    char buf[3];
    sprintf(buf, "%2X", ar[i]);
    s += buf;
    if (i < 5) s += ':';
  }
  return s;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void printMACAddress() {
  WiFi.macAddress(mac);
  MACAddress = mac2String(mac);
  Serial.println(MACAddress);
}

char message[45];
char pesan[45];
char address[40];

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void callback(char* topic, byte* payload, unsigned int length) {
  delay(10);
  Serial.print("Message arrived : ");
  payload[length] = '\0';
  String payloadStr = String((char*)payload);
  DynamicJsonDocument doc(200);
  DeserializationError error = deserializeJson(doc, payloadStr);
  if (error) {
    Serial.print(F("Failed to parse JSON: "));
    Serial.println(error.c_str());
    return;
  }
  const char* commandType = doc["command"];
  if (strcmp(commandType, "RESET") == 0) {
   wifiManager.resetSettings();
   delay(2000);
   ESP.restart();
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void reconnect() {
  // Loop until we're reconnected
  printMACAddress();
  const char* CL;
  CL = MACAddress.c_str();
  Serial.println(CL);
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(CL, mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      client.subscribe(input_guid.c_str());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      ESP.restart();
      delay(5000);
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  setup_wifi();
  printMACAddress();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}
String prevpubmsg = "";
String pubmsg = "";


/////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
  }

  pubmsg = input_guid + "#" + WindSpeed + "#" + WindSpeed / 3.6;
  Serial.println(pubmsg);
  client.publish(mqtt_pub_topic, pubmsg.c_str());
  Serial.println(pubmsg);
  Serial.println("\t");
  delay(1000);

  meassure();
  Serial.print("Wind Speed: ");
  Serial.print(WindSpeed);  //Speed in km/h
  Serial.print(" km/h - ");
  Serial.print(WindSpeed / 3.6);  //Speed in m/s
  Serial.println(" m/s");
}

void ICACHE_RAM_ATTR countup() {
  InterruptCounter++;
}

void meassure() {
  InterruptCounter = 0;
  attachInterrupt(digitalPinToInterrupt(SensorPin), countup, RISING);
  delay(1000 * RecordTime);
  detachInterrupt(digitalPinToInterrupt(SensorPin));
  WindSpeed = (float)InterruptCounter / (float)RecordTime * 2.4;
}

