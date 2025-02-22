#include <Preferences.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiManager.h>
#define Vout 32

bool shouldSaveConfig = false;

// const char* ssid = "LSKK_Lantai2";
// const char* password 
const char* mqtt_server = "rmq2.pptik.id";
const char* mqtt_user = "*********";
const char* mqtt_pass = "*********";
const char* CL = "ElectroDrive";  //ini diganti
const char* mqttQueuePublish = "Log";
const char* mqttQueueSensor = "Sensor";
const float VCC = 5.0;  // supply voltage 5V or 3.3V. If using PCB, set to 5V only.
const int model = 2;    // enter the model (see below)
char mqtt_pub_topic[40];
float cutOffLimit = 1.50;  // reading cutt off current. 1.00 is 1 Amper
float sensitivity[] = {
  40.0,  // for ACS758LCB-050B
  60.0,  // for ACS758LCB-050U
  34.0,  // for ACS758LCB-100B
  40.0,  // for ACS758LCB-100U
  13.3,  // for ACS758KCB-150B
  16.7,  // for ACS758KCB-150U
  10.0,  // for ACS758ECB-200B
  20.0,  // for ACS758ECB-200U
};

float quiescent_Output_voltage[] = {
  0.5,   // for ACS758LCB-050B
  0.12,  // for ACS758LCB-050U
  0.5,   // for ACS758LCB-100B
  0.12,  // for ACS758LCB-100U
  0.5,   // for ACS758KCB-150B
  0.12,  // for ACS758KCB-150U
  0.5,   // for ACS758ECB-200B
  0.12,  // for ACS758ECB-200U
};
const float FACTOR = sensitivity[model] / 1000;           // set sensitivity for selected model
const float QOV = quiescent_Output_voltage[model] * VCC;  // set quiescent Output voltage for selected model
float voltage;                                            // internal variable for voltage
float cutOff = FACTOR / cutOffLimit;                      // convert current cut off to mV
const int num = 160;

const char* input_guid = "2f0adc25-cba3-4220-81a6-3b144b801531";
String client_id = "ELECTRO-DRIVE-01";
String APName = "ESP32-AP-" + client_id;
int loop_count = 0;
// Motor A and B
int RPWM_PIN_L = 14;  //MAJU KIRI
int LPWM_PIN_L = 15;  //MUNDUR KIRI
int enable1PinL_L = 25;
int enable1PinR_L = 26;

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 255;
unsigned long lastMsg = 0;
unsigned long interval = 5000;

byte mac[6];
String MACAddress;
uint8_t MAC_array[6];
char MAC_char[18];
String SpeedLog;
String button_status[4];
String statusDevice;
String prevpubmsg = "";
WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wifiManager;
Preferences preferences;
//callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void printMACAddress() {
  WiFi.macAddress(mac);
  MACAddress = mac2String(mac);
  Serial.println(MACAddress);
}

void setup_wifi() {
  delay(10);
  preferences.begin("my-app", false);

  String ssid = preferences.getString("ssid", "");  //second parameter is default value
  String password = preferences.getString("password", "");

  // Check if we have stored WiFi credentials
  if (ssid == "" || password == "") {
    Serial.println("No values saved for ssid or password");
    // Uncomment and run it once, if you want to erase all the stored information
    // wifiManager.resetSettings();
    // If it cannot connect in a certain time, it starts an access point with the specified name
    if (!wifiManager.autoConnect(APName.c_str())) {
      Serial.println("Failed to connect and hit timeout");
      // Reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }

    // Save the WiFi credentials in the preferences
    preferences.putString("ssid", WiFi.SSID());
    preferences.putString("password", WiFi.psk());

  } else {
    WiFi.begin(ssid.c_str(), password.c_str());
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("Failed to connect with saved credentials");
      preferences.begin("my-app", false);
      preferences.clear();
      wifiManager.resetSettings();
      preferences.end();
      ESP.restart();
    }
  }
  preferences.end();
  Serial.print("ESP32 MAC ADDRESS: ");
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


char sPayload[100];
char message[40];
char address[40];


void setup() {
  Serial.begin(115200);
  pinMode(RPWM_PIN_L, OUTPUT);
  pinMode(LPWM_PIN_L, OUTPUT);
  pinMode(enable1PinL_L, OUTPUT);
  pinMode(enable1PinR_L, OUTPUT);
  digitalWrite(RPWM_PIN_L, LOW);
  digitalWrite(LPWM_PIN_L, LOW);
  digitalWrite(enable1PinL_L, LOW);
  digitalWrite(enable1PinR_L, LOW);
  strcpy(mqtt_pub_topic, input_guid);

  setup_wifi();
  printMACAddress();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1PinR_L, pwmChannel);
  ledcAttachPin(enable1PinL_L, pwmChannel);
}


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
  const char* speed = doc["speed"];
  if (strcmp(commandType, "D") == 0) {
    DynamicJsonDocument logD(110);
    digitalWrite(RPWM_PIN_L, HIGH);
    digitalWrite(LPWM_PIN_L, LOW);
    Serial.println("Maju");
    if (strcmp(speed, "L") == 0) {
      dutyCycle = 140;
      SpeedLog = "LOW";
    } else if (strcmp(speed, "M") == 0) {
      dutyCycle = 185;
      SpeedLog = "MEDIUM";
    } else if (strcmp(speed, "H") == 0) {
      dutyCycle = 215;
      SpeedLog = "HIGH";
    }
    ledcWrite(pwmChannel, dutyCycle);
    logD["guid"] = input_guid;
    logD["mac"] = WiFi.macAddress();
    logD["message"] = "Maju";
    logD["speed"] = SpeedLog;
    char outD[110];
    serializeJson(logD, Serial);
    Serial.println("\t");
    serializeJson(logD, outD);
    client.publish(mqttQueuePublish, outD);
  }
  if (strcmp(commandType, "N") == 0) {
    DynamicJsonDocument logN(100);
    while (dutyCycle >= 125) {
      ledcWrite(pwmChannel, dutyCycle);
      dutyCycle = dutyCycle - 20;
      delay(200);
    }
    digitalWrite(RPWM_PIN_L, LOW);
    digitalWrite(LPWM_PIN_L, LOW);
    Serial.println("Berhenti");
    logN["guid"] = input_guid;
    logN["mac"] = WiFi.macAddress();
    logN["message"] = "Berhenti";
    logN["speed"] = "0";
    char outN[110];
    serializeJson(logN, Serial);
    Serial.println("\t");
    serializeJson(logN, outN);
    client.publish(mqttQueuePublish, outN);
  }
  if (strcmp(commandType, "R") == 0) {
    DynamicJsonDocument logR(110);
    digitalWrite(RPWM_PIN_L, LOW);
    digitalWrite(LPWM_PIN_L, HIGH);
    Serial.println("Mundur");
    if (strcmp(speed, "L") == 0) {
      dutyCycle = 140;
      SpeedLog = "LOW";
    } else if (strcmp(speed, "M") == 0) {
      dutyCycle = 185;
      SpeedLog = "MEDIUM";
    } else if (strcmp(speed, "H") == 0) {
      dutyCycle = 215;
      SpeedLog = "HIGH";
    }
    ledcWrite(pwmChannel, dutyCycle);
    logR["guid"] = input_guid;
    logR["mac"] = WiFi.macAddress();
    logR["message"] = "Mundur";
    logR["speed"] = SpeedLog;
    char outR[110];
    serializeJson(logR, Serial);
    Serial.println("\t");
    serializeJson(logR, outR);
    client.publish(mqttQueuePublish, outR);
  }
  if (strcmp(commandType, "RESET") == 0) {
    preferences.begin("my-app", false);
    preferences.clear();
    wifiManager.resetSettings();
    preferences.end();
    ESP.restart();
  }
}

void reconnect() {
  printMACAddress();
  const char* CL;
  CL = MACAddress.c_str();
  Serial.println(CL);
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(CL, mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      client.subscribe(mqtt_pub_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      ESP.restart();
      delay(5000);
    }
  }
}

void loop() {

  String pubmsg = "";
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > interval) {
    lastMsg = now;
    int adcsum = 0;
    for (int i = 0; i < num; i++) {
      adcsum += analogRead(Vout);
      delay(10);
    }
    int adcaverage = adcsum / num;
    float voltage_raw = (5.0 / 4095.0) * adcaverage;  // Read the voltage from sensor
    voltage = voltage_raw - QOV - 1.084;              // 0.007 is a value to make voltage zero when there is no current
    float current = voltage / FACTOR;
    Serial.println(voltage, 3);
    DynamicJsonDocument Current(100);
    char outC[110];
    if (abs(voltage) > cutOff) {
      Serial.print(" I: ");
      Serial.print(current, 2);  // print the current with 2 decimal places
      Serial.println("A");
      String currentStr = String(current, 2);
      Current["guid"] = input_guid;
      Current["mac"] = WiFi.macAddress();
      Current["current"] = currentStr;
      serializeJson(Current, outC);
      client.publish(mqttQueueSensor, outC);
      delay(1000);
    } else {
      Serial.println("No Current");
      Current["guid"] = input_guid;
      Current["mac"] = WiFi.macAddress();
      Current["current"] = "0";
      serializeJson(Current, outC);
      client.publish(mqttQueueSensor, outC);

    }
  }
}
