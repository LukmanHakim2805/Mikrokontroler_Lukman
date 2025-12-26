#include <WiFi.h>
#include <PubSubClient.h>

/* ================= PIN ================= */
#define LED_WIFI 2        // LED onboard ESP32
#define MOTOR_IN1 27
#define MOTOR_IN2 26
#define MOTOR_EN  12

/* ================= WIFI ================= */
const char* ssid     = "realme 10";
const char* password = "bdaazqqae";

/* ================= MQTT ================= */
const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_topic  = "iot/esp32/motor";

/* ================= PWM ================= */
const int pwmChannel = 0;
const int pwmFreq    = 30000;
const int pwmRes     = 8;

WiFiClient espClient;
PubSubClient client(espClient);

/* ================= WIFI SETUP ================= */
void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_WIFI, HIGH);
    delay(300);
    digitalWrite(LED_WIFI, LOW);
    delay(300);
    Serial.print(".");
  }

  digitalWrite(LED_WIFI, HIGH);
  Serial.println("\nWiFi Connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

/* ================= MQTT CALLBACK ================= */
void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";

  for (int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  int speed = constrain(msg.toInt(), 0, 255);

  if (speed > 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    ledcWrite(pwmChannel, speed);

    Serial.print("Motor ON | Speed: ");
    Serial.println(speed);
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    ledcWrite(pwmChannel, 0);

    Serial.println("Motor OFF");
  }
}

/* ================= MQTT RECONNECT ================= */
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting MQTT... ");

    String clientId = "ESP32-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("Connected");
      client.subscribe(mqtt_topic);
      Serial.print("Subscribe: ");
      Serial.println(mqtt_topic);
    } else {
      Serial.print("Failed, retry...");
      delay(2000);
    }
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  pinMode(LED_WIFI, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);

  ledcSetup(pwmChannel, pwmFreq, pwmRes);
  ledcAttachPin(MOTOR_EN, pwmChannel);

  setupWiFi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

/* ================= LOOP ================= */
void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
}