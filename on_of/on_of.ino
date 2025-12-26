#include <WiFi.h>
#include <PubSubClient.h>

/* ================= WIFI ================= */
const char* ssid = "Rajo Kos ext1-2 luar";
const char* password = "set163204";

/* ================= MQTT ================= */
const char* mqtt_server = "broker.hivemq.com";
const char* mqtt_topic  = "esp32/motor/control";

WiFiClient espClient;
PubSubClient client(espClient);

/* ================= PIN MOTOR ================= */
#define MOTOR_IN1 27
#define MOTOR_IN2 26
#define MOTOR_EN  12

/* ================= PWM ================= */
#define PWM_CHANNEL 0
#define PWM_FREQ    30000
#define PWM_RES     8
#define TARGET_SPEED 200   // 🔥 KECEPATAN OTOMATIS (0–255)

bool motorOn = false;
int currentSpeed = 0;

/* ================= WIFI ================= */
void setup_wifi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Connected");
  Serial.println(WiFi.localIP());
}

/* ================= MQTT CALLBACK ================= */
void callback(char* topic, byte* message, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) {
    msg += (char)message[i];
  }

  msg.trim();
  Serial.println("MQTT: " + msg);

  if (msg == "on") {
    motorOn = true;
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    Serial.println("Motor ON");
  } 
  else if (msg == "off") {
    motorOn = false;
    currentSpeed = 0;
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    ledcWrite(PWM_CHANNEL, 0);
    Serial.println("Motor OFF");
  }
}

/* ================= MQTT RECONNECT ================= */
void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32MotorClient")) {
      client.subscribe(mqtt_topic);
      Serial.println("MQTT Connected");
    } else {
      delay(3000);
    }
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_EN, PWM_CHANNEL);

  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

/* ================= LOOP ================= */
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  /* ======== LOGIKA OTOMATIS SPEED ======== */
  if (motorOn && currentSpeed < TARGET_SPEED) {
    currentSpeed++;
    ledcWrite(PWM_CHANNEL, currentSpeed);
    delay(10);  // soft start
  }
}
