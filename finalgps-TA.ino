#include <WiFiManager.h>
#include <WiFiClient.h>
#include <MQTT.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ctime>


const int RELAY_PIN = 4;
const int BUZZER_PIN = 5;
const int LED = 18;

const char* ssid = "AndriHostop";
const char* password = "qwerty12345";
WiFiManager wm;

WiFiClient net;
MQTTClient client;
const char* mqttBroker = "broker.emqx.io";
const char* topic_location = "Andri/TA/Simor/GPS";
const char* topic_control = "Andri/TA/Simor/Control";
const char* topic_alarm = "Andri/TA/Simor/Alarm";

TinyGPSPlus gps;
HardwareSerial serial_gps(2); 
String gmapsLink;
String clientId;
String jsonString;
unsigned long lastSendTime = 0;

String phoneNumber = "628973015810";
String WAAPI = "4580393";

#define telegram_token "7479673290:AAEr-KRP7hZfUpgIk9lnrcK02PGYV0nry00"
#define chat_id "7272562844"

bool sendGPSData = false; 

WiFiUDP ntpUDP;
NTPClient ntpClient(ntpUDP, "pool.ntp.org");

void connectWiFi() {
  Serial.print("Menghubungkan ke WiFi...");
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 60000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nTerhubung ke WiFi");
    digitalWrite(LED, HIGH);
    delay(3000);
  } else {
    Serial.println("\nGagal terhubung ke WiFi dalam 30 detik. Memulai WiFiManager...");
    bool res = wm.autoConnect("GPS Tracker", "12345678");
    if (!res) {
      Serial.println("Failed to connect using WiFiManager");
    } else {
      Serial.println("Connected using WiFiManager...yeey :)");
      digitalWrite(LED, HIGH);
      delay(3000);
    }
  }
}

void connectMQTT() {
  Serial.println("Menghubungkan ke broker MQTT...");
  clientId = "ESP32Client-";
  clientId += String(random(0xffff), HEX);

  while (!client.connect(clientId.c_str())) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nTerhubung ke broker MQTT!");
  client.subscribe(topic_control);
  digitalWrite(LED, LOW);
}

void setup() {
  Serial.begin(115200);
  serial_gps.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED, OUTPUT);

  connectWiFi();
  digitalWrite(LED, HIGH);
  delay(3000);

  client.begin(mqttBroker, net);
  connectMQTT();
  client.onMessage(messageReceived);

  ntpClient.begin();
  ntpClient.setTimeOffset(25200); // Adjust the time offset in seconds as per your timezone (e.g., 25200 for GMT+7)
}

void loop() {
  client.loop();

  if (!client.connected()) {
    connectMQTT();
  }

  while (serial_gps.available() > 0) {
    gps.encode(serial_gps.read());
  }

  unsigned long currentMillis = millis();

  // Kirim data JSON setiap 10 detik
  if (currentMillis - lastSendTime >= 10000) {
    lastSendTime = currentMillis;

    if (gps.location.isUpdated()) {
      double latitude = gps.location.lat();
      double longitude = gps.location.lng();
      double altitude = gps.altitude.meters();
      double speed = gps.speed.kmph();
      int satellites = gps.satellites.value();
      int hdop = gps.hdop.value();
      ntpClient.update();
      time_t epochTime = ntpClient.getEpochTime();

      StaticJsonDocument<200> jsonDoc;
      jsonDoc["lat"] = latitude;
      jsonDoc["lng"] = longitude;
      jsonDoc["alt"] = altitude;
      jsonDoc["spd"] = speed;
      jsonDoc["sat"] = satellites;
      jsonDoc["hdop"] = hdop / 100.0;
      jsonDoc["timestamp"] = epochTime;

      serializeJson(jsonDoc, jsonString);

      // Untuk debugging
      Serial.println("JSON Data:");
      Serial.println(jsonString);

      client.publish(topic_location, jsonString);

      jsonString = "";

      if (sendGPSData) {
        gmapsLink = "https://maps.google.com/?q=" + String(latitude, 8) + "," + String(longitude, 8);

        // Untuk debugging
        Serial.println("Google Maps Link:");
        Serial.println(gmapsLink);

        // Kirim link ke WhatsApp dan Telegram 
        sendNotification(gmapsLink, "whatsapp"); 
        sendNotification(gmapsLink, "telegram"); 

        gmapsLink = "";
      }
    } else {
      Serial.println("Lokasi tidak valid");
    }

    Serial.println();
  }

  delay(1000);
}


void messageReceived(String &topic, String &payload) {
  Serial.println("Pesan diterima dari topic: " + topic);
  Serial.println("Payload: " + payload);
 
  if (topic == topic_control) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.println("deserializeJson() gagal: " + String(error.c_str()));
      return;
    }
 
    if (doc.containsKey("relay")) {
      bool relayState = doc["relay"]; // true means relay off
      digitalWrite(RELAY_PIN, relayState ? LOW : HIGH); // Adjusted logic
      Serial.println("Relay state: " + String(relayState ? "OFF" : "ON"));
    }
    if (doc.containsKey("buzzer")) {
      bool buzzerState = doc["buzzer"]; // true means buzzer on
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW); // Adjusted logic
      Serial.println("Buzzer state: " + String(buzzerState ? "ON" : "OFF"));
    }
    if (doc.containsKey("sendGPS")) {
      sendGPSData = doc["sendGPS"];
      Serial.println("Send GPS data: " + String(sendGPSData ? "ENABLED" : "DISABLED"));
    }
  }
}

void sendNotification(String message, String bot) {
  HTTPClient clien;
  String url;
  if (bot == "whatsapp") {
    url = "https://api.callmebot.com/whatsapp.php?phone=" + String(phoneNumber) + "&text=" + message + "&apikey=" + String(WAAPI);
  } else if (bot == "telegram") {
    url = "https://api.telegram.org/bot" + String(telegram_token) + "/sendMessage?chat_id=" + String(chat_id) + "&text=" + message;
  }
 
  clien.begin(url);
  int httpResponseCode = clien.GET();
  if (httpResponseCode > 0) {
    Serial.printf("Message Sent to %s, HTTP Code: %d\n", bot.c_str(), httpResponseCode);
  } else {
    Serial.printf("Message Failed to %s, Error: %s\n", bot.c_str(), clien.errorToString(httpResponseCode).c_str());
  }
  clien.end();
}