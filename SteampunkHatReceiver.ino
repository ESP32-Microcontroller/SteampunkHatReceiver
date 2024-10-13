/*
    ESP-NOW Broadcast Slave
    Lucas Saavedra Vaz - 2024

    This sketch demonstrates how to receive broadcast messages from a master device using the ESP-NOW protocol.

    The master device will broadcast a message every 5 seconds to all devices within the network.

    The slave devices will receive the broadcasted messages. If they are not from a known master, they will be registered as a new master
    using a callback function.
*/

#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#include <vector>

#include <SPI.h>
#include <Wire.h>

#include <ESP32Servo.h>
Servo myservo; 

/* Definitions */
#define ESPNOW_WIFI_CHANNEL 6

#define SCL0_Pin 19
#define SDA0_Pin 20

/* Classes */

// Creating a new class that inherits from the ESP_NOW_Peer class is required.
class ESP_NOW_Peer_Class : public ESP_NOW_Peer {
public:
  // Constructor of the class
  ESP_NOW_Peer_Class(const uint8_t *mac_addr, uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(mac_addr, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Peer_Class() {}

  // Function to register the master peer
  bool add_peer() {
    if (!add()) {
      log_e("Failed to register the broadcast peer");
      return false;
    }
    return true;
  }

  int pos = 0; // servo position
  int ledMode = 0; // default to "off"

  // Function to print the received messages from the master
  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    Serial.printf("Received a message from master " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
    Serial.printf("  Message: %s\n", (char *)data);

    char *buffer = (char*)data;

    int stepSize = 6; int delayFor = 4;
    // 5, 10 => 249ms
    // 5, 8 => 199ms
    // 5, 6 => 149ms
    // 5, 5 => 124ms
    // 5, 4 => 99ms
    // 5, 3 => 74ms unstable
    // 6, 4 => 83ms
    // 7, 4 => 71ms unstable
    // 4, 3 => 92ms
    // 4, 2 => 61ms unstable

    int startPos = 0;
    int endPos = 120;
    if (strcmp(buffer, "FIRST_FINGER_BUTTON_DOWN") == 0) {
      // digitalWrite(22, HIGH);
      uint64_t startTime = esp_timer_get_time();
      myservo.write(endPos);
      // for (pos = startPos; pos <= endPos; pos += stepSize) {
      //   myservo.write(pos);
      //   delay(delayFor); // (8, 15) works well, (8, 5) is ok
      // }
      uint64_t endTime = esp_timer_get_time();
      Serial.print("time for 180: ");
      Serial.print(String((endTime - startTime) / 1000));
      Serial.println("ms");
    } else if (strcmp(buffer, "FIRST_FINGER_BUTTON_UP") == 0) {
      // digitalWrite(22, LOW);
      myservo.write(startPos);
      // for (pos = endPos; pos >= startPos; pos -= stepSize) {
      //   myservo.write(pos);
      //   delay(delayFor);
      // }
    }

    if (strcmp(buffer, "SECOND_FINGER_BUTTON_DOWN") == 0) {
      if (ledMode == 0) {
        digitalWrite(17, HIGH);
      } else if (ledMode == 1) {
        digitalWrite(17, LOW);
      }
    } else if (strcmp(buffer, "SECOND_FINGER_BUTTON_UP") == 0) {
      if (ledMode == 0) {
        digitalWrite(17, LOW);
      } else if (ledMode == 1) {
        digitalWrite(17, HIGH);
      }
    }

    if (strcmp(buffer, "THIRD_FINGER_BUTTON_DOWN") == 0) {
      if (ledMode == 0) {
        digitalWrite(16, HIGH);
      } else if (ledMode == 1) {
        digitalWrite(16, LOW);
      }
    } else if (strcmp(buffer, "THIRD_FINGER_BUTTON_UP") == 0) {
      if (ledMode == 0) {
        digitalWrite(16, LOW);
      } else if (ledMode == 1) {
        digitalWrite(16, HIGH);
      }
    }

    if (strcmp(buffer, "TOGGLE_LED_MODE") == 0) {
      if (ledMode == 0) {
        ledMode = 1;
        digitalWrite(16, HIGH);
        digitalWrite(17, HIGH);
      } else if (ledMode == 1) {
        ledMode = 0;
        digitalWrite(16, LOW);
        digitalWrite(17, LOW);
      }
    }

    Serial.println(buffer);
  }
};

/* Global Variables */

// List of all the masters. It will be populated when a new master is registered
std::vector<ESP_NOW_Peer_Class> masters;

/* Callbacks */

// Callback called when an unknown peer sends a message
void register_new_master(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) == 0) {
    Serial.printf("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));
    Serial.println("Registering the peer as a master");

    ESP_NOW_Peer_Class new_master(info->src_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL);

    masters.push_back(new_master);
    if (!masters.back().add_peer()) {
      Serial.println("Failed to register the new master");
      return;
    }
  } else {
    // The slave will only receive broadcast messages
    log_v("Received a unicast message from " MACSTR, MAC2STR(info->src_addr));
    log_v("Igorning the message");
  }
}

String getDefaultMacAddress() {

  String mac = "";

  unsigned char mac_base[6] = {0};

  if (esp_efuse_mac_get_default(mac_base) == ESP_OK) {
    char buffer[18];  // 6*2 characters for hex + 5 characters for colons + 1 character for null terminator
    sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);
    mac = buffer;
  }

  return mac;
}

/* Main */
void setup() {
  pinMode(2, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  // pinMode(22, OUTPUT);

  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	// myservo.attach(21, 1000, 2000);
  myservo.attach(21, 500, 2450);
  myservo.write(0);
  
  Serial.begin(115200);
  Wire.begin(SDA0_Pin, SCL0_Pin);
  Serial.println(F("SSD1306 allocation OK"));
  while (!Serial) {
    delay(10);
  }

  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }
  
  char buffer[256];
  String text = getDefaultMacAddress();
  text.toCharArray(buffer, text.length()+1);
  Serial.println(buffer);
  delay(2000);

  Serial.println("ESP-NOW Example - Broadcast Slave");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    Serial.println("Failed to initialize ESP-NOW");
    Serial.println("Reeboting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  // Register the new peer callback
  ESP_NOW.onNewPeer(register_new_master, NULL);

  Serial.println("Setup complete. Waiting for a master to broadcast a message...");
}

void loop() {
  delay(1000);
}
