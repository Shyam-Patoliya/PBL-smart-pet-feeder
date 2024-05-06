#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Servo.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_LiquidCrystal.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000);

Servo servo;

// Adafruit IO configuration
#define IO_USERNAME  "aschoudhary"
#define IO_KEY       "1ac95cb8580b4271bbb6d9f75d0668f1"
#define WIFI_SSID    "Galaxy-M20"
#define WIFI_PASS    "ac312124"

// Define servo pin and angles
#define SERVO_PIN    D3
#define CLOSE_ANGLE  0
#define OPEN_ANGLE   60

// Define LCD pins
#define I2C_ADDR     0x27
#define LCD_COLS     16
#define LCD_ROWS     2

Adafruit_LiquidCrystal lcd(I2C_ADDR);

// Initialize the MQTT client
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, "io.adafruit.com", 1883, IO_USERNAME, IO_KEY);

// Set up the feed you're subscribing to
Adafruit_MQTT_Subscribe onoff(&mqtt, IO_USERNAME "/f/onoff");

// Time variables
int hh, mm, ss;
int feed_hour = 0;
int feed_minute = 0;

// Feed condition
boolean feed = true;

void setup() {
  Serial.begin(9600);
  timeClient.begin();

  // Connect to Wi-Fi
  Serial.print("\n\nConnecting Wifi... ");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println("OK!");

  // Initialize LCD
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.setBacklight(LOW);

  // Attach servo
  servo.attach(SERVO_PIN);
  servo.write(CLOSE_ANGLE);

  // Subscribe to the onoff feed
  mqtt.subscribe(&onoff);
}

void loop() {
  MQTT_connect();
  timeClient.update();
  hh = timeClient.getHours();
  mm = timeClient.getMinutes();
  ss = timeClient.getSeconds();

  // Display current time on LCD
  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  if (hh > 12) {
    hh = hh - 12;
    lcd.print(hh);
    lcd.print(":");
    lcd.print(mm);
    lcd.print(":");
    lcd.print(ss);
    lcd.print(" PM");
  } else {
    lcd.print(hh);
    lcd.print(":");
    lcd.print(mm);
    lcd.print(":");
    lcd.print(ss);
    lcd.print(" AM");
  }

  // Display feed time on LCD
  lcd.setCursor(0, 1);
  lcd.print("Feed Time: ");
  lcd.print(feed_hour);
  lcd.print(':');
  lcd.print(feed_minute);

  // Handle MQTT messages
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    if (subscription == &onoff) {
      Serial.println((char*) onoff.lastread);
      if (!strcmp((char*) onoff.lastread, "ON")) {
        open_door();
        delay(1000);
        close_door();
      }
      if (!strcmp((char*) onoff.lastread, "Morning")) {
        feed_hour = 10;
        feed_minute = 30;
      }
      if (!strcmp((char*) onoff.lastread, "Afternoon")) {
        feed_hour = 1;
        feed_minute = 30;
      }
      if (!strcmp((char*) onoff.lastread, "Evening")) {
        feed_hour = 6;
        feed_minute = 30;
      }
    }
  }

  // Trigger feed if current time matches feed time and feed condition is true
  if (hh == feed_hour && mm == feed_minute && feed == true) {
    open_door();
    delay(1000);
    close_door();
    feed = false;
  }
}

void MQTT_connect() {
  int8_t ret;
  // Stop if already connected
  if (mqtt.connected()) {
    return;
  }
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    mqtt.disconnect();
    delay(5000);  // Wait 5 seconds before retrying
    retries--;
    if (retries == 0) {
      while (1);  // Basically die and wait for WDT to reset
    }
  }
}

void open_door() {
  servo.write(OPEN_ANGLE);   // Open the trap door
}

void close_door() {
  servo.write(CLOSE_ANGLE);  // Close the trap door
}
