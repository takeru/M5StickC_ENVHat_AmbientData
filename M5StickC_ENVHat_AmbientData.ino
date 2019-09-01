#include <M5StickC.h>
#include "DHT12.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include "Ambient.h"
#include <Preferences.h>
#include <esp_wifi.h>
#include <esp_bt_main.h>
#include <esp_bt.h>
#include <driver/adc.h>

Preferences preferences;
char wifi_ssid[33];
char wifi_key[65];

DHT12 dht12; 
Adafruit_BMP280 bme;
WiFiClient client;

struct Profile {
  int index;
  char* macAddress;
  unsigned int channelId;
  char* writeKey;
};

#include "config.h"

struct Profile *profile = NULL;

#define LED_BUILTIN 10

void blink(int n){
  for(int i=0; i<n; i++){
    digitalWrite(LED_BUILTIN, LOW);
    delay(10);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
  }
}

void setup() {
  M5.begin(false);
  M5.Axp.ScreenBreath(7);
  M5.Lcd.begin();

  preferences.begin("Wi-Fi", true);
  preferences.getString("ssid", wifi_ssid, sizeof(wifi_ssid));
  preferences.getString("key", wifi_key, sizeof(wifi_key));
  preferences.end();

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setRotation(3);
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 1);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Wire.begin(0,26);
  Serial.begin(115200);

  byte macAddress[6];
  WiFi.macAddress(macAddress);
  char mac[18];
  sprintf(mac, "%02x:%02x:%02x:%02x:%02x:%02x", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
  Serial.printf("macAddress: %s\n", mac);

  for(int i=0; i<PROFILE_COUNT; i++){
    if(strcmp(mac, profiles[i].macAddress) == 0){
      profile = &profiles[i];
      break;
    }
  }
  if(profile){
    M5.Lcd.printf("ENV(%d)\n", profile->index);
    Serial.printf("ENV(%d)\n", profile->index);
  }

  pinMode(M5_BUTTON_HOME, INPUT);

  while (!bme.begin(0x76)){
    M5.Lcd.println("BMP280 init fail\n");
    Serial.printf("BMP280 init fail\n");
    blink(4);
    delay(1000);
  }
}

void loop() {
  float temperature = dht12.readTemperature();
  float humidity    = dht12.readHumidity();
  float pressure    = bme.readPressure() / 100.0;
  double vbatAxp    = M5.Axp.GetVbatData() * 1.1 / 1000;
  float tempAxp     = -144.7 + M5.Axp.GetTempData() * 0.1;
  double vusbinAxp  = GetVusbinData() * 1.7 /1000;

  Serial.printf("%2.1f'C %2.0f%% %2.1fhPa\n", temperature, humidity, pressure);
  Serial.printf("%4.2fV %2.1f'C %4.2fV\n", vbatAxp, tempAxp, vusbinAxp);
  M5.Lcd.printf("%2.1f'C %2.0f%% %2.1fhPa\n", temperature, humidity, pressure);
  M5.Lcd.printf("%4.2fV %2.1f'C %4.2fV\n", vbatAxp, tempAxp, vusbinAxp);

  if(profile){
    WiFi.begin(wifi_ssid, wifi_key);

    for(int i=0; i<30; i++){
      if(WiFi.status() == WL_CONNECTED){
        break;
      }
      delay(1000);
      Serial.print(".");
    }
    Serial.printf("\n");

    if(WiFi.status() == WL_CONNECTED){
      IPAddress ipAddress = WiFi.localIP();
      M5.Lcd.printf("%s\n", ipAddress.toString().c_str());
      Serial.printf("%s\n", ipAddress.toString().c_str());
  
      Ambient ambient;
      ambient.begin(profile->channelId, profile->writeKey, &client);
      ambient.set(1, temperature);
      ambient.set(2, humidity);
      ambient.set(3, pressure);
      ambient.set(4, vbatAxp);
      ambient.set(5, tempAxp);
      ambient.set(6, vusbinAxp);
      bool ok = ambient.send();
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
  
      M5.Lcd.printf("ok=%d\n", ok);
      Serial.printf("ok=%d\n", ok);
  
      if(ok){ blink(1); }else{ blink(2); }
    }else{
      blink(3);
    }
  }

  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;
  M5.Rtc.GetTime(&time);
  M5.Rtc.GetData(&date);
  char datetime[20];
  sprintf(datetime, "%04d-%02d-%02d %02d:%02d:%02d", date.Year, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds);
  M5.Lcd.printf("%s\n", datetime);
  Serial.printf("%s\n", datetime);
  delay(1000);

  esp_wifi_stop();
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  esp_bt_mem_release(ESP_BT_MODE_BTDM);
  adc_power_off();
  M5.Axp.DeepSleep(SLEEP_SEC(60 - time.Seconds));

//  if(digitalRead(M5_BUTTON_HOME) == LOW){
//    // something
//    while(digitalRead(M5_BUTTON_HOME) == LOW);
//  }
}

uint16_t GetVusbinData(void)
{
  uint16_t vin = 0;

  Wire1.beginTransmission(0x34);
  Wire1.write(0x5a);
  Wire1.endTransmission();
  Wire1.requestFrom(0x34, 1);
  uint8_t buf = Wire1.read();

  Wire1.beginTransmission(0x34);
  Wire1.write(0x5b);
  Wire1.endTransmission();
  Wire1.requestFrom(0x34, 1);
  uint8_t buf2 = Wire1.read();

  vin = ((buf << 4) + buf2); // V
  return vin;
}
