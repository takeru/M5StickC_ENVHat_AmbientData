#include <M5StickC.h>
#include "DHT12.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "Ambient.h"
#include <Preferences.h>

DHT12 dht12; 
Adafruit_BMP280 bme;
WiFiClient client;

struct Profile {
  int index;
  uint8_t* mac;
  uint32_t channelId;
  char*    writeKey;
  uint32_t interval;
  bool battery;
};

#include "config.h"

struct Profile *profile = NULL;

#define LED_BUILTIN 10
#define LED_IR       9

void blink(int n){
  for(int i=0; i<n; i++){
    set_led_red(true);
    delay(10);
    set_led_red(false);
    delay(200);
  }
}

void setup() {
  // M5.begin(...);
  M5.Axp.begin();
  if(M5.Axp.GetBatVoltage() < 3.7){
    battery_charge_ctrl(3.7, 3.9, 100); // ON
  }else{
    battery_charge_ctrl(0.0, 0.0, 100); // OFF
  }
  M5.Axp.ScreenBreath(0);
  M5.Lcd.begin();
  M5.Rtc.begin();
  M5.Lcd.fillScreen(BLACK);

  if(4.5 < M5.Axp.GetVBusVoltage()){
    M5.Axp.ScreenBreath(8);
  }else{
    M5.Axp.ScreenBreath(7);
  }

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_IR, OUTPUT);
  pinMode(M5_BUTTON_HOME, INPUT);
  set_led_red(false);
  set_led_ir(false);

  Wire.begin(0,26);
  Serial.begin(115200);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("mac: %s\n", mac_string(mac).c_str());

  for(int i=0; i<PROFILE_COUNT; i++){
    if(memcmp(mac, profiles[i].mac, 6)==0){
      profile = &profiles[i];
      break;
    }
  }

  while (!bme.begin(0x76)){
    M5.Lcd.println("BMP280 init fail\n");
    Serial.printf("BMP280 init fail\n");
    blink(5);
    delay(1000);
  }
}

void loop() {
  static int state = 5;
  static int last_sec = rtc_seconds();
  static bool wifiOK = false;
  static bool led_and_udp = false;
  static int drain_mode = 0;
  #define WIFI_UDP     1
  #define CHARGE_190   2
  static unsigned long timeout_ms0 = 0;
  static unsigned long timeout_ms1 = 0;
  int sec = rtc_seconds();
  if(sec != last_sec){
    update_lcd();
    last_sec = sec;
  }

  static unsigned long last_counter = 0;
  unsigned long counter = millis() / 100;
  if(counter != last_counter){
    print_status();
    last_counter = counter;
  }

  switch(state){
  case 1:
    {
      wifiOK = wifi_connect(10*1000);
      if(wifiOK){
        bool ok = ntp_to_rtc();
        Serial.printf("ntp_to_rtc=%d\n", ok);
      }
      state = 2;
    }
    break;
  case 2:
    if(1 <= sec && sec < 15){
      if(wifiOK){
        post_sensor_values();
      }
      state = 3;
    }
    delay(1);
    break;
  case 3:
    RTC_TimeTypeDef time;
    M5.Rtc.GetTime(&time);
    // time.Minutes%3==0 &&
    if(profile->battery && 4.5 < M5.Axp.GetVBusVoltage()){
      if(M5.Axp.GetBatVoltage() < 3.8 || wifiOK==false){
        wifi_disconnect();
        drain_mode = CHARGE_190;
        Serial.printf("drain_mode = CHARGE_190\n");

        battery_charge_ctrl(5.0, 5.0, 190); // force ON

        timeout_ms0 = millis() + 5000;
        timeout_ms1 = 0;
      }else{
        drain_mode = WIFI_UDP;
        Serial.printf("drain_mode = WIFI_UDP\n");
        
        set_lcd_brightness(15);
        set_led_red(true);
        set_led_ir(true);

        timeout_ms0 = millis() + 5000;
        timeout_ms1 = 0;
      }
      state = 4;
    }else{
      state = 5;
    }
    break;
  case 4:
    if(drain_mode==WIFI_UDP){
      WiFiUDP udp;
      udp.beginPacket("192.168.0.1", 65535);
      udp.printf("millis=%d", millis());
      udp.endPacket();
      delay(1);
    }
    if(timeout_ms1==0 && 150 <= M5.Axp.GetVBusCurrent()){
      timeout_ms1 = millis();
      if(drain_mode==CHARGE_190){ timeout_ms1 += 1500; }
      if(drain_mode==WIFI_UDP  ){ timeout_ms1 += 1500; }
    }
    if(timeout_ms0 < millis() || (0 < timeout_ms1 && timeout_ms1 < millis())){
      state = 5;
    }
    break;
  case 5:
    wifi_disconnect();
    if(M5.Axp.GetBatVoltage() < 3.7){
      battery_charge_ctrl(3.7, 3.9, 100); // ON
    }else{
      battery_charge_ctrl(0.0, 0.0, 100); // OFF
    }
    set_lcd_brightness(0);
    set_led_red(false);
    set_led_ir(false);

    int32_t sleep_ms = (57-rtc_seconds())*1000;
    if(1000<sleep_ms){
      Serial.printf("DeepSleep sleep_ms=%d\n", sleep_ms);
      Serial.flush();
      M5.Axp.DeepSleep(sleep_ms*1000);
    }
    state = 1;
    break;
  }
}

void post_sensor_values(){
  float temperature = dht12.readTemperature();
  float humidity    = dht12.readHumidity();
  float pressure    = bme.readPressure() / 100.0;
  float vbatAxp     = M5.Axp.GetBatVoltage();
  float tempAxp     = M5.Axp.GetTempInAXP192();
  float vusbinAxp   = M5.Axp.GetVBusVoltage();
  Serial.printf("temperature=%.2f humidity=%.2f pressure=%.2f\n", temperature, humidity, pressure);

  IPAddress ipAddress = WiFi.localIP();
  Serial.printf("%s\n", ipAddress.toString().c_str());

  if(profile){
    bool ok = send_to_ambient(temperature, humidity, pressure, vbatAxp, tempAxp, vusbinAxp);
    Serial.printf("ok=%d\n", ok);
    if(ok){ blink(1); }else{ blink(2); }
  }else{
    blink(3);
  }
}

String mac_string(const uint8_t *mac)
{
  char macStr[18] = { 0 };
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

bool send_to_ambient(double temperature, double humidity, double pressure, double vbatAxp, double tempAxp, double vusbinAxp)
{
  Ambient ambient;
  ambient.begin(profile->channelId, profile->writeKey, &client);
  ambient.set(1, temperature);
  ambient.set(2, humidity);
  ambient.set(3, pressure);
  ambient.set(4, vbatAxp);
  ambient.set(5, tempAxp);
  ambient.set(6, vusbinAxp);
  bool ok = ambient.send();
  return ok;
}

bool ntp_to_rtc(){
  configTime(9 * 3600, 0, "ntp.nict.jp");
  struct tm i;
  if (getLocalTime(&i)) {
    RTC_TimeTypeDef t;
    t.Hours   = i.tm_hour;
    t.Minutes = i.tm_min;
    t.Seconds = i.tm_sec;
    M5.Rtc.SetTime(&t);

    RTC_DateTypeDef d;
    d.Year    = i.tm_year + 1900;
    d.Month   = i.tm_mon + 1;
    d.Date    = i.tm_mday;
    d.WeekDay = i.tm_wday;
    M5.Rtc.SetData(&d);

    return true;
  }else{
    return false;
  }
}

bool wifi_connect(int timeout_ms)
{
  if(WiFi.status() == WL_CONNECTED){
    return true;
  }

  Preferences preferences;
  char wifi_ssid[33];
  char wifi_key[65];

  preferences.begin("Wi-Fi", true);
  preferences.getString("ssid", wifi_ssid, sizeof(wifi_ssid));
  preferences.getString("key", wifi_key, sizeof(wifi_key));
  preferences.end();

  WiFi.begin(wifi_ssid, wifi_key);
  unsigned long start_ms = millis();
  bool connected = false;
  while(1){
    connected = WiFi.status() == WL_CONNECTED;
    if(connected || (start_ms+timeout_ms)<millis()){
      break;
    }
    delay(1);
  }
  return connected;
}

void wifi_disconnect(){
  if(WiFi.status() != WL_DISCONNECTED){
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
  }
}

String rtc_datetime_string(void){
  RTC_TimeTypeDef time;
  RTC_DateTypeDef date;
  M5.Rtc.GetTime(&time);
  M5.Rtc.GetData(&date);
  char datetime[20];
  sprintf(datetime, "%04d-%02d-%02d %02d:%02d:%02d", date.Year, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds);
  return String(datetime);
}

int rtc_seconds(void){
  RTC_TimeTypeDef time;
  M5.Rtc.GetTime(&time);
  return time.Seconds;
}

void print_status(void){
  String datetime = rtc_datetime_string();
  Serial.printf("ms=%d rtc=%s bat=%.2fV,%.2fmA vbus=%.2fV,%.2fmA temp=%.2f\n",
    millis(),
    datetime.c_str(),
    M5.Axp.GetBatVoltage(),
    M5.Axp.GetBatCurrent(),
    M5.Axp.GetVBusVoltage(),
    M5.Axp.GetVBusCurrent(),
    M5.Axp.GetTempInAXP192()
  );
}

void update_lcd(){
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setRotation(3);
  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(0, 1);
  if(profile){
    M5.Lcd.printf("Profile:%d\n", profile->index);
  }
  String datetime = rtc_datetime_string();
  M5.Lcd.printf("%s %d\n", datetime.c_str(), millis()/1000);

  M5.Lcd.setTextColor(WHITE, BLACK);
  if(0 < M5.Axp.GetBatCurrent()){ M5.Lcd.setTextColor(GREEN, BLACK); }
  if(0 > M5.Axp.GetBatCurrent()){ M5.Lcd.setTextColor(RED,   BLACK); }
  M5.Lcd.printf("bat= %4.2fV,%6.2fmA\n", M5.Axp.GetBatVoltage(), M5.Axp.GetBatCurrent());

  M5.Lcd.setTextColor(WHITE, BLACK);
  if(0 < M5.Axp.GetVBusCurrent()){ M5.Lcd.setTextColor(GREEN, BLACK); }
  if(0 > M5.Axp.GetVBusCurrent()){ M5.Lcd.setTextColor(RED,   BLACK); }
  M5.Lcd.printf("vbus=%4.2fV,%6.2fmA\n", M5.Axp.GetVBusVoltage(), M5.Axp.GetVBusCurrent());

  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.printf("temp=%.2f\n", M5.Axp.GetTempInAXP192());
}

uint8_t wire1_read(uint8_t from, uint8_t addr){
  Wire1.beginTransmission(from);
  Wire1.write(addr);
  Wire1.endTransmission();
  Wire1.requestFrom(from, (uint8_t)1);
  return Wire1.read();
}

void wire1_write(uint8_t from, uint8_t addr, uint8_t value){
  Wire1.beginTransmission(from);
  Wire1.write(addr);
  Wire1.write(value);
  Wire1.endTransmission();
}

void battery_charge_ctrl(float vbat_min, float vbat_max, int curr)
{
  float vbat = M5.Axp.GetBatVoltage();
  uint8_t reg0x33 = wire1_read(0x34, 0x33);
  uint8_t reg0x33_orig = reg0x33;
  if(reg0x33 & 0x80){
    if(vbat_max < vbat){ reg0x33 &= ~(1<<7); } // OFF
  }else{
    if(vbat < vbat_min){ reg0x33 |=  (1<<7); } // ON
  }
  if(curr==100){
    reg0x33 = (reg0x33 & 0xF0) | 0b0000;
  }else if(curr==190){
    reg0x33 = (reg0x33 & 0xF0) | 0b0001;
  }else if(curr==280){
    reg0x33 = (reg0x33 & 0xF0) | 0b0011;
  }else{
    reg0x33 = (reg0x33 & 0xF0) | 0b0000;
  }
  wire1_write(0x34, 0x33, reg0x33);
  reg0x33 = wire1_read(0x34, 0x33);
  if(reg0x33 != reg0x33_orig){
    Serial.printf("battery_charge_ctrl reg0x33: %02X->%02X\n", reg0x33_orig, reg0x33); // default=0xC0 OFF=0x40
  }
}

void set_lcd_brightness(uint8_t brightness){
  uint8_t reg0x28 = wire1_read(0x34, 0x28);
  wire1_write(0x34, 0x28, ((reg0x28 & 0x0f) | (brightness << 4)));
}

void set_led_red(bool on){
  digitalWrite(LED_BUILTIN, on ? LOW : HIGH);
}

void set_led_ir(bool on){
  digitalWrite(LED_IR, on ? LOW : HIGH);
}
