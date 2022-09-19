// Wifi
#ifndef STASSID
#define STASSID "Chez_FloEtFlo"
#define STAPSK "Vyygop8j!"
// #define STASSID "Dialler"
// #define STAPSK "123456789"
#endif

// BME280
#ifndef BME280_ADDR
#define BME280_ADDR 0x76
#endif

// Display
#ifndef SCREEN_ADDR
#define SCREEN_ADDR 0x3C
#endif

#ifndef TIME_PAGE
#define TIME_PAGE 0
#define TEMPERATURE_PAGE 1
#define HUMIDITY_PAGE 2
#define PRESSURE_PAGE 3
#endif

// Button
#ifndef BUTTON_PIN
#define BUTTON_PIN 15
#endif

// MQTT
#ifndef MQTT_SERVER
#define MQTT_SERVER "192.168.0.253"
#define MQTT_USER "out"
#define MQTT_PASS "out"
#define MQTT_ID "out"
#endif

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 600

#include <Wire.h>

// Display
#include "SSD1306Wire.h"
#include "OLEDDisplayUi.h"
#include "Lato_Black_16.h"
#include "Lato_Black_45.h"

// Wifi
#include <WiFi.h>

// BME280
#include <Adafruit_BME280.h>

// NTP
#include <time.h>

// MQTT
#include <PubSubClient.h>

// Button
#include <Bounce2.h>

#include "icons.h"

// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
SSD1306Wire display(SCREEN_ADDR, SDA, SCL);
OLEDDisplayUi ui(&display);

// Wifi
WiFiClient wifi;
int wifiAttempt = 0;

// Display
RTC_DATA_ATTR bool displayOn = true;
unsigned long displayLastOn = 0;
const long displayTimeOn = 30 * 1000; // 30 seconds

// NTP
time_t now;   // this is the epoch
tm localTime; // the structure tm holds time information in a more convient way

// BME280
Adafruit_BME280 bme; // use I2C interface

// Button
Bounce button = Bounce();

// MQTT
PubSubClient mqtt(wifi);
unsigned long mqttLastReconnect = 0;
unsigned long lastPublish = 0;

void mqttSetup()
{
  mqtt.setServer(MQTT_SERVER, 1883);
  mqtt.setSocketTimeout(5);
  mqtt.setKeepAlive(20);
  // mqtt.setCallback(mqttCallback);
}

bool mqttReconnect()
{
  if (!mqtt.connect(MQTT_ID, MQTT_USER, MQTT_PASS))
  {
    Serial.print("failed, rc=");
    Serial.print(mqtt.state());
    Serial.println(" try again in 5 seconds");
  }
  return mqtt.connected();
}

void publishSensor(unsigned long millis)
{
  if (lastPublish == 0 | millis - lastPublish >= 10 * 60 * 1000)
  {
    Serial.println("publish sensor");
    char currentTemp[6];
    sprintf(currentTemp, "%.1f", bme.readTemperature());
    mqtt.publish("home/out/temperature", currentTemp);

    char currentHumidity[5];
    sprintf(currentHumidity, "%.1f", bme.readHumidity());
    mqtt.publish("home/out/humidity", currentHumidity);

    char currentPressure[5];
    sprintf(currentPressure, "%d", (int)bme.readPressure() / 100);
    mqtt.publish("home/out/pressure", currentPressure);
    lastPublish = millis;
  }
}

void displaySetup(FrameCallback *frames, int frameCount, OverlayCallback *overlays, int overlaysCount)
{
  Serial.println(F("DisplaySetup"));

  ui.disableAllIndicators();

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);

  // Add frames
  ui.setFrames(frames, frameCount);

  // Add overlays
  ui.setOverlays(overlays, overlaysCount);

  // Initialising the UI will init the display too.
  ui.init();
  display.flipScreenVertically();
  display.setBrightness(255);
}

bool wifiSetup()
{
  const char ssid[] PROGMEM = STASSID;
  const char password[] PROGMEM = STAPSK;
  Serial.printf("Connecting to: %s", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && wifiAttempt <= 10)
  {
    delay(500);
    Serial.print(".");
    wifiAttempt++;
  }
  Serial.println("");
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.printf("Connected\n");
    return true;
  }
  else
  {
    Serial.println(F("Not connected"));
    return false;
  }
}

void sleep()
{
  if (displayOn)
  {
    display.displayOff();
  }

  if (mqtt.connected())
  {
    mqtt.disconnect();
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    WiFi.disconnect();
  }

  Serial.println(F("Deep Sleep"));
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_deep_sleep_start();
}

void overlay(OLEDDisplay *display, OLEDDisplayUiState *state)
{
  char currentDate[11];
  sprintf(currentDate, "%02d/%02d/%d", localTime.tm_mday, localTime.tm_mon, localTime.tm_year - 100);

  display->setTextAlignment(TEXT_ALIGN_RIGHT);
  display->setFont(Lato_Black_16);
  display->drawString(128, 0, currentDate);

  if (state->currentFrame == TIME_PAGE)
  {
    display->drawXbm(0, 0, 16, 16, clock_bits);
  }
  else if (state->currentFrame == TEMPERATURE_PAGE)
  {
    display->drawXbm(0, 0, 16, 16, temperature_bits);
  }
  else if (state->currentFrame == HUMIDITY_PAGE)
  {
    display->drawXbm(0, 0, 16, 16, humidity_bits);
  }
  else if (state->currentFrame == PRESSURE_PAGE)
  {
    display->drawXbm(0, 0, 16, 16, pressure_bits);
  }
}

void drawTime(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
  char currentTime[6];
  sprintf(currentTime, "%02d:%02d", localTime.tm_hour, localTime.tm_min);
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(Lato_Black_45);
  display->drawString(x + (128 / 2), y + 10, currentTime);
}

void drawTemperature(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
  char currentTemp[6];
  sprintf(currentTemp, "%.1f", bme.readTemperature());
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(Lato_Black_45);
  display->drawString(x + (128 / 2), y + 10, currentTemp);
}

void drawHumidity(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
  char currentHumidity[5];
  sprintf(currentHumidity, "%.1f", bme.readHumidity());

  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(Lato_Black_45);
  display->drawString(x + (128 / 2), y + 10, currentHumidity);
}

void drawPressure(OLEDDisplay *display, OLEDDisplayUiState *state, int16_t x, int16_t y)
{
  char currentPressure[5];
  sprintf(currentPressure, "%d", (int)bme.readPressure() / 100);
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(Lato_Black_45);
  display->drawString(x + (128 / 2), y + 10, currentPressure);
}

void readButton(unsigned long millis)
{
  button.update();
  if (button.fell())
  {
    ui.nextFrame();
  }
}

void wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    displayOn = true;
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    displayOn = false;
    break;
  default:
    displayOn = true;
    Serial.println("Unknown wakeup reason");
    break;
  }
}

FrameCallback frames[] = {drawTime, drawTemperature, drawHumidity, drawPressure};

// how many frames are there?
int frameCount = 4;

// Overlays are statically drawn on top of a frame eg. a clock
OverlayCallback overlays[] = {overlay};
int overlaysCount = 1;

void setup()
{
  Serial.begin(115200);
  setCpuFrequencyMhz(80);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, LOW);

  wakeup_reason();
  // Loading screen
  if (displayOn)
  {
    display.init();
    display.flipScreenVertically();
    display.drawXbm((128 - 40) / 2, (64 - 40) / 2, 40, 40, loading_bits);
    display.display();
    Serial.println("Loading");
  }
  // Configure sensor
  if (!bme.begin(BME280_ADDR))
  {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1)
      delay(10);
  }

  // Configure wifi
  if (wifiSetup())
  {
    configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "fr.pool.ntp.org");
    mqttSetup();
    if (displayOn)
    {
      displaySetup(frames, frameCount, overlays, overlaysCount);
    }
    button.attach(BUTTON_PIN, INPUT_PULLUP);
    button.interval(5);
  }
}

void loop()
{
  if (WiFi.status() == WL_CONNECTED && getLocalTime(&localTime))
  {
    const unsigned long milliseconds = millis();
    readButton(milliseconds);

    if (displayOn)
    {
      int remainingTimeBudget = ui.update();
      if (remainingTimeBudget > 0)
      {
        delay(remainingTimeBudget);
      }
    }

    if (milliseconds - displayLastOn > displayTimeOn)
    {
      sleep();
    }

    if (!mqtt.connected())
    {
      if (milliseconds - mqttLastReconnect > 5000)
      {
        Serial.println("Trying to reconnect");
        mqttLastReconnect = milliseconds;
        if (mqttReconnect())
        {
          Serial.println("reconnected");
          mqttLastReconnect = 0;
        }
      }
    }
    else
    {
      mqtt.loop();
      publishSensor(milliseconds);
    }
  }
}
