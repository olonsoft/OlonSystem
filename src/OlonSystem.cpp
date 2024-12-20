// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou and others
 * olonsoft: Added changes for my own purpose
 */
#include <LittleFS.h>
#include <OlonSystem.h>
#include <Preferences.h>
#include "ResetReason.h"

#ifdef ESP32
  #include <WiFi.h>
  #include <esp_system.h>
  #include <nvs_flash.h>
  #include <soc/rtc.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <user_interface.h>
#endif

#ifdef OLON_DEBUG_SYSTEM
  #define LOGD(tag, format, ...) {                              \
      Serial.printf("\033[0;36m%6lu [D] [%s] ", millis(), tag); \
      Serial.printf(format "\033[0m\n", ##__VA_ARGS__); }
  #define LOGI(tag, format, ...) {                              \
      Serial.printf("\033[0;32m%6lu [I] [%s] ", millis(), tag); \
      Serial.printf(format "\033[0m\n", ##__VA_ARGS__); }
  #define LOGW(tag, format, ...) {                              \
      Serial.printf("\033[0;33m%6lu [W] [%s]", millis(), tag);  \
      Serial.printf(format "\033[0m\n", ##__VA_ARGS__); }
  #define LOGE(tag, format, ...) {                              \
      Serial.printf("\033[0;31m%6lu [E] [%s] ", millis(), tag); \
      Serial.printf(format "\033[0m\n", ##__VA_ARGS__); }
  #define LOGBULK(...) Serial.printf(PSTR("%s"), ##__VA_ARGS__);
#else
  #define LOGD(tag, format, ...)
  #define LOGI(tag, format, ...)
  #define LOGW(tag, format, ...)
  #define LOGE(tag, format, ...)
  #define LOGBULK(...)
#endif

static const char* TAG = "SYSTEM";
#define CONFIG_NAME   "system"
#define KEY_POWER_ONS "power"

#define RTC_OFFSET 100

struct rtcData_t {
  uint16_t valid;
  size_t   restarts;
};

#ifdef ESP8266
rtcData_t rtcData;
#elif defined(ESP32)
RTC_NOINIT_ATTR rtcData_t rtcData;
#endif

const uint16_t RTC_MEM_VALID = 0xA55A;

void Olon::SystemClass::init() {
  _initialFreeHeap = getFreeHeap();
  LOGI(TAG, "Initializing File System...");
#ifdef ESP8266
  if (LittleFS.begin()) {
    LOGD(TAG, "File System initialized.");
  } else {
    LOGW(TAG, "File System initialization failed. Trying to format...");
    if (LittleFS.format()) {
      LOGW(TAG, "Successfully formatted and initialized");
    } else {
      LOGE(TAG, "Failed to format");
    }
  }
#elif defined(ESP32)

  if (LittleFS.begin(false)) {
    LOGD(TAG, "File System initialized");
  } else {
    LOGW(TAG, "File System initialization failed. Trying to format...");
    if (LittleFS.begin(true)) {
      LOGW(TAG, "Successfully formatted and initialized");
    } else {
      LOGE(TAG, "Failed to format");
    }
  }
#endif

  _rtcDataRead();
  Preferences prefs;
  prefs.begin(CONFIG_NAME, false);

  _powerOns = (prefs.isKey(KEY_POWER_ONS) ? prefs.getULong(KEY_POWER_ONS, 0) : 0);

  if (rtcData.restarts == 0) {  // device powered on
    prefs.putULong(KEY_POWER_ONS, ++_powerOns);
  }

  prefs.end();

  rtcData.restarts++;
  _rtcDataWrite();
  LOGI(TAG, "Powered on %" PRIu32 " times", _powerOns);
  LOGI(TAG, "Restarted  %" PRIu32 " times", rtcData.restarts - 1);
}

void Olon::SystemClass::reset() {
  LOGD(TAG, "Triggering System Reset...");
#ifdef ESP8266
  // LittleFS.format();
  clear();
  restart();
#elif defined(ESP32)
  // LittleFS.format();
  nvs_flash_erase();  // erase the NVS partition and...
  nvs_flash_init();   // initialize the NVS partition.
  restart();
#endif
}

void Olon::SystemClass::restart() {
  LOGD(TAG, "Triggering System Restart...");
  ESP.restart();
}

void Olon::SystemClass::clear() {
  Preferences prefs;
  prefs.begin(CONFIG_NAME);
  prefs.remove(KEY_POWER_ONS);
  prefs.end();
}

void Olon::SystemClass::deepSleep(uint32_t delaySeconds) {
  LOGI(TAG, "Deep Sleep for %d secs!", delaySeconds);
#ifdef ESP8266
  ESP.deepSleep((uint64_t)delaySeconds * 1000000ULL);
  delay(10000);
#elif defined(ESP32)
  #if SOC_UART_NUM > 2
  Serial2.end();
  #endif
  #if SOC_UART_NUM > 1
  Serial1.end();
  #endif
  Serial.end();

  // #if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
  // esp_bt_controller_disable();
  // esp_bluedroid_disable();
  // #endif

  // equiv to esp_deep_sleep(delayMicros);
  // esp_sleep_enable_timer_wakeup(seconds * (uint64_t)1000000ULL);
  // esp_deep_sleep_start();

  esp_deep_sleep((uint64_t)delaySeconds * 1000000ULL);

  esp_restart();
#endif
}

uint32_t Olon::SystemClass::getUptimeSeconds() const {
  static unsigned long last_uptime      = 0;
  static unsigned char uptime_overflows = 0;

  if (millis() < last_uptime)
    ++uptime_overflows;
  last_uptime = millis();

  return uptime_overflows * (UINT32_MAX / 1000) + (last_uptime / 1000);
}

String Olon::SystemClass::getUptimePretty() const {
  char     buffer[15];  // "99999T23:59:59"
  uint32_t s       = getUptimeSeconds();
  uint8_t  seconds = s % 60;
  s /= 60;                   // now it is minutes
  uint8_t minutes = s % 60;
  s /= 60;                   // now it is hours
  uint8_t  hours = s % 24;
  uint16_t days  = s /= 24;  // now it is days
  snprintf_P(buffer, sizeof(buffer), PSTR("%uT%02u:%02u:%02u"), days, hours,
             minutes, seconds);
  return buffer;
}

uint32_t Olon::SystemClass::getFreeHeap() {
  _freeHeap = ESP.getFreeHeap();
  return _freeHeap;
}

size_t Olon::SystemClass::getRestartCount() const {
  return rtcData.restarts - 1;
}

String Olon::SystemClass::getEspID() {
  uint32_t chipId = 0;
#ifdef ESP8266
  chipId = ESP.getChipId();
#elif defined(ESP32)
  for (int i = 0; i < 17; i += 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
#endif
  String espId = String(chipId, HEX);
  espId.toUpperCase();
  return espId;
}

bool Olon::SystemClass::startedAfterDeepSleep() {
  #ifdef ESP8266
    rst_info *resetInfo = ESP.getResetInfoPtr();
    if (resetInfo->reason == REASON_DEEP_SLEEP_AWAKE) return true;
  #elif defined(ESP32)
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) return true;
  #endif
  return false;
}

#ifdef OLON_JSON_SUPPORT
void Olon::SystemClass::toJson(const JsonObject& root) const {
  ResetReasonClass resetReason;

  root[F("CPU chip id")] = getEspID();
  root[F("CPU freq")]    = String(ESP.getCpuFreqMHz()) + "MHz";

  #ifdef ESP32
  root[F("Chip revision")] = ESP.getChipRevision();
  #endif

  root[F("SDK version")] = String(ESP.getSdkVersion());  // esp32 ??? ESP.getFullVersion()

  #ifdef ESP8266
  root[F("Core version")] = ESP.getCoreVersion();
  #endif

  const char compile_date[] = __DATE__ " " __TIME__;
  root[F("Built")]          = String(compile_date);

  root[F("Power On count")] = _powerOns;
  root[F("Restart count")]  = rtcData.restarts - 1;

  // ====== RAM =====
  root[F("Initial Free Heap")] = _initialFreeHeap;
  #ifdef ESP32
  root[F("Heap size")]      = ESP.getHeapSize();
  root[F("Heap free")]      = ESP.getFreeHeap();
  root[F("Heap min")]       = ESP.getMinFreeHeap();
  root[F("Heap max alloc")] = ESP.getMaxAllocHeap();
  #endif

  #ifdef ESP8266
  root[F("Heap free")]       = ESP.getFreeHeap();
  root[F("Heap free block")] = ESP.getMaxFreeBlockSize();
  root[F("Heap fragm")]      = String(ESP.getHeapFragmentation()) + "%";
  #endif

  // ====== SPI ======
  #ifdef ESP32
  root[F("SPI size")]      = ESP.getPsramSize();
  root[F("SPI Free")]      = ESP.getFreePsram();
  root[F("SPI min")]       = ESP.getMinFreePsram();
  root[F("SPI max alloc")] = ESP.getMaxAllocPsram();
  #endif

  // ====== Flash ======
  root[F("Flash chip speed")] = String(ESP.getFlashChipSpeed() / 1000000.0) + "MHz";  // ? on esp32 return 0

  #ifdef ESP8266
  root[F("Flash chip id")] = int32_t((ESP.getFlashChipId()));
  root[F("Flash size")]    = ESP.getFlashChipRealSize();
  #else
  root[F("Flash size")] = ESP.getFlashChipSize();
  #endif
  root[F("Firmware size")] = ESP.getSketchSize();
  root[F("Free FW space")] = ESP.getFreeSketchSpace();

  #ifdef ESP8266
  FSInfo fs_info;
  bool   fs = LittleFS.info(fs_info);
  if (fs) {
    root[F("FS size")]      = fs_info.totalBytes;
    root[F("FS used size")] = fs_info.usedBytes;

  } else {
    root[F("FS")] = F("No LittleFS partition");
  }
  #elif defined(ESP32)
  root[F("FS size")]      = LittleFS.totalBytes();
  root[F("FS used size")] = LittleFS.usedBytes();
  #endif

  // ================= WiFi ================
  if (WiFi.getMode() == WIFI_STA) {
    root[F("SSID")]  = WiFi.SSID();
    root[F("BSSID")] = WiFi.BSSIDstr();
    root[F("RSSI")]  = WiFi.RSSI();

    root[F("MAC")]    = WiFi.macAddress();
    root[F("IP")]     = WiFi.localIP().toString();
    root[F("SUBNET")] = WiFi.subnetMask().toString();
    root[F("GW")]     = WiFi.gatewayIP().toString();
  } else {
    root[F("IP")]  = WiFi.softAPIP().toString();
    root[F("MAC")] = WiFi.softAPmacAddress();
  }
  root[F("UpTime")] = getUptimePretty();

  #ifdef ESP32 // todo check boards with 1 core
  root[F("Reset Reason")] = "CPU1: " + resetReason.getResetReason(0) + " CPU2: " + resetReason.getResetReason(1);
  #else
  root[F("Reset Reason")] = resetReason.getResetReason(0);
  #endif
}
#endif

void Olon::SystemClass::_rtcDataRead() {
#ifdef ESP8266
  ESP.rtcUserMemoryRead(RTC_OFFSET, (uint32_t*)&rtcData, sizeof(rtcData));
#endif
  if (rtcData.valid != RTC_MEM_VALID) {  // new data detected after power on
    memset(&rtcData, 0, sizeof(rtcData));
    rtcData.valid    = RTC_MEM_VALID;
    // default values
    rtcData.restarts = 0;
    _rtcDataWrite();
  }
  _rtcDataCRC = _getDataCRC();
}

void Olon::SystemClass::_rtcDataWrite() {
  // save on ESP8266 only when new data detected. Don't wear the flash
  if (_getDataCRC() != _rtcDataCRC) {
    rtcData.valid = RTC_MEM_VALID;
#ifdef ESP8266
    ESP.rtcUserMemoryWrite(RTC_OFFSET, (uint32_t*)&rtcData, sizeof(rtcData));
#endif
    _rtcDataCRC = _getDataCRC();
  }
}

uint32_t Olon::SystemClass::_getDataCRC() {
  uint32_t crc   = 0;
  uint8_t* bytes = (uint8_t*)&rtcData;

  for (uint32_t i = 0; i < sizeof(rtcData); i++) {
    crc += bytes[i] * (i + 1);
  }
  return crc;
}

// namespace Olon {
// SystemClass System;
// }  // namespace Olon
