/*
 * original code for esp32 found at espressif site:
 * https://docs.espressif.com/projects/arduino-esp32/en/latest/api/reset_reason.html
 */
#include "ResetReason.h"

#if defined(ESP32)
  #if CONFIG_IDF_TARGET_ESP32  // ESP32/PICO-D4
  #include "esp32/rom/rtc.h"
  #elif CONFIG_IDF_TARGET_ESP32S2
  #include "esp32s2/rom/rtc.h"
  #elif CONFIG_IDF_TARGET_ESP32C2
  #include "esp32c2/rom/rtc.h"
  #elif CONFIG_IDF_TARGET_ESP32C3
  #include "esp32c3/rom/rtc.h"
  #elif CONFIG_IDF_TARGET_ESP32S3
  #include "esp32s3/rom/rtc.h"
  #elif CONFIG_IDF_TARGET_ESP32C6
  #include "esp32c6/rom/rtc.h"
  #elif CONFIG_IDF_TARGET_ESP32H2
  #include "esp32h2/rom/rtc.h"
  #else
  #error Target CONFIG_IDF_TARGET is not supported
  #endif
#endif

String ResetReasonClass::getResetReason(uint8_t cpu_id) {
#if defined(ESP32)
  RESET_REASON reason;
  reason = rtc_get_reset_reason(cpu_id);

  String reason_str;

  switch (reason) {
    case 1:  reason_str = "Vbat power on reset"; break;
    case 3:  reason_str = "Software reset digital core"; break;
    case 4:  reason_str = "Legacy watch dog reset digital core"; break;
    case 5:  reason_str = "Deep Sleep reset digital core"; break;
    case 6:  reason_str = "Reset by SLC module, reset digital core"; break;
    case 7:  reason_str = "Timer Group0 Watch dog reset digital core"; break;
    case 8:  reason_str = "Timer Group1 Watch dog reset digital core"; break;
    case 9:  reason_str = "RTC Watch dog Reset digital core"; break;
    case 10: reason_str = "Instrusion tested to reset CPU"; break;
    case 11: reason_str = "Time Group reset CPU"; break;
    case 12: reason_str = "Software reset CPU"; break;
    case 13: reason_str = "RTC Watch dog Reset CPU"; break;
    case 14: reason_str = "for APP CPU, reset by PRO CPU"; break;
    case 15: reason_str = "Reset when the vdd voltage is not stable"; break;
    case 16: reason_str = "RTC Watch dog reset digital core and rtc module"; break;
    default: reason_str = "NO_MEAN";
  }

  return reason_str;
#else // eps8266
  return ESP.getResetReason();
#endif
}

