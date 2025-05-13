#pragma once

#ifdef OLON_JSON_SUPPORT
  #include <ArduinoJson.h>
#endif

#define OLON_SYSTEM_VERSION "1.0.4"

namespace Olon {

class SystemClass {
 public:
  void init();

  void reset();
  void restart();
  void clear();  // clear settings made by this class
  void deepSleep(uint32_t delaySeconds);

  // returns the uptime in seconds
  uint32_t getUptimeSeconds() const;
  String   getUptimePretty() const;
  uint32_t getFreeHeap();
  uint32_t getInitialFreeHeap() const {
    return _initialFreeHeap;
  };
  size_t getPowerOnCount() const {
    return _powerOns;
  }
  size_t getRestartCount() const;
  static String getEspID();
  bool          startedAfterDeepSleep();
#ifdef OLON_JSON_SUPPORT
  void toJson(const JsonObject& root) const;
#endif

 private:
  size_t   _powerOns        = 0;
  uint32_t _freeHeap        = 0;
  uint32_t _initialFreeHeap = 0;
  uint32_t _rtcDataCRC      = 0;
  void     _rtcDataRead();
  void     _rtcDataWrite();
  uint32_t _getDataCRC();
};

// extern SystemClass System;
}  // namespace Olon