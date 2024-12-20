#pragma once

#include <Arduino.h>

class ResetReasonClass {
public:
    String getResetReason(uint8_t cpu_id);
};
