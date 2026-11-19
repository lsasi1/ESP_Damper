#include "helpers.h"
#include "config.h"
#include "serial_intercept.h"

// Convert 56-bit uint64_t to binary string (only called when debug logging is active)
String frame64ToBinaryString(uint64_t frameBits, uint8_t bitCount) {
    if (bitCount == 0 || bitCount > 64) return "";
    char buf[65];
    for (uint8_t i = 0; i < bitCount; ++i) {
        uint8_t shift = (bitCount - 1) - i;
        buf[i] = ((frameBits >> shift) & 0x1ULL) ? '1' : '0';
    }
    buf[bitCount] = '\0';
    return String(buf);
}

// Convert 56-bit uint64_t directly to formatted HEX string ("14 19 40 F4 FF FF 42 ")
String frame64ToHexGroups(uint64_t frameBits, uint8_t byteCount) {
    if (byteCount == 0 || byteCount > 8) return "";
    char buf[32];
    size_t pos = 0;
    for (int i = byteCount - 1; i >= 0; --i) {
        uint8_t b = (frameBits >> (i * 8)) & 0xFF;
        pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X ", b);
    }
    return String(buf);
}

// Legacy helper kept for compatibility
String binaryToHexGroups(const String &binStr) {
    String padded = binStr;
    if (padded.length() == 0) return "";
    while (padded.length() % 8 != 0) padded = "0" + padded;
    String result;
    for (int i = 0; i < (int)padded.length(); i += 8) {
        String byteStr = padded.substring(i, i + 8);
        uint8_t byteVal = (uint8_t)strtoul(byteStr.c_str(), nullptr, 2);
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X ", byteVal);
        result += buf;
    }
    return result;
}

uint32_t binToDec(const String &binStr) {
    return strtoul(binStr.c_str(), nullptr, 2);
}

// Zero-allocation validation & parsing on 56-bit frames (MSB-first):
// Bit layout (55 down to 0):
//   [55..48] Byte 0: Header (0x14)
//   [47..40] Byte 1: Raw temperature (previously bits 8..16)
//   [39]     Bit 16: Reserved/Status MSB
//   [38]     Bit 17: Power State (1 = ON, 0 = OFF)
//   [37..36] Bits 18..19: Damper angle (0..3)
//   [35..0]  Remaining bytes + parity trailer
bool validateAndParseFrames64(const uint64_t frames[RX_FRAMES],
                              const uint8_t bitCounts[RX_FRAMES],
                              rx_data_t &rx_data,
                              const char* model) {
    bool valid0 = (bitCounts[0] == 56);
    bool valid1 = (bitCounts[1] == 56);
    bool valid2 = (bitCounts[2] == 56);

    bool match12 = valid0 && valid1 && (frames[0] == frames[1]);
    bool match13 = valid0 && valid2 && (frames[0] == frames[2]);
    bool match23 = valid1 && valid2 && (frames[1] == frames[2]);

    uint64_t selected = 0;
    if (match12 || match13) {
        selected = frames[0];
    } else if (match23) {
        selected = frames[1];
    } else {
        Serial.println("No valid matching frames. Ignored.");
        return false;
    }

    if (device_config.debug_verbose) {
        Serial.println("Valid frame (binary): " + frame64ToBinaryString(selected, 56));
        Serial.println("Valid frame (hex): " + frame64ToHexGroups(selected, 7));
    }

    // Extract fields directly via bit shifts & masks (O(1), zero heap)
    uint8_t rawTemperature = (uint8_t)((selected >> 40) & 0xFFULL);
    bool state             = ((selected >> 38) & 0x1ULL) != 0;
    uint8_t damperAngle    = (uint8_t)((selected >> 36) & 0x3ULL);

    AcModel ac_model   = ac_model_from_string(model);
    AcMode mode        = ac_rmt_mode_to_mqtt(ac_model, rawTemperature);
    uint8_t temperature = ac_rmt_temp_to_mqtt(ac_model, mode, rawTemperature);

    rx_data.temp  = temperature;
    rx_data.state = state ? "on" : "off";
    rx_data.fan   = damperAngle;
    rx_data.mode  = ac_mode_to_string(mode);

    if (device_config.debug_verbose) {
        Serial.printf("Temperature: %d C\n", rx_data.temp);
        Serial.printf("State: %s\n", rx_data.state.c_str());
        Serial.printf("Damper Angle: %d\n", rx_data.fan);
        Serial.printf("Mode: %s\n", rx_data.mode.c_str());
    }

    return true;
}

String damper_angle_to_fan_label(uint8_t angle) {
    switch (angle) {
        case 0: return "Low";
        case 1: return "Medium";
        case 2: return "High";
        case 3: return "Auto";
        default: return "Low";
    }
}

uint8_t fan_label_to_damper_angle(const String& label) {
    String v = label;
    v.trim();
    v.toLowerCase();
    if (v == "low")    return 0;
    if (v == "medium") return 1;
    if (v == "high")   return 2;
    if (v == "auto")   return 3;
    return 0;
}