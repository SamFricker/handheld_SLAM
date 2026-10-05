/**
 * @file atech_helpers.h
 * @brief Common firmware patterns the LLM should reuse instead of writing by hand.
 *
 * Auto-included in every generated main.cpp. The LLM is told about these via
 * the codegen system prompt and uses them via the names in this header.
 *
 * Design rule: every helper here replaces a pattern the LLM has been observed
 * to write (and sometimes mis-write) repeatedly. Adding helpers is fine; the
 * cost is one cached prompt line per helper. Removing helpers requires
 * confirming no generated code references them.
 */

#pragma once
#include <Arduino.h>

namespace atech {

// ---------------------------------------------------------------------------
// Arithmetic
// ---------------------------------------------------------------------------

/// Float linear remap (Arduino's map() is integer-only).
/// Example: mapRangeF(adcReading, 0, 4095, 0.0, 3.3) → voltage.
inline float mapRangeF(float x, float inLo, float inHi, float outLo, float outHi) {
    if (inHi == inLo) return outLo;
    return outLo + (x - inLo) * (outHi - outLo) / (inHi - inLo);
}

/// Linear interpolation, t clamped to [0,1]. lerp(a, b, 0)=a, lerp(a, b, 1)=b.
inline float lerp(float a, float b, float t) {
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return a + (b - a) * t;
}

/// Modulo that handles negative dividends correctly. wrap(-1, 10) = 9.
inline int wrap(int x, int period) {
    if (period <= 0) return 0;
    int r = x % period;
    return r < 0 ? r + period : r;
}

// ---------------------------------------------------------------------------
// Time / events
// ---------------------------------------------------------------------------

/// "Every N ms, do this." Use instead of hand-rolled millis() bookkeeping.
/// Handles millis() rollover correctly via unsigned arithmetic.
///
///   EveryTimer t;
///   if (t.ready(500)) { /* runs every 500ms */ }
struct EveryTimer {
    unsigned long _last = 0;

    /// True at most once per period. Updates internal clock on success.
    bool ready(unsigned long ms) {
        unsigned long now = millis();
        if ((unsigned long)(now - _last) >= ms) {
            _last = now;
            return true;
        }
        return false;
    }

    /// Reset so the next ready() call returns false until the period elapses.
    void reset() { _last = millis(); }
};

/// Fires once on the false→true transition of a boolean. Replaces the
/// "remember last_state" pattern for button presses, threshold crossings, etc.
///
///   RisingEdge buttonEdge;
///   if (buttonEdge.fired(button_1.isPressed())) { /* single click */ }
struct RisingEdge {
    bool _last = false;

    bool fired(bool now) {
        bool out = (now && !_last);
        _last = now;
        return out;
    }
};

/// Fires once on the true→false transition.
struct FallingEdge {
    bool _last = false;

    bool fired(bool now) {
        bool out = (!now && _last);
        _last = now;
        return out;
    }
};

/// Software debounce for noisy mechanical inputs (buttons, limit switches).
/// Holds state for `threshold_ms` before accepting a change.
///
///   Debouncer button;
///   bool isPressed = button.update(button_1.isRawPressed(), 30);
struct Debouncer {
    bool _stable = false;
    unsigned long _last_change_ms = 0;

    bool update(bool raw, unsigned long threshold_ms = 30) {
        unsigned long now = millis();
        if (raw != _stable) {
            if ((unsigned long)(now - _last_change_ms) > threshold_ms) {
                _stable = raw;
                _last_change_ms = now;
            }
        } else {
            _last_change_ms = now;
        }
        return _stable;
    }
};

}  // namespace atech

// Pull commonly-named structs into the global namespace so LLM-generated code
// can write `EveryTimer t;` without `atech::` prefix. Arithmetic helpers stay
// namespaced to avoid colliding with Arduino's `map`, `constrain`, etc.
using atech::EveryTimer;
using atech::RisingEdge;
using atech::FallingEdge;
using atech::Debouncer;
