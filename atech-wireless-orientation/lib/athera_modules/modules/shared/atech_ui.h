/**
 * @file atech_ui.h
 * @brief Opinionated UI primitives for Atech displays.
 *
 * Goal: the LLM composes screens by calling these helpers instead of doing
 * setCursor/setTextColor/print/math by hand. Pre-baked layout + colors mean
 * generated UIs look consistent and don't depend on the LLM picking sensible
 * pixel positions.
 *
 * All helpers are templated on the display so they work across any
 * Adafruit-GFX-compatible driver (today: ST7735_TFT; future: OLEDs etc.).
 *
 * Auto-included in every generated main.cpp.
 */

#pragma once
#include <Arduino.h>
#include "atech_helpers.h"

// ---------------------------------------------------------------------------
// Atech display palette (RGB565)
//
// Use these instead of raw hex literals so generated projects look consistent
// and the design can be retuned in one place. The LLM may pick any of these
// or use raw colors if a project explicitly needs a different palette.
// ---------------------------------------------------------------------------

#define ATECH_BG        0x0000  // Background — black
#define ATECH_FG        0xFFFF  // Primary text — white
#define ATECH_ACCENT    0x07FF  // Accent / live value — cyan
#define ATECH_MUTED     0x8410  // Labels, hints — soft gray
#define ATECH_OK        0x07E0  // Success / nominal — green
#define ATECH_WARN      0xFD20  // Warning — orange
#define ATECH_ALERT     0xF800  // Critical — red

namespace atech {

// ---------------------------------------------------------------------------
// Rate-limited render pattern
//
// Wraps the canonical canvas-mode loop body:
//     clear → draws → display
// in a rate-limited shell so the LLM does not have to write timing logic.
// Pass the display, a timer, the desired interval in ms, and the draw lambda.
// The lambda body is the ONLY thing the LLM has to think about.
//
// Use:
//     EveryTimer screenTimer;        // in global_variables
//     // in loop_code or task body:
//     atech::renderEvery(screenTimer, 33, screen, [&]() {
//         atech::drawHeader(screen, "Status");
//         // ... more draws ...
//     });
// ---------------------------------------------------------------------------

template <typename Display, typename DrawFn>
inline void renderEvery(EveryTimer& timer, unsigned long ms, Display& d, DrawFn draw) {
    if (timer.ready(ms)) {
        d.clear();
        draw();
        d.display();
    }
}

// ---------------------------------------------------------------------------
// drawHeader — top title bar with accent underline.
//
//   atech::drawHeader(screen, "Temperature");
//
// Draws the title at the top using the default small font in ATECH_FG, with
// a 1px ATECH_ACCENT underline at y=12.
// ---------------------------------------------------------------------------

template <typename Display>
inline void drawHeader(Display& d, const char* title) {
    d.setFont(NULL);
    d.setTextSize(1);
    d.setTextColor(ATECH_FG);
    d.setCursor(4, 2);
    d.print(title);
    d.drawLine(0, 12, 160, 12, ATECH_ACCENT);
}

template <typename Display>
inline void drawHeader(Display& d, const String& title) { drawHeader(d, title.c_str()); }

// ---------------------------------------------------------------------------
// drawValueRow — one labeled value with units, on a numbered row.
//
//   atech::drawValueRow(screen, "Temp",    23.4, "°C", 1);
//   atech::drawValueRow(screen, "Humidity", 51.0, "%",  2);
//
// Rows are 1-indexed and start under the header. Label is ATECH_MUTED,
// value is ATECH_ACCENT, units are ATECH_MUTED.
// ---------------------------------------------------------------------------

namespace _detail {
inline int rowY(int row) { return 18 + (row - 1) * 14; }
}  // namespace _detail

template <typename Display, typename Value>
inline void drawValueRow(Display& d, const char* label, Value value, const char* units, int row) {
    int y = _detail::rowY(row);
    d.setFont(NULL);
    d.setTextSize(1);
    d.setTextColor(ATECH_MUTED);
    d.setCursor(4, y);
    d.print(label);
    d.setTextColor(ATECH_ACCENT);
    d.setCursor(60, y);
    d.print(value);
    if (units && units[0]) {
        d.setTextColor(ATECH_MUTED);
        d.print(' ');
        d.print(units);
    }
}

// Overload for floats with 1 decimal place — the common dashboard case.
template <typename Display>
inline void drawValueRow(Display& d, const char* label, float value, const char* units, int row) {
    int y = _detail::rowY(row);
    d.setFont(NULL);
    d.setTextSize(1);
    d.setTextColor(ATECH_MUTED);
    d.setCursor(4, y);
    d.print(label);
    d.setTextColor(ATECH_ACCENT);
    d.setCursor(60, y);
    d.print(value, 1);
    if (units && units[0]) {
        d.setTextColor(ATECH_MUTED);
        d.print(' ');
        d.print(units);
    }
}

// ---------------------------------------------------------------------------
// drawProgressBar — horizontal bar showing pct in [0, 1].
//
//   atech::drawProgressBar(screen, 60, batteryPct);  // y=60, pct in [0,1]
//
// Full width minus 8px margin. 8px tall. ATECH_MUTED outline, ATECH_ACCENT fill.
// ---------------------------------------------------------------------------

template <typename Display>
inline void drawProgressBar(Display& d, int y, float pct, uint16_t fillColor = ATECH_ACCENT) {
    const int x = 4;
    const int w = 152;
    const int h = 8;
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 1.0f) pct = 1.0f;
    d.drawRect(x, y, w, h, ATECH_MUTED);
    int filled = (int)(pct * (w - 2));
    if (filled > 0) {
        d.fillRect(x + 1, y + 1, filled, h - 2, fillColor);
    }
}

// ---------------------------------------------------------------------------
// drawCenteredText — centered text on a given y, for titles, splashes, etc.
// Uses the default small font; for big text use setFont + drawCenteredText.
// ---------------------------------------------------------------------------

template <typename Display>
inline void drawCenteredText(Display& d, int y, const char* text, uint16_t color = ATECH_FG) {
    // Default font glyph is 6x8 pixels.
    int charCount = 0;
    for (const char* p = text; *p; ++p) ++charCount;
    int textWidth = charCount * 6;
    int x = (160 - textWidth) / 2;
    if (x < 0) x = 0;
    d.setFont(NULL);
    d.setTextSize(1);
    d.setTextColor(color);
    d.setCursor(x, y);
    d.print(text);
}

}  // namespace atech

using atech::renderEvery;
using atech::drawHeader;
using atech::drawValueRow;
using atech::drawProgressBar;
using atech::drawCenteredText;
