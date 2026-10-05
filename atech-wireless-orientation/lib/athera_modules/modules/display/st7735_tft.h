/**
 * @file st7735_tft.h
 * @brief ST7735 160x80 TFT Color Display for Athera
 *
 * Color TFT display using the Adafruit ST7735 library over SPI.
 * Double module spanning two adjacent ports (4 SPI pins).
 *
 * Specifications:
 * - Resolution: 160x80 pixels
 * - Color: RGB565 (16-bit, 65K colors)
 * - Interface: SPI (40MHz)
 * - Controller: ST7735
 *
 * Athera Connector (double module):
 * - Port 1 Line A: DC (data/command)
 * - Port 1 Line B: CS (chip select)
 * - Port 2 Line A: MOSI (SPI data)
 * - Port 2 Line B: SCLK (SPI clock)
 */

#ifndef ST7735_TFT_MODULE_H
#define ST7735_TFT_MODULE_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

class ST7735_TFT {
public:
    // Color constants (RGB565)
    static constexpr uint16_t COLOR_BLACK   = 0x0000;
    static constexpr uint16_t COLOR_WHITE   = 0xFFFF;
    static constexpr uint16_t COLOR_RED     = 0xF800;
    static constexpr uint16_t COLOR_GREEN   = 0x07E0;
    static constexpr uint16_t COLOR_BLUE    = 0x001F;
    static constexpr uint16_t COLOR_CYAN    = 0x07FF;
    static constexpr uint16_t COLOR_MAGENTA = 0xF81F;
    static constexpr uint16_t COLOR_YELLOW  = 0xFFE0;
    static constexpr uint16_t COLOR_ORANGE  = 0xFD20;

    /**
     * @brief Construct ST7735 display
     * @param sclkPin SPI clock pin (Line A of primary port)
     * @param csPin Chip select pin (Line B of primary port)
     * @param mosiPin SPI data pin (Line A of secondary port)
     * @param dcPin Data/command pin (Line B of secondary port)
     */
    ST7735_TFT(int sclkPin, int csPin, int mosiPin, int dcPin);

    /** @brief Initialize display (160x80, landscape, 40MHz SPI) */
    void begin();

    /** @brief Clear display to black */
    void clear();

    /** @brief Fill entire screen with a color */
    void fillScreen(uint16_t color);

    /** @brief Draw a single pixel */
    void drawPixel(int16_t x, int16_t y, uint16_t color);

    /** @brief Draw a line between two points */
    void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);

    /** @brief Draw a rectangle outline */
    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

    /** @brief Draw a filled rectangle */
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

    /** @brief Draw a circle outline */
    void drawCircle(int16_t x, int16_t y, int16_t r, uint16_t color);

    /** @brief Draw a filled circle */
    void fillCircle(int16_t x, int16_t y, int16_t r, uint16_t color);

    /** @brief Draw a rounded-rectangle outline (radius is corner radius in pixels) */
    void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t radius, uint16_t color);

    /** @brief Draw a filled rounded rectangle */
    void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t radius, uint16_t color);

    /** @brief Draw a triangle outline through three points */
    void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);

    /** @brief Draw a filled triangle through three points */
    void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);

    /**
     * @brief Draw an ellipse outline
     * @param x Center X
     * @param y Center Y
     * @param rx Horizontal radius (pixels)
     * @param ry Vertical radius (pixels)
     */
    void drawEllipse(int16_t x, int16_t y, int16_t rx, int16_t ry, uint16_t color);

    /** @brief Draw a filled ellipse (see drawEllipse for parameters) */
    void fillEllipse(int16_t x, int16_t y, int16_t rx, int16_t ry, uint16_t color);

    /**
     * @brief Enable/disable direct drawing mode
     * @param direct true = draw directly to display (fast partial updates, for games/animations),
     *               false = draw to off-screen canvas, call display() to flush (flicker-free)
     *
     * Direct mode: each draw call goes straight to the TFT. Only changed pixels are sent.
     *              Best for games/animations with partial redraws. Do NOT call display().
     * Canvas mode (default): all draws go to RAM buffer, display() pushes entire frame.
     *              Best for static UIs or full-screen redraws. Flicker-free.
     */
    void setDirectMode(bool direct);

    /** @brief Print at current cursor position (accepts any type) */
    template<typename T>
    void print(T value) { _gfx()->print(value); }
    void print(float value, int decimals);
    void print(double value, int decimals);

    /** @brief Print with newline (accepts any type) */
    template<typename T>
    void println(T value) { _gfx()->println(value); }
    void println(float value, int decimals);
    void println(double value, int decimals);

    /**
     * @brief Display text at position with size and color (convenience method)
     * @param text Text to display
     * @param x X position
     * @param y Y position
     * @param size Text size multiplier (1-4)
     * @param color Text color (RGB565)
     */
    void displayText(const char* text, int16_t x, int16_t y, uint8_t size, uint16_t color);
    void displayText(const String& text, int16_t x, int16_t y, uint8_t size, uint16_t color);

    /** @brief Flush canvas buffer to display (call after drawing a frame) */
    void display();

    /** @brief Set text cursor position */
    void setCursor(int16_t x, int16_t y);

    /** @brief Set text color */
    void setTextColor(uint16_t color);

    /** @brief Set text color with background */
    void setTextColor(uint16_t color, uint16_t bg);

    /** @brief Set text size multiplier (1-4) */
    void setTextSize(uint8_t size);

    /**
     * @brief Set font for smooth text rendering
     * @param font Pointer to GFXfont (nullptr = default bitmap font)
     *
     * Available fonts (from Adafruit GFX):
     *   FreeSans9pt7b, FreeSans12pt7b, FreeSansBold9pt7b, FreeSansBold12pt7b,
     *   FreeSansBold18pt7b, FreeSansBold24pt7b,
     *   FreeSerif9pt7b, FreeSerif12pt7b,
     *   FreeMono9pt7b, FreeMono12pt7b
     *
     * NOTE: With custom fonts, setTextSize(1) is the native size.
     *       Y coordinate is the text BASELINE (not top-left).
     */
    void setFont(const GFXfont* font);

    /** @brief Set display rotation (0-3) */
    void setRotation(uint8_t r);

    /** @brief Get display width in pixels */
    int16_t width();

    /** @brief Get display height in pixels */
    int16_t height();

private:
    int _sclkPin, _csPin, _mosiPin, _dcPin;
    Adafruit_ST7735* _tft;
    GFXcanvas16* _canvas;       // Off-screen framebuffer for flicker-free drawing
    bool _directMode;           // true = draw to TFT directly, false = draw to canvas

    // Hardware-SPI peripherals are auto-assigned to displays in the order they
    // call begin(): 1st → default SPI (FSPI on ESP32-S3, VSPI on ESP32 classic),
    // 2nd → HSPI. 3rd+ displays fall back to software SPI (slower full-frame
    // push, but no bus conflict). HW SPI is ~5–8× faster than the previous
    // software-SPI-only path — full 160×80 frame at 40 MHz ≈ 6 ms vs ~40 ms.
    static int _nextHwSpiBus;
    static SPIClass _hspi;

    /** @brief Returns the active drawing target (canvas or TFT) */
    Adafruit_GFX* _gfx() { return _directMode ? (Adafruit_GFX*)_tft : (Adafruit_GFX*)_canvas; }

    void _initDisplay();
    void _drawSplash();
};

#endif // ST7735_TFT_MODULE_H
