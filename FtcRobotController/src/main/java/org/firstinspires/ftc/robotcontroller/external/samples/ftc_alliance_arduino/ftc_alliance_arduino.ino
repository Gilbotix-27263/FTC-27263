#include <Adafruit_NeoPixel.h>

#define LED_PIN 6       // NeoPixel data pin
#define LED_COUNT 24    // Number of LEDs in the strip
#define SERVO_PIN 9     // PWM signal from FTC

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
bool locked = false;  // Track if the color is locked
int lastR = 128, lastG = 0, lastB = 128;  // Default Purple

void setup() {
    pinMode(SERVO_PIN, INPUT);
    strip.begin();
    strip.show();  // Initialize LEDs

    // **Ensure Purple is the default on INIT**
    resetToPurple();
}

void loop() {
    int pwmValue = pulseIn(SERVO_PIN, HIGH, 25000); // Check for PWM signal

    // **If no PWM signal is detected, assume FTC program has stopped**
    if (pwmValue == 0) {
        resetToPurple();
        return;
    }

    if (!locked) { // Allow color selection before match starts
        if (pwmValue < 1300) {  
            lastR = 255; lastG = 0; lastB = 0;  // Red Team
        } else if (pwmValue > 1700) {  
            lastR = 0; lastG = 0; lastB = 255;  // Blue Team
        } else {  
            lastR = 128; lastG = 0; lastB = 128;  // Default Purple
        }

        // **Run smooth fading animation**
        smoothBrightnessAnimation(lastR, lastG, lastB);  
    }

    // **Lock the last color when match starts (PWM = 1500)**
    if (!locked && pwmValue == 1500) {  
        locked = true;
        setColor(lastR, lastG, lastB); // Lock the final color
    }
}

// 🚀 **Smooth Brightness Animation**
void smoothBrightnessAnimation(int r, int g, int b) {
    for (int i = 0; i < LED_COUNT; i++) {
        for (int j = 0; j < LED_COUNT; j++) {
            // **Calculate brightness based on position**
            float brightnessFactor = (float)((j + i) % 12) / 12.0;  // Smooth fade over 12 LEDs
            int dimR = r * brightnessFactor;
            int dimG = g * brightnessFactor;
            int dimB = b * brightnessFactor;

            strip.setPixelColor(j, strip.Color(dimR, dimG, dimB));  
        }
        strip.show();
        delay(50);  // Speed of the effect
    }
}

// 🔥 **Locks the LEDs in the final color**
void setColor(int r, int g, int b) {
    for (int i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, strip.Color(r, g, b));
    }
    strip.show();
}

// **🔄 Reset to Purple When FTC Program Stops**
void resetToPurple() {
    locked = false;
    lastR = 128; lastG = 0; lastB = 128; // Purple
    setColor(lastR, lastG, lastB);
}
