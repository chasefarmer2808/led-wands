#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel_ZeroDMA.h>

#define PIXEL_PIN 4
#define NUM_PIXELS 10
#define ACTIVATE_SPELL_PIN 3

Adafruit_NeoPixel_ZeroDMA pixels(NUM_PIXELS, PIXEL_PIN, NEO_GRBW);
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

const byte INTERRUPT_PIN = 1;
volatile byte state = LOW;

uint8_t maxAccel = 0;

void blink()
{
  state = !state;
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), blink, RISING);

  pinMode(ACTIVATE_SPELL_PIN, INPUT_PULLUP);

  Serial.begin(115200);

  pixels.begin();
  pixels.setBrightness(32);
  pixels.show();

  if (!lis.begin(0x18))
  { // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1)
      yield();
  }
  Serial.println("LIS3DH found!");

  // lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = ");
  Serial.print(2 << lis.getRange());
  Serial.println("G");
}

void loop()
{
  if (digitalRead(ACTIVATE_SPELL_PIN) == LOW)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    lis.read();
    Serial.print("X:  ");
    Serial.print(lis.x);
    Serial.print("  \tY:  ");
    Serial.print(lis.y);
    Serial.print("  \tZ:  ");
    Serial.print(lis.z);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    uint16_t i;
    // Rainbow cycle
    uint32_t elapsed, t, startTime = micros();
    for (;;)
    {
      t = micros();
      elapsed = t - startTime;
      if (elapsed > 5000000)
        break; // Run for 5 seconds
      uint32_t firstPixelHue = elapsed / 32;
      for (i = 0; i < pixels.numPixels(); i++)
      {
        uint32_t pixelHue = firstPixelHue + (i * 65536L / pixels.numPixels());
        pixels.setPixelColor(i, pixels.gamma32(pixels.ColorHSV(pixelHue)));
      }
      pixels.show();
    }
  }

  // digitalWrite(LED_BUILTIN, state);

  // lis.read(); // get X Y and Z data at once
  // // Then print out the raw data
  // Serial.print("X:  ");
  // Serial.print(lis.x);
  // Serial.print("  \tY:  ");
  // Serial.print(lis.y);
  // Serial.print("  \tZ:  ");
  // Serial.print(lis.z);

  // /* Or....get a new sensor event, normalized */
  // sensors_event_t event;
  // lis.getEvent(&event);

  // /* Display the results (acceleration is measured in m/s^2) */
  // Serial.print("\t\tX: ");
  // Serial.print(event.acceleration.x);
  // Serial.print(" \tY: ");
  // Serial.print(event.acceleration.y);
  // Serial.print(" \tZ: ");
  // Serial.print(event.acceleration.z);
  // Serial.println(" m/s^2 ");

  // Serial.println();

  // delay(200);
}