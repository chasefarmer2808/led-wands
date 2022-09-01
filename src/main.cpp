#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>

#define PIXEL_PIN 4
#define NUM_PIXELS 10
#define ACTIVATE_SPELL_PIN 3

Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_GRBW + NEO_KHZ800);
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

const uint32_t WHITE = pixels.Color(0, 0, 0, 255);
const uint32_t BLACK = pixels.Color(0, 0, 0, 0);

const byte INTERRUPT_PIN = 1;

float currAccel = 0;
uint8_t currPower = 0;
uint8_t maxPower = 0;
uint8_t currMaxPower = 0;
bool resetMax = false;
bool resetSpell = true;

float sampleAccel()
{
  sensors_event_t event;
  lis.getEvent(&event);
  return sqrt((event.acceleration.x * event.acceleration.x) + (event.acceleration.y * event.acceleration.y) + (event.acceleration.z * event.acceleration.z));
}

uint8_t accelToSpellPower(float accel)
{
  uint8_t power = 0;

  if (accel > 11 && accel <= 16)
  {
    power = 1;
  }
  else if (accel > 16 && accel <= 22)
  {
    power = 2;
  }
  else if (accel > 22)
  {
    power = 3;
  }

  return power;
}

void lumosSpell(uint8_t power)
{
  if (power == 0)
  {
    pixels.fill(BLACK);
    pixels.show();
    return;
  }

  if (!resetSpell)
  {
    return;
  }
  Serial.println(power);

  uint16_t i;
  for (i = 0; i < pixels.numPixels(); i++)
  {
    if (i > 0)
    {
      pixels.setPixelColor(i - 1, BLACK);
    }
    pixels.setPixelColor(i, WHITE);
    pixels.show();
    delay(20);
  }

  if (power >= 1)
  {
    pixels.setPixelColor(pixels.numPixels() - 1, WHITE);
  }

  if (power >= 2)
  {
    pixels.setPixelColor(pixels.numPixels() - 2, WHITE);
  }

  if (power >= 3)
  {
    pixels.setPixelColor(pixels.numPixels() - 3, WHITE);
  }

  pixels.show();
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), blink, RISING);

  pinMode(ACTIVATE_SPELL_PIN, INPUT_PULLUP);

  Serial.begin(115200);

  pixels.begin();
  pixels.setBrightness(32);
  pixels.show();
  pixels.fill(WHITE);
  pixels.show();
  delay(1000);

  if (!lis.begin(0x18))
  { // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1)
      yield();
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_4_G); // 2, 4, 8 or 16 G!

  Serial.print("Range = ");
  Serial.print(2 << lis.getRange());
  Serial.println("G");
}

void loop()
{
  if (digitalRead(ACTIVATE_SPELL_PIN) == LOW)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    resetSpell = true;

    if (resetMax)
    {
      maxPower = 0;
      resetMax = false;
    }

    currAccel = sampleAccel();
    currPower = accelToSpellPower(currAccel);

    if (currPower > maxPower)
    {
      maxPower = currPower;
    }

    Serial.println(currPower);
    delay(100);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);

    resetMax = true;
    currMaxPower = maxPower;

    if (currMaxPower == 0)
    {
      pixels.show();
    }

    lumosSpell(currMaxPower);
    resetSpell = false;
  }
}