#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include "colors.h"

#define PIXEL_PIN 4
#define NUM_PIXELS 20
#define ACTIVATE_SPELL_PIN 3
#define CHANGE_SPELL_PIN 1

enum Spell
{
  LUMOS,
  GREEN_FIRE,
  RED_FIRE,
  ZAP
};

int colors[] = {BLUE, GREEN, TEAL, LIME, SPRINGGREEN, CYAN, INDIGO, MAROON, OLIVE, YELLOWGREEN, PLUM, SALMON, DEEPPINK, FUCHSIA, YELLOW};

Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_GRBW + NEO_KHZ800);
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

const uint8_t BRIGHTNESS = 80;

const int8_t GREEN_FIRE_R = 74;
const int8_t GREEN_FIRE_G = 150;
const int8_t GREEN_FIRE_B = 12;
const int8_t RED_FIRE_R = 226;
const int8_t RED_FIRE_G = 121;
const int8_t RED_FIRE_B = 35;

float currAccel = 0;
uint8_t currPower = 0;
uint8_t maxPower = 0;
uint8_t currMaxPower = 0;
uint8_t currSpell = LUMOS;
uint8_t zapColorIndex = 0;
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

  // Use this condition when we want the spell to only animate once on every button press.
  if (!resetSpell)
  {
    return;
  }

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

void fireSpell(uint8_t power, uint8_t fireR, uint8_t fireG, uint8_t fireB)
{
  if (power == 0)
  {
    pixels.fill(BLACK);
    pixels.show();
    return;
  }

  for (uint16_t i = 0; i < pixels.numPixels(); i++)
  {
    int flicker = random(0, 55);
    int r1 = fireR - flicker;
    int g1 = fireG - flicker;
    int b1 = fireB - flicker;
    if (g1 < 0)
      g1 = 0;
    if (r1 < 0)
      r1 = 0;
    if (b1 < 0)
      b1 = 0;
    pixels.setPixelColor(i, pixels.Color(g1, r1, b1, 0));
  }
  pixels.show();
  delay(random(10, 113));
}

void zapSpell(uint8_t power)
{
  if (power == 0)
  {
    pixels.fill(BLACK);
    pixels.show();
    return;
  }

  // Use this condition when we want the spell to only animate once on every button press.
  if (!resetSpell)
  {
    return;
  }

  uint8_t i;
  for (i = 0; i < pixels.numPixels(); i++)
  {
    if (i > 0)
    {
      pixels.setPixelColor(i - 1, BLACK);
    }
    pixels.setPixelColor(i, colors[zapColorIndex]);
    pixels.show();
    delay(20);
  }

  zapColorIndex++;
  zapColorIndex %= sizeof(colors);

  delay(750);
  pixels.fill(BLACK);
  pixels.show();
}

void changeSpell()
{
  currSpell++;
  currSpell %= 4;
  maxPower = 0; // Prevents the next spell from animating automatically.
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(ACTIVATE_SPELL_PIN, INPUT_PULLUP);
  pinMode(CHANGE_SPELL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CHANGE_SPELL_PIN), changeSpell, RISING);

  Serial.begin(115200);

  pixels.begin();
  pixels.setBrightness(BRIGHTNESS);
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

    switch (currSpell)
    {
    case LUMOS:
      lumosSpell(currMaxPower);
      break;
    case GREEN_FIRE:
      fireSpell(currMaxPower, GREEN_FIRE_R, GREEN_FIRE_G, GREEN_FIRE_B);
      break;
    case RED_FIRE:
      fireSpell(currMaxPower, RED_FIRE_R, RED_FIRE_G, RED_FIRE_B);
      break;
    case ZAP:
      zapSpell(currMaxPower);
      break;
    }
    resetSpell = false;
  }
}