/**
 * Author: MathZtew
 * Licensed under MIT-license
 * 2021-07
 */

// Code for Arduino Nano
#include <LCD16x2.h>
#include <Wire.h>
#include <SPI.h>

// Controller functions, only stops execution data persists
#define SCREEN
#define WHEEL_SENSOR

// Button shorthands
#define button1 0x01
#define button2 0x02
#define button3 0x04
#define button4 0x08

// LCD object
LCD16x2 lcd;

// button status
int buttons;

// Counts the number of changes in the optical sensor
// Because of the restrictions of the counter, a max
// distance of about 35000 km is possible
uint32_t counter = 20350;

// Slits in the encoder wheel for the sensor
const int rot_slits = 64;
// Diameter of wheel to measure in mm
const int wheel_diam = 700;
// The circumference of the wheel
const float circ = 3.14 * (float) wheel_diam / 1000;

// The distance travelled in m
uint32_t dist_m;
// 100m part of the distance travelled, for one decimal precision print
uint8_t dist_hm;
// Distance travelled in km, used for easier display
uint16_t dist_km;

void setup() {
  // Start up setup

  Serial.begin(9600);

  #ifdef WHEEL_SENSOR
  // Attach the interrupt at D2, D3 is also available for interrupts
  // This is for the wheel counter, for calculating speed and distance
  attachInterrupt(digitalPinToInterrupt(2), count, RISING);
  #endif
  
  #ifdef SCREEN
  // Start i2c
  Wire.begin();
  // Reset LCD and print information
  reset_lcd();
  #endif
}

void loop() {
  // Main program loop

  #ifdef SCREEN
  // Read buttons from screen
  buttons = lcd.readButtons();
  
  // Print encoder wheel counter
  lcd.lcdGoToXY(7,1); 
  lcd.lcdWrite(counter);

  // Calculate the distance travelled
  calculate_distance_m();
  display_distance(7, 2);

  // Clear distance travelled 
  if (button_pressed(button1)) {  
    counter = 0;
    reset_lcd();
  }
  #endif
    
  delay(100);

}

bool button_pressed(int button) {
  return !(buttons & button);
}

void reset_lcd(){
  lcd.lcdClear();

  lcd.lcdGoToXY(1,1);
  lcd.lcdWrite("Slits:");

  lcd.lcdGoToXY(1,2);
  lcd.lcdWrite("Dist.:");
}

void count() {
  counter++;
}

/**
 * Send the distance counted to the screen, distance
 * is displayed in km with one decimal and unit with a comma as
 * the separator, minimally 5 character displayed.
 */
void display_distance(int x, int y) {
  lcd.lcdGoToXY(x, y);
  x += get_int_len(dist_km);
  lcd.lcdWrite(dist_km);
  lcd.lcdGoToXY(x, y);
  lcd.lcdWrite(",");
  x += 1;
  lcd.lcdGoToXY(x, y);
  lcd.lcdWrite(dist_hm);
  x += get_int_len(dist_hm);
  lcd.lcdGoToXY(x, y);
  lcd.lcdWrite("km");
}

uint32_t calculate_distance_m() {
  float res = (float) counter / (float) rot_slits;
  res = res * circ;
  dist_m = (long) res;
  calculate_distance_km();
  calculate_distance_hm();
  return (long) res;
}

uint16_t calculate_distance_km() {
  dist_km = dist_m / 1000;
  return dist_km;
}

uint8_t calculate_distance_hm() {
  dist_hm = (dist_m / 100) % 10;
  return dist_hm;
}

uint8_t get_int_len(int num) {
  int i = 0;
  int copy = num;
  do {
    i++;
    copy = copy / 10;
  } while (copy != 0);
  return i;
}
