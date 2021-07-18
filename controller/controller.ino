/**
 * Author: MathZtew
 * Licensed under MIT-license
 * 2021-07
 */

// Code for Arduino Nano
//#include <LCD16x2.h>
#include <Wire.h>
#include <SPI.h>
#include "lcd.h"

// Controller functions, only stops execution, data persists
#define SCREEN // Screen output and speed calculations
#define WHEEL_SENSOR // Sensor to count wheel rotations
#define DIG_POT // Write values to the digital potentiometer

// Button shorthands
#define button1 0x01
#define button2 0x02
#define button3 0x04
#define button4 0x08

// Screen modes
#define SPEED_MODE 0
#define POT_MODE 1

// SPI constants for digital potentiometer
// Digital pot MCP41XXX
#define CHIP_SELECT 10
#define POT_WRITE_COMMAND 0x13
#define SENSORPIN A0
#define POT_CHANGE 5

// Digital pin for the relay that controls the power switch of the motor controller
#define RELAY_PIN 5
// Number of delta cycles that need to pass before updating the speed value
#define SPEED_UPDATE_CYCLE 5

// Slits in the encoder wheel for the sensor
#define rot_slits 64
// Diameter of wheel to measure in mm
#define wheel_diam 700
// Time period for the loop
#define period 100

// Counts the number of changes in the optical sensor.
// Because of the restrictions of the counter, a max
// distance of about 35000 km is possible
uint32_t counter = 1;
uint32_t old_counter = counter - 1;
float old_speed = 1;

bool change = true;

#ifdef DIG_POT
// The byte that is to be sent to the digital potentiometer
uint8_t pot_value = 0;
#endif

#ifdef SCREEN
// The circumference of the wheel
const float circ = 3.14 * (float) wheel_diam / 1000;
// Screen mode
uint8_t mode = 0;
// Light value of the LCD, 256 values possible
uint8_t light = 0;
// Last time the delta cycle started
uint32_t old_time;
// Distance travelled in meters
uint32_t dist_m;
// Set the old distance so that the screen updates when first turned on
uint32_t old_dist_m = ~dist_m;
// Set the update cycles number so that it updates on first startup
uint8_t speed_upd = SPEED_UPDATE_CYCLE;
#endif

bool relay = false;

/**
 * Runs on start-up
 */
void setup() {
  // Serial debug
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
  lcdSetBlacklight(light);
  // First time delta start
  old_time = millis();
  #endif

  #ifdef DIG_POT
  // Setup the SPI for the digital potentiometer
  pinMode(CHIP_SELECT, OUTPUT);
  digitalWrite(CHIP_SELECT, HIGH);
  SPI.begin();
  write_pot(pot_value);
  #endif

  // Relay for controlling power state of the motor controller
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, relay);
}

/**
 * Main program loop
 */
void loop() {
  #ifdef DIG_POT
  // 10-bit value conversion to 8-bit
  //pot_value = analogRead(SENSORPIN) >> 2;
  write_pot(pot_value);
  change = false;
  #endif

  #ifdef SCREEN
  // Get the current time and distance
  uint32_t new_time = millis();
  // Compare them to the old time and distance and calculate the speed
  float seconds = (new_time - old_time) * 0.001;

  // testing
  counter = counter + rot_slits * seconds * 5;

  // Calculate distance travelled within time delta
  float dist_m_delta = calculate_distance_m_float(circ, counter - old_counter);
  // Calculate the average speed within the time delta
  float speed = (float) (dist_m_delta) / seconds;
  // Conversion from m/s to km/h
  speed = speed * 3.6;
  // Set the new time and distance as the old ones
  dist_m = calculate_distance_m(circ, counter);
  old_time = new_time;
  // Update the old counter with the current value
  old_counter = counter;
  // Update the speed display update variable
  speed_upd++;

  // Read buttons from screen
  uint8_t buttons = readButtons();
  // Switch mode for the screen
  switch (mode) {
    case SPEED_MODE:
    // Only update the screen every n'th update cycle
    if (old_speed != speed && speed_upd >= SPEED_UPDATE_CYCLE) {
      // Print encoder wheel counter
      lcdGoToXY(7,1);
      lcdWrite(speed, 1, 7);
      lcdGoToXY(13, 1);
      lcdWrite("km/h");
      old_speed = speed;
      speed_upd = 0;
    }
    
    // If value has not changed, don't update screen
    if (display_value_changed(dist_m, old_dist_m)) {
      // Calculate the distance travelled and display it
      display_distance(7, 2, dist_m);
      old_dist_m = dist_m;
    }

    // Clear distance travelled 
    if (button_pressed(buttons, button1)) {  
      counter = 0;
      old_counter = ~counter;
      old_speed = 0.1;
      reset_lcd();
    }
    // Open/close the relay for the motor controller
    if (button_pressed(buttons, button2)){
      relay = !relay;
      digitalWrite(RELAY_PIN, relay);
      if (relay) {
        lcdGoToXY(16, 2);
        lcdWrite("*");
      }
      else {
        lcdGoToXY(16, 2);
        lcdWrite(" ");
      }
    }
    // Enable/disable screen backlight
    if (button_pressed(buttons, button3)) {
      light = light == 0x00 ? 0xFF : 0x00;
      lcdSetBlacklight(light);
    }
    break;
  case POT_MODE:
    change = false;
    if (button_pressed(buttons, button1)) {
      pot_value = 0;
      change = true;
    }
    if (button_pressed(buttons, button2)) {
      pot_value = pot_value - POT_CHANGE < 0 ? 0 : pot_value - POT_CHANGE;
      change = true;
    }
    if (button_pressed(buttons, button3)) {
      pot_value = pot_value + POT_CHANGE > 255 ? 255 : pot_value + POT_CHANGE;
      change = true;
    }
    if (change) {
      lcdGoToXY(1,2);
      lcdWrite(pot_value);
      lcdWrite("  ");
    }
    break;
  }
  
  // change screen modes
  if (button_pressed(buttons, button4)) {
    mode = mode == 0 ? 1 : 0;
    change_screen_mode(mode);
  }

  // The minimum period for the calculations
  while (millis() < old_time + period) {    
  }

  #endif
}

/**
 * Changes the screen behaviour to the correct mode.
 */
void change_screen_mode(uint8_t mode) {
  switch (mode)
  {
  case SPEED_MODE:
    // Update screen values with old values
    old_speed = 0.1;
    old_dist_m = ~old_dist_m;
    reset_lcd();
    break;
  case POT_MODE:
    lcd_pot();
    break;
  
  default:
    break;
  }
}

/**
 * Convenience function
 * Check the status of the buttons on the LCD.
 */
bool button_pressed(uint8_t buttons, uint8_t button) {
  return !(buttons & button);
}

/**
 * Reset the LCD to the start-up behaviour
 */
void reset_lcd(){
  lcdClear();

  lcdGoToXY(1,1);
  lcdWrite("Speed:");

  lcdGoToXY(1,2);
  lcdWrite("Dist.:");
}

/**
 * Resets the LCD to the potentiometer mode.
 */
void lcd_pot() {
  lcdClear();

  lcdGoToXY(1,1);
  lcdWrite("pot value:");

  lcdGoToXY(1,2);
  lcdWrite(pot_value);
  lcdWrite("  ");
}

/**
 * Increase the counter for the wheel sensor
 */
void count() {
  counter++;
}

/**
 * Send the distance counted to the screen, distance
 * is displayed in km with one decimal and unit, with a comma as
 * the separator, minimally 5 characters displayed.
 */
void display_distance(uint8_t x, uint8_t y, uint32_t dist_m) {
  lcdGoToXY(x, y);
  uint16_t dist_km = calculate_distance_km(dist_m);
  lcdWrite(dist_km);
  lcdWrite(".");
  uint8_t dist_hm = calculate_distance_hm(dist_m);
  lcdWrite(dist_hm);
  lcdGoToXY(13, y);
  lcdWrite("km");
}

/**
 * Calculate the distance travelled in meters based on the sensor counter.
 */
uint32_t calculate_distance_m(float circ, uint32_t counter) {
  float res = (float) counter / (float) rot_slits;
  res *= circ;
  return (uint32_t) res;
}

/**
 * Calculate the distance travelled in meters based on the sensor counter.
 */
float calculate_distance_m_float(float circ, uint32_t counter) {
  float res = (float) counter / (float) rot_slits;
  res *= circ;
  return res;
}

/**
 * Distance travelled in km, used for easier displayability.
 */
uint16_t calculate_distance_km(uint32_t dist_m) {
  return dist_m / 1000;
}

/**
 * 100m part of the distance travelled, for one decimal precision print.
 */
uint8_t calculate_distance_hm(uint32_t dist_m) {
  return (dist_m / 100) % 10;
}

/**
 * Calculates the length of the integer, 
 * used to calculate display offsets.
 */
uint8_t get_int_len(int num) {
  int i = 0;
  int copy = num;
  do {
    i++;
    copy = copy / 10;
  } while (copy != 0);
  return i;
}

/**
 * Check if the distance travelled has changed significantly enough
 * to update the screen.
 */
bool display_value_changed(uint32_t dist_m, uint32_t old_dist_m) {
  return calculate_distance_km(dist_m) != calculate_distance_km(old_dist_m) ||
         calculate_distance_hm(dist_m) != calculate_distance_hm(old_dist_m);
}

/**
 * Write a value to the digital potentiometer.
 */
void write_pot(uint8_t value) {
  digitalWrite(CHIP_SELECT, LOW);
  // 4 MHz clock, MSB first, mode 0
  SPI.beginTransaction (SPISettings (4000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(POT_WRITE_COMMAND);
  SPI.transfer(value);
  SPI.endTransaction();
  digitalWrite(CHIP_SELECT, HIGH);
}