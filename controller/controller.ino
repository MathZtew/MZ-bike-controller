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

// Digital pin for the relay that controls the power switch of the motor controller
#define RELAY_PIN 5
// Digital pin for the dead man switch
#define DEAD_MAN_SWITCH_PIN 8
#define POT_UP_PIN 7
#define POT_DOWN_PIN 6
// Number of delta cycles that need to pass before updating the speed value
#define SPEED_UPDATE_CYCLE 5
// Milliseconds to check pedal changes
#define PEDAL_DELTA_MIN 500
// Number of pedal changes that is enough to signal pedalling
#define PEDAL_CHANGE_THRESHOLD 3
// Value speed_update is set to to significate a zero speed state
#define SPEED_ZERO_VALUE 200
// Maximum speed in km/h
#define MAX_SPEED 25

// Potentiometer start and stop values
#define MIN_POT_VALUE 90
#define MAX_POT_VALUE 235
#define POT_CHANGE 5

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
uint32_t last_wheel_time = 0;
uint32_t old_counter = counter - 1;
float old_speed = 1;

// Values for the counting of pedalling
uint32_t pedal_change_time = 0;
// Counter for the changes in the pedal
uint8_t pedal_counter = 0;
// Mode for the sensors
bool sensor_mode = false;
// Records whether the pot has changed
bool change = true;

#ifdef DIG_POT
// The byte that is to be sent to the digital potentiometer
uint8_t pot_value = 0;
#endif

#ifdef SCREEN
// The circumference of the wheel
const float circ = 3.14 * (float) wheel_diam / 1000;
// Speed in km/h
float speed = 0;
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

uint16_t old_battery;
#endif

// Statuses for the speed values for the potentiometer
bool relay = false;
bool dead_man_switch = false;
bool speedo_limiter = false;
uint32_t voltage_wait = 0;

/**
 * Runs on start-up
 */
void setup() {
  // Serial debug
  Serial.begin(9600);

  #ifdef WHEEL_SENSOR
  // Attach the interrupt at D2,
  // This is for the wheel counter, for calculating speed and distance
  attachInterrupt(digitalPinToInterrupt(2), wheel_sensor, FALLING);
  // Attach interrupt for the pedal sensor
  attachInterrupt(digitalPinToInterrupt(3), pedal_change, CHANGE);
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
  // Pins for handle controls
  pinMode(DEAD_MAN_SWITCH_PIN, INPUT);
  pinMode(POT_UP_PIN, INPUT);
  pinMode(POT_DOWN_PIN, INPUT);
}

/**
 * Main program loop
 */
void loop() {
  #ifdef DIG_POT
  if (read_pedal(true) && (!digitalRead(DEAD_MAN_SWITCH_PIN))) {
    // Speed limiter
    if (limit_speed() && !speedo_limiter) {
      // Cut motor power
      write_pot(0);
      // Lower potentiometer value to not exceed speed next time
      decrement_pot();
      // Dead man switch is pressed
      dead_man_switch = true;
      // Speed limiter is engaged
      speedo_limiter = true;
    }
    else if (limit_speed()){
      // Here we have already set the speed to 0
    }
    // Check whether this is the first time the dead man switch was
    // pressed or the pot value was changed
    else if (!dead_man_switch || change) {
      write_pot(pot_value);
      // We have now taken care of the potentiometer change
      change = false;
      // dead man switch is now pressed
      dead_man_switch = true;
      // Speed limiter is not engaged
      speedo_limiter = false;
    }
    else {
      change = false;
    }
  }
  else {
    // Check whether the dead man switch was pressed previously
    if (dead_man_switch) {
      write_pot(0);
      dead_man_switch = false;
      speedo_limiter = false;
    }
  }
  #endif

  #ifdef SCREEN
  // Read buttons from screen
  uint8_t buttons = readButtons();
  // Switch mode for the screen
  switch (mode) {
    case SPEED_MODE:
    // Only update the screen every n'th update cycle
    if (old_speed != speed && speed_upd >= SPEED_UPDATE_CYCLE) {
      // Print speed value
      lcdGoToXY(1,1);
      lcdWrite(speed, 1, 5);
      old_speed = speed;
      speed_upd = 0;
    }
    else if (last_wheel_time + 8000 < millis() && speed_upd != SPEED_ZERO_VALUE) {
      // Print zero speed value
      lcdGoToXY(1,1);
      speed = 0;
      lcdWrite(speed, 1, 5);
      old_speed = speed;
      speed_upd = SPEED_ZERO_VALUE;
    }
    else if (speed_upd < SPEED_UPDATE_CYCLE) {
      speed_upd++;
    }
    
    if (voltage_wait + 2000 <= millis()) {
      uint16_t voltage = get_battery_voltage();
      if (old_battery != voltage) {
        write_battery_voltage(12, 2, voltage);
        old_battery = voltage;
      }
      voltage_wait = millis();
    }
    
    // If value has not changed, don't update screen
    if (display_value_changed(dist_m, old_dist_m)) {
      // Calculate the distance travelled and display it
      display_distance(1, 2, dist_m);
      old_dist_m = dist_m;
    }

    //**************************************************
    // Button routines
    //**************************************************
    
    // Clear distance travelled 
    if (button_pressed(buttons, button1)) {  
      counter = 0;
      old_counter = ~counter;
      old_dist_m = ~old_dist_m;
      old_speed = 0.1;
      reset_lcd();
    }
    // Open/close the relay for the motor controller
    if (button_pressed(buttons, button2)){
      relay = !relay;
      digitalWrite(RELAY_PIN, relay);
      write_disp_relay();
    }
    // Enable/disable screen backlight
    if (button_pressed(buttons, button3)) {
      light = light == 0x00 ? 0xFF : 0x00;
      lcdSetBlacklight(light);
    }
    // Set the sensor mode
    if (button_pressed(buttons, button3) && digitalRead(POT_UP_PIN) && digitalRead(POT_DOWN_PIN)) {
      sensor_mode = true;
      lcdGoToXY(6, 1);
      lcdWrite("K");
    }
    // Decrease the value sent to the potentiometer
    if (digitalRead(POT_DOWN_PIN)) {
      decrement_pot();
    }
    // Increase the value sent to the potentiometer
    if (digitalRead(POT_UP_PIN)) {
      increment_pot();
    }
    // Check whether to update the screen
    if (change) {
      write_disp_pot_value(15, 2);
    }
    break;
  case POT_MODE:
    if (button_pressed(buttons, button1)) {
      pot_value = 0;
      change = true;
    }
    if (button_pressed(buttons, button2) || digitalRead(POT_DOWN_PIN)) {
      decrement_pot();
    }
    if (button_pressed(buttons, button3) || digitalRead(POT_UP_PIN)) {
      increment_pot();
    }
    // Check whether to update the screen
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
 * Writes the potentiometer value to the screen, adjusted to
 * increments and not the actual value sent to the pot.
 */
void write_disp_pot_value(uint8_t x, uint8_t y) {
  lcdGoToXY(x, y);
  if (pot_value == 0) {
    lcdWrite(0);
  }
  else {
    lcdWrite((pot_value - MIN_POT_VALUE) / POT_CHANGE);
  }
  if (pot_value < MIN_POT_VALUE + POT_CHANGE * 10) {
    lcdWrite(" ");
  }
}

void write_battery_voltage(int x, int y, float voltage) {
  lcdGoToXY(x, y);
  lcdWrite(voltage);//, 1, 4);
}

uint16_t get_battery_voltage() {
  uint16_t voltage = analogRead(SENSORPIN) >> 4;
  return voltage;//((float)50 / (float)256) * voltage;
}

/**
 * Writes the relay value to the screen.
 */
void write_disp_relay(){
  if (relay) {
    lcdGoToXY(14, 1);
    lcdWrite(" on");
  }
  else {
    lcdGoToXY(14, 1);
    lcdWrite("off");
  }
}

/**
 * Increment the potentiometer value by POT_CHANGE.
 */
void increment_pot() {
  if (pot_value >= 0 && pot_value <= 85) {
    // Jump to a value where the motor starts working
    pot_value = 90;
  }
  else {
    pot_value = pot_value + POT_CHANGE > 235 ? 235 : pot_value + POT_CHANGE;
  }
  change = true;
}

/**
 * Decrement the potentiometer value by POT_CHANGE.
 */
void decrement_pot(){
  if (pot_value >= 0 && pot_value <= 90) {
    // Jump to 0
    pot_value = 0;
  }
  else {
    pot_value = pot_value - POT_CHANGE < 0 ? 0 : pot_value - POT_CHANGE;
  }
  change = true;
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
 * Convenience function:
 * Check the status of one of the buttons on the LCD.
 */
bool button_pressed(uint8_t buttons, uint8_t button) {
  return !(buttons & button);
}

/**
 * Reset the LCD to the start-up behaviour.
 * 
 * *----------------*
 * *NN.NNkm/h    sss*
 * *DDD.D  km     pp*
 * *----------------*
 * 
 * s = potentiometer status
 * p = potentiometer value
 * N = speed in km/h
 * D = distance travelled in km
 */
void reset_lcd(){
  lcdClear();
  write_disp_pot_value(15, 2);
  write_disp_relay();
  lcdGoToXY(6, 1);
  lcdWrite("km/h");
  lcdGoToXY(10, 2);
  lcdWrite("B:");
}

/**
 * Resets the LCD to the potentiometer mode.
 * 
 * *----------------*
 * *pot value:      *
 * *PPP             *
 * *----------------*
 * 
 * P = actual potentiometer value in decimal
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
 * Interrup routine:
 * Increase the counter for the wheel sensor
 * and calculate speed
 */
void wheel_sensor() {
  counter++;
  float seconds = millis() - last_wheel_time;
  seconds = seconds * 0.001;
  speed = circ / seconds;
  speed *= 3.6;
  last_wheel_time = millis();
  dist_m = calculate_distance_m(circ, counter);
}

/**
 * Interrupt routine for the pedal sensor, sets the time that
 * the pedals were activated.
 */
void pedal_change() {
  if (++pedal_counter >= PEDAL_CHANGE_THRESHOLD) {
    pedal_change_time = millis();
    pedal_counter = 0;
  }
}

/**
 * Checks whether the pedals have been activated in the sufficient
 * time period to engage the motor.
 */
bool read_pedal(bool reset) {
  if (sensor_mode) {
    return true;
  }
  return pedal_change_time + PEDAL_DELTA_MIN >= millis();
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
  lcdGoToXY(6, y);
  lcdWrite("km");
}

/**
 * Calculate the distance travelled in meters based on the sensor counter.
 */
uint32_t calculate_distance_m(float circ, uint32_t counter) {
  float res = (float) counter * circ;
  return (uint32_t) res;
}

/**
 * Calculate the distance travelled in meters based on the sensor counter.
 */
float calculate_distance_m_float(float circ, uint32_t counter) {
  float res = (float) counter * circ;
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

/**
 * returns whether the controller should limit the speed.
 */
bool limit_speed() {
  return speed > MAX_SPEED && !sensor_mode;
}