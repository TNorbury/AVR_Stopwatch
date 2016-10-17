//*****************************************************************
//*
//* Name : Tyler Norbury
//* Program : Program 6
//* Class : ENGE 320
//* Date : 10/18/16
//* Description : Stopwatch that can hold up to 3 lap times, and display any of
//*               the lap times, or the current time, based on user selection.
//*
//* =============================================================
//* Program Grading Criteria
//* =============================================================
//* Stopwatch displays on the seven seg: (10) ____
//* Display doesn't ghost: (10) ____
//* Stopwatch timer operates at the correct speed: (10) ____
//* Start/Stop button works: (10) ____
//* Stop time displayed for 2 seconds (if display set to a lap time): (10) ____
//* Lap button works: (10) ____
//* Lap time displayed for 2 seconds: (10) ____
//* Potentiometer controls the Lap time display: (10) ____
//* Lap times queue properly: (10) ____
//* LEDs indicate the Lap time: (10) ___
//* LEDs fade properly at 250ms: (20) ____
//* Switch turns off the leds: (10) ____
//* Serial port functions properly at 1MHz: (10) ____
//* Serial start prints properly (10) ____
//* Serial lap prints properly (10) ____
//* Serial stop prints properly (10) ____
//* Pressing "s" performs a start/stop: (10) ____
//* Pressing "l" performs a lap: (10) ____
//* Messages don't cause system delay 1MHz: (10) ____
//* =============================================================
//*
//* TOTAL (200)_________
//*
//* =============================================================
//*****************************************************************


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "adc.h"
#include "led.h"
#include "pwm.h"
#include "timer.h"
#include "tlc.h"
#include "buttons.h"
#include "serial.h"
#include "switch.h"

//-----------------------------------------------------------------------------
//      __   ___  ___         ___  __
//     |  \ |__  |__  | |\ | |__  /__`
//     |__/ |___ |    | | \| |___ .__/
//
//-----------------------------------------------------------------------------

#define RED_FADE (0)
#define YELLOW_FADE (1)
#define GREEN_FADE (2)
#define CYAN_FADE (3)

#define DIGIT0_ANODE (1)
#define DIGIT1_ANODE (2)
#define DIGIT2_ANODE (3)
#define DIGIT3_ANODE (4)

#define STARTSTOP ('s')
#define LAP ('l')

//-----------------------------------------------------------------------------
//     ___      __   ___  __   ___  ___  __
//      |  \ / |__) |__  |  \ |__  |__  /__`
//      |   |  |    |___ |__/ |___ |    .__/
//
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//                __          __        ___  __
//     \  /  /\  |__) |  /\  |__) |    |__  /__`
//      \/  /~~\ |  \ | /~~\ |__) |___ |___ .__/
//
//-----------------------------------------------------------------------------
const uint8_t SEVEN_SEG[10] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d,
0x7d, 0x07, 0x7f, 0x6f};

const uint8_t FADE_SPECTRUM[250] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
  0x02, 0x03, 0x03, 0x03, 0x03, 0x03, 0x04, 0x04, 0x04, 0x04, 0x05, 0x05, 0x05,
  0x06, 0x06, 0x07, 0x07, 0x08, 0x08, 0x08, 0x09, 0x09, 0x0A, 0x0A, 0x0B, 0x0B,
  0x0C, 0x0C, 0x0D, 0x0D, 0x0E, 0x0F, 0x0F, 0x10, 0x11, 0x11, 0x12, 0x13, 0x14,
  0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1F, 0x20, 0x21, 0x23,
  0x24, 0x26, 0x27, 0x29, 0x2B, 0x2C, 0x2E, 0x30, 0x32, 0x34, 0x36, 0x38, 0x3A,
  0x3C, 0x3E, 0x40, 0x43, 0x45, 0x47, 0x4A, 0x4C, 0x4F, 0x51, 0x54, 0x57, 0x59,
  0x5C, 0x5F, 0x62, 0x64, 0x67, 0x6A, 0x6D, 0x70, 0x73, 0x76, 0x79, 0x7C, 0x7F,
  0x82, 0x85, 0x88, 0x8B, 0x8E, 0x91, 0x94, 0x97, 0x9A, 0x9C, 0x9F, 0xA2, 0xA5,
  0xA7, 0xAA, 0xAD, 0xAF, 0xB2, 0xB4, 0xB7, 0xB9, 0xBB, 0xBE, 0xC0, 0xC2, 0xC4,
  0xC6, 0xC8, 0xCA, 0xCC, 0xCE, 0xD0, 0xD2, 0xD3, 0xD5, 0xD7, 0xD8, 0xDA, 0xDB,
  0xDE, 0xDF, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB,
  0xEC, 0xED, 0xED, 0xEE, 0xEF, 0xEF, 0xF0, 0xF1, 0xF1, 0xF2, 0xF2, 0xF3, 0xF3,
  0xF4, 0xF4, 0xF5, 0xF5, 0xF6, 0xF6, 0xF6, 0xF7, 0xF7, 0xF7, 0xF8, 0xF8, 0xF8,
  0xF9, 0xF9, 0xF9, 0xF9, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFB, 0xFB, 0xFB, 0xFB,
  0xFB, 0xFB, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFC, 0xFD, 0xFD,
  0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD, 0xFD,
0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF};

enum TIMERS
{
  TIMER_STOP_WATCH,
  TIMER_FADE,
  TIMER_DEBOUNCE,
  TIMER_TLC,
  TIMER_LAP_DISPLAY,
  TIMER_LAP,
  NUM_TIMERS
};

const uint8_t STOP_WATCH_DELAY = 1;
const uint8_t FADE_DELAY = 1;
const uint8_t DEBOUNCE_DELAY = 20;
const uint8_t TLC_DELAY = 5;
const uint16_t LAP_DISPLAY_DELAY = 2000;

uint8_t new_pot_pos, old_pot_pos = 0;
uint8_t fade_to;
bool fade_flag;

//Indices for the fade spectrum above
uint8_t red_fade_i, green_fade_i, blue_fade_i = 0;

uint32_t stop_watch_time;
uint32_t time_to_display = 0;
uint64_t event_timer[NUM_TIMERS];

uint32_t lap_times[3];

//-----------------------------------------------------------------------------
//      __   __   __  ___  __  ___      __   ___  __
//     |__) |__) /  \  |  /  \  |  \ / |__) |__  /__`
//     |    |  \ \__/  |  \__/  |   |  |    |___ .__/
//
//-----------------------------------------------------------------------------

static void set_timer_display(uint16_t pot_value);
static void fade_lights(uint8_t fade_color);

static uint8_t inc_fade_index(uint8_t fade_index);
static uint8_t dec_fade_index(uint8_t fade_index);

//Functions for fading lights
static void fade_red();
static void fade_yellow();
static void fade_green();
static void fade_cyan();

//Functions for displaying on the TLC display
static uint8_t display_number(uint32_t number, uint8_t digit);
static uint8_t decimal_to_SevSeg(uint32_t decimal, uint8_t digit);
static void write_to_digit(uint32_t decimal, uint8_t digit, uint8_t anode);

static void save_lap_time(uint32_t lap_time);

static int uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

//-----------------------------------------------------------------------------
//      __        __          __
//     |__) |  | |__) |    | /  `
//     |    \__/ |__) |___ | \__,
//
//-----------------------------------------------------------------------------

//=============================================================================
int main(void)
{
  uint8_t active_digit = 0;
  bool button[2] = {false,false};
  bool old_button[2] = {false, false};
  bool is_started = false;
  bool save_lap = false;
  bool display_lap = false;
  char my_string[MAX_SIZE];
  char input;
  uint16_t lap_counter = 0;
  
  //Initialize Drivers
  adc_init();
  led_init();
  pwm_init();
  timer_init();
  tlc_init();
  buttons_init();
  serial_init();
  switch_init();
  
  stdout = &mystdout;

  
  //Initialize the timers
  for (int i = 0; i < NUM_TIMERS; i++)
  {
    event_timer[i] = timer_get();
  }
  
  //Turn on global interrupts
  sei();
  
  while (1)
  {
    
    /////////////////// CHECK USER INPUT ///////////////////
    //Check to see if the buttons were pressed
    button[0] = buttons_get_debounce(0, timer_get());
    button[1] = buttons_get_debounce(1, timer_get());
    
    //Read the serial input
    input = serial_read();
    
    //Look for a falling edge on both buttons
    if ((true == button[0]) && (false == old_button[0]) &&
    (true == button[1]) && (false == old_button[1]))
    {
      
      //The watch has been reset, set the watch time back to 0 and reinitialize
      //all event timers
      lap_counter = 0;
      is_started = false;
      lap_times[0] = 0;
      stop_watch_time = 0;
      lap_times[1] = 0;
      lap_times[2] = 0;
      
      timer_set(0);
      for (int i = 0; i < NUM_TIMERS; i++)
      {
        event_timer[i] = timer_get();
      }
    }
    
    //If the first button was pressed, or "s" was entered, start/stop the watch.
    else if (((true == button[0]) && (false == old_button[0]))
    || (STARTSTOP == input))
    {
      is_started = !is_started;
      
      //If the system was started, print a message
      if (is_started)
      {
        sprintf(my_string, "\r\nStopwatch Started\r\n");
        
        while (serial_write_string(my_string) != 0)
        {
          ;
        }
        
        event_timer[TIMER_LAP] = timer_get();
      }
      
      //Otherwise, print the final lap time, as well as the total time
      else
      {
        lap_counter ++;
        
        //Save the lap time
        save_lap_time(stop_watch_time);
        
        //Print the final lap time
        sprintf(my_string, "Lap time %u: %u", lap_counter,
          (timer_get() - event_timer[TIMER_LAP]));
        while (serial_write_string(my_string) != 0)
        {
          ; //Wait for the serial write to not be busy
        }
        
        //Print the total time
        sprintf(my_string, "\r\nFinal Time: %u\r\nStopwatch Completed\r\n", 
                stop_watch_time);
        while (serial_write_string(my_string) != 0)
        {
          ;
        }
      }
    }
    
    //If the second button was pressed, or "l" was entered, record a lap.
    else if (((true == button[1]) && (false == old_button[1])) || (LAP == input))
    {
      
      //If the stop watch isn't in start mode, don't save the lap time.
      if (is_started)
      {
        save_lap = true;
        display_lap = true;
        event_timer[TIMER_LAP_DISPLAY] = timer_get();
      }
    }
    
    //Save the button value
    old_button[0] = button[0];
    old_button[1] = button[1];

    /////////////////// Check switch position ///////////////////
    if (UP == switch_get_position())
    {
      led_enable();
    }
    else
    {
      led_disable();
    }

    //If the timer has started,
    if (is_started)
    {
      
      /////////////////// Increment stop watch timer ///////////////////
      if (STOP_WATCH_DELAY <= (timer_get() - event_timer[TIMER_STOP_WATCH]))
      {
        event_timer[TIMER_STOP_WATCH] = timer_get();
        stop_watch_time ++;
      }
      
      /////////////////// Save lap times ///////////////////
      //Take the time since the last lap was saved and save it.
      if (save_lap)
      {
        lap_counter ++;
        
        //Save the lap time
        save_lap_time(timer_get() - event_timer[TIMER_LAP]);
        
        //Print the lap time
        sprintf(my_string, "Lap time %u: %u\r\n", lap_counter,
        (timer_get() - event_timer[TIMER_LAP]));
        while (serial_write_string(my_string) != 0)
        {
          ;
        }
        
        event_timer[TIMER_LAP] = timer_get();
        save_lap = false;
      }
      
      /////////////////// Update 7-seg display and LEDs ///////////////////
      //Update the display and, if necessary, fade the lights.
      event_timer[TIMER_STOP_WATCH] = timer_get();
      set_timer_display(adc_get_value());
      if (true == fade_flag)
      {
        //If enough time has passed, fade the lights.
        if (FADE_DELAY <= (timer_get() - event_timer[TIMER_FADE]))
        {
          event_timer[TIMER_FADE] = timer_get();
          fade_lights(fade_to);
        }
      }
    }
    
    //Otherwise, if the watch is stopped, just display the main time
    else
    {
      
      //Passing a pot value of 0 will tell the system that the pot is in the 1st
      //quadrant
      set_timer_display(0);
      
      //Fade the LEDs to red
      if (true == fade_flag)
      {
        //If enough time has passed, fade the lights.
        if (FADE_DELAY <= (timer_get() - event_timer[TIMER_FADE]))
        {
          event_timer[TIMER_FADE] = timer_get();
          fade_lights(fade_to);
        }
      }
    }
    
    
    /////////////////// Display new lap times ///////////////////
    //If the display_lap flag is set, and two seconds has elapsed since the flag
    //was set, display the most recent lap. Otherwise, display the selected time
    if (display_lap)
    {
      
      //If enough time has passed, turn the flag off. Otherwise, display the
      //most recent lap.
      if (LAP_DISPLAY_DELAY <= (timer_get() - event_timer[TIMER_LAP_DISPLAY]))
      {
        display_lap = false;
      }
      else
      {
        if (TLC_DELAY <= (timer_get() - event_timer[TIMER_TLC]))
        {
          event_timer[TIMER_TLC] = timer_get();
          active_digit = display_number(lap_times[0], active_digit);
        }
      }
    }
    else
    {
      if (TLC_DELAY <= (timer_get() - event_timer[TIMER_TLC]))
      {
        event_timer[TIMER_TLC] = timer_get();
        active_digit = display_number(time_to_display, active_digit);
      }
    }
    
  }
}

//-----------------------------------------------------------------------------
//      __   __              ___  ___
//     |__) |__) | \  /  /\   |  |__
//     |    |  \ |  \/  /~~\  |  |___
//
//-----------------------------------------------------------------------------

//=============================================================================
static void set_timer_display(uint16_t pot_value)
{
  
  //Determine the position of the pot, and turn on the correct led and display
  //the correct time
  if (pot_value <= 255)
  {
    new_pot_pos = 1;
    fade_to = RED_FADE;
    time_to_display = stop_watch_time;
  }
  else if (pot_value <= 511)
  {
    new_pot_pos = 2;
    fade_to = YELLOW_FADE;
    
    //Replace w/ lap time 1
    time_to_display = lap_times[0];
  }
  else if (pot_value <= 767)
  {
    new_pot_pos = 3;
    fade_to = GREEN_FADE;
    
    //Replace w/ lap time 2
    time_to_display = lap_times[1];
  }
  else if (pot_value <= 1023)
  {
    new_pot_pos = 4;
    fade_to = CYAN_FADE;
    
    //Replace w/ lap time 3
    time_to_display = lap_times[2];
  }
  
  //If the potentiometer position has changed, update the position and set a
  //flag indicating that the lights need to be faded.
  if (new_pot_pos != old_pot_pos)
  {
    old_pot_pos = new_pot_pos;
    fade_flag = true;
  }
}

//=============================================================================
static void fade_lights(uint8_t fade_color)
{
  if (RED_FADE == fade_color)
  {
    fade_red();
  }
  else if (YELLOW_FADE == fade_color)
  {
    fade_yellow();
  }
  else if (GREEN_FADE == fade_color)
  {
    fade_green();
  }
  else if (CYAN_FADE == fade_color)
  {
    fade_cyan();
  }
}

//=============================================================================
static uint8_t inc_fade_index(uint8_t fade_index)
{
  
  //If the index hasn't reached the max threshold, increment it.
  if (249 > fade_index)
  {
    fade_index++;
  }
  
  return fade_index;
}

//=============================================================================
static uint8_t dec_fade_index(uint8_t fade_index)
{
  
  //If the index hasn't reached the min threshold, decrement it.
  if (0 < fade_index)
  {
    fade_index--;
  }
  
  return fade_index;
}

//=============================================================================
static void fade_red()
{
  
  //Increase the red LED, and decrease the green and blue LED
  red_fade_i = inc_fade_index(red_fade_i);
  green_fade_i = dec_fade_index(green_fade_i);
  blue_fade_i = dec_fade_index(blue_fade_i);
  
  pwm_set_value(RED_LED, FADE_SPECTRUM[red_fade_i]);
  pwm_set_value(GREEN_LED, FADE_SPECTRUM[green_fade_i]);
  pwm_set_value(BLUE_LED, FADE_SPECTRUM[blue_fade_i]);
  
  //If each light has reached it's intended fade index, then turn off the fade
  //flag.
  if ((249 == red_fade_i) && (0 == green_fade_i) && (0 == blue_fade_i))
  {
    fade_flag = false;
  }
}

//=============================================================================
static void fade_yellow()
{
  
  //Increase the red and green LED, and decrease the blue LED
  red_fade_i = inc_fade_index(red_fade_i);
  green_fade_i = inc_fade_index(green_fade_i);
  blue_fade_i = dec_fade_index(blue_fade_i);
  
  pwm_set_value(RED_LED, FADE_SPECTRUM[red_fade_i]);
  pwm_set_value(GREEN_LED, FADE_SPECTRUM[green_fade_i]);
  pwm_set_value(BLUE_LED, FADE_SPECTRUM[blue_fade_i]);
  
  //If each light has reached it's intended fade index, then turn off the fade
  //flag.
  if ((249 == red_fade_i) && (249 == green_fade_i) && (0 == blue_fade_i))
  {
    fade_flag = false;
  }
}

//=============================================================================
static void fade_green()
{
  //Increase the green LED, and decrease the blue and red LED
  red_fade_i = dec_fade_index(red_fade_i);
  green_fade_i = inc_fade_index(green_fade_i);
  blue_fade_i = dec_fade_index(blue_fade_i);
  
  pwm_set_value(RED_LED, FADE_SPECTRUM[red_fade_i]);
  pwm_set_value(GREEN_LED, FADE_SPECTRUM[green_fade_i]);
  pwm_set_value(BLUE_LED, FADE_SPECTRUM[blue_fade_i]);
  
  //If each light has reached it's intended fade index, then turn off the fade
  //flag.
  if ((0 == red_fade_i) && (249 == green_fade_i) && (0 == blue_fade_i))
  {
    fade_flag = false;
  }
}

//=============================================================================
static void fade_cyan()
{
  
  //Increase the blue and green LED, and decrease the red LED.
  red_fade_i = dec_fade_index(red_fade_i);
  green_fade_i = inc_fade_index(green_fade_i);
  blue_fade_i = inc_fade_index(blue_fade_i);
  
  pwm_set_value(RED_LED, FADE_SPECTRUM[red_fade_i]);
  pwm_set_value(GREEN_LED, FADE_SPECTRUM[green_fade_i]);
  pwm_set_value(BLUE_LED, FADE_SPECTRUM[blue_fade_i]);
  
  //If each light has reached it's intended fade index, then turn off the fade
  //flag.
  if ((0 == red_fade_i) && (249 == green_fade_i) && (249 == blue_fade_i))
  {
    fade_flag = false;
  }
}

//=============================================================================
static uint8_t display_number(uint32_t number, uint8_t digit)
{
  tlc_turn_off_anodes();
  
  if (0 == digit)
  {
    
    //display the first digit
    write_to_digit(number, digit, DIGIT3_ANODE);
    digit = 1;
  }
  else if (1 == digit)
  {
    
    //Display the second digit
    write_to_digit(number, digit, DIGIT2_ANODE);
    digit = 2;
  }
  else if (2 == digit)
  {
    
    //Display the third digit
    write_to_digit(number, digit, DIGIT1_ANODE);
    digit = 3;
  }
  else if (3 == digit)
  {
    
    //Display the fourth digit
    write_to_digit(number, digit, DIGIT0_ANODE);
    digit = 0;
  }
  
  
  return digit;
}

//=============================================================================
static void write_to_digit(uint32_t decimal, uint8_t digit, uint8_t anode)
{
  uint8_t number_to_display;
  
  //Convert the digit to a seven segment value.
  number_to_display = decimal_to_SevSeg(decimal, digit);

  //Write the value to the sev_seg without the lights
  while ((tlc_write((0x00 << 8)|number_to_display)) != 0)
  {
    ; // Try to write until the tlc isn't busy
  }

  tlc_turn_on_anode(anode);
}

//=============================================================================
static uint8_t decimal_to_SevSeg(uint32_t decimal, uint8_t digit)
{
  uint8_t digit_value = 0;
  uint8_t sev_seg;
  
  //Get the value of the specified digit
  switch (digit)
  {
    case 0:
    digit_value = decimal % 100 / 10;
    break;
    
    case 1:
    digit_value = decimal % 1000 / 100;
    break;
    
    case 2:
    digit_value = decimal % 10000 / 1000;
    break;
    
    case 3:
    digit_value = decimal % 100000 / 10000;
    break;
  }
  
  //Get the seven segment representation of the digit and return it.
  sev_seg = SEVEN_SEG[digit_value];
  
  //If it's the digit in 100's place, set the 8th bit to turn on the
  //decimal point
  if (digit == 2)
  {
    sev_seg |= (1<<7);
  }
  
  return sev_seg;
}

//=============================================================================
static void save_lap_time(uint32_t lap_time)
{
  
  //Shift the lap times down one spot
  lap_times[2] = lap_times[1];
  lap_times[1] = lap_times[0];
  
  //Add the new lap time
  lap_times[0] = lap_time;
  
}

//=============================================================================
static int uart_putchar(char c, FILE *stream)
{
  //serial_write(c);
  return 0;
}

//-----------------------------------------------------------------------------
//        __   __   __
//     | /__` |__) /__`
//     | .__/ |  \ .__/
//
//-----------------------------------------------------------------------------

