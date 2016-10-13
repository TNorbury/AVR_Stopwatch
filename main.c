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
#include <stdint.h>
#include <avr/interrupt.h>

#include "adc.h"
#include "led.h"
#include "pwm.h"
#include "timer.h"

//-----------------------------------------------------------------------------
//      __   ___  ___         ___  __
//     |  \ |__  |__  | |\ | |__  /__`
//     |__/ |___ |    | | \| |___ .__/
//
//-----------------------------------------------------------------------------

#define RED_FADE 0
#define YELLOW_FADE 1
#define GREEN_FADE 2
#define CYAN_FADE 3

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

enum TIMERS
{
  TIMER_STOP_WATCH,
  TIMER_FADE,
  NUM_TIMERS
};

//-----------------------------------------------------------------------------
//      __   __   __  ___  __  ___      __   ___  __
//     |__) |__) /  \  |  /  \  |  \ / |__) |__  /__`
//     |    |  \ \__/  |  \__/  |   |  |    |___ .__/
//
//-----------------------------------------------------------------------------

void set_timer_display(uint16_t pot_value);

//-----------------------------------------------------------------------------
//      __        __          __
//     |__) |  | |__) |    | /  `
//     |    \__/ |__) |___ | \__,
//
//-----------------------------------------------------------------------------

//=============================================================================
int main(void)
{
  
  //Initialize Drivers
  adc_init();
  led_init();
  pwn_init();
  timer_init();
  
  //Turn on global interrupts
  sei();
  
  while (1)
  {
    set_timer_display(adc_get_value());
  }
}

//-----------------------------------------------------------------------------
//      __   __              ___  ___
//     |__) |__) | \  /  /\   |  |__
//     |    |  \ |  \/  /~~\  |  |___
//
//-----------------------------------------------------------------------------

//=============================================================================
void set_timer_display(uint16_t pot_value)
{
  //Turn off all LEDs
  led_turn_all_off();
  
  
  //Determine the position of the pot, and turn on the correct led and display
  //the correct time
  if (pot_value <= 255)
  {
    led_set_value(RED_LED, 1);
  }
  else if (pot_value <= 511)
  {
    led_set_value(RED_LED, 1);
    led_set_value(GREEN_LED, 1);
  }
  else if (pot_value <= 767)
  {
    led_set_value(GREEN_LED, 1);
  }
  else if (pot_value <= 1023)
  {
    led_set_value(GREEN_LED, 1);
    led_set_value(BLUE_LED, 1);
  }
}

//-----------------------------------------------------------------------------
//        __   __   __
//     | /__` |__) /__`
//     | .__/ |  \ .__/
//
//-----------------------------------------------------------------------------

