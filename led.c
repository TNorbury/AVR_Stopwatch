//------------------------------------------------------------------------------
//             __             __   ___  __
//     | |\ | /  ` |    |  | |  \ |__  /__`
//     | | \| \__, |___ \__/ |__/ |___ .__/
//
//------------------------------------------------------------------------------

#include "led.h"
#include <avr/io.h>

//------------------------------------------------------------------------------
//      __   ___  ___         ___  __
//     |  \ |__  |__  | |\ | |__  /__`
//     |__/ |___ |    | | \| |___ .__/
//
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//     ___      __   ___  __   ___  ___  __
//      |  \ / |__) |__  |  \ |__  |__  /__`
//      |   |  |    |___ |__/ |___ |    .__/
//
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//                __          __        ___  __
//     \  /  /\  |__) |  /\  |__) |    |__  /__`
//      \/  /~~\ |  \ | /~~\ |__) |___ |___ .__/
//
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//      __   __   __  ___  __  ___      __   ___  __
//     |__) |__) /  \  |  /  \  |  \ / |__) |__  /__`
//     |    |  \ \__/  |  \__/  |   |  |    |___ .__/
//
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//      __        __          __
//     |__) |  | |__) |    | /  `
//     |    \__/ |__) |___ | \__,
//
//------------------------------------------------------------------------------

void led_init()
{
    // R0 = PD5 - PWM
    // G0 = PD6 - PWM
    // B0 = PB1 - PWM
    
    //Turn on the green and red lights
    DDRD |= 0x60;
    
    //Turn on the blue light
    DDRB |= 0x02;
}

//==============================================================================
void led_set_value(uint8_t led, uint8_t value)
{
  
  //Set the value of the given led.
  if (RED_LED == led)
  {
    if (1 == value)
    {
      PORTD |= (1 << PORTD5);
    }
    else
    {
      PORTD &= ~(1 << PORTD5);
    }
  }
  else if (GREEN_LED == led)
  {
    if (1 == value)
    {
      PORTD |= (1 << PORTD6);
    }
    else
    {
      PORTD &= ~(1 << PORTD6);
    }
  }
  else if (BLUE_LED == led)
  {
    if (1 == value)
    {
      PORTB |= (1 << PORTB1);
    }
    else
    {
      PORTB &= ~(1 << PORTB1);
    }
  }
}

//==============================================================================
void led_turn_all_off()
{
  
  //Set the value of all the LEDS to 0
  PORTD &= ~(0x60);
  PORTB &= ~(0x02);
}

//==============================================================================
void led_enable()
{
  
    //Switch the direction of all the leds so that they can display lights.
    DDRD |= 0x60;
    DDRB |= 0x02;
}

//==============================================================================
void led_disable()
{
  
  //Switch the direction of all the leds so that they can't display any lights.
  DDRD &= ~(0x60);
  DDRB &= ~(0x02);
}

//------------------------------------------------------------------------------
//      __   __              ___  ___
//     |__) |__) | \  /  /\   |  |__
//     |    |  \ |  \/  /~~\  |  |___
//
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//      __                  __        __        __
//     /  `  /\  |    |    |__)  /\  /  ` |__/ /__`
//     \__, /~~\ |___ |___ |__) /~~\ \__, |  \ .__/
//
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//        __   __  , __
//     | /__` |__)  /__`   
//     | .__/ |  \  .__/
//
//------------------------------------------------------------------------------
