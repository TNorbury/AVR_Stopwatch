//------------------------------------------------------------------------------
//             __             __   ___  __
//     | |\ | /  ` |    |  | |  \ |__  /__`
//     | | \| \__, |___ \__/ |__/ |___ .__/
//
//------------------------------------------------------------------------------

#include "pwm.h"
#include <avr/io.h>


//------------------------------------------------------------------------------
//      __   ___  ___         ___  __
//     |  \ |__  |__  | |\ | |__  /__`
//     |__/ |___ |    | | \| |___ .__/
//
//------------------------------------------------------------------------------

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2

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

//==============================================================================
void pwn_init()
{
  //Set up TCCR0A
  TCCR0A = 0xA3;
  
  //Set up TCCR0B w/ no pre-scaler
  TCCR0B = 0x01;
  
  //Set up TCCR1A for WGM
  TCCR1A |= 0xA1;
  
  //Set up WGM and pre-scaler
  TCCR1B |= 0x09;
}

//==============================================================================
void pwn_set_value(uint8_t led, uint8_t value)
{
  if (RED_LED == led)
  {
    OCR0B = value;
  }
  else if (GREEN_LED == led)
  {
    OCR0A = value;
  }
  else if (BLUE_LED == led)
  {
    OCR1A = value;
  }
}

//==============================================================================
void pwn_get_value(uint8_t led)
{
  uint8_t retval;
  
  if (RED_LED == led)
  {
    retval = OCR0B;
  }
  else if (GREEN_LED == led)
  {
    retval = OCR0A;
  }
  else if (BLUE_LED == led)
  {
    retval = OCR1A;
  }
  
  return retval;
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
