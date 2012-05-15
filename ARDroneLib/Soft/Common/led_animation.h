/******************************************************************************
 *                        COPYRIGHT PARROT 2010
 ******************************************************************************
 *       PARROT                MODULES
 *---------------------------------------------------------------------------*/
/**
 * @file   leds_animation.h
 * @date   25th February 2010
 * @brief  Data types and functions to communicate with the drone.
 * @author Pierre Eline
 *
 ******************************************************************************/

// LED_ANIMATION(#name, {#nb_cycle,#nb_state,{{#led_pattern1,#delay1},{#led_pattern2,#delay2},{...,...}}})
// #name = name, example : BLINK
// #nb_cycle = number of times the animation is played (0 means infinite), example : 3
// #nb_state = number of led patterns in the animation, example : 2
// #led_pattern = led bitfield (G1 | R1 | G2 | R2 | G3 | R3 | G4 | R4), example : 0xAA all green led turned on
// #delay = delay in ms for the associated led pattern, example : 500

LED_ANIMATION(BLINK_GREEN_RED,               { 0,2, { {0x55,500},{0xAA,500} } } )
LED_ANIMATION(BLINK_GREEN,                   { 0,2, { {0x00,500},{0xAA,500} } } )
LED_ANIMATION(BLINK_RED,                     { 0,2, { {0x55,500},{0x00,500} } } )
LED_ANIMATION(BLINK_ORANGE,                  { 0,2, { {0xFF,500},{0x00,500} } } )
LED_ANIMATION(SNAKE_GREEN_RED,               { 0,8, { {0x90,200},{0x48,200},{0x24,200},{0x12,200},{0x9,200},{0x84,200},{0x42,200},{0x21,200}}})
LED_ANIMATION(FIRE,                          { 0,2, { {0x35,50},{0xC5,50} } } )
LED_ANIMATION(STANDARD,                      { 1,1, { {0xA5,100} } } )
LED_ANIMATION(RED,                           { 1,1, { {0x55,100} } } )
LED_ANIMATION(GREEN,                         { 1,1, { {0xAA,100} } } )
LED_ANIMATION(RED_SNAKE,                     { 0,4, { {0x40,500},{0x10,500},{0x04,500},{0x01,500}}})
LED_ANIMATION(BLANK,                         { 1,1, { {0x00,100} } } )
LED_ANIMATION(RIGHT_MISSILE,                 { 1,5, { {0x00,500},{0x04,300},{0x1C,100},{0x30,300},{0x00,500}}})
LED_ANIMATION(LEFT_MISSILE,                  { 1,5, { {0x00,500},{0x01,300},{0x43,100},{0xC0,300},{0x00,500}}})
LED_ANIMATION(DOUBLE_MISSILE,                { 1,5, { {0x00,500},{0x05,300},{0x5F,100},{0xF0,300},{0x00,500}}})
LED_ANIMATION(FRONT_LEFT_GREEN_OTHERS_RED,   { 1,1, { {0x95,100} } } )
LED_ANIMATION(FRONT_RIGHT_GREEN_OTHERS_RED,  { 1,1, { {0x65,100} } } )
LED_ANIMATION(REAR_RIGHT_GREEN_OTHERS_RED,   { 1,1, { {0x59,100} } } )
LED_ANIMATION(REAR_LEFT_GREEN_OTHERS_RED,    { 1,1, { {0x56,100} } } )
LED_ANIMATION(LEFT_GREEN_RIGHT_RED,          { 1,1, { {0x96,100} } } )
LED_ANIMATION(LEFT_RED_RIGHT_GREEN,          { 1,1, { {0x69,100} } } )
LED_ANIMATION(BLINK_STANDARD,                { 0,2, { {0x00,500},{0xA5,500} } } )






