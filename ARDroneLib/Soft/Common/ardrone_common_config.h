/**
 *  \file     ardrone_common_config.h
 *  \brief    Ardrone Specific data for configuration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.com>
 *  \version  1.0
 */

#ifndef _ARDRONE_COMMON_CONFIG_H_
#define _ARDRONE_COMMON_CONFIG_H_

#define CURRENT_NUM_VERSION_CONFIG  1

#define CAMIF_NUM_BUFFERS               2
#define COM_INPUT_LANDING_TIME          (2)         /* Time drone is waiting for input before landing */

/**
 * \enum print_mask_t
 * \brief mask to choose where to syslog
*/
typedef enum {
  UART_PRINT = 1,
  WIFI_PRINT = 2,
  FLASH_PRINT = 4
} print_mask_t;

/**
 * \enum  ADC_COMMANDS
 * \brief ADC commands.
*/
typedef enum
{
  ADC_CMD_STARTACQ                = 1,  /**< command to start acquisition with ADC                            **/
  ADC_CMD_STOPACQ                 = 2,  /**< command to stop acquisition with ADC                             **/
  ADC_CMD_RESYNC                  = 3,  /**< command to resync acquisition with ADC                           **/
  ADC_CMD_TEST                    = 4,  /**< command to ADC send a test frame (123456)                        **/
  ADC_CMD_VERSION                 = 5,  /**< command to ADC send his number : version (MSB) subversion (LSB)  **/
  ADC_CMD_SELECT_ULTRASOUND_22Hz  = 7,  /**set the ultrasound at 22,22Hz                                      **/
  ADC_CMD_SELECT_ULTRASOUND_25Hz  = 8,  /**set the ultrasound at 25Hz                                         **/
  ADC_CMD_SEND_CALIBRE            = 13, /**command to ADC to send the calibration                             **/
  ADC_CMD_RECEVED_CALIBRE         = 14, /**command to ADC to receved a new calibration                        **/
  ADC_CMD_GET_HARD_VERSION        = 15, /**get the hard version of the navboard                               **/
  ADC_CMD_ACTIVE_SEPARATION       = 16, /**enabled the separation of sources ultrasound                       **/
  ADC_CMD_STOP_SEPARATION         = 17, /**disables the ultrasound source separation                          **/
  ADC_CMD_RECEVED_PROD            = 18, /**command to ADC to receved the prod data                            **/
  ADC_CMD_SEND_PROD               = 19, /**command to ADC to send the prod data                               **/
  ADC_CMD_ACTIVE_ETALONAGE        = 20, /**command to ADC to send PWM ultrasond in continue                   **/
  ADC_CMD_ACTIVE_ULTRASON         = 21, /**command to ADC to stop send PWM ultrasond in continue              **/
  ADC_CMD_ACTIVE_TEST_ULTRASON    = 22, /**teste de la perturbation de l'ultrason par le wifi                 **/
} ADC_COMMANDS;


#endif // _ARDRONE_COMMON_CONFIG_H_
