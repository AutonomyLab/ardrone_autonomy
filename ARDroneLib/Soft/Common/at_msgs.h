/**
 *  \file     at_msgs.h
 *  \brief    ATCodec messages declaration
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.com>
 *  \date     2007/04/03
 *  \version  1.0
 */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_RC_REF_EXE,        "AT*REF=%d,%d\r",                                0, at_rc_ref_exe,         3 )
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PMODE_EXE,         "AT*PMODE=%d,%d\r",                           0, at_pmode_exe,          3 )
// old school
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_MISC_EXE,          "AT*MISC=%d,%d,%d,%d,%d\r",                      0, at_misc_exe,           3 )
// gains
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_GAIN_EXE,          "AT*GAIN=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r", 0, at_gain_exe,           3 )
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_ANIM_EXE,          "AT*ANIM=%d,%d,%d\r",                          0, at_anim_exe,         3 )
// vision params
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_VISP_EXE,          "AT*VISP=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r",       0, at_visp_exe,           3 )
// vision params
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_VISO_EXE,          "AT*VISO=%d,%d\r",                               0, at_viso_exe,           3 )
// capture params
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_RAWC_EXE,          "AT*CAP=%d\r",                                   0, at_cap,                3 )
// zapper
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_ZAP_EXE,           "AT*ZAP=%d,%d\r",                                0, at_zap,                3 )
// Change camera for arwiz detection
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_CAD_EXE,           "AT*CAD=%d,%d,%d\r",                             0, at_cad,                3 )
// flat trim
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_FTRIM_EXE,         "AT*FTRIM=%d\r",                                 0, at_flat_trim_exe,      3 )
// manual trims
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_MTRIM_EXE,          "AT*MTRIM=%d,%d,%d,%d\r",                       0, at_manual_trims_exe,	  3 )
// send attitude
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_POLARIS_EXE,       "AT*POL=%d,%d,%d,%d,%d,%d\r",                    0, at_pol_exe,            3 )
// sends iphone command for all axes
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PCMD_EXE,       	  "AT*PCMD=%d,%d,%d,%d,%d,%d\r",		           0, at_pcmd_exe,           3 )
// sends Radiocommand values for all 4 axis.
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_CONFIG_EXE,        "AT*CONFIG=%d,\"%s\",\"%s\"\r",                  0, at_toy_configuration_exe, 3)
// control command
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_CONFIG_IDS,        "AT*CONFIG_IDS=%d,\"%s\",\"%s\",\"%s\"\r",       0, at_toy_configuration_ids, 4)
// control command
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_CTRL_EXE,          "AT*CTRL=%d,%d,%d\r",                            0, at_control_exe,        3)
// led animation command
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_LED_EXE,           "AT*LED=%d,%d,%d,%d\r",                          0, at_led_animation_exe,  3)
// reset com watchdog
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_RESET_COM_WATCHDOG,"AT*COMWDG=%d\r",                                0, at_reset_com_watchdog, 3)

ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PWM_EXE,           "AT*PWM=%d,%d,%d,%d,%d\r",                       0, at_pwm,                3)

ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_AUTONOMOUS_FLIGHT_EXE, "AT*AFLIGHT=%d,%d\r",                        0, at_autonomous_flight_exe,  3 )

// Vicon information
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_VICON_EXE, 		   "AT*VICON=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r",  0, at_vicon_exe,  3 )

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Result messages
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_OK,       "OK\r",        AT_MSG_ATCMD_DEFAULT,      atresu_ok)
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_ERROR,    "ERROR\r",     AT_MSG_ATCMD_DEFAULT,      atresu_error)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
