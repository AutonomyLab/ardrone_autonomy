/**
 * @file ATcodec_Messages_ex.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/06
 */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// General commands
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_CGMI,      "AT+CGMI\r",                  0, at_cgmi, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_CGMM,      "AT+CGMM\r",                  0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_CGMR,      "AT+CGMR\r",                  0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PR,        "AT*PR\r",                    0, at_default_cb, 7)
// Motors commands
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PM,        "AT*PM\r",                    0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PM_QW,     "AT*PM=!%d\r",                0, at_pm_qw,      0)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PM_QR,     "AT*PM=?\r",                  0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PM_QV,     "AT*PM?\r",                   0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PM_EXE,    "AT*PM=%d,%d\r",              0, at_pm_exe,     0)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PM_CAL,    "AT*PM!\r",                   0, at_default_cb, 6)
// Lights commands
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PL,        "AT*PL\r",                    0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PL_QW,     "AT*PL=!%d\r",                0, at_default_cb, 2)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PL_QR,     "AT*PL=?\r",                  0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PL_QV,     "AT*PL?\r",                   0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PL_EXE,    "AT*PL=%d\r",                 0, at_default_cb, 2)
// Audio/Video commands : streaming
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PAVS_QR,   "AT*PAVS=?\r",                0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PAVS_QV,   "AT*PAVS?\r",                 0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PAVS_EXE,  "AT*PAVS=%d[,\"%s\"]\r",      0, at_default_cb, 7)
// Audio/Video commands : input settings
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PAVI_QR,   "AT*PAVI=?\r",                0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PAVI_QV,   "AT*PAVI?\r",                 0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PAVI_EXE,  "AT*PAVI=%l{[%d]}\r",         0, at_default_cb, 7)
// Audio/Video commands : audio output
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PAO_QR,    "AT*PAO=?\r",                 0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PAO_QV,    "AT*PAO?\r",                  0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PAO_EXE,   "AT*PAO=%d[,\"%s\"]\r",       0, at_default_cb, 1)
// Codecs commands : codecs choosing
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PCC_QR,    "AT*PCC=?\r",                 0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PCC_QV,    "AT*PCC?\r",                  0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PCC_EXE,   "AT*PCC=[%d],[%d]\r",         0, at_default_cb, 8)
// Codecs commands : codecs settings
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PCS_QR,    "AT*PCS=?\r",                 0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PCS_QV,    "AT*PCS?\r",                  0, at_default_cb, 9)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PCS_EXE,   "AT*PCS=%l{[%d]}\r",          0, at_default_cb, 8)
// Test SDCard
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_SDT_EXE,   "AT*SDT\r",                   0, at_default_cb, 7)
// PID
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PIP_EXE,   "AT*PIP=%d\r",                0, at_default_cb, 6)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PII_EXE,   "AT*PII=%d\r",                0, at_default_cb, 6)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_PID_EXE,   "AT*PID=%d\r",                0, at_default_cb, 6)
// change calibration
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_DEM_EXE,   "AT*DEM\r",                   0, at_default_cb, 6)
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_INM_EXE,   "AT*INM\r",                   0, at_default_cb, 6)
// get rssi
ATCODEC_DEFINE_AT_CMD(AT_MSG_ATCMD_RSS_EXE,   "AT*RSS=%d:%d:%d:%d:%d:%d\r", 0, at_default_cb, 3)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Result messages
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_OK,      "OK\r",                                                            AT_MSG_ATCMD_DEFAULT, atresu_ok)
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_ERROR,   "ERROR\r",                                                         AT_MSG_ATCMD_DEFAULT, atresu_error)
// General commands results
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_CGMI,    "+CGMI: \"%s\"\r",                                                 AT_MSG_ATCMD_CGMI,    atresu_cgmi)
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_CGMM,    "+CGMM: (\"%s\",\"%s\")\r",                                        AT_MSG_ATCMD_CGMM,    at_default_cb)
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_CGMR,    "+CGMR: (\"%s\",\"%s\")\r",                                        AT_MSG_ATCMD_CGMR,    at_default_cb)
// Motors commands results
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PM,      "*PM: (%l{\"%s\"})\r",                                             AT_MSG_ATCMD_PM,      at_default_cb)
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PM_QR,   "*PM: %d,(\"%s\",(%l{%d})),(\"%s\",<int>)\r",                      AT_MSG_ATCMD_PM_QR,   at_default_cb)
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PM_QV,   "*PM: %d,%d\r",                                                    AT_MSG_ATCMD_PM_QV,   at_default_cb)
// Lights commands results
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PL,      "*PL: (%l{\"%s\"})\r",                                             AT_MSG_ATCMD_PL,      at_default_cb)
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PL_QR,   "*PL: %d,(%l{%d})\r",                                              AT_MSG_ATCMD_PL_QR,   at_default_cb)
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PL_QV,   "*PL: %d,%d\r",                                                    AT_MSG_ATCMD_PL_QV,   at_default_cb)
// Audio/Video commands results : streaming
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PAVS_QR, "*PAVS: (\"%s\",(%l{\"%s\"}))\\[,(\"%s\",<string>)\\]\r",          AT_MSG_ATCMD_PAVS_QR, at_default_cb)
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PAVS_QV, "*PAVS: %d,\"[%s]\",%d,\"[%s]\"\r",                                AT_MSG_ATCMD_PAVS_QV, at_default_cb)
// Audio/Video commands results : input settings
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PAVI_QR, "*PAVI: %l{\\[(\"%s\"\\,(%d..%d))\\]}\r",                          AT_MSG_ATCMD_PAVI_QR, at_default_cb)
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PAVI_QV, "*PAVI: %l{%d}\r",                                                 AT_MSG_ATCMD_PAVI_QV, at_default_cb)
// Audio/Video commands results : audio output
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PAO_QR,  "*PAO: (\"%s\",(%l{\"%s\"}))\\[,(\"%s\",<string>)\\]\r",           AT_MSG_ATCMD_PAO_QR,  at_default_cb)
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PAO_QV,  "*PAO: %d,\"[%s]\"\r",                                             AT_MSG_ATCMD_PAO_QV,  at_default_cb)
// Codecs commands results : codecs choosing
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PCC_QR,  "*PCC: \\[(\"%s\",(%l{\"%s\"}))\\],\\[(\"%s\",(%l{\"%s\"}))\\]\r", AT_MSG_ATCMD_PCC_QR,  at_default_cb)
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PCC_QV,  "*PCC: %d,%d\r",                                                   AT_MSG_ATCMD_PCC_QV,  at_default_cb)
// Codecs commands results : codecs settings
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PCS_QR,  "*PCS: %l{\\[(\"%s\"\\,(%l{%d}))\\]}\r",                           AT_MSG_ATCMD_PCS_QR,  at_default_cb)
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_PCS_QV,  "*PCS: %l{%d}\r",                                                  AT_MSG_ATCMD_PCS_QV,  at_default_cb)
// RSSI command result : value
ATCODEC_DEFINE_AT_RESU(AT_MSG_ATRESU_RSS,     "*RSS: %d\r",                                                      AT_MSG_ATCMD_RSS_EXE, at_default_cb)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

