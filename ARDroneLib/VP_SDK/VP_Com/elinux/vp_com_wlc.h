#ifndef _VP_COM_WLC_H_
#define _VP_COM_WLC_H_

#include <VP_Os/vp_os_types.h>
#include <wlioctl.h>


// Begin a call sequence
int32_t vp_wlc_begin(const char* itfname);

// End a call sequence
int32_t vp_wlc_end(void);

#define VP_WLC_BEGIN(itfname) vp_wlc_begin(itfname); {
#define VP_WLC_END            } vp_wlc_end();

// Takes the driver out of the UP state. 
// Be aware that the DOWN state, resets all driver calibration states.
// To take the driver out of the UP state and keep any run time calibration information,
// use the wlc_out.
int32_t vp_wlc_down(void);

// Enables the driver after a WLC_DOWN or WLC_OUT command.
int32_t vp_wlc_up(void);


// Gets the driver's magic id 
int32_t vp_wlc_get_magic(void);

// Sets the default BSS channel. For IEEE 802.11b and
// IEEE 802.11g solutions, valid channels are generally 1­14.
// Note that the exact valid channel setting is dictated by the
// current country setting.
int32_t vp_wlc_set_channel(int32_t channel);

// Returns the pointer to a channel_info_t structure.
channel_info_t* vp_wlc_get_channel(int32_t* channel);


// Sets the IEEE 802.11d country string. The input parameter can 
// either be the full name of the country (CHINA), or a two-
// character abbreviation (CN).
int32_t vp_wlc_set_country(const char* country);

// Returns the current IEEE 802.11d country string used by the driver.
// The return value is always 4 bytes long.
const char* vp_wlc_get_country(void);


// Returns the current instant power of the chip.
int32_t vp_wlc_get_instant_power(uint32_t *power);

// Sets the current instant power of the chip.
int32_t vp_wlc_set_instant_power(uint32_t power);


// Radio states
typedef enum
{
  VP_WLC_RADIO_ENABLE  = 0x10000,
  VP_WLC_RADIO_DISABLE = 0x10001
} VP_WLC_RADIO_STATE;

// Sets the software radio state.
int32_t vp_wlc_set_radio(VP_WLC_RADIO_STATE state);

// Returns the radio state.
// -1 = Error
int32_t vp_wlc_get_radio(void);


typedef enum
{
  VP_WLC_ADHOC           = 0,
  VP_WLC_INFRASTRUCTURE  = 1
} VP_WLC_INFRA_MODE;

// Pass 0 to disable infrastructure mode; Pass 1 to enable it.
int32_t vp_wlc_set_infrastructure(VP_WLC_INFRA_MODE value);

// Returns 1 if in infrastructure mode; 0 otherwise or -1 if error.
int32_t vp_wlc_get_infrastructure(void);


typedef enum
{
  VP_WLC_AUTH_OPEN   = 0,
  VP_WLC_AUTH_SHARED = 1
} VP_WLC_AUTH_MODE;

// Sets the authentication mode
int32_t vp_wlc_set_authentication(VP_WLC_AUTH_MODE mode);

// Returns the authentication mode
int32_t vp_wlc_get_authentication(void);


// Sets the ssid
int32_t vp_wlc_set_ssid(const char* ssid);

// Returns the current SSID. Note that after an SSID has been set, 
// the BSS is created. The BSS is with the rate, channel, and other
// settings in their current state. This function should be called only
// after all other parameters have been set.
wlc_ssid_t* vp_wlc_get_ssid(void);


// Returns the bssid
int32_t vp_wlc_get_bssid(bdaddr_t* bssid);


// Sets the wireless security mode to be used.
// Possible values are
//    WEP_ENABLED
//    TKIP_ENABLED
//    AES_ENABLED
int32_t vp_wlc_set_security_mode(int32_t wsec);

// Returns a value containing a bit vector representing which
// wireless security modes are currently enabled
int32_t vp_wlc_get_security_mode(void);


// Inserts a security key
int32_t vp_wlc_set_wep128_key(const char* wep_key);

// Sets the mode of the MAC list
// Possible values are
//  WLC_MACMODE_DISABLED
//  WLC_MACMODE_DENY
//  WLC_MACMODE_ALLOW
int32_t vp_wlc_set_mac_mode(int32_t mode);

// Returns the mac mode
int32_t vp_wlc_get_mac_mode(void);

/// Roaming

// Enables roaming
int32_t vp_wlc_enable_roaming(void);

// Disables roaming
int32_t vp_wlc_disable_roaming(void);

// Returns true if roaming is enable. 0 otherwise.
bool_t vp_wlc_roaming_enable(void);


// Begins a scan sequence
int32_t vp_wlc_scan(void);

// Gets the result of a previously launched scan
wl_scan_results_t* vp_wlc_get_scan_results(void);

#endif // _VP_COM_WLC_H_
