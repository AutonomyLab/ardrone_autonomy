//#include <network.h>

#include <sys/ioctl.h>
#include <unistd.h>
#include <net/if.h>  

#include <sys/types.h>
#include <sys/socket.h>

#include "vp_com_wlc.h"

#include <VP_Com/vp_com.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>

static int32_t wlsock = 0;
static wl_ioctl_t wldata;
static struct ifreq ifr;
static char buffer[WLC_IOCTL_MAXLEN];

typedef union
{
  char*    aschar;
  int32_t* asint32;
} buffer_repr_t;

static buffer_repr_t buffer_repr = { .aschar = buffer };

//////////////////////////////////////////////////////////////////////////////////////////////////////

static int vp_wlc_ioctl(int cmd, void *buffer, int length, bool set)
{
  if(wlsock < 0)
    return -1;

  wldata.cmd      = cmd;
  wldata.buf      = buffer;
  wldata.len      = length;
  wldata.set      = set;

  int result = ioctl(wlsock, SIOCDEVPRIVATE, &ifr);
  if (result<0)
  {
    perror("ioctl(wlsock, SIOCDEVPRIVATE, &ifr)");
  }
  return result;
}

static inline int vp_wlc_get(int cmd, void *data, int len)
{
  return vp_wlc_ioctl(cmd, data, len, FALSE);
}

static inline int vp_wlc_set(int cmd, void *data, int len)
{
  return vp_wlc_ioctl(cmd, data, len, TRUE);
}

static inline int vp_wlc_set_var(void* data, int len)
{
  return vp_wlc_set(WLC_SET_VAR, data, len);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

int32_t vp_wlc_begin(const char* itfname)
{
  strcpy(ifr.ifr_name, itfname);
  ifr.ifr_data  = (caddr_t) &wldata;

  if(wlsock <= 0)
    wlsock = socket(AF_INET, SOCK_DGRAM, 0);

  return wlsock;
}

int32_t vp_wlc_end(void)
{
  if(wlsock > 0)
  {
    close(wlsock);
    wlsock = 0;
  }

  return wlsock;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

int32_t vp_wlc_down(void)
{
  return vp_wlc_set(WLC_DOWN, NULL, 0);
}

int32_t vp_wlc_up(void)
{
  return vp_wlc_set(WLC_UP, NULL, 0);
}

int32_t vp_wlc_get_magic(void)
{
  if(vp_wlc_get(WLC_GET_MAGIC, buffer, sizeof(int32_t)) < 0)
    return -1;

  return *(buffer_repr.asint32);
}

int32_t vp_wlc_set_channel(int32_t channel)
{
  return vp_wlc_set(WLC_SET_CHANNEL, &channel, sizeof(int32_t));
}

channel_info_t* vp_wlc_get_channel(int32_t* channel)
{
  channel_info_t* channel_info = (channel_info_t*) buffer;

  *channel = -1;

  if(vp_wlc_get(WLC_GET_CHANNEL, buffer, sizeof(channel_info_t)) < 0)
    return NULL;

  *channel = channel_info->target_channel;

  return channel_info;
}

int32_t vp_wlc_set_country(const char* country)
{
  char *tmp = (char*)country;
  return vp_wlc_set(WLC_SET_COUNTRY, (void*)tmp, strlen(country) + 1);
}

const char* vp_wlc_get_country(void)
{
  if(vp_wlc_get(WLC_GET_COUNTRY, buffer, 4) < 0)
    return NULL;

  return (const char*) buffer;
}

int32_t vp_wlc_set_radio(VP_WLC_RADIO_STATE state)
{
  return vp_wlc_set(WLC_SET_RADIO, &state, sizeof(int32_t));
}

int32_t vp_wlc_get_radio(void)
{
  int32_t radio_state;

  if(vp_wlc_get(WLC_GET_RADIO, buffer, sizeof(int32_t)) < 0)
    return -1;

  radio_state = *(buffer_repr.asint32);
  if(radio_state == 0)
    return VP_WLC_RADIO_ENABLE;

  return VP_WLC_RADIO_DISABLE;
}

int32_t vp_wlc_set_infrastructure(VP_WLC_INFRA_MODE value)
{
  return vp_wlc_set(WLC_SET_INFRA, &value, sizeof(int32_t));
}

int32_t vp_wlc_get_infrastructure(void)
{
  if(vp_wlc_get(WLC_GET_INFRA, buffer, sizeof(int32_t)) < 0)
    return -1;

  return *(buffer_repr.asint32);
}

int32_t vp_wlc_set_authentication(VP_WLC_AUTH_MODE mode)
{
  return vp_wlc_set(WLC_SET_AUTH, &mode, sizeof(int32_t));
}

int32_t vp_wlc_get_authentication(void)
{
  if(vp_wlc_get(WLC_GET_AUTH, buffer, sizeof(int32_t)) < 0)
    return -1;

  return *(buffer_repr.asint32);
}

int32_t vp_wlc_set_ssid(const char* name)
{
  wlc_ssid_t ssid;

  vp_os_memset(&ssid, 0, sizeof(wlc_ssid_t));

  if(name)
  {
    ssid.SSID_len = strlen(name);
    strncpy((char *)ssid.SSID, name, ssid.SSID_len);
  }

  return vp_wlc_set(WLC_SET_SSID, &ssid, sizeof(wlc_ssid_t));
}

wlc_ssid_t* vp_wlc_get_ssid(void)
{
  if(vp_wlc_get(WLC_GET_SSID, buffer, sizeof(wlc_ssid_t)) < 0)
    return NULL;

  return (wlc_ssid_t*) buffer;
}


int32_t vp_wlc_get_bssid(bdaddr_t* bssid)
{
  if(vp_wlc_get(WLC_GET_BSSID, buffer, sizeof(bdaddr_t)) < 0)
    return -1;

  vp_com_copy_address((bdaddr_t*) buffer, bssid);

  return 0;
}


int32_t vp_wlc_set_security_mode(int32_t wsec)
{
  return vp_wlc_set(WLC_SET_WSEC, &wsec, sizeof(int32_t));
}

int32_t vp_wlc_get_security_mode(void)
{
  if(vp_wlc_get(WLC_GET_WSEC, buffer, sizeof(int32_t)) < 0)
    return -1;

  return *(buffer_repr.asint32);
}

int32_t vp_wlc_set_wep128_key(const char* wep_key)
{
  wl_wsec_key_t key;

  vp_os_memset(&key, 0, sizeof(wl_wsec_key_t));
  vp_os_memcpy(&key.data, wep_key, WEP128_KEY_SIZE);

  key.len     = WEP128_KEY_SIZE;
  key.algo    = CRYPTO_ALGO_WEP128;
  key.flags  |= WL_PRIMARY_KEY;

  return vp_wlc_set(WLC_SET_KEY, &key, sizeof(key));
}

int32_t vp_wlc_set_mac_mode(int32_t mode)
{
  return vp_wlc_set(WLC_SET_MACMODE, &mode, sizeof(int32_t));
}

int32_t vp_wlc_get_mac_mode(void)
{
  if( vp_wlc_get(WLC_GET_MACMODE, buffer, sizeof(int32_t)) < 0)
    return -1;

  return *(buffer_repr.asint32);
}

static int32_t vp_wlc_set_roam_trigger(int32_t value)
{
  return vp_wlc_set(WLC_SET_ROAM_TRIGGER, &value, sizeof(int32_t));
}

static int32_t vp_wlc_get_roam_trigger(int32_t* value)
{
  if( vp_wlc_get(WLC_GET_MACMODE, buffer, sizeof(int32_t)) < 0)
    return -1;

  *value = *(buffer_repr.asint32);

  return 0;
}

#define ROAMING_ENABLE_VALUE  0
#define ROAMING_DISABLE_VALUE -99

// Enables roaming
int32_t vp_wlc_enable_roaming(void)
{
  return vp_wlc_set_roam_trigger(ROAMING_ENABLE_VALUE);
}

// Disables roaming
int32_t vp_wlc_disable_roaming(void)
{
  return vp_wlc_set_roam_trigger(ROAMING_DISABLE_VALUE);
}

// Returns true if roaming is enable. 0 otherwise.
bool_t vp_wlc_roaming_enable(void)
{
  bool_t b = FALSE;
  int32_t value;

  if( vp_wlc_get_roam_trigger(&value) == 0 )
  {
    switch(value)
    {
      case ROAMING_ENABLE_VALUE:
        b = TRUE;
        break;

      case ROAMING_DISABLE_VALUE:
        b = FALSE;
        break;

      default:
        b = FALSE;
        break;
    }
  }

  return b;
}

int32_t vp_wlc_scan(void)
{
  int32_t ret = 0;
  int32_t ap = 0, oldap = 0;
  wl_scan_params_t params;

  vp_os_memset(&params, 0, sizeof(params));
  vp_os_memset(&params.bssid, 0xff, sizeof(params.bssid));

  params.bss_type     = DOT11_BSSTYPE_ANY;
  params.scan_type    = -1;
  params.nprobes      = -1;
  params.active_time  = -1;
  params.passive_time = -1;
  params.home_time    = -1;

  if( vp_wlc_get( WLC_GET_AP, &oldap, sizeof(int32_t) ) < 0 )
    ret = -1;

  if( oldap > 0 )
    vp_wlc_set( WLC_SET_AP, &ap, sizeof(int32_t) );

  if( vp_wlc_get(WLC_SCAN, &params, 64) < 0 )
    ret = -1;

  if( oldap > 0 )
    vp_wlc_set( WLC_SET_AP, &oldap, sizeof(oldap) );

  return ret;
}

wl_scan_results_t* vp_wlc_get_scan_results(void)
{
  wl_scan_results_t* results = (wl_scan_results_t *) buffer;

  results->buflen = WLC_IOCTL_MAXLEN - sizeof(wl_scan_results_t);

  if( vp_wlc_get(WLC_SCAN_RESULTS, buffer, WLC_IOCTL_MAXLEN) < 0 )
    return NULL;

  return results;
}

static bool swap = FALSE;
#define dtoh32(i) (swap?bcmswap32(i):i)
#define htod32(i) (swap?bcmswap32(i):i)
#define DIV_QUO(num, div) ((num)/div)  /* Return the quotient of division to avoid floats */
#define DIV_REM(num, div) (((num%div) * 100)/div) /* Return the remainder of division */

/* Byte swap a 32 bit value */
#define BCMSWAP32(val) \
	((uint32)(\
		(((uint32)(val) & (uint32)0x000000ffUL) << 24) | \
		(((uint32)(val) & (uint32)0x0000ff00UL) <<  8) | \
		(((uint32)(val) & (uint32)0x00ff0000UL) >>  8) | \
		(((uint32)(val) & (uint32)0xff000000UL) >> 24)))

static INLINE uint32
bcmswap32(uint32 val)
{
	return BCMSWAP32(val);
}

#define QDBM_TABLE_LEN 40
#define QDBM_OFFSET 153

/* Smallest mW value that will round up to the first table entry, QDBM_OFFSET.
 * Value is ( mW(QDBM_OFFSET - 1) + mW(QDBM_OFFSET) ) / 2
 */
#define QDBM_TABLE_LOW_BOUND 6493 /* QDBM_TABLE_LOW_BOUND */

static const uint16_t nqdBm_to_mW_map[QDBM_TABLE_LEN] = {
/* qdBm:        +0	+1	+2	+3	+4	+5	+6	+7	*/
/* 153: */      6683,	7079,	7499,	7943,	8414,	8913,	9441,	10000,
/* 161: */      10593,	11220,	11885,	12589,	13335,	14125,	14962,	15849,
/* 169: */      16788,	17783,	18836,	19953,	21135,	22387,	23714,	25119,
/* 177: */      26607,	28184,	29854,	31623,	33497,	35481,	37584,	39811,
/* 185: */      42170,	44668,	47315,	50119,	53088,	56234,	59566,	63096
};

static uint16_t wl_qdbm_to_mw(uint8_t qdbm)
{
  uint32_t factor = 1;
  int idx = qdbm - QDBM_OFFSET;

  if (idx >= QDBM_TABLE_LEN) {
    /* clamp to max uint16 mW value */
    return 0xFFFF;
  }

  /* scale the qdBm index up to the range of the table 0-40
   * where an offset of 40 qdBm equals a factor of 10 mW.
   */
  while (idx < 0) {
          idx += 40;
          factor *= 10;
  }

  /* return the mW value scaled down to the correct factor of 10,
   * adding in factor/2 to get proper rounding.
   */
  return ((nqdBm_to_mW_map[idx] + factor/2) / factor);
}

static uint8 wl_mw_to_qdbm(uint16 mw)
{
  uint8 qdbm;
  int offset;
  uint mw_uint = mw;
  uint boundary;

  /* handle boundary case */
  if (mw_uint <= 1)
    return 0;

  offset = QDBM_OFFSET;

  /* move mw into the range of the table */
  while (mw_uint < QDBM_TABLE_LOW_BOUND) {
    mw_uint *= 10;
    offset -= 40;
  }

  for (qdbm = 0; qdbm < QDBM_TABLE_LEN-1; qdbm++) {
    boundary = nqdBm_to_mW_map[qdbm] + (nqdBm_to_mW_map[qdbm+1] - nqdBm_to_mW_map[qdbm])/2;
    if (mw_uint < boundary) break;
  }

  qdbm += (uint8)offset;

  return (qdbm);
}

int32_t vp_wlc_get_instant_power(uint32_t *power)
{
  int val;
  uint8_t buffer[WLC_IOCTL_MAXLEN];
  uint32 *int_ptr;
  bool override = TRUE;

  strcpy((char*)&buffer[0], "qtxpower");
  if( vp_wlc_get(WLC_GET_VAR, &buffer[0], strlen((char *)&buffer[0]) + 5) < 0)
    return -1;

  int_ptr = (uint32 *)&buffer[0];
  val = dtoh32(*int_ptr);
  override = (bool)(val & WL_TXPWR_OVERRIDE);
  val &= ~WL_TXPWR_OVERRIDE;
  *power = wl_qdbm_to_mw((uint8)((val<0xff?val:0xff)));

  return 0;
}

int32_t vp_wlc_set_instant_power(uint32_t power)
{
  unsigned int new_val = 0;
  uint8_t pwr_buf[WLC_IOCTL_MAXLEN];
  uint8_t *int_ptr;
  bool override = TRUE;

  strcpy((char *)&pwr_buf[0], "qtxpower");

  new_val = wl_mw_to_qdbm((uint16)(power<=0xffff?power:0xffff));

  int_ptr = &pwr_buf[strlen((char *)&pwr_buf[0]) + 1];
  if (override)
    new_val |= WL_TXPWR_OVERRIDE;
  new_val = htod32(new_val);
  memcpy(int_ptr, (const void *)&new_val, sizeof(new_val));

  if( vp_wlc_set(WLC_SET_VAR, &pwr_buf[0], strlen((char *)&pwr_buf[0])+5) < 0 )
    return -1;

  PRINT("Power set to %d mW\n", power);
  return 0;
}
