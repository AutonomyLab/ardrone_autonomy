
/**
 *  \brief    VP Stages. Vision Stage default configurations
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.com>
 *  \version  1.0
 *  \date     first release 2007/11/06
 */


#include <VP_Os/vp_os_types.h>
#include <VP_Os/vp_os_assert.h>
#include <VP_Os/vp_os_malloc.h>

#include <VP_Stages/vp_stages_configs.h>

#include <VP_Stages/vp_stages_io_com.h>

#if defined(USE_LINUX)
#    include <VP_Stages/vp_stages_o_sdl.h>
#endif // USE_LINUX

vp_api_picture_t vp_api_picture;

#if defined(USE_LINUX)
static vp_stages_output_sdl_config_t
decoding_qcif_sdl_config = {
  .width         = QVGA_WIDTH,
  .height        = QVGA_HEIGHT,
  .bpp           = 16,
  .window_width  = QVGA_WIDTH,
  .window_height = QVGA_HEIGHT,
  .pic_width     = QCIF_WIDTH,
  .pic_height    = QCIF_HEIGHT,
  .y_size        = QCIF_WIDTH*QCIF_HEIGHT,
  .c_size        = (QCIF_WIDTH*QCIF_HEIGHT)>>2,
};


static vp_stages_output_sdl_config_t
raw_qcif_sdl_config = {
  .width         = QVGA_WIDTH,
  .height        = QVGA_HEIGHT,
  .bpp           = 16,
  .window_width  = QVGA_WIDTH,
  .window_height = QVGA_HEIGHT,
  .pic_width     = QCIF_WIDTH,
  .pic_height    = QCIF_HEIGHT,
  .y_size        = QCIF_WIDTH*QCIF_HEIGHT,
  .c_size        = 0,
};


static vp_stages_output_sdl_config_t
raw_qqcif_sdl_config = {
  .width         = QVGA_WIDTH,
  .height        = QVGA_HEIGHT,
  .bpp           = 16,
  .window_width  = QVGA_WIDTH,
  .window_height = QVGA_HEIGHT,
  .pic_width     = QQCIF_WIDTH,
  .pic_height    = QQCIF_HEIGHT,
  .y_size        = QQCIF_WIDTH*QQCIF_HEIGHT,
  .c_size        = 0,
};


static vp_stages_output_sdl_config_t
decoding_qvga_sdl_config = {
  .width         = QVGA_WIDTH,
  .height        = QVGA_HEIGHT,
  .bpp           = 16,
  .window_width  = QVGA_WIDTH,
  .window_height = QVGA_HEIGHT,
  .pic_width     = QVGA_WIDTH,
  .pic_height    = QVGA_HEIGHT,
  .y_size        = QVGA_WIDTH*QVGA_HEIGHT,
  .c_size        = (QVGA_WIDTH*QVGA_HEIGHT)>>2,
};


static vp_stages_output_sdl_config_t
raw_qvga_sdl_config = {
  .width         = QVGA_WIDTH,
  .height        = QVGA_HEIGHT,
  .bpp           = 16,
  .window_width  = QVGA_WIDTH,
  .window_height = QVGA_HEIGHT,
  .pic_width     = QVGA_WIDTH,
  .pic_height    = QVGA_HEIGHT,
  .y_size        = QVGA_WIDTH*QVGA_HEIGHT,
  .c_size        = 0,
};

static vp_stages_output_sdl_config_t
raw_tweaky_qqvga_sdl_config = {
  .width         = QVGA_WIDTH,
  .height        = QVGA_HEIGHT,
  .bpp           = 16,
  .window_width  = QVGA_WIDTH,
  .window_height = QVGA_HEIGHT,
  .pic_width     = TWEAKY_QQVGA_WIDTH,
  .pic_height    = TWEAKY_QQVGA_HEIGHT,
  .y_size        = TWEAKY_QQVGA_WIDTH*TWEAKY_QQVGA_HEIGHT,
  .c_size        = 0,
};
#endif // ! __linux__ || USE_MINGW32

#if defined(USE_LINUX)
static vp_com_serial_config_t
uart0_serial_config = {
#ifdef __linux__
  .itfName          = "/dev/ttyUSB0",
#else // ! __linux__
  .itfName          = "/dev/ser0",
#endif // -> __linux__
  .blocking         = 1,
  .sync             = 1,
  .sync_done        = 0,
  .initial_baudrate = VP_COM_BAUDRATE_115200,
  .baudrate         = VP_COM_BAUDRATE_460800,
};


static vp_com_serial_config_t
uart1_serial_config = {
#ifdef __linux__
  .itfName          = "/dev/ttyUSB1",
#else // ! __linux__
  .itfName          = "/dev/ser1",
#endif // -> __linux__
  .blocking         = 1,
  .sync             = 1,
  .sync_done        = 0,
  .initial_baudrate = VP_COM_BAUDRATE_115200,
  .baudrate         = VP_COM_BAUDRATE_460800,
};
#endif // ! __NDS__


#if defined(USE_BLUEZ)
static vp_com_bluetooth_config_t
bluetooth_config = {
  .itfName   = "bnep0",
#ifdef __linux__
  .localHost = "192.168.2.58",
#else // ! __linux__
  .localHost = "192.168.2.23",
#endif // -> __linux__
  .netmask   = "255.255.255.0",
  .broadcast = "192.168.2.255",
  .gateway   = "192.168.2.0",
  .server    = "192.168.2.0",
  .secure    = 1,
  .passkey   = "1234",
};
#endif // ! defined(USE_BLUEZ)


#ifdef USE_WIFI
static vp_com_wifi_config_t
wifi_conf = {
#ifdef __linux__
  .itfName              = "rausb0",
  .localHost            = "10.10.10.2",
#else // ! __linux__
  .itfName              = "wl0",
  .localHost            = "10.10.10.1",
#endif // -> __linux__
  .netmask              = "255.255.255.0",
  .broadcast            = "10.10.10.255",
  .gateway              = "10.10.10.2",
  .server               = "10.10.10.1",
  .secure               = 1,
  .passkey              = "9F1C3EE11CBA230B27BF1C1B6F",
  .infrastructure       = 1,
};
#endif // ! USE_WIFI


#define _internal_vp_stages_fill_default_config(cfg, config, size) \
  do                                                               \
  {                                                                \
    VP_OS_ASSERT(size >= sizeof(config));                          \
    vp_os_memcpy(cfg, &config, sizeof(config));                    \
  }                                                                \
  while(0)


C_RESULT
vp_stages_fill_default_config(DEFAULT_STAGE_CONFIG config, void *cfg, size_t size)
{
  C_RESULT res = VP_SUCCESS;

  VP_OS_ASSERT(cfg);
  VP_OS_ASSERT(size);

  vp_os_memset(cfg, 0, size);

  switch(config)
    {

#if defined(USE_LINUX)
    case SDL_RAW_QCIF_CONFIG:
      _internal_vp_stages_fill_default_config(cfg, raw_qcif_sdl_config, size);
      break;
    case SDL_RAW_QQCIF_CONFIG:
      _internal_vp_stages_fill_default_config(cfg, raw_qqcif_sdl_config, size);
      break;
    case SDL_DECODING_QCIF_CONFIG:
      _internal_vp_stages_fill_default_config(cfg, decoding_qcif_sdl_config, size);
      break;
    case SDL_RAW_QVGA_CONFIG:
      _internal_vp_stages_fill_default_config(cfg, raw_qvga_sdl_config, size);
      break;
    case SDL_RAW_TWEAKY_QQVGA_CONFIG:
      _internal_vp_stages_fill_default_config(cfg, raw_tweaky_qqvga_sdl_config, size);
      break;
    case SDL_DECODING_QVGA_CONFIG:
      _internal_vp_stages_fill_default_config(cfg, decoding_qvga_sdl_config, size);
      break;
#endif // ! __linux__ || USE_MINGW32 || USE_ELINUX

#if defined(USE_LINUX)
    case UART0_COM_CONFIG:
      _internal_vp_stages_fill_default_config(cfg, uart0_serial_config, size);
      break;
    case UART1_COM_CONFIG:
      _internal_vp_stages_fill_default_config(cfg, uart1_serial_config, size);
      break;
#endif // ! __NDS__

#if defined(USE_BLUEZ)
    case BLUETOOTH_COM_CONFIG:
      _internal_vp_stages_fill_default_config(cfg, bluetooth_config, size);
      break;
#endif // ! defined(USE_BLUEZ)
#ifdef USE_WIFI
    case WIFI_COM_CONFIG:
      _internal_vp_stages_fill_default_config(cfg, wifi_conf, size);
      break;
#endif // ! USE_WIFI
    default:
      res = VP_FAILURE;
      break;
    }

  return res;
}

#undef _internal_vp_stages_fill_default_config

