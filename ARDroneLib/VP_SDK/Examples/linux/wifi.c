#include <Com/com.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_delay.h>
#include <VP_Api/vp_api_error.h>

#include <iwlib.h>
#include <netdb.h>
#include <sys/socket.h>

#include <sys/timeb.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>

#define DRONE_HOST "192.168.1.2"
#define DRONE_PORT 5555

#define TIME_TO_SEND 1024*10
#define SIZE_TO_SEND 1024

static com_socket_t     clt;
static Read             my_read;
static Write            my_write;

static int8_t buffer[SIZE_TO_SEND];
static int32_t size;

static uint32_t timeGetTime()
{
  struct timeval tv; struct timezone tz;

  gettimeofday(&tv,&tz);

  return (uint32_t)( tv.tv_sec*1000 + tv.tv_usec/10000 );
}

int main(void)
{
  com_config_t config;
  vp_com_connection_t connection;

  config.connection = VP_COM_WIFI;
  config.localAdapterName = "rausb0";

  connection.essid = "Drone";
  connection.channel = 10;

  if(FAILED(com_init(&config)))
    PRINT("com_init failed\n");

  if(FAILED(com_passKey("9F1C3EE11CBA230B27BF1C1B6F")))
    PRINT("com_passKey failed\n");

  if(FAILED(com_connect(&connection,1)))
    PRINT("com_connect failed\n");

  clt.socket      = VP_COM_CLIENT;
  clt.port        = DRONE_PORT;
  clt.serverHost  = DRONE_HOST;

  if(SUCCEED(com_open(&clt,&my_read,&my_write)))
  {
    int32_t i = 0;
    float st = timeGetTime();
    float et = timeGetTime();
    float db = 0.0f;
    float d = 0.0f;

    for(i=0; i < TIME_TO_SEND;i++)
    {
      int32_t received;

      PRINT("\r reception n° %d... ",i);

      received  = 0;
      size      = SIZE_TO_SEND;

      while(received != SIZE_TO_SEND)
      {
        my_read(&clt,buffer,&size);
        received += size;
        size = SIZE_TO_SEND - received;
      }

      PRINT("%d bytes           ",received);
    }

    et = timeGetTime();
    d = (et - st) / 1000.0f;
    if(d > 0)
    {
      float tx = SIZE_TO_SEND * TIME_TO_SEND / 1024.0f;
      db = tx / d;
    }
    PRINT("\n---------------\n");
    PRINT("Start Time : %f\n",st);
    PRINT("End Time : %f\n",et);
    PRINT("%d Kbytes sent in %f time\n",SIZE_TO_SEND * TIME_TO_SEND / 1024,d);
    PRINT("Debit: %f\n",db);
    PRINT("\n---------------\n");
  }
  else
  {
    PRINT("snif... pas connecte a la socket\n");
  }

  PRINT("Waiting for disconnection\n");
  vp_delay(5000);

  com_disconnect();
  PRINT("Disconnected\n");

  com_shutdown();

  return 0;
}
