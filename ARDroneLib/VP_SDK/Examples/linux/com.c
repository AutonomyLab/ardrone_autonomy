#include <Com/com.h>
#include <VP_Api/vp_api_error.h>
#include <Examples/common/common.h>

#include <stdio.h>
#include <unistd.h>

int main(int argc,char* argv[])
{
  com_config_t cfg;
  vp_com_connection_t conn;

  printf("---------- Network Adapter Inquiry ----------\n");
  com_networkAdapterLookUp(COM_BLUETOOTH,adapterinquiry_df);
  printf("---------------------------------------------\n");

  cfg.connection        = COM_BLUETOOTH;
  cfg.localAdapterName  = DEVICENAME;
  cfg.localIpAddress    = LOCALHOST;
  cfg.localIpSubmask    = "255.255.255.0";

  printf("--------------- INITIALISATION --------------\n");
  if(FAILED(com_init(&cfg)))
  {
    printf("Failed to init\n");
    com_shutdown();
    return -1;
  }
/*
  printf("----------- Remote Device Inquiry -----------\n");
  com_inquire(deviceinquiry,60000);
  printf("---------------------------------------------\n");
*/
  
  printf("---------- Tentative de connection ----------\n");
  com_strToAddress(BTADDR_SERVER,&conn.address);
  com_passKey("1234");
  if(FAILED(com_connect(&conn,1)))
  {
    printf("Failed to connect\n");
    com_shutdown();
    return -1;
  }

  printf("Connected to BTT\n");

  printf("---------- BNEP Connection ----------\n");
  {// Test d'execution bnep
    com_socket_t socket;
    Read read;

    socket.socket     = VP_COM_CLIENT;
    socket.protocol   = COM_BNEP;
    socket.serverHost = SERVERHOST;
    socket.port       = BTADDR_PORT;
  
    if(SUCCEED(com_open(&socket,&read,0)))
    {
      char buffer[10];
      int r = 10;
      printf("Connection BNEP succeeded\n");
      if(SUCCEED(read((void*)&socket,buffer,&r)))
        printf("Read succeed\n");
  
      printf("socket closed\n");
      com_close(&socket);
    }
  }

  sleep(1);

  printf("---------- RFCOMM Connection ----------\n");
  {// Test d'execution rfcomm
    com_socket_t socket;
    Write write;

    socket.socket    = VP_COM_CLIENT;
    socket.protocol  = COM_RFCOMM;
    socket.scn       = BTADDR_SCN;
  
    if(SUCCEED(com_open(&socket,0,&write)))
    {
      char buffer[10];
      int r = 10;
      printf("Connection RFCOMM succeeded\n");
      if(SUCCEED(write((void*)&socket,buffer,&r)))
        printf("Write succeed\n");
  
      printf("socket closed\n");
      com_close(&socket);
    }
  }

  sleep(1);

  printf("Deconnection\n");
  com_disconnect();

  printf("End of program\n");
  com_shutdown();

  return 0;
}
