/**
 *  \brief    VP Stages. Com stage declaration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \author   Aurelien Morelle <aurelien.morelle@parrot.fr>
 *  \author   Thomas Landais <thomas.landais@parrot.fr>
 *  \version  2.0
 *  \date     first release 16/03/2007
 *  \date     modification  19/03/2007
 */

#include <VP_Stages/vp_stages_io_com.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>

C_RESULT
vp_stages_input_com_stage_open(vp_stages_input_com_config_t *cfg)
{
  C_RESULT res;

  res = vp_com_init(cfg->com);

  if( VP_SUCCEEDED(res) )
  {
    vp_com_local_config(cfg->com, cfg->config);

    if(cfg->connection && !cfg->connection->is_up)
    {
      res = vp_com_connect(cfg->com, cfg->connection, 1);
    }
  }

  if( VP_SUCCEEDED(res) )
    res = vp_com_open(cfg->com, &cfg->socket, &cfg->read, 0);

  if( VP_SUCCEEDED(res) )
  {
    if(cfg->socket.type == VP_COM_SERVER)
    {
      res = vp_com_wait_connections(cfg->com, &cfg->socket, &cfg->socket_client, 1);
    }
    else
    {
      vp_os_memcpy(&cfg->socket_client, &cfg->socket, sizeof(vp_com_socket_t));
    }
    vp_com_sockopt(cfg->com, &cfg->socket_client, cfg->sockopt);
  }

  // \todo test
  return res;
}


C_RESULT
vp_stages_input_com_stage_transform(vp_stages_input_com_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  vp_os_mutex_lock(&out->lock);

  if(out->status == VP_API_STATUS_INIT)
  {
    out->numBuffers = 1;
    out->size = cfg->buffer_size;
    out->buffers = (uint8_t **) vp_os_malloc (sizeof(uint8_t *)+out->size*sizeof(uint8_t));
    out->buffers[0] = (uint8_t *)(out->buffers+1);
    out->indexBuffer = 0;
    // out->lineSize not used

    out->status = VP_API_STATUS_PROCESSING;
  }

  if(out->status == VP_API_STATUS_PROCESSING && cfg->read != NULL)
  {
    out->size = cfg->buffer_size;

    if(VP_FAILED(cfg->read(&cfg->socket_client, out->buffers[0], &out->size)))
      out->status = VP_API_STATUS_ERROR;
  }

  vp_os_mutex_unlock(&out->lock);

  return (VP_SUCCESS);
}


C_RESULT
vp_stages_input_com_stage_close(vp_stages_input_com_config_t *cfg)
{
  // \todo test

  // \todo Faire ca dans le transform en detectant la fin d'une connection
  vp_com_close(cfg->com, &cfg->socket_client);

  if(cfg->socket.type == VP_COM_SERVER)
    vp_com_close(cfg->com, &cfg->socket);

  if(cfg->connection && cfg->connection->is_up)
    vp_com_disconnect(cfg->com);

  vp_com_shutdown(cfg->com);

  return (VP_SUCCESS);
}


C_RESULT
vp_stages_output_com_stage_open(vp_stages_output_com_config_t *cfg)
{
  C_RESULT res;

  res = vp_com_init(cfg->com);

  if( VP_SUCCEEDED(res) )
  {
    vp_com_local_config(cfg->com, cfg->config);

    if(cfg->connection && !cfg->connection->is_up)
    {
      res = vp_com_connect(cfg->com, cfg->connection, 1);
    }
  }

  if( VP_SUCCEEDED(res) && VP_FAILED(vp_com_open(cfg->com, &cfg->socket, 0, &cfg->write)))
    res = C_FAIL;

  if( VP_SUCCEEDED(res) )
  {
    if(cfg->socket.type == VP_COM_SERVER)
    {
      res = vp_com_wait_connections(cfg->com, &cfg->socket, &cfg->socket_client, 1);
    }
    else
    {
      vp_os_memcpy(&cfg->socket_client, &cfg->socket, sizeof(vp_com_socket_t));
    }

    vp_com_sockopt(cfg->com, &cfg->socket_client, cfg->sockopt);
  }

  // \todo test
  return res;
}


C_RESULT
vp_stages_output_com_stage_transform(vp_stages_output_com_config_t *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  vp_os_mutex_lock(&out->lock);

  if(out->status == VP_API_STATUS_INIT)
  {
    out->status = VP_API_STATUS_PROCESSING;
  }

  // \todo test
  if(out->status != VP_API_STATUS_ERROR)
  {
    out->size = in->size;
    if(in->size > 0 && cfg->write != NULL)
    {
      cfg->write(&cfg->socket_client, &in->buffers[in->indexBuffer][0], &out->size);
    }

    out->status = in->status;
  }
  // \todo test

  vp_os_mutex_unlock(&out->lock);

  return (VP_SUCCESS);
}


C_RESULT
vp_stages_output_com_stage_close(vp_stages_output_com_config_t *cfg)
{
  // \todo Faire ca dans le transform en detectant la fin d'une connection
  vp_com_close(cfg->com, &cfg->socket_client);

  if(cfg->socket.type == VP_COM_SERVER)
    vp_com_close(cfg->com, &cfg->socket);

  // \todo test
  if(cfg->connection && cfg->connection->is_up)
     vp_com_disconnect(cfg->com);

  vp_com_shutdown(cfg->com);

  return (VP_SUCCESS);
}
