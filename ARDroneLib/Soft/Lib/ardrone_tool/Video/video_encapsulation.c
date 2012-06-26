#include <stdio.h>
#include <VP_Os/vp_os_types.h>
#include <video_encapsulation.h>

C_RESULT init_parrot_video_encapsulation_header(parrot_video_encapsulation_t * header)
{
	header->signature[0]='P'; /* Parrot */
	header->signature[1]='a';
	header->signature[2]='V'; /* Video */
	header->signature[3]='E'; /* Encapsulation */

	header->version = PAVE_CURRENT_VERSION;
	return C_OK;
}

int pave_is_same_frame(parrot_video_encapsulation_t * header1 , parrot_video_encapsulation_t * header2 )
{
  int res;
  
  if (!header1) { return 0; }
  if (!header2) { return 0; }
 
  res = ( (header1->stream_id    == header2->stream_id)  &&
          (header1->frame_number == header2->frame_number  )  );
         
  return ( res );
}

void dumpPave (parrot_video_encapsulation_t *PaVE)
{
  printf ("Signature : \"%c%c%c%c\" [0x%02x][0x%02x][0x%02x][0x%02x]\n", PaVE->signature[0], PaVE->signature[1],
          PaVE->signature[2], PaVE->signature[3], PaVE->signature[0], PaVE->signature[1], PaVE->signature[2], PaVE->signature[3]);
  printf ("Frame Type / Number : %s : %d : slice %d/%d\n",
     (PaVE->frame_type == FRAME_TYPE_P_FRAME) ? "P-Frame" : ((PaVE->frame_type == FRAME_TYPE_I_FRAME) ? "I-Frame" : "IDR-Frame"),
     PaVE->frame_number,
     PaVE->slice_index+1,
     PaVE->total_slices);
  printf ("Codec : %s\n", (PaVE->video_codec == CODEC_MPEG4_VISUAL) ? "MP4" : ((PaVE->video_codec == CODEC_MPEG4_AVC) ? "H264" : "Unknown"));
  printf ("StreamID : %d \n", PaVE->stream_id);
  printf ("Encoded dims : %d x %d\n", PaVE->encoded_stream_width, PaVE->encoded_stream_height);
  printf ("Display dims : %d x %d\n", PaVE->display_width, PaVE->display_height);
  printf ("Header size  : %d\n", PaVE->header_size);
  printf ("Payload size : %d\n", PaVE->payload_size);
  printf ("Control : 0x%08x\n", PaVE->control);
}
