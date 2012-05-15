#ifndef _VIDEO_HUFFMAN_H_
#define _VIDEO_HUFFMAN_H_

#include <VP_Os/vp_os_types.h>
#include <VLIB/video_controller.h>

# pragma pack (1)

typedef struct _huffman_code_t {
  int32_t index;
  union {
    struct {
      uint8_t   length;
      uint32_t  vlc:24;
    };
    int32_t code;
  };
} huffman_code_t;

# pragma pack () // resets packsize to default value

typedef struct _huffman_tree_data_t {
  huffman_code_t* code;
  int32_t         weight;
} huffman_tree_data_t;

typedef struct _huffman_tree_t {
  int32_t num_used_codes;
  int32_t num_max_codes;
  int32_t max_code_length;

  huffman_tree_data_t data[];
} huffman_tree_t;

huffman_tree_t* huffman_alloc( int32_t num_max_codes, int32_t max_code_length );
void huffman_free( huffman_tree_t* tree );

C_RESULT huffman_add_codes( huffman_tree_t* tree, huffman_code_t* codes, int32_t num_codes );
C_RESULT huffman_sort_codes( huffman_tree_t* tree );

C_RESULT huffman_check_code( huffman_tree_t* tree, uint32_t code, uint32_t length );
int32_t  huffman_stream_code( huffman_tree_t* tree, video_stream_t* stream );

#endif // _VIDEO_HUFFMAN_H_
