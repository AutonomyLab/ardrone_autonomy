#include <VLIB/Platform/video_utils.h>

#include <VP_Os/vp_os_malloc.h>
#include <VLIB/video_huffman.h>
#include <VLIB/video_packetizer.h>

huffman_tree_t* huffman_alloc( int32_t num_max_codes, int32_t max_code_length )
{
  int32_t tree_size = sizeof(huffman_tree_t) + num_max_codes * sizeof(huffman_tree_data_t);
  huffman_tree_t* tree = (huffman_tree_t*) vp_os_malloc( tree_size );

  tree->num_used_codes    = 0;
  tree->num_max_codes     = num_max_codes;
  tree->max_code_length   = max_code_length;

  return tree;
}

void huffman_free( huffman_tree_t* tree )
{
  vp_os_free( tree );
}

C_RESULT huffman_add_codes( huffman_tree_t* tree, huffman_code_t* codes, int32_t num_codes )
{
  while( (tree->num_used_codes < tree->num_max_codes) && num_codes-- )
  {
    tree->data[tree->num_used_codes].code = codes++;
    tree->data[tree->num_used_codes].weight = 0;

    tree->num_used_codes ++;
  }

  return C_OK;
}

static inline int32_t compare_codes( huffman_code_t* c1, huffman_code_t* c2, int32_t max_length )
{
  int32_t i1, i2;

  i1 = c1->vlc << (max_length - c1->length);
  i2 = c2->vlc << (max_length - c2->length);

  return i1 < i2 ? -1 : 1;
}

static C_RESULT huffman_sort_codes_internal( huffman_tree_data_t *begin, huffman_tree_data_t *end, int32_t max_length )
{
  huffman_tree_data_t *pivot  = begin;
  huffman_tree_data_t *left   = begin;
  huffman_tree_data_t *right  = end;

  while( right != left )
  {
    if( compare_codes(right->code, left->code, max_length) < 0 )
    {
      huffman_code_t* temp_code = right->code;
      right->code = left->code;
      left->code  = temp_code;

      pivot = pivot == left ? right : left;
    }

    pivot == left ? right-- : left++;
  }

  if( begin < left - 1 )
    huffman_sort_codes_internal( begin, left - 1, max_length );
  if( end > right + 1 )
    huffman_sort_codes_internal( right + 1, end, max_length );

  return C_OK;
}

C_RESULT huffman_sort_codes( huffman_tree_t* tree )
{
  int32_t i;

  // Sort codes (basic qsort implementation)
  huffman_sort_codes_internal( &tree->data[0], &tree->data[tree->num_used_codes-1], tree->max_code_length);

  // Generates weight & counts for each code
  for(i = 0; i < tree->num_used_codes; i++ )
  {
    tree->data[i].weight  = (1 << (1 + tree->max_code_length - tree->data[i].code->length)) - 1;
    tree->data[i].weight += tree->data[i].code->vlc << (1 + tree->max_code_length - tree->data[i].code->length);
  }

  return C_OK;
}

C_RESULT huffman_check_code( huffman_tree_t* tree, uint32_t code, uint32_t length )
{
  int32_t i;
  uint32_t w;

  w  = (1 << (1 + tree->max_code_length - length)) - 1;
  w += code << (1 + tree->max_code_length - length);

  for(i = 0; i < tree->num_used_codes && w != tree->data[i].weight; i++ );

  if( i == tree->num_used_codes )
    return C_FAIL;

  return tree->data[i].weight == w ? C_OK : C_FAIL;
}

int32_t huffman_stream_code( huffman_tree_t* tree, video_stream_t* stream )
{
  huffman_code_t* huffman_code;
  int32_t i, w;
  uint32_t c;

  c = 0;

  video_peek_data( stream, &c, tree->max_code_length );

  w = (c << 1) + 1;

  for(i = 0; i < tree->num_used_codes && w > tree->data[i].weight; i++ );

  huffman_code = tree->data[i].code;

  // Update stream with read data
  video_read_data( stream, &c, huffman_code->length );

  return tree->data[i].code->index;
}
