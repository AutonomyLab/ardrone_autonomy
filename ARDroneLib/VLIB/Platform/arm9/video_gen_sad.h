#ifndef _VIDEO_GEN_SAD_H_
#define _VIDEO_GEN_SAD_H_

// Returns the sum of absolute difference between reference and source
typedef uint32_t (*sad_fn)(void* ref, void* src);

//
// bytes1 : number of bytes for ref (1-bytes)
// offset_ref : number of elements to skip to process next row ( 176 in QCIF for example, or at least row_length for a contiguous array )
// square_length : number of elements per row or column
// offset_ref & offset_src must be less than 1024
//
sad_fn gen_sad( uint32_t* buffer, uint32_t buffer_length,
                uint32_t bytes_per_entry_ref, uint32_t bytes_per_entry_src,
                uint16_t offset_ref, uint16_t offset_src,
                uint32_t square_length);

/*
  Usage:

int32_t sad_ref[] = { 1, 2, 3, 4, 5, 6, 7, 8,
                      1, 2, 3, 4, 5, 6, 7, 8,
                      1, 2, 3, 4, 5, 6, 7, 8,
                      1, 2, 3, 4, 5, 6, 7, 8,
                      1, 2, 3, 4, 5, 6, 7, 8,
                      1, 2, 3, 4, 5, 6, 7, 8,
                      1, 2, 3, 4, 5, 6, 7, 8,
                      1, 2, 3, 4, 5, 6, 7, 8 };

int16_t sad_mem[] =  { 9, 2, 3, 4, 5, 6, 7, 8,
                      1, 2, 3, 4, 5, 6, 7, 8,
                      1, 2, 3, 4, 5, 6, 7, 8,
                      1, 2, 3, 4, 5, 6, 7, 8,
                      1, 2, 3, 12, 5, 6, 7, 8,
                      1, 2, 3, 4, 5, 6, 7, 8,
                      1, 2, 3, 4, 5, 6, 7, 8,
                      1, 2, 3, 4, 5, 6, 7, 8 };

  uint32_t ramcode_buffer_sad[256];
  sad_fn sad_8_32_16 = gen_sad( ramcode_buffer_sad, 256, MEM_FMT_32, MEM_FMT_16, 8, 8, 8);

  uint32_t sum = sad_8_32_16(sad_ref, sad_mem);
*/


#endif // _VIDEO_GENSAD_H_
