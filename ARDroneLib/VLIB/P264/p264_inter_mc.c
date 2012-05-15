#include <VLIB/P264/p264_inter_mc.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_assert.h>
// partition dimension
typedef struct _part_dim_t {
  uint32_t x;
  uint32_t y;
} part_dim_t;

// LUT table to retrieve partition dimension. part_dim[inter_partition_mode_t]
static part_dim_t part_dim[NB_PARTITION] =
{
  {16,16},{16,8},{8,16},{8,8},{8,4},{4,8},{4,4}
};

// return pixel(i,j) from picture. if (i,j) is out of the image, boundary pixel are returned.
static uint8_t get_pixel(int32_t x, int32_t y, uint8_t* picture, uint32_t linesize, uint32_t picture_width, uint32_t picture_height)
{
  if (x<0 && y<0)
  {
    // return top left reference pixel
    return  *picture;
  }
  else if (x>=(int32_t)picture_width && y>=(int32_t)picture_height)
  {
    // return bottom right pixel
    return  *(picture + picture_height*linesize - 1);
  }
  else if (x>=(int32_t)picture_width && y<0)
  {
    // return top right ref pixel
    return  *(picture + picture_width - 1);
  }
  else if (x<0 && y>=(int32_t)picture_height)
  {
    // return bottom left ref pixel
    return  *(picture + (picture_height-1)*linesize );
  }
  else if (x<0)
  {
    // return first column pixel
    return  *(picture+y*linesize);
  }
  else if (y<0)
  {
    // return first line pixel
    return  *(picture+x);
  }
  else if (x>=(int32_t)picture_width)
  {
    // return last column pixel
    return  *(picture + y*linesize + picture_width - 1);
  }
  else if (y>=(int32_t)picture_height)
  {
    // return last line pixel
    return  *(picture + (picture_height-1)*linesize + x);
  }
  else
  {
    //return ref pixel
    return  *(picture + y*linesize + x);
  }
}

//  ________________           ________________
// | ref_picture              | picture being decoding
// |  _________               |
// | | refblock|              |
// | |_________|        ===>  |
// |       |                  |  _________
// |       | (mv)             | | refblock|     [Motion compensation]
// |       V                  | |_________|
// |________________          |________________


// move a block from picture_ref to picture at position (x,y).
void p264_inter_mc_luma (inter_partition_mode_t partition, MV_XY_t mv,uint8_t *picture_ref , uint8_t *picture, uint32_t x, uint32_t y, uint32_t picture_width, uint32_t picture_height, uint32_t linesize)
{
  // partition : partition mode of the block to be moved (16x16, 16x8, 8x16, ...)
  // mv : motion vector (in half pixel unit)
  // picture_ref : picture reference
  // picture : the dest picture to be motion compensated.
  // x, y : coordinates of the destination block
  // picture_width, picture_height : picture dimensions
  // linesize : pixel line length inside picture. (it is assumed that picture and picture_ref have the same resolution/format)
  // Note : this function supports only integer MV on luma component

  uint8_t *picture_ref_start = picture_ref;
  uint8_t *picture_start = picture;
  //uint8_t temp_pixel;
  // compute coordinates of ref_block
  int32_t x_ref,y_ref;
  int32_t block_dim_x,block_dim_y;

  x_ref = ((int32_t)x)+mv.x;
  y_ref = ((int32_t)y)+mv.y;

  // retrieve block dimensions
  block_dim_x = part_dim[partition].x;
  block_dim_y = part_dim[partition].y;

  // jump to destination position in picture
  picture = picture_start + y*linesize + x;
  // jump to source position in picture_ref
  picture_ref = picture_ref_start + y_ref*linesize + x_ref;

  if ( x_ref > 0 &&
      (x_ref+block_dim_x) <= (int32_t)picture_width &&
       y_ref > 0 &&
      (y_ref+block_dim_y) <= (int32_t)picture_height)
  {
    // ref block is not out of boundary
    // make a simple copy on the luma sample
    while (block_dim_y--)
    {
      // copy line
      vp_os_memcpy(picture,picture_ref,block_dim_x);
      // jump to next line
      picture_ref += linesize;
      picture += linesize;
    }
  }
  else
  {
    // ref block is out of boundary
    // ref picture boundary pixel are extended when necessary
    int32_t i,j;
    for (j=y_ref;j<(block_dim_y+y_ref);j++)
    {
      for (i=x_ref;i<(block_dim_x+x_ref);i++)
      {
        *picture++ = get_pixel(i,j,picture_ref_start,linesize,picture_width,picture_height);
      }
      picture += linesize - block_dim_x;
    }
  }
}


// chroma interpollation
//
// A -  -  -  -  -  -  -  B
// -        |
// -       (dy)
// -        |
// - <-dx-> a <-(8-dx)->
// -        |
// -      (8-dy)
// -        |
// C -  -  -  -  -  -  -  D
//
// a = round([(8-dx)(8-dy)A + dx(8-dy)B + (8-dx)dyC + dxdyD]/64)
// if a is an half pel the formula is ://
// A a B
// b c
// C   D
//
// a = round[(A+B)>>1]
// b = round[(A+C)>>1]
// c = round[(A+B+C+D)>>2]

void p264_inter_mc_chroma (inter_partition_mode_t partition, MV_XY_t mv,uint8_t *picture_ref , uint8_t *picture, uint32_t x, uint32_t y, uint32_t picture_width, uint32_t picture_height, uint32_t linesize)
{
  // partition : partition mode of the block to be moved (16x16, 16x8, 8x16, ...)
  // mv : motion vector (in 1/2 pixel)
  // picture_ref : picture reference
  // picture : the dest picture to be motion compensated.
  // x,y : coordinates of the destination block
  // linesize : pixel line length inside picture. (it is assumed that picture and picture_ref have the same resolution/format
  // Note : this function supports only half pixel MV on chroma

  //uint8_t *picture_start = picture;

  // compute integer coordinates of ref_block
  int32_t x_ref,y_ref;
  bool_t x_half_pel,y_half_pel;
  uint32_t scale=0;
  int32_t block_dim_x,block_dim_y;

  if (mv.x > 0)
    x_ref = ((int32_t)x)+mv.x/2;
  else
    x_ref = ((int32_t)x)+(mv.x-1)/2;

  if ((mv.x&0x01) == 0)
    x_half_pel = FALSE;
  else
  {
    scale++;
    x_half_pel = TRUE;
  }

  if (mv.y > 0)
    y_ref = ((int32_t)y)+mv.y/2;
  else
    y_ref = ((int32_t)y)+(mv.y-1)/2;

  if ((mv.y&0x01) == 0)
    y_half_pel = FALSE;
  else
  {
    y_half_pel = TRUE;
    scale++;
  }

  // jump to destination position in picture
  picture += y*linesize + x;

  // retrieve block dimensions
  block_dim_x = part_dim[partition].x>>1;
  block_dim_y = part_dim[partition].y>>1;

  // interpolate chroma from picture_ref to picture
  int32_t i,j;
  uint32_t pel;
  for (j=y_ref;j<(block_dim_y+y_ref);j++)
  {
    for (i=x_ref;i<(block_dim_x+x_ref);i++)
    {
      // add pixel A
      pel = get_pixel(i,j,picture_ref,linesize,picture_width,picture_height);
      //pel = *picture_ref;
      // add pixel B
      if (x_half_pel == TRUE)
        //pel += *(picture_ref+1);
        pel += get_pixel(i+1,j,picture_ref,linesize,picture_width,picture_height);
      // add pixel C
      if (y_half_pel == TRUE)
        //pel += *(picture_ref+linesize);
        pel += get_pixel(i,j+1,picture_ref,linesize,picture_width,picture_height);
      // add pixel D
      if (y_half_pel == TRUE && x_half_pel == TRUE)
        //pel += *(picture_ref+linesize+1);
        pel += get_pixel(i+1,j+1,picture_ref,linesize,picture_width,picture_height);

      // scale pel and set picture pixel
      *picture++ = pel>>scale;
    }
    picture += linesize - block_dim_x;
  }
}
