/**
 * @file ATcodec_tree.c
 * @author aurelien.morelle@parrot.com
 * @date 2007/08/20
 */


/*
------- Builds a tree which factorises the beginning of AT command strings ------
----  and allows quick search of the callback corresponding to an AT command ----

Example:

A
 . T
 .  . *
 .  .  . A
 .  .  .  . AT*ANIM=<\0>"
 .  .  . C
 .  .  .  . A
 .  .  .  .  . D
 .  .  .  .  .  . AT*CAD=<\0>"
 .  .  .  .  . P
 .  .  .  .  .  . AT*CAP=<\0>"
 .  .  .  . O
 .  .  .  .  . M
 .  .  .  .  .  . AT*COMWDG=<\0>"
 .  .  .  .  . N
 .  .  .  .  .  . AT*CONFIG=<\0>"
 .  .  .  . T
 .  .  .  .  . AT*CTRL=<\0>"
 .  .  . F
 .  .  .  . AT*FTRIM=<\0>"
 .  .  . G
 .  .  .  . AT*GAIN=<\0>"
 .  .  . L
 .  .  .  . AT*LED=<\0>"
 .  .  . M
 .  .  .  . I
 .  .  .  .  . AT*MISC=<\0>"
 .  .  .  . T
 .  .  .  .  . AT*MTRIM=<\0>"
 .  .  . P
 .  .  .  . C
 .  .  .  .  . AT*PCMD=<\0>"
 .  .  .  . O
 .  .  .  .  . AT*POL=<\0>"
 .  .  .  . W
 .  .  .  .  . AT*PWM=<\0>"
 .  .  . R
 .  .  .  . AT*REF=<\0>"
 .  .  . V
 .  .  .  . I
 .  .  .  .  . S
 .  .  .  .  .  . O
 .  .  .  .  .  .  . AT*VISO=<\0>"
 .  .  .  .  .  . P
 .  .  .  .  .  .  . AT*VISP=<\0>"
 .  .  . Z
 .  .  .  . AT*ZAP=<\0>"
E
 . ERROR<CR><\0>"
O
 . OK<CR><\0>"

 ----------------------------------------------------------*/

#ifndef _AT_CODEC_TREE_INCLUDE_
#define _AT_CODEC_TREE_INCLUDE_


//#include <ATcodec/ATcodec_api.h>
#include <ATcodec/ATcodec_Buffer.h>
#include <ATcodec/ATcodec.h>

#define AT_CODEC_TREE_MAX_SONS 256

typedef enum _ATCODEC_TREE_NODE_TYPE_
{
  ATCODEC_TREE_NODE_TYPE_EMPTY, // just for tree root, when nothing has been added yet
  ATCODEC_TREE_NODE_TYPE_NODE,  
  ATCODEC_TREE_NODE_TYPE_LEAF,
  ATCODEC_TREE_NODE_TYPE_MULTILEAVES,
}
ATCODEC_TREE_NODE_TYPE;


typedef struct _ATcodec_Tree_Node_
{
  ATCODEC_TREE_NODE_TYPE type;
  int depth;
  int strkey;

  int sons[AT_CODEC_TREE_MAX_SONS];

  int nb_sons; // only when type is MULTILEAVES

  int data; // only when type is LEAF; index of the parameter string in the 'data' buffer.
}
ATcodec_Tree_Node_t;


typedef struct _ATcodec_Tree_
{
  int root;

  ATcodec_Buffer_t leaves;
  ATcodec_Buffer_t strs;
  ATcodec_Buffer_t sons;
}
ATcodec_Tree_t;


void
ATcodec_Tree_init(ATcodec_Tree_t *tree, size_t leaf_size, int nb_start);


int
ATcodec_Tree_insert(ATcodec_Tree_t *tree, char *str, void *data);


ATcodec_Tree_Node_t *
ATcodec_Tree_Node_get(ATcodec_Tree_t *tree, int node);


void
ATcodec_Tree_print(ATcodec_Tree_t *tree);


/* void */
/* ATcodec_Tree_remove(ATcodec_Tree_t *tree, char *str, void *data); */


/* void */
/* ATcodec_Tree_destroy(ATcodec_Tree_t *tree); */


#endif // -> _AT_CODEC_TREE_INCLUDE_

