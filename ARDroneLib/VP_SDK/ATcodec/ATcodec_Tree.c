/**
 * @file ATcodec_tree.c
 * @author aurelien.morelle@parrot.com
 * @date 2007/08/20
 * modified on 2010/07/19 by stephane.piskorski.ext@parrot.fr (bug fix+comments)
 */

#include <VP_Os/vp_os_types.h>
#include <VP_Os/vp_os_assert.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <ATcodec/ATcodec_Tree.h>
#include <ATcodec/ATcodec_api.h>
#include <ATcodec/ATcodec_Buffer.h>
#include <ATcodec/ATcodec.h>


/* \todo Remove nodes fields initializations that are not necessary in each function */




static void
ATcodec_Tree_Node_init(ATcodec_Tree_Node_t *node)
{
  VP_OS_ASSERT(node);

  node->data = -1;
  node->depth = 0;
  node->nb_sons = 0;
  vp_os_memset(node->sons,0,sizeof(node->sons));
  node->strkey = 0;
  node->type = ATCODEC_TREE_NODE_TYPE_EMPTY;
  
}



/********************************************************************
 * @brief Initialises the AT command search tree.
 * @param[in] tree The tree to initialise.
*/
void
ATcodec_Tree_init(ATcodec_Tree_t *tree, size_t leaf_size, int nb_start)
{
  ATcodec_Tree_Node_t s_node;

  VP_OS_ASSERT(tree);

  ATcodec_Buffer_init(&tree->leaves, leaf_size, nb_start);
  ATcodec_Buffer_init(&tree->strs, sizeof(char), nb_start);
  ATcodec_Buffer_init(&tree->sons, sizeof(ATcodec_Tree_Node_t), nb_start);

  ATcodec_Tree_Node_init(&s_node);
  tree->root = tree->sons.nbElements;
  ATcodec_Buffer_pushElement(&tree->sons, &s_node);
}




/********************************************************************
 * @param[in] tree Tree in which the node is inserted.
 * @brief Creates a new node in the tree.
*/
static int
ATcodec_Tree_Node_insert(ATcodec_Tree_t *tree, ATCODEC_TREE_NODE_TYPE type, int depth, int str_index, int data, ATcodec_Tree_Node_t *father, int son_index)
{
	/* The new node's value */
	ATcodec_Tree_Node_t s_node;
	/* Index of the new node inside the tree's nodes list */
	int node_index;

  /* Clears the new node */
	vp_os_memset(&s_node,0,sizeof(s_node));
  
  node_index = tree->sons.nbElements;
  if(father)
    father->sons[son_index] = node_index;

  s_node.type = type;
  s_node.depth = depth;
  s_node.strkey = str_index;
  s_node.data = data;
  
  ATcodec_Buffer_pushElement(&tree->sons, &s_node);

  return node_index;
}



/********************************************************************
 * @param[in] tree Tree in which the command is inserted.
 * @brief Inserts an AT command string in the tree.
*/
static int
ATcodec_Tree_Node_search(ATcodec_Tree_t *tree, char *str)
{
  ATcodec_Tree_Node_t *node;
  int searching = 1;
  int depth = 0;
  int str_index;
  int node_index = -1;
  int temp_data;
  int father_index;

  VP_OS_ASSERT(str);
  VP_OS_ASSERT(*str);
  VP_OS_ASSERT(tree);

  str_index = tree->strs.nbElements;
  
  /* Adds the new string to the list of commands */
  ATcodec_Buffer_pushElements(&tree->strs, str, strlen(str)+1);

  /* !! WARNING !! this pointer might be invalid after inserting an element in the tree */
  node = ATcodec_Tree_Node_get(tree, tree->root);

  while(searching)
  {
      switch(node->type)
	{
	
	/* Tree is empty -> we insert the string as the only element */
	case ATCODEC_TREE_NODE_TYPE_EMPTY:
	  {
	    /* Initialise the root element */
			node->type = ATCODEC_TREE_NODE_TYPE_LEAF;
			node->depth = depth;
			node->strkey = str_index; /* str_index = place of the newly inserted string */ 
			node_index = 0;
		/* Stop the search */
			searching = 0;
	  }
	  break;

    /* 
	Parse an intermediate node, which means that previously inserted
	command strings began with the same characters.
	*/
	case ATCODEC_TREE_NODE_TYPE_NODE:
	  {
		  /* Gets the node representing the next character of the command string */
			 node_index = node->sons[(int)*str];
	    
		 /* If there is no such node, the remaining part of the string is stored in the
		 tree as a terminal node (a leaf of the tree).*/
			 if(node_index == -1)
			  {
				node_index = ATcodec_Tree_Node_insert(tree, 
														ATCODEC_TREE_NODE_TYPE_LEAF, 
														depth+1, 
														str_index, /* str_index = place of the newly inserted string */ 
														/*data*/-1, 
														/*father*/node, 
														/*index*/(int)*(str));
				searching = 0;
			  }
		 /* Otherwise, we get the node representing this character and continue scanning the tree.*/
			 node = ATcodec_Tree_Node_get(tree, node_index);
		     depth++;
			 str++;
	  }
	  break;

	/* 
	Scanning arrives on a leaf -> this leaf must be split to store the two
	possible command-string endings.
	*/
	case ATCODEC_TREE_NODE_TYPE_LEAF:
	  {
		/* 
		Retrieves the already-stored string.
		We have to insert as many additional intermediate nodes as there are common
		characters in the two string remainders.
		Two terminal leaves will then be added to store the parts of those strings which differ.
		*/
	    char *leaf_str = (char *)ATcodec_Buffer_getElement(&tree->strs, node->strkey);
	    int i;

	    /* As far as strings endings are the same, insert the intermediate nodes  ...*/
		while(*str && *str == leaf_str[depth])
	      {
			/* Change the current leaf node to an intermediate node and initialises the list of sons. */
			  node->type = ATCODEC_TREE_NODE_TYPE_NODE;
			  for(i = 0 ; i < AT_CODEC_TREE_MAX_SONS ; i++) {  node->sons[i] = -1; }
				
				/* Create a new leaf */
				node_index = ATcodec_Tree_Node_insert(tree, 
													 /*node type*/ATCODEC_TREE_NODE_TYPE_LEAF, 
													 /*node depth*/depth+1, 
													 /*str_index*/node->strkey, /* Points to the old command-string by default */
													 /*data*/node->data, 
													 /*father*/node, 
													 /*son_index*/(int)leaf_str[depth]);
				depth++;

				node = ATcodec_Tree_Node_get(tree, node_index);
				str++;
	      }

		/* If the new string was in fact a subpart of the old one ...*/
		if(*str == leaf_str[depth])
	      {
		   /* Change the leaf to a multileaves */
			node->type = ATCODEC_TREE_NODE_TYPE_MULTILEAVES;
			node->nb_sons = 2;
			temp_data = node->data;
			node->data = -1;
			father_index = node_index;

			/* Insert a leaf for the old longer string */
			node_index = ATcodec_Tree_Node_insert(tree, 
													ATCODEC_TREE_NODE_TYPE_LEAF, 
													++depth, 
													node->strkey, 
													temp_data, 
													/*father*/node, 
													/*son_index*/0);
			
			/* Pointer 'node' is invalid from here because it points
			to a buffer which might have been resized by 'ATcodec_Tree_Node_insert'.
			We fetch a valid address computed from the index.*/
			node = ATcodec_Tree_Node_get(tree,father_index);
						
			/* Insert a leaf for the new, shorter string */
			node_index = ATcodec_Tree_Node_insert(tree, 
													ATCODEC_TREE_NODE_TYPE_LEAF, 
													depth, 
													str_index, 
													/*data*/-1, 
													/*father*/node, 
													/*son_index*/1);
	      }
	    
		/* If the two strings have different endings ...*/
		else
	      {
			/* The old leaf becomes an intermediate node with two daughter leaves */
			node->type = ATCODEC_TREE_NODE_TYPE_NODE;
			for(i = 0 ; i < AT_CODEC_TREE_MAX_SONS ; i++) { node->sons[i] = -1; }
			father_index = node_index;
			
			/* Recreate a leaf for the old command string */
			node_index = ATcodec_Tree_Node_insert(tree, 
													ATCODEC_TREE_NODE_TYPE_LEAF, 
													node->depth+1, 
													node->strkey, /* strkey was propagated from the original leaf */
													node->data, /* keeps the data of the old command string */ 
													/*father*/node, 
													/*son_index*/(int)leaf_str[depth]
											);
		
			/* Pointer 'node' is invalid from here because it points
			to a buffer which might have been resized by 'ATcodec_Tree_Node_insert'.
			We fetch a valid address computed from the index.*/
			node = ATcodec_Tree_Node_get(tree,father_index);

			
			//node->sons[(int)leaf_str[depth]] = node_index;

			/* Create a new leaf for the new command string */
			node_index = ATcodec_Tree_Node_insert(tree, 
													ATCODEC_TREE_NODE_TYPE_LEAF, 
													node->depth+1, 
													str_index, /* str_index = place of the newly inserted string */
													-1, /* data is initialised later in the ATcodec_Tree_insert function */
													/*father*/node, 
													/*son_index*/(int)*str);
	      }
			
		searching = 0;
	  }
	  break;

  /* 
  If there are already at leat two several end-of-command-strings starting with the current character,
  these are represented by a 'multileaves' node and we just add a new son to this node.
  */
	case ATCODEC_TREE_NODE_TYPE_MULTILEAVES:
	  {
	    node_index = ATcodec_Tree_Node_insert(tree, 
												ATCODEC_TREE_NODE_TYPE_LEAF, 
												++depth, 
												str_index, /* str_index = place of the newly inserted string */
												/*data*/-1, /* data is initialised later in the ATcodec_Tree_insert function */
												/*father*/node, 
												/*son_index*/node->nb_sons++
										);
	    searching = 0;
	  }
	  break;
	}
    }

  return node_index;
}


/********************************************************************
 * @param[in] tree Tree in which the command is inserted.
 * @brief Inserts an AT command string in the AT tree with its associated parameters.
*/
int
ATcodec_Tree_insert(ATcodec_Tree_t *tree, char *str, void *data)
{
  ATcodec_Tree_Node_t *node;
  int node_i;

  VP_OS_ASSERT(tree);

  /* 
  Insert the 'command part' (until the '=' symbol, without parameter)
  in the search tree.
  */
	  node_i = ATcodec_Tree_Node_search(tree, str);
	  node = ATcodec_Tree_Node_get(tree, node_i);
	  node->data = tree->leaves.nbElements;
  /* Stores the string containing the parameters */
	  ATcodec_Buffer_pushElement(&tree->leaves, data);

  return node_i;
}



/********************************************************************
 * @param[in] tree Tree from which the node is retrieved.
 * @param[in] node Index of the node whose pointer is to be returned.
 * @brief Gets a pointer the node-th node of the tree.
 * @return Pointer to the asked node.
 * Be extremely careful not to use this pointer anymore after a tree->sons buffer resize.
*/
ATcodec_Tree_Node_t *
ATcodec_Tree_Node_get(ATcodec_Tree_t *tree, int node)
{
  ATcodec_Tree_Node_t * res = (ATcodec_Tree_Node_t *)ATcodec_Buffer_getElement(&tree->sons, node);
  return res;
}


#define AT_CODEC_PRINT_NODE_CHAR_CASE(CARAC_SRC,STR_DST) \
  case CARAC_SRC:                                        \
    PRINT(STR_DST);                                      \
    break

#define AT_CODEC_PRINT_NODE_CHAR(CARAC)                  \
  switch(CARAC)                                          \
  {                                                      \
    AT_CODEC_PRINT_NODE_CHAR_CASE('\r',"<CR>");          \
    AT_CODEC_PRINT_NODE_CHAR_CASE('\n',"<LF>");          \
    AT_CODEC_PRINT_NODE_CHAR_CASE('\0',"<\\0>");         \
    AT_CODEC_PRINT_NODE_CHAR_CASE(' ',"< >");            \
    default:                                             \
      PRINT("%c", CARAC);                                \
      break;                                             \
  }

void
ATcodec_Tree_Node_print(ATcodec_Tree_t *tree, ATcodec_Tree_Node_t *node)
{
#ifdef ATCODEC_DEBUG
  static int tab = 0;
  int i, j;
  char *sta;

  if(node->type == ATCODEC_TREE_NODE_TYPE_LEAF)
    {
	 /* Get the end-of-command-string stored on the leaf */
      sta = ATcodec_Buffer_getElement(&tree->strs, node->strkey);
      for(j = 0 ; j < tab ; j++)
	ATCODEC_PRINT(" . ");
      do
	{
	  /* Print the end-of-command-string */
	  AT_CODEC_PRINT_NODE_CHAR(*sta);
	}
      while(*sta++);
      ATCODEC_PRINT("\"\n");
    }
  else if(node->type == ATCODEC_TREE_NODE_TYPE_NODE)
    {
      /* For each possible character of the ascii map, check if a command-string is stored. */
		for(i = 0 ; i < AT_CODEC_TREE_MAX_SONS ; i++)
	   {
		if(node->sons[i] != -1)
	    {
	      for(j = 0 ; j < tab ; j++)
		ATCODEC_PRINT(" . ");
	      AT_CODEC_PRINT_NODE_CHAR((char)i);
	      ATCODEC_PRINT("\n");
	      tab++;
	      ATcodec_Tree_Node_print(tree, ATcodec_Tree_Node_get(tree, node->sons[i]));
	      tab--;
	    }
	}
    }
  else if(node->type == ATCODEC_TREE_NODE_TYPE_MULTILEAVES)
    {
      for(i = 0 ; i < node->nb_sons ; i++)
	{
	  tab++;
	  ATcodec_Tree_Node_print(tree, ATcodec_Tree_Node_get(tree, node->sons[i]));
	  tab--;
	}
    }
#endif // > ATCODEC_DEBUG
}


void
ATcodec_Tree_print(ATcodec_Tree_t *tree)
{
#ifdef ATCODEC_DEBUG
  ATcodec_Tree_Node_print(tree, ATcodec_Tree_Node_get(tree, tree->root));
#endif // > ATCODEC_DEBUG
}

