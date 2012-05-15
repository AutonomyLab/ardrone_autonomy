/**
 * @file ATcodec_Buffer.h
 * @author aurelien.morelle@parrot.fr
 * @date 2006/12/06
 */

#ifndef _AT_CODEC_BUFFER_INCLUDE_
#define _AT_CODEC_BUFFER_INCLUDE_

typedef struct _ATcodec_Buffer_
{
	void *data;
	void *topElement;

	int nbElements;

	size_t elementSize;
	size_t totalSize;
}
ATcodec_Buffer_t;


void
ATcodec_Buffer_init (ATcodec_Buffer_t *buffer, size_t elementSize, int nbElementsStart);

void
ATcodec_Buffer_destroy (ATcodec_Buffer_t *buffer);


void
ATcodec_Buffer_popElement (ATcodec_Buffer_t *buffer, void *dest);

void
ATcodec_Buffer_justPopElement (ATcodec_Buffer_t *buffer);


void
ATcodec_Buffer_pushElement (ATcodec_Buffer_t *buffer, const void *element);

void
ATcodec_Buffer_pushElements (ATcodec_Buffer_t *buffer, const void *elements, int nb);


void *
ATcodec_Buffer_topElement (ATcodec_Buffer_t *buffer);

void *
ATcodec_Buffer_getElement (ATcodec_Buffer_t *buffer, int index);


#endif // ! _AT_CODEC_BUFFER_INCLUDE_
