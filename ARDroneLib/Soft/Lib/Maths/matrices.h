/**
 *  \file     matrices.h
 *  \brief    Matrices library used by ARDrone
 *  \author   Jean-Baptiste Lanfrey <jean-baptiste.lanfrey@parrot.com>
 *  \version  1.0
 */

#ifndef _MATRICES_H_
#define _MATRICES_H_

#include <VP_Os/vp_os_types.h>

typedef struct _matrix33_t
{
  float32_t m11;
  float32_t m12;
  float32_t m13;
  float32_t m21;
  float32_t m22;
  float32_t m23;
  float32_t m31;
  float32_t m32;
  float32_t m33;
} matrix33_t;

typedef struct _vector31_t {
  union {
    float32_t v[3];
    struct
    {
      float32_t x;
      float32_t y;
      float32_t z;
    };
  };
} vector31_t;

typedef union _vector21_t {
  float32_t v[2];
  struct
  {
    float32_t x;
    float32_t y;
  };
} vector21_t;

extern const matrix33_t matrix_id3;
extern const vector31_t vector31_zero;
extern const vector31_t vector31_z;

// TODO Documentation

// Multiplies two matrices m1 & m2. Stores result in out.
void mul_mat( matrix33_t* out, matrix33_t *m1, matrix33_t *m2 );


void add_mat( matrix33_t* out, matrix33_t *m1, matrix33_t *m );


void mulvec_mat( vector31_t* out, matrix33_t *m1, vector31_t *v1 );


void transpose_mat( matrix33_t *out, matrix33_t *m1 );


void mulconst_mat( matrix33_t *out, matrix33_t *m1, float32_t k );


void cross_vec( vector31_t* out, vector31_t *v1, vector31_t *v2 );


void dot_vec( float32_t* out, const vector31_t *v1, const vector31_t *v2 );


void add_vec( vector31_t* out, vector31_t *v1, vector31_t *v2 );


void mulconst_vec( vector31_t *out, vector31_t *V1, float32_t k );


void skew_anti_symetric_vec( matrix33_t *out, vector31_t *v );


void vex( vector31_t *out, matrix33_t *m );


float32_t norm_vec( vector31_t *v );


bool_t normalize_vec( vector31_t* v );


#endif // _MATRICES_H_
