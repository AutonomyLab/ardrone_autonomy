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
extern const matrix33_t matrix_null3;
extern const vector31_t vector31_zero;
extern const vector31_t vector31_z;

// TODO Documentation

// Multiplies two matrices m1 & m2. Stores result in out.
void mul_mat( matrix33_t* out, matrix33_t *m1, matrix33_t *m2 );


void add_mat( matrix33_t* out, matrix33_t *m1, matrix33_t *m );


void mulvec_mat( vector31_t* out, matrix33_t *m1, vector31_t *v1 );


void transpose_mat( matrix33_t *out, matrix33_t *m1 );


void mulconst_mat( matrix33_t *out, matrix33_t *m1, float32_t k );


void det_mat3(float32_t *out, matrix33_t *m1);


void comatrice33(matrix33_t *out, matrix33_t *m1);


void inv_mat33(matrix33_t *out, matrix33_t* m1);


void cross_vec( vector31_t* out, vector31_t *v1, vector31_t *v2 );


void dot_vec( float32_t* out, const vector31_t *v1, const vector31_t *v2 );


void add_vec( vector31_t* out, vector31_t *v1, vector31_t *v2 );


void mulconst_vec( vector31_t *out, vector31_t *V1, float32_t k );


void skew_anti_symetric_vec( matrix33_t *out, vector31_t *v );


void vex( vector31_t *out, matrix33_t *m );


float32_t norm_vec( vector31_t *v );


bool_t normalize_vec( vector31_t* v );


void display_matrix33(matrix33_t *m1);
void display_vector31(vector31_t *v1);


// 4x4 matrix

typedef struct _matrix44_t
{
  float32_t m11;
  float32_t m12;
  float32_t m13;
  float32_t m14;
  float32_t m21;
  float32_t m22;
  float32_t m23;
  float32_t m24;
  float32_t m31;
  float32_t m32;
  float32_t m33;
  float32_t m34;
  float32_t m41;
  float32_t m42;
  float32_t m43;
  float32_t m44;
} matrix44_t;

typedef struct _vector41_t {
  union {
    float32_t v[4];
    struct
    {
      float32_t x1;
      float32_t x2;
      float32_t x3;
      float32_t x4;
    };
  };
} vector41_t;

extern const matrix44_t matrix_id4;
extern const matrix44_t matrix_null4;
extern const vector41_t vector41_zero;

void mul_mat44( matrix44_t* out, matrix44_t *m1, matrix44_t *m2 );
void add_mat44( matrix44_t* out, matrix44_t *m1, matrix44_t *m2 );
void mulvec_mat4( vector41_t* out, matrix44_t *m1, vector41_t *v1 );
void transpose_mat44( matrix44_t *out, matrix44_t *m1 );
void mulconst_mat44( matrix44_t *out, matrix44_t *m1, float32_t k );
void add_vec41( vector41_t* out, vector41_t *v1, vector41_t *v2 );
void mulconst_vec41( vector41_t *out, vector41_t *v1, float32_t k );
void comatrice44(matrix44_t *out, matrix44_t *m1);
void det_mat4(float32_t *out, matrix44_t *m1);
void inv_mat44(matrix44_t *out, matrix44_t* m1);
void display_matrix44(matrix44_t *m1);
void display_vector41(vector41_t *v1);



// 6x6 matrices

typedef struct _matrix66_t
{
  float32_t m11;
  float32_t m12;
  float32_t m13;
  float32_t m14;
  float32_t m15;
  float32_t m16;
  float32_t m21;
  float32_t m22;
  float32_t m23;
  float32_t m24;
  float32_t m25;
  float32_t m26;
  float32_t m31;
  float32_t m32;
  float32_t m33;
  float32_t m34;
  float32_t m35;
  float32_t m36;
  float32_t m41;
  float32_t m42;
  float32_t m43;
  float32_t m44;
  float32_t m45;
  float32_t m46;
  float32_t m51;
  float32_t m52;
  float32_t m53;
  float32_t m54;
  float32_t m55;
  float32_t m56;
  float32_t m61;
  float32_t m62;
  float32_t m63;
  float32_t m64;
  float32_t m65;
  float32_t m66;
} matrix66_t;

typedef struct _vector61_t {
  union {
    float32_t v[6];
    struct
    {
      float32_t x1;
      float32_t x2;
      float32_t x3;
      float32_t x4;
      float32_t x5;
      float32_t x6;
    };
  };
} vector61_t;

extern const matrix66_t matrix_id6;
extern const matrix66_t matrix_null6;
extern const vector61_t vector61_zero;

void mul_mat66( matrix66_t* out, matrix66_t *m1, matrix66_t *m2 );
void add_mat66( matrix66_t* out, matrix66_t *m1, matrix66_t *m2 );
void mulvec_mat6( vector61_t* out, matrix66_t *m1, vector61_t *v1 );
void transpose_mat66( matrix66_t *out, matrix66_t *m1 );
void mulconst_mat66( matrix66_t *out, matrix66_t *m1, float32_t k );
void add_vec61( vector61_t* out, vector61_t *v1, vector61_t *v2 );
void mulconst_vec61( vector61_t *out, vector61_t *v1, float32_t k );

// 2x2 matrices

typedef struct _matrix22_t
{
  float32_t m11;
  float32_t m12;
  float32_t m21;
  float32_t m22;

} matrix22_t;


extern const matrix22_t matrix_id2;
extern const matrix22_t matrix_null2;
extern const vector21_t vector21_zero;

void mul_mat22( matrix22_t* out, matrix22_t *m1, matrix22_t *m2 );
void add_mat22( matrix22_t* out, matrix22_t *m1, matrix22_t *m2 );
void mulvec_mat2( vector21_t* out, matrix22_t *m1, vector21_t *v1 );
void transpose_mat22( matrix22_t *out, matrix22_t *m1 );
void mulconst_mat22( matrix22_t *out, matrix22_t *m1, float32_t k );
void add_vec21( vector21_t* out, vector21_t *v1, vector21_t *v2 );
void mulconst_vec21( vector21_t *out, vector21_t *v1, float32_t k );
void det_mat2(float32_t *out, matrix22_t *m1);
void comatrice22(matrix22_t *out, matrix22_t *m1);
void inv_mat22(matrix22_t *out, matrix22_t* m1);


// NON-square matrices and operations between diferrent-sized matrices


typedef struct _matrix26_t
{
  float32_t m11;
  float32_t m12;
  float32_t m13;
  float32_t m14;
  float32_t m15;
  float32_t m16;
  float32_t m21;
  float32_t m22;
  float32_t m23;
  float32_t m24;
  float32_t m25;
  float32_t m26;
} matrix26_t;

typedef struct _matrix62_t
{
  float32_t m11;
  float32_t m12;
  float32_t m21;
  float32_t m22;
  float32_t m31;
  float32_t m32;
  float32_t m41;
  float32_t m42;
  float32_t m51;
  float32_t m52;
  float32_t m61;
  float32_t m62;
} matrix62_t;

typedef struct _matrix46_t
{
  float32_t m11;
  float32_t m12;
  float32_t m13;
  float32_t m14;
  float32_t m15;
  float32_t m16;

  float32_t m21;
  float32_t m22;
  float32_t m23;
  float32_t m24;
  float32_t m25;
  float32_t m26;

  float32_t m31;
  float32_t m32;
  float32_t m33;
  float32_t m34;
  float32_t m35;
  float32_t m36;

  float32_t m41;
  float32_t m42;
  float32_t m43;
  float32_t m44;
  float32_t m45;
  float32_t m46;

} matrix46_t;

typedef struct _matrix64_t
{
  float32_t m11;
  float32_t m12;
  float32_t m13;
  float32_t m14;

  float32_t m21;
  float32_t m22;
  float32_t m23;
  float32_t m24;

  float32_t m31;
  float32_t m32;
  float32_t m33;
  float32_t m34;

  float32_t m41;
  float32_t m42;
  float32_t m43;
  float32_t m44;

  float32_t m51;
  float32_t m52;
  float32_t m53;
  float32_t m54;

  float32_t m61;
  float32_t m62;
  float32_t m63;
  float32_t m64;

} matrix64_t;

extern const matrix46_t matrix_null46;
extern const matrix26_t matrix_null26;

void mulmat26vec61( vector21_t* out, matrix26_t *m1, vector61_t *v1 );
void mulmat46vec61( vector41_t* out, matrix46_t *m1, vector61_t *v1 );
void mulmat46mat66( matrix46_t* out, matrix46_t *m1, matrix66_t *m2 );
void mulmat66mat64( matrix64_t* out, matrix66_t *m1, matrix64_t *m2 );
void mulmat46mat64( matrix44_t* out, matrix46_t *m1, matrix64_t *m2 );
void mulmat64mat44( matrix64_t* out, matrix64_t *m1, matrix44_t *m2 );
void mulmat62mat22( matrix62_t* out, matrix62_t *m1, matrix22_t *m2 );
void mulmat26mat66( matrix26_t* out, matrix26_t *m1, matrix66_t *m2 );
void mulmat66mat62( matrix62_t* out, matrix66_t *m1, matrix62_t *m2 );
void mulmat26mat62( matrix22_t* out, matrix26_t *m1, matrix62_t *m2 );
void mulmat64vec41( vector61_t* out, matrix64_t *m1, vector41_t *v1 );
void mulmat62vec21( vector61_t* out, matrix62_t *m1, vector21_t *v1 );
void mulmat64mat46( matrix66_t* out, matrix64_t *m1, matrix46_t *m2 );
void mulmat62mat26( matrix66_t* out, matrix62_t *m1, matrix26_t *m2 );

void transpose_mat26( matrix62_t *out, matrix26_t *m1 );
void transpose_mat62( matrix26_t *out, matrix62_t *m1 );
void transpose_mat46( matrix64_t *out, matrix46_t *m1 );
void transpose_mat64( matrix46_t *out, matrix64_t *m1 );


#endif // _MATRICES_H_
