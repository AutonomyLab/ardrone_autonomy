/**
 *  \brief    Matrix 3d implementation
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \version  1.0
 *  \date     04/06/2007
 *  \warning  Subject to completion
 */

#include <Maths/matrix3d.h>
#include <Maths/matrices.h>

#include <VP_Os/vp_os_malloc.h>

#include <math.h>

matrix3d_t matrix3d_id = {
  1.0f, 0.0f, 0.0f, 0.0f,
  0.0f, 1.0f, 0.0f, 1.0f,
  0.0f, 0.0f, 1.0f, 0.0f,
  0.0f, 0.0f, 0.0f, 1.0f
};

static void vector_X( vector31_t* v )
{
  v->x = 1.0f;
  v->y = 0.0f;
  v->z = 0.0f;
}

static void vector_Y( vector31_t* v )
{
  v->x = 0.0f;
  v->y = 1.0f;
  v->z = 0.0f;
}

#if 0 // it is not used ... fix warning
static void vector_Z( vector31_t* v )
{
  v->x = 0.0f;
  v->y = 0.0f;
  v->z = 1.0f;
}
#endif

void matrix3d_zero(matrix3d_t* m)
{
  m->m00 = m->m01 = m->m02 = m->m03 = 0.0f;
  m->m10 = m->m11 = m->m12 = m->m13 = 0.0f;
  m->m20 = m->m21 = m->m22 = m->m23 = 0.0f;
  m->m30 = m->m31 = m->m32 = m->m33 = 0.0f;
}

void matrix3d_identity(matrix3d_t* m)
{
  m->m00 = m->m11 = m->m22 = m->m33 = 1.0f;

  m->m01 = m->m02 = m->m03 = 0.0f;
  m->m10 = m->m12 = m->m13 = 0.0f;
  m->m20 = m->m21 = m->m23 = 0.0f;
  m->m30 = m->m31 = m->m32 = 0.0f;
}

C_RESULT matrix3d_euler(matrix3d_t* m, float32_t phi, float32_t theta, float32_t psi)
{
  float32_t c_phi, s_phi, c_theta, s_theta, c_psi, s_psi;

  c_phi   = cosf( phi );
  s_phi   = sinf( phi );
  c_theta = cosf( theta );
  s_theta = sinf( theta );
  c_psi   = cosf( psi );
  s_psi   = sinf( psi );

  matrix3d_identity(m);

  m->m00 = c_theta * c_psi;
  m->m01 = c_theta * s_psi;
  m->m02 = -s_theta;

  m->m10 = s_phi * s_theta * c_psi - c_phi * s_psi;
  m->m11 = s_phi * s_theta * s_psi + c_phi * c_psi;
  m->m12 = s_phi * c_theta;

  m->m20 = c_phi * s_theta * c_psi + s_phi * s_psi;
  m->m21 = c_phi * s_theta * s_psi - s_phi * c_psi;
  m->m22 = c_phi * c_theta;

  return C_OK;
}

C_RESULT matrix3d_vector(matrix3d_t* m, vector31_t* pos, vector31_t* dir, vector31_t* right, vector31_t* up)
{
  normalize_vec( dir );
  vector_Y( up );

  cross_vec( right, dir, up );
  if( normalize_vec( right ) )
  {
    cross_vec( up, right, dir );
  }
  else
  {
    vector_X( right );

    cross_vec( up, right, dir );
    cross_vec( right, dir, up );
    normalize_vec( right );
  }

  normalize_vec( up );

  return matrix3d_orientation( m, pos, dir, right, up );
}

C_RESULT matrix3d_orientation(matrix3d_t* m, const vector31_t* pos, const vector31_t* dir,
                                             const vector31_t* right, const vector31_t* up)
{
  float32_t d0, d1, d2;

  m->m00 =  right->x;
  m->m01 =  up->x;
  m->m02 = -dir->x;

  m->m10 =  right->y;
  m->m11 =  up->y;
  m->m12 = -dir->y;

  m->m20 =  right->z;
  m->m21 =  up->z;
  m->m22 = -dir->z;

  dot_vec( &d0, right, pos );
  dot_vec( &d1, up,    pos );
  dot_vec( &d2, dir,   pos );

  m->m30 = -d0;
  m->m31 = -d1;
  m->m32 = -d2;

  m->m03 =  0.0f;
  m->m13 =  0.0f;
  m->m23 =  0.0f;
  m->m33 =  1.0f;

  return C_OK;
}

#define MATRIX_EXCHANGE( out, in ) temp = out; out = in; in = temp

C_RESULT matrix3d_transpose(matrix3d_t* out, matrix3d_t* in)
{
  if( out != in )
  {
    out->m00 = in->m00;
    out->m10 = in->m01;
    out->m20 = in->m02;
    out->m30 = in->m03;

    out->m01 = in->m10;
    out->m11 = in->m11;
    out->m21 = in->m12;
    out->m31 = in->m13;

    out->m02 = in->m20;
    out->m12 = in->m21;
    out->m22 = in->m22;
    out->m32 = in->m23;

    out->m03 = in->m30;
    out->m13 = in->m31;
    out->m23 = in->m32;
    out->m33 = in->m33;
  }
  else
  {
    float32_t temp;

    MATRIX_EXCHANGE( out->m01, out->m10 );
    MATRIX_EXCHANGE( out->m02, out->m20 );
    MATRIX_EXCHANGE( out->m03, out->m30 );
    MATRIX_EXCHANGE( out->m12, out->m21 );
    MATRIX_EXCHANGE( out->m13, out->m31 );
    MATRIX_EXCHANGE( out->m23, out->m32 );
  }

  return C_OK;
}

C_RESULT matrix3d_add(matrix3d_t* out, matrix3d_t* m1, matrix3d_t* m2)
{
  out->m00 = m1->m00 + m2->m00;
  out->m01 = m1->m01 + m2->m01;
  out->m02 = m1->m02 + m2->m02;
  out->m03 = m1->m03 + m2->m03;

  out->m10 = m1->m10 + m2->m10;
  out->m11 = m1->m11 + m2->m11;
  out->m12 = m1->m12 + m2->m12;
  out->m13 = m1->m13 + m2->m13;

  out->m20 = m1->m20 + m2->m20;
  out->m21 = m1->m21 + m2->m21;
  out->m22 = m1->m22 + m2->m22;
  out->m23 = m1->m23 + m2->m23;

  out->m30 = m1->m30 + m2->m30;
  out->m31 = m1->m31 + m2->m31;
  out->m32 = m1->m32 + m2->m32;
  out->m33 = m1->m33 + m2->m33;

  return C_OK;
}

C_RESULT matrix3d_sub(matrix3d_t* out, matrix3d_t* m1, matrix3d_t* m2)
{
  out->m00 = m1->m00 - m2->m00;
  out->m01 = m1->m01 - m2->m01;
  out->m02 = m1->m02 - m2->m02;
  out->m03 = m1->m03 - m2->m03;

  out->m10 = m1->m10 - m2->m10;
  out->m11 = m1->m11 - m2->m11;
  out->m12 = m1->m12 - m2->m12;
  out->m13 = m1->m13 - m2->m13;

  out->m20 = m1->m20 - m2->m20;
  out->m21 = m1->m21 - m2->m21;
  out->m22 = m1->m22 - m2->m22;
  out->m23 = m1->m23 - m2->m23;

  out->m30 = m1->m30 - m2->m30;
  out->m31 = m1->m31 - m2->m31;
  out->m32 = m1->m32 - m2->m32;
  out->m33 = m1->m33 - m2->m33;

  return C_OK;
}

#define MATRIX_MUL_PART(a,b)          \
  t1 = ( M1[a*4 + 0] * M2[b + 0 ] );  \
  t2 = ( M1[a*4 + 1] * M2[b + 4 ] );  \
  t3 = ( M1[a*4 + 2] * M2[b + 8 ] );  \
  t4 = ( M1[a*4 + 3] * M2[b + 12] );  \
  MOUT[a*4 + b] = ( t1 + t2 + t3 + t4 );

C_RESULT matrix3d_mul(matrix3d_t* out, matrix3d_t* m1, matrix3d_t* m2)
{
  float32_t t1, t2, t3, t4;

  float32_t* MOUT = (float32_t*) out;
  float32_t* M1  = (float32_t*) m1;
  float32_t* M2  = (float32_t*) m2;

  MATRIX_MUL_PART( 0, 0 ); MATRIX_MUL_PART( 0, 1 ); MATRIX_MUL_PART( 0, 2 ); MATRIX_MUL_PART( 0, 3 );
  MATRIX_MUL_PART( 1, 0 ); MATRIX_MUL_PART( 1, 1 ); MATRIX_MUL_PART( 1, 2 ); MATRIX_MUL_PART( 1, 3 );
  MATRIX_MUL_PART( 2, 0 ); MATRIX_MUL_PART( 2, 1 ); MATRIX_MUL_PART( 2, 2 ); MATRIX_MUL_PART( 2, 3 );
  MATRIX_MUL_PART( 3, 0 ); MATRIX_MUL_PART( 3, 1 ); MATRIX_MUL_PART( 3, 2 ); MATRIX_MUL_PART( 3, 3 );

  return C_OK;
}

C_RESULT matrix3d_translate(matrix3d_t* m, vector31_t* tr)
{
  m->m30 =  tr->x;
  m->m31 =  tr->y;
  m->m32 =  tr->z;
  m->m33 =  1.0f;

  return C_OK;
}

C_RESULT matrix3d_add_translate(matrix3d_t* m, vector31_t* tr)
{
  m->m30 +=  tr->x;
  m->m31 +=  tr->y;
  m->m32 +=  tr->z;
  m->m33 =  1.0f;

  return C_OK;
}

C_RESULT matrix3d_rotate_euler(matrix3d_t* m, float32_t phi, float32_t theta, float32_t psi)
{
  matrix3d_t mat_rot;
  matrix3d_t mat_res;

  matrix3d_euler(&mat_rot, phi, theta, psi);
  matrix3d_mul(&mat_res, &mat_rot, m);

  vp_os_memcpy( (void *)m, (void *)&mat_res, sizeof(matrix3d_t) );

  return C_OK;
}

C_RESULT matrix3d_rotate_axis(matrix3d_t* m, vector31_t* axis, float32_t value)
{
  return C_OK;
}

C_RESULT matrix3d_transform(matrix3d_t* m1, struct _vector31_t* v1)
{
  float32_t x = (m1->m00) * (v1->x) + (m1->m01) * (v1->y) + (m1->m02) * (v1->z) + m1->m30;
  float32_t y = (m1->m10) * (v1->x) + (m1->m11) * (v1->y) + (m1->m12) * (v1->z) + m1->m31;
  float32_t z = (m1->m20) * (v1->x) + (m1->m21) * (v1->y) + (m1->m22) * (v1->z) + m1->m32;
  float32_t w = (m1->m30) * (v1->x) + (m1->m31) * (v1->y) + (m1->m32) * (v1->z) + m1->m33;

  v1->x = x / w;
  v1->y = y / w;
  v1->z = z / w;

  return C_OK;
}
