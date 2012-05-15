#include <VP_Os/vp_os_assert.h>

#include <Maths/matrices.h>
#include <Maths/maths.h>

const matrix33_t matrix_id3 =  { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
const vector31_t vector31_zero = { { { 0.0f, 0.0f, 0.0f } } };
const vector31_t vector31_z = { { { 0.0f, 0.0f, 1.0f } } };

void mul_mat( matrix33_t* out, matrix33_t *m1, matrix33_t *m2 )
{
  /// You can't have output & input pointing to the same location
  VP_OS_ASSERT( out != m1 );
  VP_OS_ASSERT( out != m2 );

  out->m11 = (m1->m11)*(m2->m11) + (m1->m12)*(m2->m21) + (m1->m13)*(m2->m31);
  out->m12 = (m1->m11)*(m2->m12) + (m1->m12)*(m2->m22) + (m1->m13)*(m2->m32);
  out->m13 = (m1->m11)*(m2->m13) + (m1->m12)*(m2->m23) + (m1->m13)*(m2->m33);

  out->m21 = (m1->m21)*(m2->m11) + (m1->m22)*(m2->m21) + (m1->m23)*(m2->m31);
  out->m22 = (m1->m21)*(m2->m12) + (m1->m22)*(m2->m22) + (m1->m23)*(m2->m32);
  out->m23 = (m1->m21)*(m2->m13) + (m1->m22)*(m2->m23) + (m1->m23)*(m2->m33);

  out->m31 = (m1->m31)*(m2->m11) + (m1->m32)*(m2->m21) + (m1->m33)*(m2->m31);
  out->m32 = (m1->m31)*(m2->m12) + (m1->m32)*(m2->m22) + (m1->m33)*(m2->m32);
  out->m33 = (m1->m31)*(m2->m13) + (m1->m32)*(m2->m23) + (m1->m33)*(m2->m33);
}

void add_mat( matrix33_t* out, matrix33_t *m1, matrix33_t *m2 )
{
  out->m11 = (m1->m11) + (m2->m11);
  out->m12 = (m1->m12) + (m2->m12);
  out->m13 = (m1->m13) + (m2->m13);

  out->m21 = (m1->m21) + (m2->m21);
  out->m22 = (m1->m22) + (m2->m22);
  out->m23 = (m1->m23) + (m2->m23);

  out->m31 = (m1->m31) + (m2->m31);
  out->m32 = (m1->m32) + (m2->m32);
  out->m33 = (m1->m33) + (m2->m33);
}

void mulvec_mat( vector31_t* out, matrix33_t *m1, vector31_t *v1 )
{
  out->x = (m1->m11) * (v1->x) + (m1->m12) * (v1->y) + (m1->m13) * (v1->z);
  out->y = (m1->m21) * (v1->x) + (m1->m22) * (v1->y) + (m1->m23) * (v1->z);
  out->z = (m1->m31) * (v1->x) + (m1->m32) * (v1->y) + (m1->m33) * (v1->z);
}

void transpose_mat( matrix33_t *out, matrix33_t *m1 )
{
  out->m11 = m1->m11;
  out->m12 = m1->m21;
  out->m13 = m1->m31;

  out->m21 = m1->m12;
  out->m22 = m1->m22;
  out->m23 = m1->m32;

  out->m31 = m1->m13;
  out->m32 = m1->m23;
  out->m33 = m1->m33;
}

void mulconst_mat( matrix33_t *out, matrix33_t *m1, float32_t k )
{
  out->m11 = m1->m11 * k;
  out->m12 = m1->m12 * k;
  out->m13 = m1->m13 * k;

  out->m21 = m1->m21 * k;
  out->m22 = m1->m22 * k;
  out->m23 = m1->m23 * k;

  out->m31 = m1->m31 * k;
  out->m32 = m1->m32 * k;
  out->m33 = m1->m33 * k;
}

void cross_vec( vector31_t* out, vector31_t *v1, vector31_t *v2 )
{
  /// You can't have output & input pointing to the same location
  VP_OS_ASSERT( out != v1 );
  VP_OS_ASSERT( out != v2 );

  out->x = (v1->y) * (v2->z) - (v1->z) * (v2->y);
  out->y = (v1->z) * (v2->x) - (v1->x) * (v2->z);
  out->z = (v1->x) * (v2->y) - (v1->y) * (v2->x);
}

void dot_vec( float32_t* out, const vector31_t *v1, const vector31_t *v2 )
{
  *out = v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

void add_vec( vector31_t* out, vector31_t *v1, vector31_t *v2 )
{
  out->x = (v1->x) + (v2->x);
  out->y = (v1->y) + (v2->y);
  out->z = (v1->z) + (v2->z);
}

void mulconst_vec( vector31_t *out, vector31_t *v1, float32_t k )
{
  out->x = (v1->x) * k;
  out->y = (v1->y) * k;
  out->z = (v1->z) * k;
}

void skew_anti_symetric_vec( matrix33_t *out, vector31_t *v )
{
  out->m11 = 0.0f;
  out->m12 =-v->z;
  out->m13 = v->y;

  out->m21 = v->z;
  out->m22 = 0.0f;
  out->m23 =-v->x;

  out->m31 =-v->y;
  out->m32 = v->x;
  out->m33 = 0.0f;
}

void vex( vector31_t *out, matrix33_t *m )
{
  out->x = m->m32;
  out->y = m->m13;
  out->z = m->m21;
}

float32_t norm_vec( vector31_t *v )
{
  return sqrtf( (v->x)*(v->x) + (v->y)*(v->y) + (v->z)*(v->z) );
}

bool_t normalize_vec( vector31_t* v )
{
  bool_t ret;
  float32_t l;

  l = norm_vec( v );

  if( f_is_zero( l ) )
  {
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;

    ret = FALSE;
  }
  else
  {
    v->x = f_zero( v->x / l );
    v->y = f_zero( v->y / l );
    v->z = f_zero( v->z / l );

    ret = TRUE;
  }

  return ret;
}

