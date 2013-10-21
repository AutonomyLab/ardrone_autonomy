#include <VP_Os/vp_os_assert.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>

#include <Maths/matrices.h>
#include <Maths/maths.h>
#include <stdio.h>


const matrix33_t matrix_id3 =  { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
const matrix33_t matrix_null3 =  { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
const vector31_t vector31_zero = { { { 0.0f, 0.0f, 0.0f } } };
const vector31_t vector31_z = { { { 0.0f, 0.0f, 1.0f } } };

const matrix44_t matrix_id4 =  { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
const matrix44_t matrix_null4 =  { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
const vector41_t vector41_zero = { { { 0.0f, 0.0f, 0.0f, 0.0f } } };

const matrix66_t matrix_id6 =  { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
const matrix66_t matrix_null6 =  { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
const vector61_t vector61_zero = { { { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f } } };

const matrix22_t matrix_id2 =  { 1.0f, 0.0f, 0.0f, 1.0f };
const matrix22_t matrix_null2 =  { 0.0f, 0.0f, 0.0f, 0.0f };
const vector21_t vector21_zero = { { 0.0f, 0.0f } };

const matrix46_t matrix_null46 =  { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
const matrix26_t matrix_null26 =  { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

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

void det_mat3(float32_t *out, matrix33_t *m1)
{
    *out = m1->m11*m1->m22*m1->m33 + m1->m12*m1->m23*m1->m31 + m1->m21*m1->m32*m1->m13 - m1->m31*m1->m22*m1->m13 - m1->m21*m1->m12*m1->m33 - m1->m11*m1->m32*m1->m23;
}

void comatrice33(matrix33_t *out, matrix33_t *m1)
{
    out->m11 =   m1->m22*m1->m33 - m1->m32*m1->m23;
    out->m12 = -(m1->m21*m1->m33 - m1->m31*m1->m23);
    out->m13 =   m1->m21*m1->m32 - m1->m22*m1->m31;
    out->m21 = -(m1->m12*m1->m33 - m1->m13*m1->m32);
    out->m22 =   m1->m11*m1->m33 - m1->m31*m1->m13;
    out->m23 = -(m1->m11*m1->m32 - m1->m12*m1->m31);
    out->m31 =   m1->m12*m1->m23 - m1->m13*m1->m22;
    out->m32 = -(m1->m11*m1->m23 - m1->m13*m1->m21);
    out->m33 =   m1->m11*m1->m22 - m1->m12*m1->m21;
}


// computes the inverse matrix of a square 3x3 matrix.
// if matrix is not inversible, returns null matrix
void inv_mat33(matrix33_t *out, matrix33_t* m1)
{
    float32_t det;
    matrix33_t Com, t_Com;

    det_mat3(&det,m1);

    if(f_is_zero(det))
    {
        memcpy(out, &matrix_null3, sizeof(matrix33_t));
    }
    else
    {
        comatrice33(&Com, m1);
        transpose_mat(&t_Com, &Com);
        mulconst_mat(out, &t_Com, 1.0/det);
    }
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

void display_matrix33(matrix33_t *m1)
{
    PRINT("( %f\t%f\t%f )\n( %f\t%f\t%f )\n( %f\t%f\t%f )\n",m1->m11,m1->m12,m1->m13,m1->m21,m1->m22,m1->m23,m1->m31,m1->m32,m1->m33);
}

void display_vector31(vector31_t *v1)
{
    PRINT("( %f\t%f\t%f )\n", v1->x ,v1->y, v1->z);
}

// matrix of size 4

void mul_mat44( matrix44_t* out, matrix44_t *m1, matrix44_t *m2 )
{
  /// You can't have output & input pointing to the same location
  VP_OS_ASSERT( out != m1 );
  VP_OS_ASSERT( out != m2 );

  out->m11 = (m1->m11)*(m2->m11) + (m1->m12)*(m2->m21) + (m1->m13)*(m2->m31) + (m1->m14)*(m2->m41);
  out->m12 = (m1->m11)*(m2->m12) + (m1->m12)*(m2->m22) + (m1->m13)*(m2->m32) + (m1->m14)*(m2->m42);
  out->m13 = (m1->m11)*(m2->m13) + (m1->m12)*(m2->m23) + (m1->m13)*(m2->m33) + (m1->m14)*(m2->m43);
  out->m14 = (m1->m11)*(m2->m14) + (m1->m12)*(m2->m24) + (m1->m13)*(m2->m34) + (m1->m14)*(m2->m44);

  out->m21 = (m1->m21)*(m2->m11) + (m1->m22)*(m2->m21) + (m1->m23)*(m2->m31) + (m1->m24)*(m2->m41);
  out->m22 = (m1->m21)*(m2->m12) + (m1->m22)*(m2->m22) + (m1->m23)*(m2->m32) + (m1->m24)*(m2->m42);
  out->m23 = (m1->m21)*(m2->m13) + (m1->m22)*(m2->m23) + (m1->m23)*(m2->m33) + (m1->m24)*(m2->m43);
  out->m24 = (m1->m21)*(m2->m14) + (m1->m22)*(m2->m24) + (m1->m23)*(m2->m34) + (m1->m24)*(m2->m44);

  out->m31 = (m1->m31)*(m2->m11) + (m1->m32)*(m2->m21) + (m1->m33)*(m2->m31) + (m1->m34)*(m2->m41);
  out->m32 = (m1->m31)*(m2->m12) + (m1->m32)*(m2->m22) + (m1->m33)*(m2->m32) + (m1->m34)*(m2->m42);
  out->m33 = (m1->m31)*(m2->m13) + (m1->m32)*(m2->m23) + (m1->m33)*(m2->m33) + (m1->m34)*(m2->m43);
  out->m34 = (m1->m31)*(m2->m14) + (m1->m32)*(m2->m24) + (m1->m33)*(m2->m34) + (m1->m34)*(m2->m44);

  out->m41 = (m1->m41)*(m2->m11) + (m1->m42)*(m2->m21) + (m1->m43)*(m2->m31) + (m1->m44)*(m2->m41);
  out->m42 = (m1->m41)*(m2->m12) + (m1->m42)*(m2->m22) + (m1->m43)*(m2->m32) + (m1->m44)*(m2->m42);
  out->m43 = (m1->m41)*(m2->m13) + (m1->m42)*(m2->m23) + (m1->m43)*(m2->m33) + (m1->m44)*(m2->m43);
  out->m44 = (m1->m41)*(m2->m14) + (m1->m42)*(m2->m24) + (m1->m43)*(m2->m34) + (m1->m44)*(m2->m44);
}

void add_mat44( matrix44_t* out, matrix44_t *m1, matrix44_t *m2 )
{
  out->m11 = (m1->m11) + (m2->m11);
  out->m12 = (m1->m12) + (m2->m12);
  out->m13 = (m1->m13) + (m2->m13);
  out->m14 = (m1->m14) + (m2->m14);

  out->m21 = (m1->m21) + (m2->m21);
  out->m22 = (m1->m22) + (m2->m22);
  out->m23 = (m1->m23) + (m2->m23);
  out->m24 = (m1->m24) + (m2->m24);

  out->m31 = (m1->m31) + (m2->m31);
  out->m32 = (m1->m32) + (m2->m32);
  out->m33 = (m1->m33) + (m2->m33);
  out->m34 = (m1->m34) + (m2->m34);

  out->m41 = (m1->m41) + (m2->m41);
  out->m42 = (m1->m42) + (m2->m42);
  out->m43 = (m1->m43) + (m2->m43);
  out->m44 = (m1->m44) + (m2->m44);
}

void mulvec_mat4( vector41_t* out, matrix44_t *m1, vector41_t *v1 )
{
  out->x1 = (m1->m11) * (v1->x1) + (m1->m12) * (v1->x2) + (m1->m13) * (v1->x3) + (m1->m14) * (v1->x4);
  out->x2 = (m1->m21) * (v1->x1) + (m1->m22) * (v1->x2) + (m1->m23) * (v1->x3) + (m1->m24) * (v1->x4);
  out->x3 = (m1->m31) * (v1->x1) + (m1->m32) * (v1->x2) + (m1->m33) * (v1->x3) + (m1->m34) * (v1->x4);
  out->x4 = (m1->m41) * (v1->x1) + (m1->m42) * (v1->x2) + (m1->m43) * (v1->x3) + (m1->m44) * (v1->x4);
}

void transpose_mat44( matrix44_t *out, matrix44_t *m1 )
{
  out->m11 = m1->m11;
  out->m12 = m1->m21;
  out->m13 = m1->m31;
  out->m14 = m1->m41;

  out->m21 = m1->m12;
  out->m22 = m1->m22;
  out->m23 = m1->m32;
  out->m24 = m1->m42;

  out->m31 = m1->m13;
  out->m32 = m1->m23;
  out->m33 = m1->m33;
  out->m34 = m1->m43;

  out->m41 = m1->m14;
  out->m42 = m1->m24;
  out->m43 = m1->m34;
  out->m44 = m1->m44;
}

void mulconst_mat44( matrix44_t *out, matrix44_t *m1, float32_t k )
{
  out->m11 = m1->m11 * k;
  out->m12 = m1->m12 * k;
  out->m13 = m1->m13 * k;
  out->m14 = m1->m14 * k;

  out->m21 = m1->m21 * k;
  out->m22 = m1->m22 * k;
  out->m23 = m1->m23 * k;
  out->m24 = m1->m24 * k;

  out->m31 = m1->m31 * k;
  out->m32 = m1->m32 * k;
  out->m33 = m1->m33 * k;
  out->m34 = m1->m34 * k;

  out->m41 = m1->m41 * k;
  out->m42 = m1->m42 * k;
  out->m43 = m1->m43 * k;
  out->m44 = m1->m44 * k;
}

void add_vec41( vector41_t* out, vector41_t *v1, vector41_t *v2 )
{
  out->x1 = (v1->x1) + (v2->x1);
  out->x2 = (v1->x2) + (v2->x2);
  out->x3 = (v1->x3) + (v2->x3);
  out->x4 = (v1->x4) + (v2->x4);

}

void mulconst_vec41( vector41_t *out, vector41_t *v1, float32_t k )
{
  out->x1 = (v1->x1) * k;
  out->x2 = (v1->x2) * k;
  out->x3 = (v1->x3) * k;
  out->x4 = (v1->x4) * k;
}

void comatrice44(matrix44_t *out, matrix44_t *m1)
{
    out->m11 =   m1->m22*m1->m33*m1->m44 + m1->m23*m1->m34*m1->m42 + m1->m32*m1->m43*m1->m24 - m1->m42*m1->m33*m1->m24 - m1->m32*m1->m23*m1->m44 - m1->m22*m1->m43*m1->m34;
    out->m12 =   -(m1->m21*m1->m33*m1->m44 + m1->m31*m1->m43*m1->m24 + m1->m41*m1->m23*m1->m34 - m1->m41*m1->m33*m1->m24 - m1->m43*m1->m34*m1->m21 - m1->m44*m1->m31*m1->m23);
    out->m13 =   m1->m21*m1->m32*m1->m44 + m1->m22*m1->m34*m1->m41 + m1->m24*m1->m31*m1->m42 - m1->m41*m1->m32*m1->m24 - m1->m42*m1->m34*m1->m21 - m1->m44*m1->m31*m1->m22;
    out->m14 =   -(m1->m21*m1->m32*m1->m43 + m1->m22*m1->m33*m1->m41 + m1->m31*m1->m42*m1->m23 - m1->m41*m1->m32*m1->m23 - m1->m31*m1->m22*m1->m43 - m1->m21*m1->m42*m1->m33);

    out->m21 =   -(m1->m12*m1->m33*m1->m44 + m1->m13*m1->m34*m1->m42 + m1->m14*m1->m32*m1->m43 - m1->m42*m1->m33*m1->m14 - m1->m43*m1->m34*m1->m12 - m1->m44*m1->m32*m1->m13);
    out->m22 =   m1->m11*m1->m33*m1->m44 + m1->m13*m1->m34*m1->m41 + m1->m14*m1->m31*m1->m43 - m1->m41*m1->m33*m1->m14 - m1->m43*m1->m34*m1->m11 - m1->m44*m1->m31*m1->m13;
    out->m23 =   -(m1->m11*m1->m32*m1->m44 + m1->m12*m1->m34*m1->m41 + m1->m14*m1->m31*m1->m42 - m1->m41*m1->m32*m1->m14 - m1->m42*m1->m34*m1->m11 - m1->m44*m1->m31*m1->m12);
    out->m24 =   m1->m11*m1->m32*m1->m43 + m1->m12*m1->m33*m1->m41 + m1->m31*m1->m42*m1->m13 - m1->m41*m1->m32*m1->m13 - m1->m31*m1->m12*m1->m43 - m1->m11*m1->m42*m1->m33;

    out->m31 =   m1->m12*m1->m23*m1->m44 + m1->m13*m1->m24*m1->m42 + m1->m14*m1->m22*m1->m43 - m1->m42*m1->m23*m1->m14 - m1->m43*m1->m24*m1->m12 - m1->m44*m1->m22*m1->m13;
    out->m32 =   -(m1->m11*m1->m23*m1->m44 + m1->m13*m1->m24*m1->m41 + m1->m14*m1->m21*m1->m43 - m1->m41*m1->m23*m1->m14 - m1->m43*m1->m24*m1->m11 - m1->m44*m1->m21*m1->m13);
    out->m33 =   m1->m11*m1->m22*m1->m44 + m1->m12*m1->m24*m1->m41 + m1->m14*m1->m21*m1->m42 - m1->m41*m1->m22*m1->m14 - m1->m42*m1->m24*m1->m11 - m1->m44*m1->m21*m1->m12;
    out->m34 =   -(m1->m11*m1->m22*m1->m43 + m1->m12*m1->m23*m1->m41 + m1->m13*m1->m21*m1->m42 - m1->m41*m1->m22*m1->m13 - m1->m42*m1->m23*m1->m11 - m1->m43*m1->m21*m1->m12);

    out->m41 =   -(m1->m12*m1->m23*m1->m34 + m1->m13*m1->m24*m1->m32 + m1->m14*m1->m22*m1->m33 - m1->m32*m1->m23*m1->m14 - m1->m33*m1->m24*m1->m12 - m1->m34*m1->m22*m1->m13);
    out->m42 =   m1->m11*m1->m23*m1->m34 + m1->m13*m1->m24*m1->m31 + m1->m14*m1->m21*m1->m33 - m1->m31*m1->m23*m1->m14 - m1->m33*m1->m24*m1->m11 - m1->m34*m1->m21*m1->m13;
    out->m43 =   -(m1->m11*m1->m22*m1->m34 + m1->m12*m1->m24*m1->m31 + m1->m14*m1->m21*m1->m32 - m1->m31*m1->m22*m1->m14 - m1->m32*m1->m24*m1->m11 - m1->m34*m1->m21*m1->m12);
    out->m44 =   m1->m11*m1->m22*m1->m33 + m1->m12*m1->m23*m1->m31 + m1->m13*m1->m21*m1->m32 - m1->m31*m1->m22*m1->m13 - m1->m32*m1->m23*m1->m11 - m1->m33*m1->m21*m1->m12;
}

void det_mat4(float32_t *out, matrix44_t *m1)
{
    *out = m1->m11*(m1->m22*m1->m33*m1->m44 + m1->m23*m1->m34*m1->m42 + m1->m32*m1->m43*m1->m24 - m1->m42*m1->m33*m1->m24 - m1->m32*m1->m23*m1->m44 - m1->m22*m1->m43*m1->m34)
            -m1->m12*(m1->m21*m1->m33*m1->m44 + m1->m31*m1->m43*m1->m24 + m1->m41*m1->m23*m1->m34 - m1->m41*m1->m33*m1->m24 - m1->m43*m1->m34*m1->m21 - m1->m44*m1->m31*m1->m23)
            +m1->m13*(m1->m21*m1->m32*m1->m44 + m1->m22*m1->m34*m1->m41 + m1->m24*m1->m31*m1->m42 - m1->m41*m1->m32*m1->m24 - m1->m42*m1->m34*m1->m21 - m1->m44*m1->m31*m1->m22)
            -m1->m14*(m1->m21*m1->m32*m1->m43 + m1->m22*m1->m33*m1->m41 + m1->m31*m1->m42*m1->m23 - m1->m41*m1->m32*m1->m23 - m1->m31*m1->m22*m1->m43 - m1->m21*m1->m42*m1->m33);
}

// computes the inverse matrix of a square 4x4 matrix.
// if matrix is not inversible, returns null matrix
void inv_mat44(matrix44_t *out, matrix44_t* m1)
{
    float32_t det;
    matrix44_t Com, t_Com;

    det_mat4(&det,m1);

    if(f_is_zero(det))
    {
        memcpy(out, &matrix_null4, sizeof(matrix44_t));
    }
    else
    {
        comatrice44(&Com, m1);
        transpose_mat44(&t_Com, &Com);
        mulconst_mat44(out, &t_Com, 1.0/det);
    }
}

void display_matrix44(matrix44_t *m1)
{
    PRINT("( %f\t%f\t%f\t%f )\n( %f\t%f\t%f\t%f  )\n( %f\t%f\t%f\t%f  )\n( %f\t%f\t%f\t%f  )\n",m1->m11,m1->m12,m1->m13,m1->m14,m1->m21,m1->m22,m1->m23,m1->m24,m1->m31,m1->m32,m1->m33,m1->m34,m1->m41,m1->m42,m1->m43,m1->m44);
}

void display_vector41(vector41_t *v1)
{
    PRINT("( %f\t%f\t%f\t%f )\n", v1->x1 ,v1->x2, v1->x3, v1->x4);
}


// matrix of size 6

void mul_mat66( matrix66_t* out, matrix66_t *m1, matrix66_t *m2 )
{
  /// You can't have output & input pointing to the same location
  VP_OS_ASSERT( out != m1 );
  VP_OS_ASSERT( out != m2 );

  out->m11 = (m1->m11)*(m2->m11) + (m1->m12)*(m2->m21) + (m1->m13)*(m2->m31) + (m1->m14)*(m2->m41) + (m1->m15)*(m2->m51) + (m1->m16)*(m2->m61);
  out->m12 = (m1->m11)*(m2->m12) + (m1->m12)*(m2->m22) + (m1->m13)*(m2->m32) + (m1->m14)*(m2->m42) + (m1->m15)*(m2->m52) + (m1->m16)*(m2->m62);
  out->m13 = (m1->m11)*(m2->m13) + (m1->m12)*(m2->m23) + (m1->m13)*(m2->m33) + (m1->m14)*(m2->m43) + (m1->m15)*(m2->m53) + (m1->m16)*(m2->m63);
  out->m14 = (m1->m11)*(m2->m14) + (m1->m12)*(m2->m24) + (m1->m13)*(m2->m34) + (m1->m14)*(m2->m44) + (m1->m15)*(m2->m54) + (m1->m16)*(m2->m64);
  out->m15 = (m1->m11)*(m2->m15) + (m1->m12)*(m2->m25) + (m1->m13)*(m2->m35) + (m1->m14)*(m2->m45) + (m1->m15)*(m2->m55) + (m1->m16)*(m2->m65);
  out->m16 = (m1->m11)*(m2->m16) + (m1->m12)*(m2->m26) + (m1->m13)*(m2->m36) + (m1->m14)*(m2->m46) + (m1->m15)*(m2->m56) + (m1->m16)*(m2->m66);

  out->m21 = (m1->m21)*(m2->m11) + (m1->m22)*(m2->m21) + (m1->m23)*(m2->m31) + (m1->m24)*(m2->m41) + (m1->m25)*(m2->m51) + (m1->m26)*(m2->m61);
  out->m22 = (m1->m21)*(m2->m12) + (m1->m22)*(m2->m22) + (m1->m23)*(m2->m32) + (m1->m24)*(m2->m42) + (m1->m25)*(m2->m52) + (m1->m26)*(m2->m62);
  out->m23 = (m1->m21)*(m2->m13) + (m1->m22)*(m2->m23) + (m1->m23)*(m2->m33) + (m1->m24)*(m2->m43) + (m1->m25)*(m2->m53) + (m1->m26)*(m2->m63);
  out->m24 = (m1->m21)*(m2->m14) + (m1->m22)*(m2->m24) + (m1->m23)*(m2->m34) + (m1->m24)*(m2->m44) + (m1->m25)*(m2->m54) + (m1->m26)*(m2->m64);
  out->m25 = (m1->m21)*(m2->m15) + (m1->m22)*(m2->m25) + (m1->m23)*(m2->m35) + (m1->m24)*(m2->m45) + (m1->m25)*(m2->m55) + (m1->m26)*(m2->m65);
  out->m26 = (m1->m21)*(m2->m16) + (m1->m22)*(m2->m26) + (m1->m23)*(m2->m36) + (m1->m24)*(m2->m46) + (m1->m25)*(m2->m56) + (m1->m26)*(m2->m66);

  out->m31 = (m1->m31)*(m2->m11) + (m1->m32)*(m2->m21) + (m1->m33)*(m2->m31) + (m1->m34)*(m2->m41) + (m1->m35)*(m2->m51) + (m1->m36)*(m2->m61);
  out->m32 = (m1->m31)*(m2->m12) + (m1->m32)*(m2->m22) + (m1->m33)*(m2->m32) + (m1->m34)*(m2->m42) + (m1->m35)*(m2->m52) + (m1->m36)*(m2->m62);
  out->m33 = (m1->m31)*(m2->m13) + (m1->m32)*(m2->m23) + (m1->m33)*(m2->m33) + (m1->m34)*(m2->m43) + (m1->m35)*(m2->m53) + (m1->m36)*(m2->m63);
  out->m34 = (m1->m31)*(m2->m14) + (m1->m32)*(m2->m24) + (m1->m33)*(m2->m34) + (m1->m34)*(m2->m44) + (m1->m35)*(m2->m54) + (m1->m36)*(m2->m64);
  out->m35 = (m1->m31)*(m2->m15) + (m1->m32)*(m2->m25) + (m1->m33)*(m2->m35) + (m1->m34)*(m2->m45) + (m1->m35)*(m2->m55) + (m1->m36)*(m2->m65);
  out->m36 = (m1->m31)*(m2->m16) + (m1->m32)*(m2->m26) + (m1->m33)*(m2->m36) + (m1->m34)*(m2->m46) + (m1->m35)*(m2->m56) + (m1->m36)*(m2->m66);

  out->m41 = (m1->m41)*(m2->m11) + (m1->m42)*(m2->m21) + (m1->m43)*(m2->m31) + (m1->m44)*(m2->m41) + (m1->m45)*(m2->m51) + (m1->m46)*(m2->m61);
  out->m42 = (m1->m41)*(m2->m12) + (m1->m42)*(m2->m22) + (m1->m43)*(m2->m32) + (m1->m44)*(m2->m42) + (m1->m45)*(m2->m52) + (m1->m46)*(m2->m62);
  out->m43 = (m1->m41)*(m2->m13) + (m1->m42)*(m2->m23) + (m1->m43)*(m2->m33) + (m1->m44)*(m2->m43) + (m1->m45)*(m2->m53) + (m1->m46)*(m2->m63);
  out->m44 = (m1->m41)*(m2->m14) + (m1->m42)*(m2->m24) + (m1->m43)*(m2->m34) + (m1->m44)*(m2->m44) + (m1->m45)*(m2->m54) + (m1->m46)*(m2->m64);
  out->m45 = (m1->m41)*(m2->m15) + (m1->m42)*(m2->m25) + (m1->m43)*(m2->m35) + (m1->m44)*(m2->m45) + (m1->m45)*(m2->m55) + (m1->m46)*(m2->m65);
  out->m46 = (m1->m41)*(m2->m16) + (m1->m42)*(m2->m26) + (m1->m43)*(m2->m36) + (m1->m44)*(m2->m46) + (m1->m45)*(m2->m56) + (m1->m46)*(m2->m66);

  out->m51 = (m1->m51)*(m2->m11) + (m1->m52)*(m2->m21) + (m1->m53)*(m2->m31) + (m1->m54)*(m2->m41) + (m1->m55)*(m2->m51) + (m1->m56)*(m2->m61);
  out->m52 = (m1->m51)*(m2->m12) + (m1->m52)*(m2->m22) + (m1->m53)*(m2->m32) + (m1->m54)*(m2->m42) + (m1->m55)*(m2->m52) + (m1->m56)*(m2->m62);
  out->m53 = (m1->m51)*(m2->m13) + (m1->m52)*(m2->m23) + (m1->m53)*(m2->m33) + (m1->m54)*(m2->m43) + (m1->m55)*(m2->m53) + (m1->m56)*(m2->m63);
  out->m54 = (m1->m51)*(m2->m14) + (m1->m52)*(m2->m24) + (m1->m53)*(m2->m34) + (m1->m54)*(m2->m44) + (m1->m55)*(m2->m54) + (m1->m56)*(m2->m64);
  out->m55 = (m1->m51)*(m2->m15) + (m1->m52)*(m2->m25) + (m1->m53)*(m2->m35) + (m1->m54)*(m2->m45) + (m1->m55)*(m2->m55) + (m1->m56)*(m2->m65);
  out->m56 = (m1->m51)*(m2->m16) + (m1->m52)*(m2->m26) + (m1->m53)*(m2->m36) + (m1->m54)*(m2->m46) + (m1->m55)*(m2->m56) + (m1->m56)*(m2->m66);

  out->m61 = (m1->m61)*(m2->m11) + (m1->m62)*(m2->m21) + (m1->m63)*(m2->m31) + (m1->m64)*(m2->m41) + (m1->m65)*(m2->m51) + (m1->m66)*(m2->m61);
  out->m62 = (m1->m61)*(m2->m12) + (m1->m62)*(m2->m22) + (m1->m63)*(m2->m32) + (m1->m64)*(m2->m42) + (m1->m65)*(m2->m52) + (m1->m66)*(m2->m62);
  out->m63 = (m1->m61)*(m2->m13) + (m1->m62)*(m2->m23) + (m1->m63)*(m2->m33) + (m1->m64)*(m2->m43) + (m1->m65)*(m2->m53) + (m1->m66)*(m2->m63);
  out->m64 = (m1->m61)*(m2->m14) + (m1->m62)*(m2->m24) + (m1->m63)*(m2->m34) + (m1->m64)*(m2->m44) + (m1->m65)*(m2->m54) + (m1->m66)*(m2->m64);
  out->m65 = (m1->m61)*(m2->m15) + (m1->m62)*(m2->m25) + (m1->m63)*(m2->m35) + (m1->m64)*(m2->m45) + (m1->m65)*(m2->m55) + (m1->m66)*(m2->m65);
  out->m66 = (m1->m61)*(m2->m16) + (m1->m62)*(m2->m26) + (m1->m63)*(m2->m36) + (m1->m64)*(m2->m46) + (m1->m65)*(m2->m56) + (m1->m66)*(m2->m66);
}

void add_mat66( matrix66_t* out, matrix66_t *m1, matrix66_t *m2 )
{
  out->m11 = (m1->m11) + (m2->m11);
  out->m12 = (m1->m12) + (m2->m12);
  out->m13 = (m1->m13) + (m2->m13);
  out->m14 = (m1->m14) + (m2->m14);
  out->m15 = (m1->m15) + (m2->m15);
  out->m16 = (m1->m16) + (m2->m16);

  out->m21 = (m1->m21) + (m2->m21);
  out->m22 = (m1->m22) + (m2->m22);
  out->m23 = (m1->m23) + (m2->m23);
  out->m24 = (m1->m24) + (m2->m24);
  out->m25 = (m1->m25) + (m2->m25);
  out->m26 = (m1->m26) + (m2->m26);

  out->m31 = (m1->m31) + (m2->m31);
  out->m32 = (m1->m32) + (m2->m32);
  out->m33 = (m1->m33) + (m2->m33);
  out->m34 = (m1->m34) + (m2->m34);
  out->m35 = (m1->m35) + (m2->m35);
  out->m36 = (m1->m36) + (m2->m36);

  out->m41 = (m1->m41) + (m2->m41);
  out->m42 = (m1->m42) + (m2->m42);
  out->m43 = (m1->m43) + (m2->m43);
  out->m44 = (m1->m44) + (m2->m44);
  out->m45 = (m1->m45) + (m2->m45);
  out->m46 = (m1->m46) + (m2->m46);

  out->m51 = (m1->m51) + (m2->m51);
  out->m52 = (m1->m52) + (m2->m52);
  out->m53 = (m1->m53) + (m2->m53);
  out->m54 = (m1->m54) + (m2->m54);
  out->m55 = (m1->m55) + (m2->m55);
  out->m56 = (m1->m56) + (m2->m56);

  out->m61 = (m1->m61) + (m2->m61);
  out->m62 = (m1->m62) + (m2->m62);
  out->m63 = (m1->m63) + (m2->m63);
  out->m64 = (m1->m64) + (m2->m64);
  out->m65 = (m1->m65) + (m2->m65);
  out->m66 = (m1->m66) + (m2->m66);
}

void mulvec_mat6( vector61_t* out, matrix66_t *m1, vector61_t *v1 )
{
  out->x1 = (m1->m11) * (v1->x1) + (m1->m12) * (v1->x2) + (m1->m13) * (v1->x3) + (m1->m14) * (v1->x4) + (m1->m15) * (v1->x5) + (m1->m16) * (v1->x6);
  out->x2 = (m1->m21) * (v1->x1) + (m1->m22) * (v1->x2) + (m1->m23) * (v1->x3) + (m1->m24) * (v1->x4) + (m1->m25) * (v1->x5) + (m1->m26) * (v1->x6);
  out->x3 = (m1->m31) * (v1->x1) + (m1->m32) * (v1->x2) + (m1->m33) * (v1->x3) + (m1->m34) * (v1->x4) + (m1->m35) * (v1->x5) + (m1->m36) * (v1->x6);
  out->x4 = (m1->m41) * (v1->x1) + (m1->m42) * (v1->x2) + (m1->m43) * (v1->x3) + (m1->m44) * (v1->x4) + (m1->m45) * (v1->x5) + (m1->m46) * (v1->x6);
  out->x5 = (m1->m51) * (v1->x1) + (m1->m52) * (v1->x2) + (m1->m53) * (v1->x3) + (m1->m54) * (v1->x4) + (m1->m55) * (v1->x5) + (m1->m56) * (v1->x6);
  out->x6 = (m1->m61) * (v1->x1) + (m1->m62) * (v1->x2) + (m1->m63) * (v1->x3) + (m1->m64) * (v1->x4) + (m1->m65) * (v1->x5) + (m1->m66) * (v1->x6);
}

void transpose_mat66( matrix66_t *out, matrix66_t *m1 )
{
/// You can't have output & input pointing to the same location
  VP_OS_ASSERT( out != m1 );

  out->m11 = m1->m11;
  out->m12 = m1->m21;
  out->m13 = m1->m31;
  out->m14 = m1->m41;
  out->m15 = m1->m51;
  out->m16 = m1->m61;

  out->m21 = m1->m12;
  out->m22 = m1->m22;
  out->m23 = m1->m32;
  out->m24 = m1->m42;
  out->m25 = m1->m52;
  out->m26 = m1->m62;

  out->m31 = m1->m13;
  out->m32 = m1->m23;
  out->m33 = m1->m33;
  out->m34 = m1->m43;
  out->m35 = m1->m53;
  out->m36 = m1->m63;

  out->m41 = m1->m14;
  out->m42 = m1->m24;
  out->m43 = m1->m34;
  out->m44 = m1->m44;
  out->m45 = m1->m54;
  out->m46 = m1->m64;

  out->m51 = m1->m15;
  out->m52 = m1->m25;
  out->m53 = m1->m35;
  out->m54 = m1->m45;
  out->m55 = m1->m55;
  out->m56 = m1->m65;

  out->m61 = m1->m16;
  out->m62 = m1->m26;
  out->m63 = m1->m36;
  out->m64 = m1->m46;
  out->m65 = m1->m56;
  out->m66 = m1->m66;
}

void mulconst_mat66( matrix66_t *out, matrix66_t *m1, float32_t k )
{
  out->m11 = m1->m11 * k;
  out->m12 = m1->m12 * k;
  out->m13 = m1->m13 * k;
  out->m14 = m1->m14 * k;
  out->m15 = m1->m15 * k;
  out->m16 = m1->m16 * k;

  out->m21 = m1->m21 * k;
  out->m22 = m1->m22 * k;
  out->m23 = m1->m23 * k;
  out->m24 = m1->m24 * k;
  out->m25 = m1->m25 * k;
  out->m26 = m1->m26 * k;

  out->m31 = m1->m31 * k;
  out->m32 = m1->m32 * k;
  out->m33 = m1->m33 * k;
  out->m34 = m1->m34 * k;
  out->m35 = m1->m35 * k;
  out->m36 = m1->m36 * k;

  out->m41 = m1->m41 * k;
  out->m42 = m1->m42 * k;
  out->m43 = m1->m43 * k;
  out->m44 = m1->m44 * k;
  out->m45 = m1->m45 * k;
  out->m46 = m1->m46 * k;

  out->m51 = m1->m51 * k;
  out->m52 = m1->m52 * k;
  out->m53 = m1->m53 * k;
  out->m54 = m1->m54 * k;
  out->m55 = m1->m55 * k;
  out->m56 = m1->m56 * k;

  out->m61 = m1->m61 * k;
  out->m62 = m1->m62 * k;
  out->m63 = m1->m63 * k;
  out->m64 = m1->m64 * k;
  out->m65 = m1->m65 * k;
  out->m66 = m1->m66 * k;

}

void add_vec61( vector61_t* out, vector61_t *v1, vector61_t *v2 )
{
  out->x1 = (v1->x1) + (v2->x1);
  out->x2 = (v1->x2) + (v2->x2);
  out->x3 = (v1->x3) + (v2->x3);
  out->x4 = (v1->x4) + (v2->x4);
  out->x5 = (v1->x5) + (v2->x5);
  out->x6 = (v1->x6) + (v2->x6);

}

void mulconst_vec61( vector61_t *out, vector61_t *v1, float32_t k )
{
  out->x1 = (v1->x1) * k;
  out->x2 = (v1->x2) * k;
  out->x3 = (v1->x3) * k;
  out->x4 = (v1->x4) * k;
  out->x5 = (v1->x5) * k;
  out->x6 = (v1->x6) * k;
}


// matrix of size 2

void mul_mat22( matrix22_t* out, matrix22_t *m1, matrix22_t *m2 )
{
  /// You can't have output & input pointing to the same location
  VP_OS_ASSERT( out != m1 );
  VP_OS_ASSERT( out != m2 );

  out->m11 = (m1->m11)*(m2->m11) + (m1->m12)*(m2->m21);
  out->m12 = (m1->m11)*(m2->m12) + (m1->m12)*(m2->m22);

  out->m21 = (m1->m21)*(m2->m11) + (m1->m22)*(m2->m21);
  out->m22 = (m1->m21)*(m2->m12) + (m1->m22)*(m2->m22);
}

void add_mat22( matrix22_t* out, matrix22_t *m1, matrix22_t *m2 )
{
  out->m11 = (m1->m11) + (m2->m11);
  out->m12 = (m1->m12) + (m2->m12);

  out->m21 = (m1->m21) + (m2->m21);
  out->m22 = (m1->m22) + (m2->m22);
}

void mulvec_mat2( vector21_t* out, matrix22_t *m1, vector21_t *v1 )
{
  out->x = (m1->m11) * (v1->x) + (m1->m12) * (v1->y);
  out->y = (m1->m21) * (v1->x) + (m1->m22) * (v1->y);
}

void transpose_mat22( matrix22_t *out, matrix22_t *m1 )
{
/// You can't have output & input pointing to the same location
  VP_OS_ASSERT( out != m1 );

  out->m11 = m1->m11;
  out->m12 = m1->m21;
  out->m21 = m1->m12;
  out->m22 = m1->m22;
}

void mulconst_mat22( matrix22_t *out, matrix22_t *m1, float32_t k )
{
  out->m11 = m1->m11 * k;
  out->m12 = m1->m12 * k;
  out->m21 = m1->m21 * k;
  out->m22 = m1->m22 * k;
}

void add_vec21( vector21_t* out, vector21_t *v1, vector21_t *v2 )
{
  out->x = (v1->x) + (v2->x);
  out->y = (v1->y) + (v2->y);
}

void mulconst_vec21( vector21_t *out, vector21_t *v1, float32_t k )
{
  out->x = (v1->x) * k;
  out->y = (v1->y) * k;
}

void det_mat2(float32_t *out, matrix22_t *m1)
{
    *out = m1->m11*m1->m22 - m1->m12*m1->m21;
}

void comatrice22(matrix22_t *out, matrix22_t *m1)
{
    out->m11 =   m1->m22;
    out->m12 =  -m1->m21;
    out->m21 =  -m1->m12;
    out->m22 =   m1->m11;
}

void inv_mat22(matrix22_t *out, matrix22_t* m1)
{
    float32_t det;
    matrix22_t Com, t_Com;

    det_mat2(&det,m1);

    if(f_is_zero(det))
    {
        memcpy(out, &matrix_null2, sizeof(matrix22_t));
    }
    else
    {
        comatrice22(&Com, m1);
        transpose_mat22(&t_Com, &Com);
        mulconst_mat22(out, &t_Com, 1.0/det);
    }
}

// NON-square matrices and operations between diferrent-sized matrices

void mulmat26vec61( vector21_t* out, matrix26_t *m1, vector61_t *v1 )
{
  out->x = (m1->m11) * (v1->x1) + (m1->m12) * (v1->x2) + (m1->m13) * (v1->x3) + (m1->m14) * (v1->x4) + (m1->m15) * (v1->x5) + (m1->m16) * (v1->x6);
  out->y = (m1->m21) * (v1->x1) + (m1->m22) * (v1->x2) + (m1->m23) * (v1->x3) + (m1->m24) * (v1->x4) + (m1->m25) * (v1->x5) + (m1->m26) * (v1->x6);
}

void mulmat46vec61( vector41_t* out, matrix46_t *m1, vector61_t *v1 )
{
  out->x1 = (m1->m11) * (v1->x1) + (m1->m12) * (v1->x2) + (m1->m13) * (v1->x3) + (m1->m14) * (v1->x4) + (m1->m15) * (v1->x5) + (m1->m16) * (v1->x6);
  out->x2 = (m1->m21) * (v1->x1) + (m1->m22) * (v1->x2) + (m1->m23) * (v1->x3) + (m1->m24) * (v1->x4) + (m1->m25) * (v1->x5) + (m1->m26) * (v1->x6);
  out->x3 = (m1->m31) * (v1->x1) + (m1->m32) * (v1->x2) + (m1->m33) * (v1->x3) + (m1->m34) * (v1->x4) + (m1->m35) * (v1->x5) + (m1->m36) * (v1->x6);
  out->x4 = (m1->m41) * (v1->x1) + (m1->m42) * (v1->x2) + (m1->m43) * (v1->x3) + (m1->m44) * (v1->x4) + (m1->m45) * (v1->x5) + (m1->m46) * (v1->x6);
}

void mulmat46mat66( matrix46_t* out, matrix46_t *m1, matrix66_t *m2 )
{
  /// You can't have output & input pointing to the same location
  VP_OS_ASSERT( out != m1 );

  out->m11 = (m1->m11)*(m2->m11) + (m1->m12)*(m2->m21) + (m1->m13)*(m2->m31) + (m1->m14)*(m2->m41) + (m1->m15)*(m2->m51) + (m1->m16)*(m2->m61);
  out->m12 = (m1->m11)*(m2->m12) + (m1->m12)*(m2->m22) + (m1->m13)*(m2->m32) + (m1->m14)*(m2->m42) + (m1->m15)*(m2->m52) + (m1->m16)*(m2->m62);
  out->m13 = (m1->m11)*(m2->m13) + (m1->m12)*(m2->m23) + (m1->m13)*(m2->m33) + (m1->m14)*(m2->m43) + (m1->m15)*(m2->m53) + (m1->m16)*(m2->m63);
  out->m14 = (m1->m11)*(m2->m14) + (m1->m12)*(m2->m24) + (m1->m13)*(m2->m34) + (m1->m14)*(m2->m44) + (m1->m15)*(m2->m54) + (m1->m16)*(m2->m64);
  out->m15 = (m1->m11)*(m2->m15) + (m1->m12)*(m2->m25) + (m1->m13)*(m2->m35) + (m1->m14)*(m2->m45) + (m1->m15)*(m2->m55) + (m1->m16)*(m2->m65);
  out->m16 = (m1->m11)*(m2->m16) + (m1->m12)*(m2->m26) + (m1->m13)*(m2->m36) + (m1->m14)*(m2->m46) + (m1->m15)*(m2->m56) + (m1->m16)*(m2->m66);

  out->m21 = (m1->m21)*(m2->m11) + (m1->m22)*(m2->m21) + (m1->m23)*(m2->m31) + (m1->m24)*(m2->m41) + (m1->m25)*(m2->m51) + (m1->m26)*(m2->m61);
  out->m22 = (m1->m21)*(m2->m12) + (m1->m22)*(m2->m22) + (m1->m23)*(m2->m32) + (m1->m24)*(m2->m42) + (m1->m25)*(m2->m52) + (m1->m26)*(m2->m62);
  out->m23 = (m1->m21)*(m2->m13) + (m1->m22)*(m2->m23) + (m1->m23)*(m2->m33) + (m1->m24)*(m2->m43) + (m1->m25)*(m2->m53) + (m1->m26)*(m2->m63);
  out->m24 = (m1->m21)*(m2->m14) + (m1->m22)*(m2->m24) + (m1->m23)*(m2->m34) + (m1->m24)*(m2->m44) + (m1->m25)*(m2->m54) + (m1->m26)*(m2->m64);
  out->m25 = (m1->m21)*(m2->m15) + (m1->m22)*(m2->m25) + (m1->m23)*(m2->m35) + (m1->m24)*(m2->m45) + (m1->m25)*(m2->m55) + (m1->m26)*(m2->m65);
  out->m26 = (m1->m21)*(m2->m16) + (m1->m22)*(m2->m26) + (m1->m23)*(m2->m36) + (m1->m24)*(m2->m46) + (m1->m25)*(m2->m56) + (m1->m26)*(m2->m66);

  out->m31 = (m1->m31)*(m2->m11) + (m1->m32)*(m2->m21) + (m1->m33)*(m2->m31) + (m1->m34)*(m2->m41) + (m1->m35)*(m2->m51) + (m1->m36)*(m2->m61);
  out->m32 = (m1->m31)*(m2->m12) + (m1->m32)*(m2->m22) + (m1->m33)*(m2->m32) + (m1->m34)*(m2->m42) + (m1->m35)*(m2->m52) + (m1->m36)*(m2->m62);
  out->m33 = (m1->m31)*(m2->m13) + (m1->m32)*(m2->m23) + (m1->m33)*(m2->m33) + (m1->m34)*(m2->m43) + (m1->m35)*(m2->m53) + (m1->m36)*(m2->m63);
  out->m34 = (m1->m31)*(m2->m14) + (m1->m32)*(m2->m24) + (m1->m33)*(m2->m34) + (m1->m34)*(m2->m44) + (m1->m35)*(m2->m54) + (m1->m36)*(m2->m64);
  out->m35 = (m1->m31)*(m2->m15) + (m1->m32)*(m2->m25) + (m1->m33)*(m2->m35) + (m1->m34)*(m2->m45) + (m1->m35)*(m2->m55) + (m1->m36)*(m2->m65);
  out->m36 = (m1->m31)*(m2->m16) + (m1->m32)*(m2->m26) + (m1->m33)*(m2->m36) + (m1->m34)*(m2->m46) + (m1->m35)*(m2->m56) + (m1->m36)*(m2->m66);

  out->m41 = (m1->m41)*(m2->m11) + (m1->m42)*(m2->m21) + (m1->m43)*(m2->m31) + (m1->m44)*(m2->m41) + (m1->m45)*(m2->m51) + (m1->m46)*(m2->m61);
  out->m42 = (m1->m41)*(m2->m12) + (m1->m42)*(m2->m22) + (m1->m43)*(m2->m32) + (m1->m44)*(m2->m42) + (m1->m45)*(m2->m52) + (m1->m46)*(m2->m62);
  out->m43 = (m1->m41)*(m2->m13) + (m1->m42)*(m2->m23) + (m1->m43)*(m2->m33) + (m1->m44)*(m2->m43) + (m1->m45)*(m2->m53) + (m1->m46)*(m2->m63);
  out->m44 = (m1->m41)*(m2->m14) + (m1->m42)*(m2->m24) + (m1->m43)*(m2->m34) + (m1->m44)*(m2->m44) + (m1->m45)*(m2->m54) + (m1->m46)*(m2->m64);
  out->m45 = (m1->m41)*(m2->m15) + (m1->m42)*(m2->m25) + (m1->m43)*(m2->m35) + (m1->m44)*(m2->m45) + (m1->m45)*(m2->m55) + (m1->m46)*(m2->m65);
  out->m46 = (m1->m41)*(m2->m16) + (m1->m42)*(m2->m26) + (m1->m43)*(m2->m36) + (m1->m44)*(m2->m46) + (m1->m45)*(m2->m56) + (m1->m46)*(m2->m66);
}

void mulmat46mat64( matrix44_t* out, matrix46_t *m1, matrix64_t *m2 )
{
  out->m11 = (m1->m11)*(m2->m11) + (m1->m12)*(m2->m21) + (m1->m13)*(m2->m31) + (m1->m14)*(m2->m41) + (m1->m15)*(m2->m51) + (m1->m16)*(m2->m61);
  out->m12 = (m1->m11)*(m2->m12) + (m1->m12)*(m2->m22) + (m1->m13)*(m2->m32) + (m1->m14)*(m2->m42) + (m1->m15)*(m2->m52) + (m1->m16)*(m2->m62);
  out->m13 = (m1->m11)*(m2->m13) + (m1->m12)*(m2->m23) + (m1->m13)*(m2->m33) + (m1->m14)*(m2->m43) + (m1->m15)*(m2->m53) + (m1->m16)*(m2->m63);
  out->m14 = (m1->m11)*(m2->m14) + (m1->m12)*(m2->m24) + (m1->m13)*(m2->m34) + (m1->m14)*(m2->m44) + (m1->m15)*(m2->m54) + (m1->m16)*(m2->m64);

  out->m21 = (m1->m21)*(m2->m11) + (m1->m22)*(m2->m21) + (m1->m23)*(m2->m31) + (m1->m24)*(m2->m41) + (m1->m25)*(m2->m51) + (m1->m26)*(m2->m61);
  out->m22 = (m1->m21)*(m2->m12) + (m1->m22)*(m2->m22) + (m1->m23)*(m2->m32) + (m1->m24)*(m2->m42) + (m1->m25)*(m2->m52) + (m1->m26)*(m2->m62);
  out->m23 = (m1->m21)*(m2->m13) + (m1->m22)*(m2->m23) + (m1->m23)*(m2->m33) + (m1->m24)*(m2->m43) + (m1->m25)*(m2->m53) + (m1->m26)*(m2->m63);
  out->m24 = (m1->m21)*(m2->m14) + (m1->m22)*(m2->m24) + (m1->m23)*(m2->m34) + (m1->m24)*(m2->m44) + (m1->m25)*(m2->m54) + (m1->m26)*(m2->m64);

  out->m31 = (m1->m31)*(m2->m11) + (m1->m32)*(m2->m21) + (m1->m33)*(m2->m31) + (m1->m34)*(m2->m41) + (m1->m35)*(m2->m51) + (m1->m36)*(m2->m61);
  out->m32 = (m1->m31)*(m2->m12) + (m1->m32)*(m2->m22) + (m1->m33)*(m2->m32) + (m1->m34)*(m2->m42) + (m1->m35)*(m2->m52) + (m1->m36)*(m2->m62);
  out->m33 = (m1->m31)*(m2->m13) + (m1->m32)*(m2->m23) + (m1->m33)*(m2->m33) + (m1->m34)*(m2->m43) + (m1->m35)*(m2->m53) + (m1->m36)*(m2->m63);
  out->m34 = (m1->m31)*(m2->m14) + (m1->m32)*(m2->m24) + (m1->m33)*(m2->m34) + (m1->m34)*(m2->m44) + (m1->m35)*(m2->m54) + (m1->m36)*(m2->m64);

  out->m41 = (m1->m41)*(m2->m11) + (m1->m42)*(m2->m21) + (m1->m43)*(m2->m31) + (m1->m44)*(m2->m41) + (m1->m45)*(m2->m51) + (m1->m46)*(m2->m61);
  out->m42 = (m1->m41)*(m2->m12) + (m1->m42)*(m2->m22) + (m1->m43)*(m2->m32) + (m1->m44)*(m2->m42) + (m1->m45)*(m2->m52) + (m1->m46)*(m2->m62);
  out->m43 = (m1->m41)*(m2->m13) + (m1->m42)*(m2->m23) + (m1->m43)*(m2->m33) + (m1->m44)*(m2->m43) + (m1->m45)*(m2->m53) + (m1->m46)*(m2->m63);
  out->m44 = (m1->m41)*(m2->m14) + (m1->m42)*(m2->m24) + (m1->m43)*(m2->m34) + (m1->m44)*(m2->m44) + (m1->m45)*(m2->m54) + (m1->m46)*(m2->m64);
}

void mulmat64mat46( matrix66_t* out, matrix64_t *m1, matrix46_t *m2 )
{
  out->m11 = (m1->m11)*(m2->m11) + (m1->m12)*(m2->m21) + (m1->m13)*(m2->m31) + (m1->m14)*(m2->m41);
  out->m12 = (m1->m11)*(m2->m12) + (m1->m12)*(m2->m22) + (m1->m13)*(m2->m32) + (m1->m14)*(m2->m42);
  out->m13 = (m1->m11)*(m2->m13) + (m1->m12)*(m2->m23) + (m1->m13)*(m2->m33) + (m1->m14)*(m2->m43);
  out->m14 = (m1->m11)*(m2->m14) + (m1->m12)*(m2->m24) + (m1->m13)*(m2->m34) + (m1->m14)*(m2->m44);
  out->m15 = (m1->m11)*(m2->m15) + (m1->m12)*(m2->m25) + (m1->m13)*(m2->m35) + (m1->m14)*(m2->m45);
  out->m16 = (m1->m11)*(m2->m16) + (m1->m12)*(m2->m26) + (m1->m13)*(m2->m36) + (m1->m14)*(m2->m46);

  out->m21 = (m1->m21)*(m2->m11) + (m1->m22)*(m2->m21) + (m1->m23)*(m2->m31) + (m1->m24)*(m2->m41);
  out->m22 = (m1->m21)*(m2->m12) + (m1->m22)*(m2->m22) + (m1->m23)*(m2->m32) + (m1->m24)*(m2->m42);
  out->m23 = (m1->m21)*(m2->m13) + (m1->m22)*(m2->m23) + (m1->m23)*(m2->m33) + (m1->m24)*(m2->m43);
  out->m24 = (m1->m21)*(m2->m14) + (m1->m22)*(m2->m24) + (m1->m23)*(m2->m34) + (m1->m24)*(m2->m44);
  out->m25 = (m1->m21)*(m2->m15) + (m1->m22)*(m2->m25) + (m1->m23)*(m2->m35) + (m1->m24)*(m2->m45);
  out->m26 = (m1->m21)*(m2->m16) + (m1->m22)*(m2->m26) + (m1->m23)*(m2->m36) + (m1->m24)*(m2->m46);

  out->m31 = (m1->m31)*(m2->m11) + (m1->m32)*(m2->m21) + (m1->m33)*(m2->m31) + (m1->m34)*(m2->m41);
  out->m32 = (m1->m31)*(m2->m12) + (m1->m32)*(m2->m22) + (m1->m33)*(m2->m32) + (m1->m34)*(m2->m42);
  out->m33 = (m1->m31)*(m2->m13) + (m1->m32)*(m2->m23) + (m1->m33)*(m2->m33) + (m1->m34)*(m2->m43);
  out->m34 = (m1->m31)*(m2->m14) + (m1->m32)*(m2->m24) + (m1->m33)*(m2->m34) + (m1->m34)*(m2->m44);
  out->m35 = (m1->m31)*(m2->m15) + (m1->m32)*(m2->m25) + (m1->m33)*(m2->m35) + (m1->m34)*(m2->m45);
  out->m36 = (m1->m31)*(m2->m16) + (m1->m32)*(m2->m26) + (m1->m33)*(m2->m36) + (m1->m34)*(m2->m46);

  out->m41 = (m1->m41)*(m2->m11) + (m1->m42)*(m2->m21) + (m1->m43)*(m2->m31) + (m1->m44)*(m2->m41);
  out->m42 = (m1->m41)*(m2->m12) + (m1->m42)*(m2->m22) + (m1->m43)*(m2->m32) + (m1->m44)*(m2->m42);
  out->m43 = (m1->m41)*(m2->m13) + (m1->m42)*(m2->m23) + (m1->m43)*(m2->m33) + (m1->m44)*(m2->m43);
  out->m44 = (m1->m41)*(m2->m14) + (m1->m42)*(m2->m24) + (m1->m43)*(m2->m34) + (m1->m44)*(m2->m44);
  out->m45 = (m1->m41)*(m2->m15) + (m1->m42)*(m2->m25) + (m1->m43)*(m2->m35) + (m1->m44)*(m2->m45);
  out->m46 = (m1->m41)*(m2->m16) + (m1->m42)*(m2->m26) + (m1->m43)*(m2->m36) + (m1->m44)*(m2->m46);

  out->m51 = (m1->m51)*(m2->m11) + (m1->m52)*(m2->m21) + (m1->m53)*(m2->m31) + (m1->m54)*(m2->m41);
  out->m52 = (m1->m51)*(m2->m12) + (m1->m52)*(m2->m22) + (m1->m53)*(m2->m32) + (m1->m54)*(m2->m42);
  out->m53 = (m1->m51)*(m2->m13) + (m1->m52)*(m2->m23) + (m1->m53)*(m2->m33) + (m1->m54)*(m2->m43);
  out->m54 = (m1->m51)*(m2->m14) + (m1->m52)*(m2->m24) + (m1->m53)*(m2->m34) + (m1->m54)*(m2->m44);
  out->m55 = (m1->m51)*(m2->m15) + (m1->m52)*(m2->m25) + (m1->m53)*(m2->m35) + (m1->m54)*(m2->m45);
  out->m56 = (m1->m51)*(m2->m16) + (m1->m52)*(m2->m26) + (m1->m53)*(m2->m36) + (m1->m54)*(m2->m46);

  out->m61 = (m1->m61)*(m2->m11) + (m1->m62)*(m2->m21) + (m1->m63)*(m2->m31) + (m1->m64)*(m2->m41);
  out->m62 = (m1->m61)*(m2->m12) + (m1->m62)*(m2->m22) + (m1->m63)*(m2->m32) + (m1->m64)*(m2->m42);
  out->m63 = (m1->m61)*(m2->m13) + (m1->m62)*(m2->m23) + (m1->m63)*(m2->m33) + (m1->m64)*(m2->m43);
  out->m64 = (m1->m61)*(m2->m14) + (m1->m62)*(m2->m24) + (m1->m63)*(m2->m34) + (m1->m64)*(m2->m44);
  out->m65 = (m1->m61)*(m2->m15) + (m1->m62)*(m2->m25) + (m1->m63)*(m2->m35) + (m1->m64)*(m2->m45);
  out->m66 = (m1->m61)*(m2->m16) + (m1->m62)*(m2->m26) + (m1->m63)*(m2->m36) + (m1->m64)*(m2->m46);
}

void mulmat62mat26( matrix66_t* out, matrix62_t *m1, matrix26_t *m2 )
{
  out->m11 = (m1->m11)*(m2->m11) + (m1->m12)*(m2->m21);
  out->m12 = (m1->m11)*(m2->m12) + (m1->m12)*(m2->m22);
  out->m13 = (m1->m11)*(m2->m13) + (m1->m12)*(m2->m23);
  out->m14 = (m1->m11)*(m2->m14) + (m1->m12)*(m2->m24);
  out->m15 = (m1->m11)*(m2->m15) + (m1->m12)*(m2->m25);
  out->m16 = (m1->m11)*(m2->m16) + (m1->m12)*(m2->m26);

  out->m21 = (m1->m21)*(m2->m11) + (m1->m22)*(m2->m21);
  out->m22 = (m1->m21)*(m2->m12) + (m1->m22)*(m2->m22);
  out->m23 = (m1->m21)*(m2->m13) + (m1->m22)*(m2->m23);
  out->m24 = (m1->m21)*(m2->m14) + (m1->m22)*(m2->m24);
  out->m25 = (m1->m21)*(m2->m15) + (m1->m22)*(m2->m25);
  out->m26 = (m1->m21)*(m2->m16) + (m1->m22)*(m2->m26);

  out->m31 = (m1->m31)*(m2->m11) + (m1->m32)*(m2->m21);
  out->m32 = (m1->m31)*(m2->m12) + (m1->m32)*(m2->m22);
  out->m33 = (m1->m31)*(m2->m13) + (m1->m32)*(m2->m23);
  out->m34 = (m1->m31)*(m2->m14) + (m1->m32)*(m2->m24);
  out->m35 = (m1->m31)*(m2->m15) + (m1->m32)*(m2->m25);
  out->m36 = (m1->m31)*(m2->m16) + (m1->m32)*(m2->m26);

  out->m41 = (m1->m41)*(m2->m11) + (m1->m42)*(m2->m21);
  out->m42 = (m1->m41)*(m2->m12) + (m1->m42)*(m2->m22);
  out->m43 = (m1->m41)*(m2->m13) + (m1->m42)*(m2->m23);
  out->m44 = (m1->m41)*(m2->m14) + (m1->m42)*(m2->m24);
  out->m45 = (m1->m41)*(m2->m15) + (m1->m42)*(m2->m25);
  out->m46 = (m1->m41)*(m2->m16) + (m1->m42)*(m2->m26);

  out->m51 = (m1->m51)*(m2->m11) + (m1->m52)*(m2->m21);
  out->m52 = (m1->m51)*(m2->m12) + (m1->m52)*(m2->m22);
  out->m53 = (m1->m51)*(m2->m13) + (m1->m52)*(m2->m23);
  out->m54 = (m1->m51)*(m2->m14) + (m1->m52)*(m2->m24);
  out->m55 = (m1->m51)*(m2->m15) + (m1->m52)*(m2->m25);
  out->m56 = (m1->m51)*(m2->m16) + (m1->m52)*(m2->m26);

  out->m61 = (m1->m61)*(m2->m11) + (m1->m62)*(m2->m21);
  out->m62 = (m1->m61)*(m2->m12) + (m1->m62)*(m2->m22);
  out->m63 = (m1->m61)*(m2->m13) + (m1->m62)*(m2->m23);
  out->m64 = (m1->m61)*(m2->m14) + (m1->m62)*(m2->m24);
  out->m65 = (m1->m61)*(m2->m15) + (m1->m62)*(m2->m25);
  out->m66 = (m1->m61)*(m2->m16) + (m1->m62)*(m2->m26);
}

void mulmat26mat66( matrix26_t* out, matrix26_t *m1, matrix66_t *m2 )
{
  /// You can't have output & input pointing to the same location
  VP_OS_ASSERT( out != m1 );

  out->m11 = (m1->m11)*(m2->m11) + (m1->m12)*(m2->m21) + (m1->m13)*(m2->m31) + (m1->m14)*(m2->m41) + (m1->m15)*(m2->m51) + (m1->m16)*(m2->m61);
  out->m12 = (m1->m11)*(m2->m12) + (m1->m12)*(m2->m22) + (m1->m13)*(m2->m32) + (m1->m14)*(m2->m42) + (m1->m15)*(m2->m52) + (m1->m16)*(m2->m62);
  out->m13 = (m1->m11)*(m2->m13) + (m1->m12)*(m2->m23) + (m1->m13)*(m2->m33) + (m1->m14)*(m2->m43) + (m1->m15)*(m2->m53) + (m1->m16)*(m2->m63);
  out->m14 = (m1->m11)*(m2->m14) + (m1->m12)*(m2->m24) + (m1->m13)*(m2->m34) + (m1->m14)*(m2->m44) + (m1->m15)*(m2->m54) + (m1->m16)*(m2->m64);
  out->m15 = (m1->m11)*(m2->m15) + (m1->m12)*(m2->m25) + (m1->m13)*(m2->m35) + (m1->m14)*(m2->m45) + (m1->m15)*(m2->m55) + (m1->m16)*(m2->m65);
  out->m16 = (m1->m11)*(m2->m16) + (m1->m12)*(m2->m26) + (m1->m13)*(m2->m36) + (m1->m14)*(m2->m46) + (m1->m15)*(m2->m56) + (m1->m16)*(m2->m66);

  out->m21 = (m1->m21)*(m2->m11) + (m1->m22)*(m2->m21) + (m1->m23)*(m2->m31) + (m1->m24)*(m2->m41) + (m1->m25)*(m2->m51) + (m1->m26)*(m2->m61);
  out->m22 = (m1->m21)*(m2->m12) + (m1->m22)*(m2->m22) + (m1->m23)*(m2->m32) + (m1->m24)*(m2->m42) + (m1->m25)*(m2->m52) + (m1->m26)*(m2->m62);
  out->m23 = (m1->m21)*(m2->m13) + (m1->m22)*(m2->m23) + (m1->m23)*(m2->m33) + (m1->m24)*(m2->m43) + (m1->m25)*(m2->m53) + (m1->m26)*(m2->m63);
  out->m24 = (m1->m21)*(m2->m14) + (m1->m22)*(m2->m24) + (m1->m23)*(m2->m34) + (m1->m24)*(m2->m44) + (m1->m25)*(m2->m54) + (m1->m26)*(m2->m64);
  out->m25 = (m1->m21)*(m2->m15) + (m1->m22)*(m2->m25) + (m1->m23)*(m2->m35) + (m1->m24)*(m2->m45) + (m1->m25)*(m2->m55) + (m1->m26)*(m2->m65);
  out->m26 = (m1->m21)*(m2->m16) + (m1->m22)*(m2->m26) + (m1->m23)*(m2->m36) + (m1->m24)*(m2->m46) + (m1->m25)*(m2->m56) + (m1->m26)*(m2->m66);
}

void mulmat66mat64( matrix64_t* out, matrix66_t *m1, matrix64_t *m2 )
{
  /// You can't have output & input pointing to the same location
  VP_OS_ASSERT( out != m2 );

  matrix66_t t_m1;
  matrix46_t t_m2;
  matrix46_t t_out;

  transpose_mat64( &t_m2, m2 );
  transpose_mat66( &t_m1, m1 );
  mulmat46mat66( &t_out, &t_m2, &t_m1 );
  transpose_mat46( out, &t_out );
}

void mulmat66mat62( matrix62_t* out, matrix66_t *m1, matrix62_t *m2 )
{
  /// You can't have output & input pointing to the same location
  VP_OS_ASSERT( out != m2 );

  matrix66_t t_m1;
  matrix26_t t_m2;
  matrix26_t t_out;

  transpose_mat62( &t_m2, m2 );
  transpose_mat66( &t_m1, m1 );
  mulmat26mat66( &t_out, &t_m2, &t_m1 );
  transpose_mat26( out, &t_out );
}


void mulmat64mat44( matrix64_t* out, matrix64_t *m1, matrix44_t *m2 )
{
  /// You can't have output & input pointing to the same location
  VP_OS_ASSERT( out != m1 );

  out->m11 = (m1->m11)*(m2->m11) + (m1->m12)*(m2->m21) + (m1->m13)*(m2->m31) + (m1->m14)*(m2->m41);
  out->m12 = (m1->m11)*(m2->m12) + (m1->m12)*(m2->m22) + (m1->m13)*(m2->m32) + (m1->m14)*(m2->m42);
  out->m13 = (m1->m11)*(m2->m13) + (m1->m12)*(m2->m23) + (m1->m13)*(m2->m33) + (m1->m14)*(m2->m43);
  out->m14 = (m1->m11)*(m2->m14) + (m1->m12)*(m2->m24) + (m1->m13)*(m2->m34) + (m1->m14)*(m2->m44);

  out->m21 = (m1->m21)*(m2->m11) + (m1->m22)*(m2->m21) + (m1->m23)*(m2->m31) + (m1->m24)*(m2->m41);
  out->m22 = (m1->m21)*(m2->m12) + (m1->m22)*(m2->m22) + (m1->m23)*(m2->m32) + (m1->m24)*(m2->m42);
  out->m23 = (m1->m21)*(m2->m13) + (m1->m22)*(m2->m23) + (m1->m23)*(m2->m33) + (m1->m24)*(m2->m43);
  out->m24 = (m1->m21)*(m2->m14) + (m1->m22)*(m2->m24) + (m1->m23)*(m2->m34) + (m1->m24)*(m2->m44);

  out->m31 = (m1->m31)*(m2->m11) + (m1->m32)*(m2->m21) + (m1->m33)*(m2->m31) + (m1->m34)*(m2->m41);
  out->m32 = (m1->m31)*(m2->m12) + (m1->m32)*(m2->m22) + (m1->m33)*(m2->m32) + (m1->m34)*(m2->m42);
  out->m33 = (m1->m31)*(m2->m13) + (m1->m32)*(m2->m23) + (m1->m33)*(m2->m33) + (m1->m34)*(m2->m43);
  out->m34 = (m1->m31)*(m2->m14) + (m1->m32)*(m2->m24) + (m1->m33)*(m2->m34) + (m1->m34)*(m2->m44);

  out->m41 = (m1->m41)*(m2->m11) + (m1->m42)*(m2->m21) + (m1->m43)*(m2->m31) + (m1->m44)*(m2->m41);
  out->m42 = (m1->m41)*(m2->m12) + (m1->m42)*(m2->m22) + (m1->m43)*(m2->m32) + (m1->m44)*(m2->m42);
  out->m43 = (m1->m41)*(m2->m13) + (m1->m42)*(m2->m23) + (m1->m43)*(m2->m33) + (m1->m44)*(m2->m43);
  out->m44 = (m1->m41)*(m2->m14) + (m1->m42)*(m2->m24) + (m1->m43)*(m2->m34) + (m1->m44)*(m2->m44);

  out->m51 = (m1->m51)*(m2->m11) + (m1->m52)*(m2->m21) + (m1->m53)*(m2->m31) + (m1->m54)*(m2->m41);
  out->m52 = (m1->m51)*(m2->m12) + (m1->m52)*(m2->m22) + (m1->m53)*(m2->m32) + (m1->m54)*(m2->m42);
  out->m53 = (m1->m51)*(m2->m13) + (m1->m52)*(m2->m23) + (m1->m53)*(m2->m33) + (m1->m54)*(m2->m43);
  out->m54 = (m1->m51)*(m2->m14) + (m1->m52)*(m2->m24) + (m1->m53)*(m2->m34) + (m1->m54)*(m2->m44);

  out->m61 = (m1->m61)*(m2->m11) + (m1->m62)*(m2->m21) + (m1->m63)*(m2->m31) + (m1->m64)*(m2->m41);
  out->m62 = (m1->m61)*(m2->m12) + (m1->m62)*(m2->m22) + (m1->m63)*(m2->m32) + (m1->m64)*(m2->m42);
  out->m63 = (m1->m61)*(m2->m13) + (m1->m62)*(m2->m23) + (m1->m63)*(m2->m33) + (m1->m64)*(m2->m43);
  out->m64 = (m1->m61)*(m2->m14) + (m1->m62)*(m2->m24) + (m1->m63)*(m2->m34) + (m1->m64)*(m2->m44);
}

void mulmat62mat22( matrix62_t* out, matrix62_t *m1, matrix22_t *m2 )
{
  /// You can't have output & input pointing to the same location
  VP_OS_ASSERT( out != m1 );

  out->m11 = (m1->m11)*(m2->m11) + (m1->m12)*(m2->m21);
  out->m12 = (m1->m11)*(m2->m12) + (m1->m12)*(m2->m22);

  out->m21 = (m1->m21)*(m2->m11) + (m1->m22)*(m2->m21);
  out->m22 = (m1->m21)*(m2->m12) + (m1->m22)*(m2->m22);

  out->m31 = (m1->m31)*(m2->m11) + (m1->m32)*(m2->m21);
  out->m32 = (m1->m31)*(m2->m12) + (m1->m32)*(m2->m22);

  out->m41 = (m1->m41)*(m2->m11) + (m1->m42)*(m2->m21);
  out->m42 = (m1->m41)*(m2->m12) + (m1->m42)*(m2->m22);

  out->m51 = (m1->m51)*(m2->m11) + (m1->m52)*(m2->m21);
  out->m52 = (m1->m51)*(m2->m12) + (m1->m52)*(m2->m22);

  out->m61 = (m1->m61)*(m2->m11) + (m1->m62)*(m2->m21);
  out->m62 = (m1->m61)*(m2->m12) + (m1->m62)*(m2->m22);
}

void mulmat26mat62( matrix22_t* out, matrix26_t *m1, matrix62_t *m2 )
{
  out->m11 = (m1->m11)*(m2->m11) + (m1->m12)*(m2->m21) + (m1->m13)*(m2->m31) + (m1->m14)*(m2->m41) + (m1->m15)*(m2->m51) + (m1->m16)*(m2->m61);
  out->m12 = (m1->m11)*(m2->m12) + (m1->m12)*(m2->m22) + (m1->m13)*(m2->m32) + (m1->m14)*(m2->m42) + (m1->m15)*(m2->m52) + (m1->m16)*(m2->m62);

  out->m21 = (m1->m21)*(m2->m11) + (m1->m22)*(m2->m21) + (m1->m23)*(m2->m31) + (m1->m24)*(m2->m41) + (m1->m25)*(m2->m51) + (m1->m26)*(m2->m61);
  out->m22 = (m1->m21)*(m2->m12) + (m1->m22)*(m2->m22) + (m1->m23)*(m2->m32) + (m1->m24)*(m2->m42) + (m1->m25)*(m2->m52) + (m1->m26)*(m2->m62);
}

void mulmat64vec41( vector61_t* out, matrix64_t *m1, vector41_t *v1 )
{
  out->x1 = (m1->m11) * (v1->x1) + (m1->m12) * (v1->x2) + (m1->m13) * (v1->x3) + (m1->m14) * (v1->x4);
  out->x2 = (m1->m21) * (v1->x1) + (m1->m22) * (v1->x2) + (m1->m23) * (v1->x3) + (m1->m24) * (v1->x4);
  out->x3 = (m1->m31) * (v1->x1) + (m1->m32) * (v1->x2) + (m1->m33) * (v1->x3) + (m1->m34) * (v1->x4);
  out->x4 = (m1->m41) * (v1->x1) + (m1->m42) * (v1->x2) + (m1->m43) * (v1->x3) + (m1->m44) * (v1->x4);
  out->x5 = (m1->m51) * (v1->x1) + (m1->m52) * (v1->x2) + (m1->m53) * (v1->x3) + (m1->m54) * (v1->x4);
  out->x6 = (m1->m61) * (v1->x1) + (m1->m62) * (v1->x2) + (m1->m63) * (v1->x3) + (m1->m64) * (v1->x4);
}

void mulmat62vec21( vector61_t* out, matrix62_t *m1, vector21_t *v1 )
{
  out->x1 = (m1->m11) * (v1->x) + (m1->m12) * (v1->y);
  out->x2 = (m1->m21) * (v1->x) + (m1->m22) * (v1->y);
  out->x3 = (m1->m31) * (v1->x) + (m1->m32) * (v1->y);
  out->x4 = (m1->m41) * (v1->x) + (m1->m42) * (v1->y);
  out->x5 = (m1->m51) * (v1->x) + (m1->m52) * (v1->y);
  out->x6 = (m1->m61) * (v1->x) + (m1->m62) * (v1->y);
}

void transpose_mat26( matrix62_t *out, matrix26_t *m1 )
{
  out->m11 = m1->m11;
  out->m12 = m1->m21;

  out->m21 = m1->m12;
  out->m22 = m1->m22;

  out->m31 = m1->m13;
  out->m32 = m1->m23;

  out->m41 = m1->m14;
  out->m42 = m1->m24;

  out->m51 = m1->m15;
  out->m52 = m1->m25;

  out->m61 = m1->m16;
  out->m62 = m1->m26;
}


void transpose_mat62( matrix26_t *out, matrix62_t *m1 )
{
  out->m11 = m1->m11;
  out->m12 = m1->m21;
  out->m13 = m1->m31;
  out->m14 = m1->m41;
  out->m15 = m1->m51;
  out->m16 = m1->m61;

  out->m21 = m1->m12;
  out->m22 = m1->m22;
  out->m23 = m1->m32;
  out->m24 = m1->m42;
  out->m25 = m1->m52;
  out->m26 = m1->m62;
}

void transpose_mat46( matrix64_t *out, matrix46_t *m1 )
{
  out->m11 = m1->m11;
  out->m12 = m1->m21;
  out->m13 = m1->m31;
  out->m14 = m1->m41;

  out->m21 = m1->m12;
  out->m22 = m1->m22;
  out->m23 = m1->m32;
  out->m24 = m1->m42;

  out->m31 = m1->m13;
  out->m32 = m1->m23;
  out->m33 = m1->m33;
  out->m34 = m1->m43;

  out->m41 = m1->m14;
  out->m42 = m1->m24;
  out->m43 = m1->m34;
  out->m44 = m1->m44;

  out->m51 = m1->m15;
  out->m52 = m1->m25;
  out->m53 = m1->m35;
  out->m54 = m1->m45;

  out->m61 = m1->m16;
  out->m62 = m1->m26;
  out->m63 = m1->m36;
  out->m64 = m1->m46;
}

void transpose_mat64( matrix46_t *out, matrix64_t *m1 )
{
  out->m11 = m1->m11;
  out->m12 = m1->m21;
  out->m13 = m1->m31;
  out->m14 = m1->m41;
  out->m15 = m1->m51;
  out->m16 = m1->m61;

  out->m21 = m1->m12;
  out->m22 = m1->m22;
  out->m23 = m1->m32;
  out->m24 = m1->m42;
  out->m25 = m1->m52;
  out->m26 = m1->m62;

  out->m31 = m1->m13;
  out->m32 = m1->m23;
  out->m33 = m1->m33;
  out->m34 = m1->m43;
  out->m35 = m1->m53;
  out->m36 = m1->m63;

  out->m41 = m1->m14;
  out->m42 = m1->m24;
  out->m43 = m1->m34;
  out->m44 = m1->m44;
  out->m45 = m1->m54;
  out->m46 = m1->m64;
}
