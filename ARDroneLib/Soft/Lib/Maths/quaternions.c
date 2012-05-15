/**
 *  \file     quaternions.c
 *  \brief    Quaternions library used by Mykonos
 *  \author   Fran�ois Callou <francois.callou@parrot.com>
 *  \version  1.0
 */

#include <VP_Os/vp_os_assert.h>
#include <Maths/quaternions.h>
#include <Maths/maths.h>

const quaternion_t quat_unitary = { 1.0f, {{{ 0.0f, 0.0f, 0.0f}}} };

void mul_quat( quaternion_t* out, quaternion_t* q1, quaternion_t* q2)
{
  vector31_t temp_v;
  
  /// You can't have output & input pointing to the same location
  VP_OS_ASSERT( out != q1 );
  VP_OS_ASSERT( out != q2 );

  // scalar result
  out->a = q1->a*q2->a - (q1->v.x*q2->v.x + q1->v.y*q2->v.y + q1->v.z*q2->v.z); 
  
  // pure quaternion result
  cross_vec( &out->v , &q1->v,  &q2->v );
  mulconst_vec( &temp_v, &q2->v, q1->a );
  add_vec( &out->v, &out->v, &temp_v);
  mulconst_vec( &temp_v, &q1->v, q2->a );
  add_vec( &out->v, &out->v, &temp_v);
}

void add_quat( quaternion_t* out, quaternion_t* q1, quaternion_t* q2 )
{
  // scalar result
  out->a = q1->a + q2->a;
  // pure quaternion result
  add_vec( &out->v, &q1->v, &q2->v );
}

void mulconst_quat( quaternion_t* out, quaternion_t* q, float32_t k )
{
  out->a = (q->a) * k;
  mulconst_vec( &out->v, &q->v, k );
}

void conjugate_quat( quaternion_t* out, quaternion_t* q )
{
  out->a = q->a;
  out->v.x = -q->v.x;
  out->v.y = -q->v.y;
  out->v.z = -q->v.z;
}

float32_t norm_quat( quaternion_t *q )
{
  return sqrtf( q->a*q->a + q->v.x * q->v.x + q->v.y * q->v.y + q->v.z * q->v.z );
}

bool_t normalize_quat( quaternion_t* q )
{
  bool_t ret;
  float32_t norm;

  norm = norm_quat( q );

  if( f_is_zero( norm ) )
  {
    q->a   = 0.0f;
    q->v.x = 0.0f;
    q->v.y = 0.0f;
    q->v.z = 0.0f;

    ret = FALSE;
  }
  else
  {
    q->a   = f_zero( q->a / norm );
    q->v.x = f_zero( q->v.x / norm );
    q->v.y = f_zero( q->v.y / norm );
    q->v.z = f_zero( q->v.z / norm );

    ret = TRUE;
  }

  return ret;
}


void quat_to_euler_rot_mat(matrix33_t* m, quaternion_t* q)
{
  //to use with normalised quaternion
  m->m11 = 1.0f - 2*q->v.y*q->v.y - 2*q->v.z*q->v.z;
  m->m12 = 2*q->v.x*q->v.y - 2*q->v.z*q->a;
  m->m13 = 2*q->v.z*q->v.x + 2*q->v.y*q->a;
  m->m21 = 2*q->v.x*q->v.y + 2*q->v.z*q->a;
  m->m22 = 1.0f - 2*q->v.x*q->v.x - 2*q->v.z*q->v.z;
  m->m23 = 2*q->v.z*q->v.y - 2*q->v.x*q->a;
  m->m31 = 2*q->v.z*q->v.x - 2*q->v.y*q->a;
  m->m32 = 2*q->v.z*q->v.y + 2*q->v.x*q->a;
  m->m33 = 1.0f - 2*q->v.x*q->v.x - 2*q->v.y*q->v.y;
}



void quat_to_euler_angles(angles_t* a, quaternion_t* q)
{ 
	//to use with normalised quaternion
	float32_t sqvx = q->v.x*q->v.x;
	float32_t sqvy = q->v.y*q->v.y;
  float32_t sqvz = q->v.z*q->v.z;

 /* if ( f_is_zero(test -0.5) ) { // singularity at north pole
		a->psi   = 2 * atan2(q->a,q->v.z);
		a->theta = PI/2;
		a->phi   = 0;
		return;
	}
	if ( f_is_zero(test + 0.5) ) { // singularity at south pole
		a->psi   = -2 * atan2(q->a,q->v.z);
		a->theta = - PI/2;
		a->phi   = 0;
		return;
	}*/
  
	a->phi   = atan2(2*q->v.y*q->v.z+2*q->a*q->v.x , 1 - 2*sqvx - 2*sqvy);
	a->theta = asin(2*(q->a*q->v.y - q->v.x*q->v.z ));
	a->psi   = atan2(2*q->v.x*q->v.y+2*q->a*q->v.z , 1 - 2*sqvy - 2*sqvz);
}
