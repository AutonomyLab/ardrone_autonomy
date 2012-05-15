/**
 *  \file     quaternions.c
 *  \brief    Quaternions library used by Mykonos
 *  \author   Fran�ois Callou <francois.callou@parrot.com>
 *  \version  1.0
 */

#ifndef _QUATERNIONS_H_
#define _QUATERNIONS_H_

#include <Maths/matrices.h>
#include <VP_Os/vp_os_types.h>

typedef struct _angles_t
{
  float32_t phi;
  float32_t theta;
  float32_t psi;
} angles_t;


typedef struct _quaternion_t {
  float32_t   a;
  vector31_t  v;
} quaternion_t;

extern const quaternion_t quat_unitary;

// TODO Documentation

// Multiplies two quaternions q1 & q2. Stores result in out.
void mul_quat( quaternion_t* out, quaternion_t* q1, quaternion_t* q2);

// Adds two quaternions q1 & q2. Stores result in out.
void add_quat( quaternion_t* out, quaternion_t* q1, quaternion_t* q2 );

// Multiplies a quaternion ny a scalar
void mulconst_quat( quaternion_t *out, quaternion_t *q, float32_t k );

// Stores the conjugate quaternion in out
void conjugate_quat( quaternion_t* out, quaternion_t* q );

// Compuets the norm of a quaternion
float32_t norm_quat( quaternion_t* q );

//Normalises a quaternion
bool_t normalize_quat( quaternion_t* q );

//transformss a quaternion to the cprrsep�nding euler rotation matrix
void quat_to_euler_rot_mat(matrix33_t* m, quaternion_t* q);

//transformss a quaternion to the corrseponding euler angles
void quat_to_euler_angles(angles_t* a, quaternion_t* q);


#endif // _QUATERNIONS_H_
