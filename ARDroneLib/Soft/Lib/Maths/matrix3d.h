/**
 *  \brief    Matrix 3d declaration
 *  \author   Sylvain Gaeremynck <sylvain.gaeremynck@parrot.fr>
 *  \version  1.0
 *  \date     04/06/2007
 *  \warning  Subject to completion
 */

#ifndef _MATRIX3D_H_
#define _MATRIX3D_H_

#include <VP_Os/vp_os_types.h>

typedef struct _matrix3d_t {
  float32_t m00, m01, m02, m03;
  float32_t m10, m11, m12, m13;
  float32_t m20, m21, m22, m23;
  float32_t m30, m31, m32, m33;
} matrix3d_t;

struct _vector31_t;

extern matrix3d_t matrix3d_id; // identity matrix

// Set m to zero
void matrix3d_zero(matrix3d_t* m);

// Set m to identity
void matrix3d_identity(matrix3d_t* m);

// Initialize m from euler angles
C_RESULT matrix3d_euler(matrix3d_t* m, float32_t phi, float32_t theta, float32_t psi);

// Initialize m from a position and a direction
C_RESULT matrix3d_vector( matrix3d_t* m, struct _vector31_t* pos, struct _vector31_t* dir,
                                         struct _vector31_t* right, struct _vector31_t* up);

// Initialize m from a position and a normalized orientation
C_RESULT matrix3d_orientation(matrix3d_t* m, const struct _vector31_t* pos, const struct _vector31_t* dir,
                                             const struct _vector31_t* right, const struct _vector31_t* up);

// Compute transposed matrix
C_RESULT matrix3d_transpose(matrix3d_t* out, matrix3d_t* in);

// Common arithmetic operation
C_RESULT matrix3d_add(matrix3d_t* out, matrix3d_t* m1, matrix3d_t* m2);
C_RESULT matrix3d_sub(matrix3d_t* out, matrix3d_t* m1, matrix3d_t* m2);
C_RESULT matrix3d_mul(matrix3d_t* out, matrix3d_t* m1, matrix3d_t* m2);

// Common 3d transformation
C_RESULT matrix3d_translate(matrix3d_t* m, struct _vector31_t* tr);
C_RESULT matrix3d_add_translate(matrix3d_t* m, struct _vector31_t* tr);

C_RESULT matrix3d_rotate_euler(matrix3d_t* m, float32_t phi, float32_t theta, float32_t psi);
C_RESULT matrix3d_rotate_axis(matrix3d_t* m, struct _vector31_t* axis, float32_t value);

C_RESULT matrix3d_transform(matrix3d_t* m, struct _vector31_t* v);

#endif // _MATRIX3D_H_
