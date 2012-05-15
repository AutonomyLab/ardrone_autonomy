
#ifndef _STAGES_VISION_MATH_INCLUDE_
#define _STAGES_VISION_MATH_INCLUDE_

#include <Maths/maths.h>
#include <Maths/matrices.h>

/**
 * @fn     Compute the matrix of uav pose in space
 * @param  float32_t theta : euler angle
 * @param  float32_t phi : euler angle
 * @param  float32_t psi : euler angle
 * @param  matrix *mat 
 * @return VOID
*/ 
void 
euler_matrix(float32_t theta, float32_t phi, float32_t psi, matrix33_t *mat);
void
horizontal_frame_euler_matrix(float32_t theta, float32_t phi, float32_t psi, matrix33_t *mat);
void
vertical_frame_euler_matrix(float32_t theta, float32_t phi, float32_t psi, matrix33_t *mat);
void
max_euler_matrix(float32_t theta, float32_t phi, float32_t psi, matrix33_t *mat);

/**
 * @fn     Compute the matrix of uav pose in space R_euler*Rc, in camera basis
 * @param  float32_t theta : euler angle
 * @param  float32_t phi : euler angle
 * @param  float32_t psi : euler angle
 * @param  matrix *mat 
 * @return VOID
*/ 
void 
frame_euler_matrix(float32_t theta, float32_t phi, float32_t psi, matrix33_t *mat);

void
frame_euler_angles(vector31_t *angles, matrix33_t *R);
void
horizontal_frame_euler_angles(vector31_t *angles, matrix33_t *R);

/**
 * @fn     Compute the matrix which projects points from frame (t-1) to frame (t) : tr(Rc)*Rot*Rc
 * @param  float32_t delta_theta : integrated gyro in theta
 * @param  float32_t delta_phi : integrated gyro in phi
 * @param  float32_t delta_psi : integrated gyro in psi
 * @param  matrix *matproj 
 * @return VOID
*/ 
void 
integrated_gyros_matrix(float32_t delta_theta, float32_t delta_phi, float32_t delta_psi, matrix33_t *matproj);

/**
 * @fn     compute projection of a point on a plane
 * @param  int x : first coordinate
 * @param  int y : second coordinate
 * @param  int centerx : first coordinate of the image center
 * @param  int centery : second coordinate of the image center
 * @param  matrix mat : projection matrix
 * @param  float *u : first coordinate of result
 * @param  float *v : second coordinate of result
 * @return VOID
*/ 
void 
proj_point(screen_point_t *point, screen_point_t *center, matrix33_t *mat, vector21_t *out);

void 
proj_pointf(vector21_t *point, screen_point_t *center, matrix33_t *mat, vector21_t *out);

void 
vision_direction_result(vector21_t *t, screen_point_t *v, int32_t  threshold);


#endif // ! _STAGES_VISION_MATH_INCLUDE_

