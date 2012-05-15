
#include <VP_Os/vp_os_print.h>
#include <Maths/vision_math.h>

extern float32_t used_focal;

void euler_matrix(float32_t theta, float32_t phi, float32_t psi, matrix33_t *mat)
{
  float32_t cthe, sthe, cphi, sphi, cpsi, spsi;

  // Euler angles are now in radians
  cthe = cosf(theta);
  sthe = sinf(theta);
  cphi = cosf(phi);
  sphi = sinf(phi);
  cpsi = cosf(psi);
  spsi = sinf(psi);
  
  mat->m11 = cpsi* cthe;
  mat->m12 = -spsi*cphi + cpsi*sthe*sphi;
  mat->m13 = spsi*sphi + cpsi*sthe*cphi;
  mat->m21 = spsi*cthe;
  mat->m22 = cpsi*cphi + spsi*sthe*sphi; 
  mat->m23 = -cpsi*sphi + spsi*sthe*cphi;
  mat->m31 = -sthe;
  mat->m32 = cthe*sphi;
  mat->m33 = cthe*cphi;
}


void frame_euler_matrix(float32_t theta, float32_t phi, float32_t psi, matrix33_t *mat)
{
  float32_t cthe, sthe, cphi, sphi, cpsi, spsi;
  float32_t m11, m12, m13, m21, m22, m23, m31, m32, m33;
  // R_euler*Rc with Rc = [0 -1 0; 1 0 0; 0 0 1]  
  
  // Euler angles are now in radians
  cthe = cosf(theta);
  sthe = sinf(theta);
  cphi = cosf(phi);
  sphi = sinf(phi);
  cpsi = cosf(psi);
  spsi = sinf(psi);
  
  m11 = cpsi* cthe;
  m12 = -spsi*cphi + cpsi*sthe*sphi;
  m13 = spsi*sphi + cpsi*sthe*cphi;
  m21 = spsi*cthe;
  m22 = cpsi*cphi + spsi*sthe*sphi; 
  m23 = -cpsi*sphi + spsi*sthe*cphi;
  m31 = -sthe;
  m32 = cthe*sphi;
  m33 = cthe*cphi;


  mat->m11= m12;
  mat->m12 = -m11;
  mat->m13 = m13;
  mat->m21 = m22;
  mat->m22 = -m21;
  mat->m23 = m23;
  mat->m31 = m32;
  mat->m32 = -m31;
  mat->m33 = m33;
}

void vertical_frame_euler_matrix(float32_t theta, float32_t phi, float32_t psi, matrix33_t *mat)
{
  float32_t cthe, sthe, cphi, sphi, cpsi, spsi;
  float32_t m11, m12, m13, m21, m22, m23, m31, m32, m33;
  // R_euler*Rc with Rc = [0 -1 0; 1 0 0; 0 0 1]
  
  // Euler angles are now in radians
  cthe = cosf(theta);
  sthe = sinf(theta);
  cphi = cosf(phi);
  sphi = sinf(phi);
  cpsi = cosf(psi);
  spsi = sinf(psi);
  
  m11 = cpsi* cthe;
  m12 = -spsi*cphi + cpsi*sthe*sphi;
  m13 = spsi*sphi + cpsi*sthe*cphi;
  m21 = spsi*cthe;
  m22 = cpsi*cphi + spsi*sthe*sphi; 
  m23 = -cpsi*sphi + spsi*sthe*cphi;
  m31 = -sthe;
  m32 = cthe*sphi;
  m33 = cthe*cphi;


  mat->m11= m12;
  mat->m12 = -m11;
  mat->m13 = m13;
  mat->m21 = m22;
  mat->m22 = -m21;
  mat->m23 = m23;
  mat->m31 = m32;
  mat->m32 = -m31;
  mat->m33 = m33;
}

void horizontal_frame_euler_matrix(float32_t theta, float32_t phi, float32_t psi, matrix33_t *mat)
{
  float32_t cthe, sthe, cphi, sphi, cpsi, spsi;
  float32_t m11, m12, m13, m21, m22, m23, m31, m32, m33;
  // R_euler*Rc with Rc = [ 0 0 1; 1 0 0; 0 1 0 ]
  
  // Euler angles are now in radians
  cthe = cosf(theta);
  sthe = sinf(theta);
  cphi = cosf(phi);
  sphi = sinf(phi);
  cpsi = cosf(psi);
  spsi = sinf(psi);
  
  m11 = cpsi* cthe;
  m12 = -spsi*cphi + cpsi*sthe*sphi;
  m13 = spsi*sphi + cpsi*sthe*cphi;
  m21 = spsi*cthe;
  m22 = cpsi*cphi + spsi*sthe*sphi; 
  m23 = -cpsi*sphi + spsi*sthe*cphi;
  m31 = -sthe;
  m32 = cthe*sphi;
  m33 = cthe*cphi;

  mat->m11 =    m12;
  mat->m12 =    m13;
  mat->m13 =    m11;
  mat->m21 =    m22;
  mat->m22 =    m23;
  mat->m23 =    m21;
  mat->m31 =    m32;
  mat->m32 =    m33;
  mat->m33 =    m31;
}

void max_euler_matrix(float32_t theta, float32_t phi, float32_t psi, matrix33_t *mat)
{
  float32_t cthe, sthe, cphi, sphi, cpsi, spsi;
  float32_t m11, m12, m13, m21, m22, m23, m31, m32, m33;
  // R_euler*Rc with Rc = [ 0 1 0; 0 0 -1; -1 0 0 ]
  
  // Euler angles are now in radians
  cthe = cosf(theta);
  sthe = sinf(theta);
  cphi = cosf(phi);
  sphi = sinf(phi);
  cpsi = cosf(psi);
  spsi = sinf(psi);
  
  m11 = cpsi* cthe;
  m12 = -spsi*cphi + cpsi*sthe*sphi;
  m13 = spsi*sphi + cpsi*sthe*cphi;
  m21 = spsi*cthe;
  m22 = cpsi*cphi + spsi*sthe*sphi; 
  m23 = -cpsi*sphi + spsi*sthe*cphi;
  m31 = -sthe;
  m32 = cthe*sphi;
  m33 = cthe*cphi;

  
  mat->m11 =    -m13;
  mat->m12 =    m11;
  mat->m13 =    -m12;
  mat->m21 =    -m23;
  mat->m22 =    m21;
  mat->m23 =    -m22;
  mat->m31 =    -m33;
  mat->m32 =    m31;
  mat->m33 =    -m32;
}

void integrated_gyros_matrix(float32_t delta_theta, float32_t delta_phi, float32_t delta_psi, matrix33_t *matproj)
{
  // for the case which used integrated gyros
  // projection matrix used for cancelling rotation between two frames
  // ie transpose(Rc) * matrot * Rc
  // with Rc = [0 -1 0; 1 0 0; 0 0 1]
  // matrot = [1 delta_psi -delta_theta ; -delta_psi 1 delta_phi; delta_theta  -delta_phi 1]

  matproj->m11 = 1.0;
  matproj->m12 = delta_psi;
  matproj->m13 = delta_phi;
  matproj->m21 = -delta_psi;
  matproj->m22 = 1.0;
  matproj->m23 = delta_theta;
  matproj->m31 = -delta_phi;
  matproj->m32 = -delta_theta;
  matproj->m33 = 1.0;
}

void frame_euler_angles(vector31_t *angles, matrix33_t *R)
{
  angles->x = (float32_t)asin(-R->m31);
  angles->y = (float32_t)atan2(R->m32, R->m33);
  angles->z = (float32_t)atan2(R->m21, R->m11);

  if (angles->z < 0)
    angles->z += 2*PI;
}

void horizontal_frame_euler_angles(vector31_t *angles, matrix33_t *R)
{
  angles->x = (float32_t)asin(-R->m33);
  angles->y = (float32_t)atan2(R->m31, R->m32);
  angles->z = (float32_t)atan2(R->m23, R->m13);

  if (angles->z < 0)
    angles->z += 2*PI;
}

void proj_point(screen_point_t *point, screen_point_t *center, matrix33_t *mat, vector21_t *out)
{
  float32_t denom;
  int32_t x_centre = point->x - center->x;
  int32_t y_centre = point->y - center->y;
   
  denom = mat->m31*x_centre/used_focal + mat->m32*y_centre/used_focal + mat->m33;

  out->x = (float32_t)center->x + ( mat->m11*x_centre + mat->m12*y_centre + mat->m13*used_focal) / denom;
  out->y = (float32_t)center->y + ( mat->m21*x_centre + mat->m22*y_centre + mat->m23*used_focal ) / denom;
}

void proj_pointf(vector21_t *point, screen_point_t *center, matrix33_t *mat, vector21_t *out)
{
  float32_t denom;
  float32_t x_centre = point->x - (float32_t)center->x;
  float32_t y_centre = point->y - (float32_t)center->y;
  
  denom = mat->m31*x_centre/used_focal + mat->m32*y_centre/used_focal + mat->m33;

  out->x = (float32_t)center->x + ( mat->m11*x_centre + mat->m12*y_centre + mat->m13*used_focal) / denom;
  out->y = (float32_t)center->y + ( mat->m21*x_centre + mat->m22*y_centre + mat->m23*used_focal ) / denom;
}


void vision_direction_result(vector21_t *t, screen_point_t *v, int32_t  threshold)
{
  float32_t norm, angle;
  
  norm = t->x*t->x + t->y*t->y;
  
  if (norm < threshold*threshold)
    {
      v->x = 0;
      v->y = 0;
    }
  else
    {
      angle = (float32_t)atan2(t->y,t->x);
      v->x = (angle >= -3*PI/8) * (angle <= 3*PI/8) - (angle <= -5*PI/8) - (angle >= 5*PI/8);
      v->y = (angle >= PI/8) * (angle <= 7*PI/8) - (angle >= - 7*PI/8) * (angle <= -PI/8);
    }
}
