#include "arm.hpp"
#include <math.h>

#define CLAMP(x, min, max) (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))

Arm::Arm(float length_0, float length_1)
{
  _length_0 = length_0;
  _length_1 = length_1;

  _angle_0 = 0;
  _angle_1 = 0;

  _min_0 = -PI / 2;
  _max_0 = PI / 2;

  _min_1 = -PI / 2;
  _max_1 = PI / 2;
}

void Arm::forwardKinematics(float points[2][2], float angle_0, float angle_1)
{
  angle_1 += angle_0;
  points[0][0] = _length_0 * cos(angle_0);
  points[0][1] = _length_0 * sin(angle_0);
  points[1][0] = points[0][0] + _length_1 * cos(angle_1);
  points[1][1] = points[0][1] + _length_1 * sin(angle_1);
}

bool Arm::inverseKinematics(float x, float y)
{
  float radius = sqrt(x*x+y*y);
  float max_radius = 20;
  bool scaled = false;
  if(radius>=max_radius){
    float scale = max_radius /  radius;
    x *= scale;
    y *= scale;
    scaled = true;
  }
  float cos_angle_1 = ((x * x) + (y * y) - (_length_0 * _length_0) - (_length_1 * _length_1)) / (2 * _length_0 * _length_1);
  float angle_1 = -acos(cos_angle_1);
  // _angle_1 = CLAMP(_angle_1, _min_1, _max_1);

  float angle_0 = atan2(y, x) - atan2(_length_1 * sin(angle_1), _length_0 + _length_1 * cos(angle_1));
  // _angle_0 = CLAMP(_angle_0, _min_0, _max_0);
  _angle_1 = angle_1;
  _angle_0 = angle_0;
  return scaled;
}

void Arm::setLinkConstraints0(float min, float max)
{
  _min_0 = min;
  _max_0 = max;
}
void Arm::setLinkConstraints1(float min, float max)
{
  _min_1 = min;
  _max_1 = max;
}

float Arm::getAngle0()
{
  return _angle_0;
}
float Arm::getAngle1()
{
  return _angle_1;
}