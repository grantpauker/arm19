#ifndef ARM_H
#define ARM_H

#include "Arduino.h"

class Arm
{
public:
  Arm(float, float);
  bool inverseKinematics(float, float);
  void forwardKinematics(float[2][2], float, float);

  void setLinkConstraints0(float, float);
  void setLinkConstraints1(float, float);
  float getAngle0();
  float getAngle1();

private:
  float _length_0;
  float _length_1;

  float _angle_0;
  float _angle_1;

  float _min_0;
  float _max_0;

  float _min_1;
  float _max_1;
};

#endif