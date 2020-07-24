#ifndef PID_H
#define PID_H

#include "common.h"
class PIDController
{
private:
  bool init_;
  double kp_;
  double ki_;
  double kd_;
  double p_error_;
  double i_error_;
  double d_error_;

public:
  PIDController(double kp, double ki, double kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    p_error_ = 0;
    i_error_ = 0;
    d_error_ = 0;
    init_ = true;
  }

  ~PIDController()
  {
  }

  double Control(double error);
  void updateError(double error);
};

#endif // PID_H
