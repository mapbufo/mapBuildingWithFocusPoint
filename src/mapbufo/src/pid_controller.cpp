#include "pid_controller.h"
double PIDController::Control(double error)
{
  updateError(error);
  return kp_ * p_error_ + ki_ * i_error_ + kd_ * d_error_;
}

void PIDController::updateError(double error)
{
  if (init_)
  {
    p_error_ = error;
    i_error_ += error;
    d_error_ = 0.0;
    init_ = false;
  }
  else
  {
    i_error_ += error;
    d_error_ = error - p_error_;
    p_error_ = error;
  }
}