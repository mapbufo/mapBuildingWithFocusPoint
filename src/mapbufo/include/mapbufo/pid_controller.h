#pragma once

#include "common.h"
class PIDController
{
private:
  bool init_;       // check if the pid controller is initialized
  double kp_;       // P factor
  double ki_;       // Integration factor
  double kd_;       // Differentiation factor
  double p_error_;  // P error
  double i_error_;  // accumulated integration error
  double d_error_;  // differential error

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

  /**
   * @brief compute correction value
   * @param input error: input error
   * @return control value: value used for correction
   */
  double Control(double error);

  /**
   * @brief update P, I, D error
   * @param input error: input error
   */
  void updateError(double error);
};