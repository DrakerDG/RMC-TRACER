#pragma once

struct PIDState {
    double alpha;
    double alpha_filtered;
    double Kp;
    double Ki;
    double Kd;
    double PID_value;
};

class PID {
public:
  PID();                                                  // Main module
  
  double compute(double error, double dt);                // Function for calculating PID with continuous adaptive gains
  PIDState getState() const;
  void reset();                                           // Function to reset the PID value

private:
  double integral;
  double prev_error;
  double alpha;
  double alpha_filtered;
  double Kp, Ki, Kd;
  double PID_value;

  double computeAlpha(double error, double d_error);      // Function for calculating curvature factor
};
