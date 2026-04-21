#include "PID.hpp"
#include "Config.hpp"                                               // Config header with configuration constants (thresholds, speeds)
#include <iostream>
#include <string>                                                   // String library to easy way to handle text 
#include <cmath>

PID::PID() {                                                        // Main module
  prev_error = 0.0;
  integral = 0.0;
  alpha_filtered = 0.0;
}

double PID::compute(double error, double dt) {                      // Function for calculating PID with continuous adaptive gains

  // Derivative error
  double d_error = (error - prev_error) / dt;

  // Calculate alpha (curvature)
  alpha = computeAlpha(error, d_error);

  double beta = BETA_FILTER + 0.2 * alpha;

  // Low Pass Filter (smooths gear changes)
  //      α_f[n] = f(α[n], α_f[n-1])
  //      alpha_filtered = (1 - β)*alpha_filtered + β*alpha;
  alpha_filtered = (1.0 - beta) * alpha_filtered + beta * alpha;

  // Gain scheduling
  Kp = KP_SOFT + alpha_filtered * (KP_STRONG - KP_SOFT);
  Ki = KI_SOFT + alpha_filtered * (KI_STRONG - KI_SOFT);
  Kd = KD_SOFT + alpha_filtered * (KD_STRONG - KD_SOFT);

  // Integral with dynamic anti-windup
  integral = integral * (2 / 3) + error * dt * (1.0 - alpha_filtered);

  PID_value = Kp * error      // P = Kp * error     <-- Proportional
            + Ki * integral   // I = Ki * integral  <-- Integral
            + Kd * d_error;   // D = Ki * d_error   <-- Derivative

  prev_error = error;

  return PID_value;
}

PIDState PID::getState() const {                                    // Function for calculating PID with continuous adaptive gains
    return { alpha, alpha_filtered, Kp, Ki, Kd, PID_value };
}

void PID::reset() {                                                 // Function to reset the PID value
  prev_error = 0.0;
  integral = 0.0;
  alpha_filtered = 0.0;
}

double PID::computeAlpha(double error, double d_error) {           // Function for calculating curvature factor

  double abs_e = fabs(error);
  double abs_de = fabs(d_error);

  // Mixture of error + dynamics: 70/30
  double curvature = 0.7 * abs_e + 0.3 * abs_de;

  // Normalization
  double alpha = curvature / E_MAX;

  // Clamp
  if (alpha > 1.0) alpha = 1.0;
  if (alpha < 0.0) alpha = 0.0;

  // Non-linearity (key to smoothness in straight lines)
  alpha = alpha * alpha;

  return alpha;
}


