#include "Sensors.hpp"                                     // Header of Sensors
#include <iomanip>                                         // Input/Output Streams to use cout to debug routines
#include "Config.hpp"                                      // Config header with configuration constants (thresholds, speeds)

using namespace webots;

int gs_min[N_SEN];                                         // Min value of every sensor (Calibration)
int gs_max[N_SEN];                                         // Max value of every sensor (Calibration)
int gs_new[N_SEN];                                         // New normalized value of every sensor
bool initValue = true;                                     // Init value flag
int S_weights[N_SEN] = { NR_GS*3,  NR_GS*2,  NR_GS,  0 };  // Weights of every sensor to detect line of track
int S_offset = NR_GS*(N_SEN - 1)/2;                        // Offset weight

Sensors::Sensors(Robot *robot, int timeStep) {             // Module to initialize sensors
  for (int i = 0; i < N_SEN; i++) {
    std::string name = "gs" + std::to_string(i);
    gs[i] = robot->getDistanceSensor(name);
    gs[i]->enable(timeStep);
  }
  
  right_s = robot->getDistanceSensor("rs");
  right_s->enable(timeStep);
 
  left_s = robot->getDistanceSensor("ls");
  left_s->enable(timeStep);
}

void Sensors::updateCalibration() {                        // Module to calibration sensors
  for (int i = 0; i < N_SEN; i++) {
    int v = gs[i]->getValue();
    if (initValue) {
      gs_min[i] = v;
      gs_max[i] = v;
      initValue = false;
    } 
    else {
      if (v < gs_min[i]) gs_min[i] = v;
      if (v > gs_max[i]) gs_max[i] = v;
    }
  }
}

int Sensors::normalize(int i) const {                      // Function to normalize the sensor value, using min, max and normal target (NR_GS)
  if (gs_max[i] == gs_min[i]) return gs[i]->getValue();
  int v = gs[i]->getValue();
  if (v < gs_min[i]) v = gs_min[i];
  else if (v > gs_max[i]) v = gs_max[i];
  return NR_GS * (v - gs_min[i]) / (gs_max[i] - gs_min[i]);
}

double Sensors::lineError() {                              // Function to get the line error
  double PosX = 0;
  double LneX = 0;
  for (int i = 0; i < N_SEN; i++) {
    gs_new[i] = normalize(i);
    if (gs_new[i] > THOLD) {
      PosX += gs_new[i] * S_weights[i];
      LneX += gs_new[i];
    } 
  }
  if (LneX == 0)
    return 0;                                              // Lost the line
  
  PosX = PosX / LneX - S_offset;
  
  // to debug:
  //std::string P_error = std::to_string(PosX);             //   <--- to debug
  //std::cout << "P error: " << P_error << std::endl;       //   <--- to debug

  //return (double)sum / count;
  return (double) PosX;
}

int Sensors::getValue(int i) const {                       // Function to get the line sensor values
  return (int)gs[i]->getValue();
}

int Sensors::getGS(int i) const {                          // Function to get the normaliced line sensor value
  return (int)gs_new[i];
}

int Sensors::getRL(int i) const {                          // Function to get the wheel sensor value (Right: 0  |  Left: 1)
  int valueRL = (int)right_s->getValue();
  if (i > 0) {
    valueRL = (int)left_s->getValue();
  } 
  return valueRL;
}

void Sensors::cout_max_min() {                             // Function to show the max and min sensor value <--- to debug
  for (int i = 0; i < N_SEN; i++) {
    std::cout << std::setw(5) << gs_min[(N_SEN - 1) - i];
  }
  std::cout << std::endl;
  for (int i = 0; i < N_SEN; i++) {
    std::cout << std::setw(5) << gs_max[(N_SEN - 1) - i];
  }
  std::cout << std::endl;  
}

void Sensors::sensor_norm() {                              // Function to show the normalized sensor value <--- to debug
  for (int i = 0; i < N_SEN; i++) {
    std::cout << std::setw(5) << gs_new[(N_SEN - 1) - i];
  }
  std::cout << std::endl;
}

void Sensors::sensor_value() {                             // Function to show the max and min sensor value <--- to debug
  for (int i = 0; i < N_SEN; i++) {
    std::cout << std::setw(5) << gs_new[(N_SEN - 1) - i];
  }
  std::cout << std::endl;

}
