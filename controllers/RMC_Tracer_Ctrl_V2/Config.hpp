#pragma once

// Senror constants
const int     N_SEN =    4;          // Number of front sensors
const int     S_SEN =    2;          // Number of left and right sensors
const int     THOLD =  100;          // IR sensor threshold     (600)
const int     NR_GS = 1000;          // Normal max value of sensors

// Time constants
const int CALIB_DLY =   14;          // Time to calibrate  | 3 tourns = 14 s
const int  STOP_DLY =    3;          // Pause | 3 s

// Graphic constants
const int  L_Margin =   10;          // Left margin of the screen
const int  T_Margin =   10;          // Top  margin of the screen
const int P_Spacing =   28;          // Paragraph spacing of the screen
const int  S_Margin =   26;          // Margin of sensor on the screen
const int S_Spacing =   24;          // Spacing of sensor on the screen
const int   S_width =    8;          // Width of sensor on the screen
const int  S_height =   14;          // Height of sensor on the screen
const int  Gap_line =   20;          // Grid line spacing
const int   K_width =   20;          // Width of key of PID on the screen
const int  K_height =  130;          // Height of key of PID on the screen
const int   led_hgt =    5;          // LED height
const int   led_pad =    2;          // LED paddle

// Soft PID keyes
const double   KP_SOFT = 0.02;       // Soft Kp key
const double   KI_SOFT = 0.08;       // Soft Ki key
const double   KD_SOFT = 0.000008;   // Soft Kd key

// Strong PID keyes
const double KP_STRONG = 0.12;       // Strong Kp key
const double KI_STRONG = 0.24;       // Strong Ki key
const double KD_STRONG = 0.000048;   // Strong Kd key

// Normalization
const double E_MAX = 200.0;          // Maximum error threshold

// Alpha filter
const double BETA_FILTER = 0.1;      // Beta key of low-pass filter

const int    SP000 =   20;           //  20 Motor rad/s  ≈   2.7 Wheel rad/s  ≈  0.047 Robot m/s  Calibration
const int    SP001 =  220;           // 220 Motor rad/s  ≈  29.3 Wheel rad/s  ≈  0.505 Robot m/s  Base speed
const int    SP002 =  440;           // 440 Motor rad/s  ≈  58.7 Wheel rad/s  ≈  1.013 Robot m/s  Max speed
const int    ST_CH =    2;           //   2 Motor rad/s  ≈   0.3 Wheel rad/s  ≈  0.005 Robot m/s  Incremental speed
