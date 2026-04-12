#pragma once

const int     N_SEN =    4;     // Number of front sensors
const int     S_SEN =    2;     // Number of left and right sensors
const int     THOLD =  100;     // IR sensor threshold     (600)
const int     NR_GS = 1000;     // Normal max value of sensors
const int CALIB_DLY =   14;     // Time to calibrate  | 3 tourns = 14 s
const int  STOP_DLY =    3;     // Pause | 3 s
const int  L_Margin =   10;     // Left margin of the screen
const int  T_Margin =   10;     // Top  margin of the screen
const int P_Spacing =   28;     // Paragraph spacing of the screen
const int  S_Margin =   26;     // Margin of sensor on the screen
const int S_Spacing =   24;     // Spacing of sensor on the screen
const int   S_width =    8;     // Width of sensor on the screen
const int  S_height =   14;     // Height of sensor on the screen

const int    SP000 =   20;      //  20 Motor rad/s  ≈   2.7 Wheel rad/s  ≈  0.047 Robot m/s  Calibration
const int    SP001 =  220;      // 220 Motor rad/s  ≈  29.3 Wheel rad/s  ≈  0.505 Robot m/s  Base speed
const int    SP002 =  440;      // 440 Motor rad/s  ≈  58.7 Wheel rad/s  ≈  1.013 Robot m/s  Max speed
const int    ST_CH =    1;      //   1 Motor rad/s  ≈   0.1 Wheel rad/s  ≈  0.002 Robot m/s  Incremental speed
