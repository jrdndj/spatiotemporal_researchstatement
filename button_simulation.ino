/*
  Button simulation via FDVV Models
  * For details of the project and the circuit required, 
  * please refer to our project page at:  
  * https://userinterfaces.aalto.fi/button-design
 
  Our paper can be found at:
  * https://dx.doi.org/10.1145/3313831.3376262 
  * https://arxiv.org/abs/2001.04352
  
  The code is supposed to be run on Adafruit ItsyBitsy M0 Express
  * https://www.adafruit.com/product/3727
  
  Created day 2020 Jan. 15
  By Yi-Chi Liao (yi-chi.liao@aalto.fi)
  Any questions regarding installing and running the system, please contact: yi-chi.liao@aalto.fi
*/

#include <cmath>
#include <Servo.h>
#include <Wire.h>                        // For communicating Arduino Uno for further vibration control


// Replace this section below with the button you want to simulate.
// ===============================================================================
// ============ THIS IS MACBOOK PRO 2011 ==========
// ============ Set the travel range according to your simulation target =========
int travel_range = 2200;

// ============= Button Data ==============
float button_v0_Down[] = {10.000,10.000,10.000,10.000,10.000,12.915,17.273,21.668,25.907,29.885,33.845,37.565,42.320,45.985,49.795,54.017,57.890,61.725,65.105,68.578,71.680,74.192,76.153,77.030,76.097,71.728,60.562,44.178,40.767,39.773,40.250,41.803,44.068,46.947,50.328,53.968,58.108,63.675,70.170,78.175,88.942,101.470,114.735,128.190,142.545};
float button_v0_Up[] =   {10.000,10.000,10.000,10.000,10.000,10.000,11.265,14.220,17.450,20.652,23.782,27.948,32.155,36.212,40.020,43.320,46.663,49.660,52.245,54.453,55.780,55.653,52.367,44.678,35.407,29.075,27.043,26.530,26.775,27.650,29.142,31.098,33.505,36.220,39.195,42.975,47.065,52.513,59.752,68.463,79.495,92.715,106.960,123.860,142.545};
float button_v1_Down[] = {10.000,10.000,10.078,12.500,15.169,18.615,22.810,27.646,32.873,37.831,42.055,45.391,48.819,51.533,54.169,56.788,58.983,60.939,62.504,64.016,65.278,66.176,66.710,66.593,65.452,62.471,55.974,46.734,43.799,41.873,40.575,39.906,39.948,41.006,43.463,47.532,53.230,60.316,67.458,74.093,80.827,87.952,95.751,103.951,112.196};
float button_v1_Up[] =   {10.000,10.235,13.436,17.204,21.356,25.721,30.073,34.077,37.352,39.457,40.386,41.035,41.370,41.657,42.035,42.410,43.028,43.664,44.269,44.820,45.007,44.506,42.456,38.241,33.284,29.877,28.736,28.500,28.786,29.537,30.764,32.413,34.480,36.858,39.453,42.444,45.516,49.292,54.325,60.807,69.108,78.567,87.896,97.624,107.481};
float button_v2_Down[] = {12.764,13.891,15.868,18.345,21.124,24.315,28.347,33.624,39.840,45.776,50.265,53.217,55.318,57.081,58.542,59.560,60.077,60.152,59.903,59.454,58.875,58.160,57.266,56.156,54.807,53.215,51.387,49.290,46.831,43.973,40.900,38.010,35.829,35.065,36.599,41.096,48.353,56.958,64.745,70.010,72.712,74.434,76.767,79.711,81.847};
float button_v2_Up[] =   {16.673,19.395,24.162,30.079,36.456,42.856,48.880,53.934,57.253,58.261,56.991,54.123,50.584,47.103,44.050,41.501,39.393,37.668,36.292,35.186,34.234,33.359,32.545,31.804,31.162,30.678,30.430,30.469,30.798,31.424,32.386,33.729,35.455,37.495,39.711,41.913,43.967,46.071,48.898,53.152,58.721,64.418,68.832,71.387,72.417};
float button_v3_Down[] = {11.410,11.986,12.997,14.279,15.801,17.850,21.159,26.600,34.220,42.531,49.346,53.478,55.317,55.981,56.318,56.594,56.734,56.649,56.336,55.837,55.180,54.342,53.257,51.849,50.095,48.045,45.779,43.349,40.748,37.970,35.094,32.361,30.257,29.544,31.066,35.370,42.376,51.210,60.276,67.812,72.924,76.181,78.822,81.308,82.968};
float button_v3_Up[] =   {17.075,19.962,25.016,31.290,38.050,44.830,51.202,56.533,60.026,61.102,59.809,56.828,53.079,49.323,46.005,43.244,40.937,38.933,37.153,35.578,34.216,33.070,32.111,31.297,30.602,30.047,29.681,29.552,29.724,30.277,31.268,32.679,34.431,36.428,38.606,40.898,43.206,45.545,48.247,51.803,56.306,61.136,65.329,68.221,69.640};
float button_v4_Down[]=  {10.057,10.080,10.126,10.214,10.479,11.385,13.971,19.576,28.599,39.285,48.427,53.739,55.317,54.882,54.094,53.627,53.392,53.146,52.769,52.221,51.484,50.524,49.248,47.542,45.383,42.875,40.172,37.408,34.666,31.967,29.288,26.712,24.684,24.024,25.533,29.643,36.398,45.463,55.808,65.615,73.136,77.929,80.877,82.904,84.088};
float button_v4_Up[]=    {17.478,20.528,25.870,32.501,39.643,46.804,53.524,59.132,62.799,63.943,62.627,59.534,55.574,51.543,47.960,44.987,42.481,40.199,38.013,35.970,34.199,32.780,31.678,30.791,30.042,29.416,28.931,28.635,28.649,29.130,30.149,31.630,33.407,35.361,37.501,39.884,42.446,45.019,47.596,50.454,53.892,57.855,61.826,65.055,66.863};
float button_v5_Down[] = {10.410,10.576,10.874,11.280,11.900,13.210,16.267,22.348,31.594,41.933,50.192,54.668,56.118,56.385,56.688,57.162,57.381,56.998,55.965,54.399,52.395,49.978,47.145,43.948,40.514,36.986,33.516,30.369,27.903,26.352,25.730,26.060,27.672,31.062,36.363,43.065,50.396,57.874,65.311,72.429,78.752,83.882,87.678,90.171,91.409};
float button_v5_Up[] =   {16.110,18.602,22.968,28.396,34.293,40.403,46.701,53.174,59.517,64.965,68.557,69.706,68.517,65.646,61.871,57.754,53.574,49.486,45.666,42.273,39.341,36.838,34.799,33.317,32.362,31.733,31.216,30.769,30.547,30.753,31.488,32.697,34.195,35.753,37.281,38.959,41.041,43.639,46.886,51.117,56.521,62.645,68.462,72.913,75.295};
float button_v6_Down[] = {10.762,11.073,11.623,12.345,13.321,15.035,18.564,25.119,34.589,44.582,51.958,55.597,56.920,57.887,59.282,60.697,61.370,60.849,59.162,56.577,53.307,49.432,45.042,40.353,35.645,31.098,26.860,23.330,21.141,20.738,22.172,25.408,30.659,38.100,47.193,56.487,64.394,70.285,74.813,79.243,84.369,89.834,94.479,97.438,98.729};
float button_v6_Up[] =   {14.742,16.675,20.065,24.291,28.943,34.001,39.878,47.217,56.235,65.986,74.487,79.877,81.461,79.749,75.781,70.521,64.667,58.774,53.320,48.575,44.483,40.896,37.920,35.843,34.683,34.051,33.501,32.903,32.444,32.376,32.827,33.763,34.984,36.145,37.062,38.035,39.637,42.260,46.177,51.780,59.150,67.436,75.098,80.771,83.727};
float button_v7_Down[] = {10.518,10.730,11.105,11.604,12.312,13.679,16.763,22.971,32.784,44.451,54.657,60.997,63.490,63.733,63.243,62.629,61.736,60.218,57.975,55.191,52.115,48.895,45.577,42.176,38.698,35.143,31.618,28.502,26.279,25.069,24.523,24.379,24.928,26.823,30.527,36.080,43.259,51.651,60.398,68.227,74.267,78.779,82.597,85.909,87.970};
float button_v7_Up[] =   {17.314,20.296,25.520,32.013,39.041,46.231,53.401,60.319,66.513,71.257,73.856,74.064,72.233,69.055,65.157,60.917,56.575,52.454,48.929,46.130,43.806,41.645,39.687,38.252,37.520,37.332,37.383,37.501,37.734,38.228,39.078,40.274,41.739,43.356,45.073,47.057,49.569,52.633,56.070,59.889,64.365,69.631,75.243,80.136,83.019};
float button_v8_Down[] = {10.275,10.387,10.587,10.862,11.303,12.323,14.962,20.822,30.978,44.321,57.356,66.397,70.060,69.579,67.205,64.560,62.102,59.586,56.788,53.806,50.924,48.357,46.112,43.999,41.751,39.188,36.376,33.673,31.418,29.399,26.875,23.349,19.196,15.545,13.861,15.673,22.123,33.018,45.984,57.210,64.166,67.724,70.715,74.379,77.211};
float button_v8_Up[] =   {19.885,23.916,30.976,39.734,49.139,58.462,66.924,73.421,76.792,76.529,73.226,68.252,63.006,58.362,54.534,51.313,48.483,46.135,44.538,43.686,43.129,42.395,41.453,40.660,40.358,40.614,41.264,42.099,43.024,44.081,45.329,46.785,48.495,50.567,53.084,56.079,59.500,63.005,65.963,67.997,69.580,71.826,75.389,79.501,82.312};
float button_v9_Down[] = {10.032,10.044,10.069,10.120,10.294,10.967,13.161,18.673,29.172,44.191,60.055,71.797,76.630,75.425,71.166,66.492,62.468,58.954,55.601,52.420,49.733,47.819,46.647,45.822,44.804,43.233,41.134,38.845,36.556,33.730,29.227,22.319,13.465,10.000,10.000,10.000,10.000,14.385,31.570,46.194,54.064,56.669,58.833,62.850,66.452};
float button_v9_Up[] =   {22.457,27.537,36.431,47.456,59.237,70.693,80.447,86.523,87.071,81.800,72.596,62.439,53.779,47.669,43.910,41.709,40.391,39.816,40.147,41.242,42.452,43.145,43.220,43.068,43.195,43.895,45.146,46.697,48.314,49.934,51.580,53.296,55.250,57.778,61.095,65.101,69.431,73.377,75.856,76.106,74.795,74.021,75.534,78.866,81.605};

// ============ Vibration settings ============
bool if_vib = true;
int vib_point_1 = 900;    // will not activate anything
int vib_point_2 = 1200;    // will not activate anything

int vib_file0 = 5; // will not activate anything
int vib_file1 = 5;
int vib_file2 = 11;
int vib_file3 = 11;
int vib_file4 = 14;
int vib_file5 = 14;
int vib_file6 = 18;
int vib_file7 = 18;
int vib_file8 = 20;
int vib_file9 = 20;

// ===============================================================================
// Replace the section above

// ============ Parameters for position sensor and force actuator which require calibrations ============
float pos_intercept = 14565.7522;        // These two parameters (pos_intercept, pos_slope) may require 
float pos_slope = -3.9103;               // mannual calibration using linear regression of the position sensor.

float pwm_par0_up = 368.211176;          // Below are for caculating the pwm signal based on the desired force level,
float pwm_par1_up = 4.213580;            // they might require calibration of force actuator.
float pwm_par2_up = -0.014787;
float pwm_par0_down = 283.607315;
float pwm_par1_down = 4.917654;
float pwm_par2_down = -0.0157675;

float travel_intercept = 158.77735;
float travel_slope = -0.021463;

// ============= Global Variables ============
int full_index = travel_range/50 + 1;
int analog_in = A1;             // Pin A1 is for receiving the raw data of displacement
int frequency_test_pin = 11;
int servo_pin = 12;
float cali_pos_sensor = 0;
int pos_sampling_times = 25;    // Every displacement is an average of 25 samples
int cali_time = 1000;           // The sampling times for real-time calibrating the position 
float velocity_compensation = 2.3; // This parameter is obtained empirically

#define DOWN 0
#define UP 1
int cur_dir = UP;               // 0 for down, and 1 for up
int press_status = 0;           // 0 for reset, 1 for detecting velocity, 2 for velocity detected
int vib_status = 0;             // 0 for reset, 1 for detecting velocity, 2 for velocity detected
unsigned long press_start_micro, press_finish_micro, press_duration;
int cur_button = 0;
int cur_vib = 0;
int pwm_adjustment = 0;
Servo myservo;
int cur_servo_deg = 90;
int next_servo_deg = 90;

// Below parameters are the threshold for detecing velocities,
// which are separated based on the velocity the button is modelled. 
int v_thres8 = 200; 
int v_thres7 = 175; 
int v_thres6 = 150; 
int v_thres5 = 125;
int v_thres4 = 100; 
int v_thres3 = 75; 
int v_thres2 = 50; 
int v_thres1 = 25; 
int v_thres0 = 5; 



void setup() 
{
  setTimers();                        // Set timers for fast PWM. More info please refer to:
                                      // https://create.arduino.cc/projecthub/voske65/high-speed-pwm-on-arduino-atsamd21-859b06
                                      
  analogReadResolution(12);           // Set a higher analog read resolution.
  set_pwm_up(420);                    // Set a default force to push it up 
  delay(200);                         // delay
  set_displacement(travel_range+100);  // Set the right travel range  
  Serial.begin(9600);                 // Start the serial communication
  Wire.begin();                       // Start the I2C communication to Arduino Uno
  pinMode(frequency_test_pin, OUTPUT);// Set pin 11 as outpur for testing the frequency easily
}

void loop() 
{
  //digitalWrite(frequency_test_pin, HIGH);                // For you to examine the operating rate with Oscilloscope.
  unsigned long main_loop_starting_time = micros();
  if (Serial.available()>0)
  {
    char inByte = Serial.read();  
    if (inByte == 'c')                                     // Press 'c' in the Serial monitor to calibrate the position sensor
    {                                                      // i.e., using the current displacement as 0. 
      Serial.println("Start calibrating position sensor"); // If the position sensor doesn't respond properly, you need a manual calibration 
      calibrate_pos_sensor();
    }
  }
  //digitalWrite(frequency_test_pin, LOW);                 // For you to examine the operating rate with Oscilloscope
  simulate_button();
  Wire.write(1);
  global_delay(main_loop_starting_time);
}

void simulate_button()
{
  // 1. Get raw data and it's already been smoothed
  int raw_pos_data = get_raw_pos_data(); 
  
  // 2. Transfer input data into displacement
  int pos = get_position(raw_pos_data); 
  
  // 3. Normalize the displacement
  pos = normal_pos(pos);
  
  // 4. Get direction & get velocity
  cur_dir = get_direction(pos);

  // 5. Switching FD curves based on the velocity
  if(pos<500)
  {press_status = 0;}
  if (pos > 500 && press_status == 0)  // Once the displacement reaches > 500 um, the detection starts
  {
    press_start_micro = micros();
    press_status = 1;
  }
  if (pos>1000 && press_status == 1)   // Once the displacement reaches > 1000 um, the detection stops
  {
    press_finish_micro = micros();
    press_duration = press_finish_micro - press_start_micro;
    int vel_tmp = 500 / (press_duration/1000) * velocity_compensation;  // Derive the pressing velocity (mm/s)
    cur_button = get_button(vel_tmp);  // Switching the FD curve according to the velocity
    if (if_vib)
    {cur_vib = get_vibration(vel_tmp);}  // Switching the vibration soundwave according to the velocity
    
    press_status = 2;
  }
  
  // 6. Get force level
  int force = get_force(pos,cur_dir, cur_button);
  
  // 7. Get the pwm level and generate the desired force
  int pwm = get_pwm_less(cur_dir,force) + pwm_adjustment;
  set_pwm_up(pwm);
  
  
  // 8. Call i2c and vibrate if the displacement is correct
  if (pos > vib_point_1 && pos < vib_point_2 && vib_status == 0 && if_vib)
  {
    Wire.beginTransmission(9); // transmit to device #9
    Wire.write(cur_vib);              // sends x 
    Wire.endTransmission();    // stop transmitting  
    vib_status = 1;
  }
  if (pos > travel_range-200 || pos< 400)
  {vib_status = 0;}
}

// This function is for switching the button force data according to the initial pressing velocity
int get_button(int vel_tmp)
{
  int button_temp = 0;
  if (vel_tmp > v_thres8)
  {button_temp = 9;}
  else if (vel_tmp < v_thres8 && vel_tmp >= v_thres7)
  {button_temp = 8;}
  else if (vel_tmp < v_thres7 && vel_tmp >= v_thres6)
  {button_temp = 7;}
  else if (vel_tmp < v_thres6 && vel_tmp >= v_thres5)
  {button_temp = 6;}
  else if (vel_tmp < v_thres5 && vel_tmp >= v_thres4)
  {button_temp = 5;}
  else if (vel_tmp < v_thres4 && vel_tmp >= v_thres3)
  {button_temp = 4;}
  else if (vel_tmp < v_thres3 && vel_tmp >= v_thres2)
  {button_temp = 3;}
  else if (vel_tmp < v_thres2 && vel_tmp >= v_thres1)
  {button_temp = 7;}
  else if (vel_tmp < v_thres1 && vel_tmp >= v_thres0)
  {button_temp = 1;}
  else
  {button_temp = 0;}
  return button_temp;
}

// This function is for switching the button vibration data according to the initial pressing velocity
int get_vibration(int vel_tmp)
{
  int vib_temp = 0;
  if (vel_tmp > v_thres8)
  {vib_temp = vib_file9;}
  else if (vel_tmp < v_thres8 && vel_tmp >= v_thres7)
  {vib_temp = vib_file8;}
  else if (vel_tmp < v_thres7 && vel_tmp >= v_thres6)
  {vib_temp = vib_file7;}
  else if (vel_tmp < v_thres6 && vel_tmp >= v_thres5)
  {vib_temp = vib_file6;}
  else if (vel_tmp < v_thres5 && vel_tmp >= v_thres4)
  {vib_temp = vib_file5;}
  else if (vel_tmp < v_thres4 && vel_tmp >= v_thres3)
  {vib_temp = vib_file4;}
  else if (vel_tmp < v_thres3 && vel_tmp >= v_thres2)
  {vib_temp = vib_file3;}
  else if (vel_tmp < v_thres2 && vel_tmp >= v_thres1)
  {vib_temp = vib_file2;}
  else if (vel_tmp < v_thres1 && vel_tmp >= v_thres0)
  {vib_temp = vib_file1;}
  else
  {vib_temp = vib_file0;}
  return vib_temp;
}

// Get the right force level according to displacement, direction and the button data (based on velocity)
float get_force(int cur_pos, int cur_dir, int button_id)
{
  int getArray = cur_pos/50;
  float temp_force = 40;
  switch (button_id)
  {
    case 0:
      if (cur_dir == 0)
      {temp_force = button_v0_Up[getArray];}
      else
      {temp_force = button_v0_Down[getArray];}
      break;
    
    case 1:
      if (cur_dir == 0)
      {temp_force = button_v1_Up[getArray];}
      else
      {temp_force = button_v1_Down[getArray];}
      break;
   
    case 2:
      if (cur_dir == 0)
      {temp_force = button_v2_Up[getArray];}
      else
      {temp_force = button_v2_Down[getArray];}
      break;
    
    case 3:
      if (cur_dir == 0)
      {temp_force = button_v3_Up[getArray];}
      else
      {temp_force = button_v3_Down[getArray];}
      break;
    
    case 4:
      if (cur_dir == 0)
      {temp_force = button_v4_Up[getArray];}
      else
      {temp_force = button_v4_Down[getArray];}
      break;
      
    case 5:
      if (cur_dir == 0)
      {temp_force = button_v5_Up[getArray];}
      else
      {temp_force = button_v5_Down[getArray];}
      break;
    
    case 6:
      if (cur_dir == 0)
      {temp_force = button_v6_Up[getArray];}
      else
      {temp_force = button_v6_Down[getArray];}
      break;
   
    case 7:
      if (cur_dir == 0)
      {temp_force = button_v7_Up[getArray];}
      else
      {temp_force = button_v7_Down[getArray];}
      break;
    
    case 8:
      if (cur_dir == 0)
      {temp_force = button_v8_Up[getArray];}
      else
      {temp_force = button_v8_Down[getArray];}
      break;
    
    case 9:
      if (cur_dir == 0)
      {temp_force = button_v9_Up[getArray];}
      else
      {temp_force = button_v9_Down[getArray];}
      break;
    
    default:
      break;
  }
  return temp_force;
}

// Get the PWM level according to the target force level and the direction
int get_pwm_less(int curr_dir, float force)
{
  int pwm;
  if (curr_dir == 0) // if direction is down
  {
    pwm = -0.0157675* pow(force,2) + 4.917654*force + 283.607315;
  }
  else // if direction is up
  {
    pwm = -0.014787* pow(force,2) + 4.213580*force + 368.211176;
  }
  return pwm;
}

// This function is for controlling the operation rate slightly below 1 kHz, which is the highest effective rate for force actuator
void global_delay(unsigned long loop_starting_time)
{
  int tmp_interval = micros() - loop_starting_time;
  if ( tmp_interval < 1000)
  {
    delayMicroseconds((unsigned int)(1000-tmp_interval));
  }
}

// Get position data
int get_raw_pos_data()
{
  int average = 0;
  for (int i =0; i<pos_sampling_times;i++)
  {
    average += analogRead(analog_in);
  }
  int raw_data = average/pos_sampling_times;
  return raw_data;
}

// Get position from raw data
int get_position(int data)
{
  int position_temp = pos_intercept + pos_slope * data - cali_pos_sensor;
  return position_temp;
}

int normal_pos(int cur_pos)
{
  if (cur_pos < 0)
  {cur_pos = 0;}
  if (cur_pos > travel_range)
  {cur_pos = travel_range;} 
  return cur_pos;
}

// Get dirction out of position. 
// Note: We assume that every press will touch bottom then return, 
//       so the direction change only in initial state and the bottom
int get_direction(int cur_pos)
{
  if (cur_pos < 300) 
  {return DOWN;}
  else if (cur_pos > (travel_range-10))
  {return UP;}
  else
  {return cur_dir;}
}

// Calibrating position sensor
void calibrate_pos_sensor()
{
  float average_tmp = 0;
  cali_pos_sensor = 0;
  for (int i=0; i < cali_time; i++)
  { 
    int pos_temp = get_raw_pos_data(); 
    int pos = get_position(pos_temp); 
    average_tmp += pos;
    delay(1);
  }
  cali_pos_sensor = average_tmp / cali_time;
  Serial.println(cali_pos_sensor);
  Serial.println("Finish position sensor calibration");  
}

// ============== Setup functions ==============

void setTimers()
{
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by divisor 1: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the PWM channel on pin D9  
  PORT->Group[g_APinDescription[9].ulPort].PINCFG[g_APinDescription[9].ulPin].bit.PMUXEN = 1;
  
  
  // Connect the TCC1 timer to the port outputs - port pins are paired odd PMUO and even PMUXE
  // F & E peripherals specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[9].ulPort].PMUX[g_APinDescription[9].ulPin >> 1].reg |= PORT_PMUX_PMUXO_E; 
  
  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Normal (single slope) PWM operation: timers countinuously count up to PER register value and then is reset to 0
  REG_TCC1_WAVE |= TCC_WAVE_WAVEGEN_NPWM;        // Setup single slope PWM on TCC1
  while (TCC1->SYNCBUSY.bit.WAVE);                // Wait for synchronization
  

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation: 959 = 50kHz
  REG_TCC1_PER = 959;      // Set the frequency of the PWM on TCC1 to 50kHz
  while(TCC1->SYNCBUSY.bit.PER);

  // The CCBx register value corresponds to the pulsewidth in microseconds (us) 
  //REG_TCC1_CC1 = 480;      // Set the duty cycle of the PWM on TCC0 to 50%
  //while(TCC1->SYNCBUSY.bit.CC1);
 
  // Enable TCC1 timer
  REG_TCC1_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

  //====== SET PD7 =======// 
  
  PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTTOM;    // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  // 400 = 20kHz
  REG_TCC0_PER = 400;      // Set the frequency of the PWM on TCC0 to 50Hz
  while(TCC0->SYNCBUSY.bit.PER);

  // The CCBx register value corresponds to the pulsewidth in microseconds (us)
  //REG_TCC0_CCB3 = 0;       // TCC0 CCB3 - 50% duty cycle on D7
  //while(TCC0->SYNCBUSY.bit.CCB3);

  // Divide the 16MHz signal by 1 giving 16MHz (62.5ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

// For faster actuating the force actuator, we set the pwm value via these functions.
void set_pwm_up(int i)
{
  REG_TCC1_CCB1 = i;
  while(TCC1->SYNCBUSY.bit.CCB1);
}
void set_pwm_down(int i)
{
  REG_TCC0_CCB3 = i;       // TCC0 CCB3 - 50% duty cycle on D7
  while(TCC0->SYNCBUSY.bit.CCB3);
}

// We manipulate the servo motor for controlling the travel range.
void set_displacement(int travel)
{
  next_servo_deg = travel_slope*travel + travel_intercept;
  if (next_servo_deg < 0)
  {next_servo_deg = 0;}
  if (next_servo_deg > 163)
  {next_servo_deg = 163;}
  
  myservo.attach(servo_pin);
  servo_return();
  delay(2000);
  servo_set(next_servo_deg);
  delay(2000);
  cur_servo_deg = next_servo_deg;
  myservo.detach();  
}
void servo_return ()
{
  for (int pos = cur_servo_deg; pos >= 0; pos -= 1) 
  {
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(1);                       // waits 15ms for the servo to reach the position
  }
}
void servo_set (int target_degree)
{
  for (int pos = 0; pos <= target_degree; pos += 1) 
  {
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(3);                       // waits 15ms for the servo to reach the position
  }
}
