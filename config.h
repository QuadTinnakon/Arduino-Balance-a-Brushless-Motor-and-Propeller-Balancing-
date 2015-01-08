/*
project_balance a brushless motor  v1.1  
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
support:  Board arduino2560
• Atmega2560
• MPU6050C Gyro Accelerometer //400kHz nois gyro +-0.05 deg/s , acc +-0.04 m/s^2
• Photomicrosensor (Transmissive) EE-SX1137
*/
int Photomicro = 2;//numbers 0 (on digital pin 2)
float RPS_Photo = 0.0;//max 195.62 Rev/s motor 1000 KV
float RPS_Photoff = 0.0;
long width1 = 2147483647;//2^32 bit/2
unsigned long start1 = 0;
//int angle_motor = 1;
//rec Data
int kj=0;
float Acc_data[201];
float Acc_dataX[201];
float Acc_dataXf = 0.0;
float phase_shift = 0.0;
int do_print = 1;
float time_p[201];
float Hz_p[201];
float degreeMotor = 0.0;
float degreeMo[201];
float degreUnBala = 0.0;
float degreUnBalaff = 0.0;
float massBala = 0.0;
float massBalaff = 0.0;
float AmpBala = 0.0;
//motor
int MOTOR_PIN = 6;
#define PWM_FREQUENCY 50   //400 in Hz
#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)
////////////////////////////////////////////////////////////////////
#define TASK_100HZ 5
#define TASK_50HZ 10
#define TASK_20HZ 25
#define TASK_10HZ 100
#define TASK_5HZ 200
#define TASK_2HZ 250
#define TASK_1HZ 500
#define RAD_TO_DEG 57.295779513082320876798154814105

// Main loop variables
unsigned long currentTime = 0;
unsigned long previousTime = 0;
unsigned long sensorPreviousTime = 0;
int frameCounter = 0;
int time_sec = 1;
float G_Dt = 0.01; 
long Dt_sensor = 1000;
long Dt_roop = 10000;
int Status_LED = LOW;
