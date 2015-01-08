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
////////////////////////////////////////////////////////////////////
float accelX;
float accelScaleFactorX = 9.80665/16182;//9.81/8192.09 9.81 / 8205  
int16_t accSamples = 0;
//int16_t accSamples2 = 0;
int16_t accelRawX;
int16_t accelRawY;
float accelSumX;
float accelSumY;
float AccX;
float AccXf;
float acc_offsetX;
float AccY;
float AccYf;
float acc_offsetY;

//sensor MPU6050 -------------------------------------
// MPU 6050 Registers
#define MPU6050_ADDRESS         0x69  //0x68
#define MPUREG_WHOAMI           0x75
#define MPUREG_SMPLRT_DIV       0x19
#define MPUREG_CONFIG           0x1A
#define MPUREG_GYRO_CONFIG      0x1B
#define MPUREG_ACCEL_CONFIG     0x1C
#define MPUREG_FIFO_EN          0x23
#define MPUREG_INT_PIN_CFG      0x37
#define MPUREG_INT_ENABLE       0x38
#define MPUREG_INT_STATUS       0x3A
#define MPUREG_ACCEL_XOUT_H     0x3B
#define MPUREG_ACCEL_XOUT_L     0x3C
#define MPUREG_ACCEL_YOUT_H     0x3D
#define MPUREG_ACCEL_YOUT_L     0x3E
#define MPUREG_ACCEL_ZOUT_H     0x3F
#define MPUREG_ACCEL_ZOUT_L     0x40
#define MPUREG_TEMP_OUT_H       0x41
#define MPUREG_TEMP_OUT_L       0x42
#define MPUREG_GYRO_XOUT_H      0x43
#define MPUREG_GYRO_XOUT_L      0x44
#define MPUREG_GYRO_YOUT_H      0x45
#define MPUREG_GYRO_YOUT_L      0x46
#define MPUREG_GYRO_ZOUT_H      0x47
#define MPUREG_GYRO_ZOUT_L      0x48
#define MPUREG_USER_CTRL        0x6A
#define MPUREG_PWR_MGMT_1       0x6B
#define MPUREG_PWR_MGMT_2       0x6C
#define MPUREG_FIFO_COUNTH      0x72
#define MPUREG_FIFO_COUNTL      0x73
#define MPUREG_FIFO_R_W         0x74
// Configuration bits
#define BIT_SLEEP               0x40
#define BIT_H_RESET             0x80
#define BITS_CLKSEL             0x07
#define MPU_CLK_SEL_PLLGYROX    0x01
#define MPU_CLK_SEL_PLLGYROZ    0x03
#define MPU_EXT_SYNC_GYROX      0x02
#define BITS_FS_250DPS          0x00
#define BITS_FS_500DPS          0x08
#define BITS_FS_1000DPS         0x10
#define BITS_FS_2000DPS         0x18
#define BITS_FS_MASK            0x18
#define BITS_DLPF_CFG_256HZ  0x00// //Default settings LPF 256Hz/8000Hz sample
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR    0x10
#define BIT_RAW_RDY_EN          0x01
#define BIT_I2C_IF_DIS          0x10
#define BIT_INT_STATUS_DATA     0x01
void mpu6050_initialize()
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_PWR_MGMT_1);    // Chip reset DEVICE_RESET 1
    Wire.write(BIT_H_RESET);//DEVICE_RESET
    Wire.endTransmission();  
    delay(10);// Startup delay      
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_PWR_MGMT_1);
    Wire.write(MPU_CLK_SEL_PLLGYROZ);//CLKSEL 3 (PLL with Z Gyro reference)
    Wire.endTransmission();    
    delay(5);
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_SMPLRT_DIV);// SAMPLE RATE
    Wire.write(0x00);//// Sample rate = 1kHz
    Wire.endTransmission();  
    delay(5);
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_CONFIG);
    Wire.write(BITS_DLPF_CFG_98HZ);//98 BITS_DLPF_CFG_188HZ, BITS_DLPF_CFG_42HZ, default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    Wire.endTransmission();  
    delay(5);   
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_GYRO_CONFIG);
    Wire.write(BITS_FS_1000DPS);//BITS_FS_1000DPS FS_SEL = 3: Full scale set to 2000 deg/sec,  BITS_FS_2000DPS
    Wire.endTransmission(); 
    delay(5);  
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_ACCEL_CONFIG);
    Wire.write(0x00);//0x10 = 1G=4096, AFS_SEL=2 (0x00 = +/-2G)  1G = 16,384 //0x10 = 1G = 4096 ,//0x08 = +-4g
    Wire.endTransmission();
    delay(5);
}
void mpu6050_Accel_Values()
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPUREG_ACCEL_XOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_ADDRESS, 2);
    int i = 0;
    byte result[4];
  while(Wire.available())    
  { 
    result[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();
    accelRawX = ((result[0] << 8) | result[1]);//65
    //accelRawY = ((result[2] << 8) | result[3])*-1;
}

void mpu6050_readAccelSum() {
    mpu6050_Accel_Values();
    accelSumX += accelRawX;
    //accelSumY += accelRawY;
    accSamples++; 
}
void mpu6050_Get_accel()
{
  if(accSamples == 0){
    accSamples = 1;
  }
    // Calculate average
    AccX = (accelSumX / accSamples)*accelScaleFactorX - acc_offsetX;  
    //AccY = (accelSumY / accSamples)*accelScaleFactorX - acc_offsetY;      
    // Apply correct scaling (at this point accel reprensents +- 1g = 9.81 m/s^2)
    // Reset SUM variables
    accelSumX = 0;
    //accelSumY = 0;
    //accSamples2 = accSamples;
    accSamples = 0; 
}
void sensor_Calibrate()
{
  Serial.print("Sensor_Calibrate");Serial.println("\t");
    accelSumX = 0;
    //accelSumY = 0;
    accSamples = 0;
    for (uint8_t i=0; i<60; i++) //Collect 60 samples
    {
        Serial.print("- ");
        mpu6050_readAccelSum();
        digitalWrite(13, HIGH);
        delay(15);
        digitalWrite(13, LOW);
        delay(15);
    }
    Serial.println("- ");
    acc_offsetX = (accelSumX/accSamples)*accelScaleFactorX;
    //acc_offsetY = (accelSumY/accSamples)*accelScaleFactorX;
    accelSumX = 0;
    //accelSumY = 0;
    accSamples = 0.0;
    Serial.print("ACC_Calibrate");Serial.print("\t");
    Serial.print(acc_offsetX);Serial.print("\t");
    Serial.print(acc_offsetY);Serial.println("\t");
    //acc_offsetX = 0.0;//-0.18 0.11 -0.36  Trim PITCH CONTROL   -10.07	-10.55	-9.82
}
