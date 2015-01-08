/*
project_balance a brushless motor  v1.1  
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
//https://www.facebook.com/tinnakonza

date: 14-09-2557(2014)  V.1 ,write read acc mpu6050
date: 17-09-2557(2014)  V.2 ,write read Photomicrosensor and write pwm motor
date: 18-09-2557(2014)  V.3 ,write phase shift
date: 19-09-2557(2014)  V.41 ,low pass filter , send data to Labview 2010
date: 19-09-2557(2014)  V.5 , send data to Labview 2010 V5

support:  Board arduino2560
• Atmega2560
• MPU6050C Gyro Accelerometer //400kHz nois gyro +-0.05 deg/s , acc +-0.04 m/s^2
• Photomicrosensor (Transmissive) EE-SX1137
 //EE-SX1137
//VCC = 5 V, Rf = 200 Ω, Anode
//VCC = 5 V, RL = 1 KΩ, E
*/
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "mpu6050.h"
void setup()
{
  Serial.begin(115200);//57600 38400
  Serial.print("TK_balance_brushless_motorV1.1");Serial.println("\t");
  pinMode(13, OUTPUT);//LEDPIN
  delay(10);
  digitalWrite(13, HIGH);
  Wire.begin();
  delay(10);
  mpu6050_initialize();
  delay(30); 
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz 
  delay(10);
  //attachInterrupt(0, ISRspeed, RISING);//numbers 0 (on digital pin 2) 
  //configure THROTTLE PIN (A8 pin) as input witch pullup and enabled PCINT interrupt
  DDRK &= ~(1<<0); PORTK |= (1<<0); PCMSK2 |= (1<<0); PCICR |= (1<<2);
  delay(10);
      for(uint8_t i=0; i<50; i++) 
    {
     mpu6050_Accel_Values();
     delay(20);
    }
    digitalWrite(13, LOW);
  sensor_Calibrate();//sensor.h
  Serial.print("TK_balance_brushless_motor");Serial.println("\t");
  Serial.print("Start_Motor");Serial.println("\t");
  delay(1000);
  previousTime = micros();
  start1 = previousTime;
}
void loop()
{
   Dt_roop = micros() - previousTime;
   if(Dt_roop <= 0)
   {
    Dt_roop = 2001; 
   }   
    if (Dt_roop >= 1000) // 1000 Hz task loop 1000 us
    {
      previousTime = micros();
      G_Dt = Dt_roop*0.000001;
      frameCounter++;
      mpu6050_Accel_Values();
      AccX = accelRawX*accelScaleFactorX - acc_offsetX;  
      RPS_Photoff = RPS_Photoff + (RPS_Photo - RPS_Photoff)*0.0145;//RC = 1/2*pi*fc ,,α := dt / (RC + dt)
///////////////////////////////////////////////////////////////
          Acc_dataX[kj] = AccX;
          Hz_p[kj] = RPS_Photoff;
          degreeMotor = degreeMotor + (RPS_Photoff*360.0*G_Dt);
          degreeMotor = constrain(degreeMotor, 0, 360);
          degreeMo[kj] = degreeMotor;
          kj++;
///////////////////////////////////////////////////////////////
    if(kj > 200)//Collect 100 samples  min 10Hz
   {
     for (int i=0; i<100; i++)
    {
      // normalise the measurements
      Acc_data[i] = sqrt(Acc_dataX[i]*Acc_dataX[i]);
      massBala = Acc_data[i];
      //Acc_dataXf = Acc_dataXf + (Acc_dataX[i] - Acc_dataXf)*0.285;//RC = 1/2*pi*fc ,,α := dt / (RC + dt)
      if(Acc_dataX[i] > AmpBala){
        AmpBala = Acc_dataX[i];
        phase_shift = (Hz_p[i] - 30)*2.57;//(Hz_p[i] - 50)*2.67  1.2  2.97 2.47
        degreUnBala =  degreeMo[i] -  phase_shift + 20;//+ phase_shift[i]
      }
      
    }
    kj = 0;
    AmpBala = 0.0;
    Acc_dataXf = 0.0;
    //phase_shift = 0.0;
   }
///////////////////////////////////////////////////////////////////////
degreUnBalaff = degreUnBalaff + (degreUnBala - degreUnBalaff)*0.00075;//0.00075 0.012
massBalaff = massBalaff + (massBala - massBalaff)*0.00075;
         if (frameCounter % TASK_10HZ == 0)//roop print  ,TASK_5HZ  TASK_10HZ
        {     
    unsigned int xAngle_d = (massBalaff*32767/50.0) + 32767;
    unsigned int yAngle_d = (degreUnBalaff*32767/360.0) + 32767;
    unsigned int xRatef_d = (AccX*32767/100.0) + 32767;
    unsigned int yRatef_d = (RPS_Photoff*32767/300.0) + 32767;
    unsigned int ARatef_d = (phase_shift*32767/1000.0) + 32767; 
    byte Header = 58;//0xFF = 255
    byte output1 = (xAngle_d & 0xFF);   
    byte output2 = ((xAngle_d >> 8) & 0xFF);
    byte output3 = (yAngle_d & 0xFF);   
    byte output4 = ((yAngle_d >> 8) & 0xFF);
    byte output5 = (xRatef_d & 0xFF);   
    byte output6 = ((xRatef_d >> 8) & 0xFF);
    byte output7 = (yRatef_d & 0xFF);   
    byte output8 = ((yRatef_d >> 8) & 0xFF);
    byte output9 = (ARatef_d & 0xFF);   
    byte output10 = ((ARatef_d >> 8) & 0xFF);
    byte Lop = 13;//0x00   
     Serial.write(Header);
     Serial.write(output1);
     Serial.write(output2);
     Serial.write(output3);
     Serial.write(output4);
     Serial.write(output5);
     Serial.write(output6);
     Serial.write(output7);
     Serial.write(output8);
     Serial.write(output9);
     Serial.write(output10);
     Serial.println();
               //Serial.print(degreBalaff);Serial.print("\t");
               //Serial.print("degree");Serial.print("\t");
               //Serial.print(massBalaff);Serial.print("\t");
               //Serial.print("G");Serial.print("\t");
               //Serial.print(phase_shift);Serial.print("\t");
            //Serial.print(sensorPreviousTime*0.000001);Serial.print("\t");//millis() micros()
            //Serial.print(AccX);Serial.print("\t");
            //Serial.print(AccY);Serial.print("\t");
            //Serial.print("m/s2");Serial.print("\t");
            //Serial.print(AccXf);Serial.print("\t");
            //Serial.print(accelRawX);Serial.print("\t");
            //Serial.print(RPS_Photo);Serial.print("\t");
            //Serial.print("Hz");Serial.print("\t");
            //Serial.print(accSamples2);Serial.print("\t");
            //Serial.print(G_Dt*1000);Serial.print("\t");
            //Serial.print("ms");Serial.print("\t");
            //Serial.print("\n"); 
        }//end roop 5 Hz 
        if (frameCounter >= TASK_1HZ) { // Reset frameCounter back to 0 after reaching 100 (1s)
            frameCounter = 0;
            time_sec++;
              if(Status_LED == LOW)
            {
            Status_LED = HIGH;
            }
            else
            {
            Status_LED = LOW;
            }
            digitalWrite(13, Status_LED);//A
        }//end roop 1 Hz
    }//end roop 1000 HZ 
}
///////////////////////////////////////////////////////
ISR(PCINT2_vect){if(PINK&(1<<0))ISRspeed();}
void ISRspeed()
{
  unsigned long now1 = micros();
  width1= now1 - start1;
  degreeMotor = 0.0;//Set zero
  if(width1 <= 0){//overflow (go back to zero)
    width1 = 2147483647;
  }
  RPS_Photo = 1000000.0/width1;//Revolutions per second ,frequency Hz
  start1 = now1;
}
