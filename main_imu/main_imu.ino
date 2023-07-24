/* 
Senior Capstone Design _ Team ZEST
 */
#include <Servo.h>
#include <Wire.h>
#include "Kalman.h" 


//*****************************************DEFINE NUMS*********************************************
//pin 설정 
#define SERVO_YAWING1_PIN 11
#define SERVO_YAWING2_PIN 10
#define SERVO_TILTING_PIN 9
#define VIBRATION_PIN 6
//안쓰는 것들
#define SERVO_PINION_PIN 12
#define SERVO_GRIPPING_PIN 13
#define FLEX1_PIN A0  //up
#define FLEX2_PIN A1  //down
#define FLEX3_PIN A2  //gripper
#define SW_PIN A3  //Switch
#define PRESS1_PIN A4  //gripper press1
#define PRESS2_PIN A5  //gripper press2
//IMU Pin -> SDA:20 SCL:21


//Gripper 임계값 설정
#define PRESS_MAX 50
//실수 방지 임계값 설정
#define FLEX_MISTAKE 10 //
#define IMU_MISTAKE  10 //

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead 
//지워도 될 듯 ????

//*****************************************CREATE INSTANCES**************************************************
// Create the Kalman instances
Kalman kalmanX; 
Kalman kalmanY;

//Create motor instances
Servo ser_yaw1;
Servo ser_yaw2;
Servo ser_tilt;
Servo ser_pinion;
Servo ser_grip;

//**************************************변수 선언***********************************************
//-----------------------Switch-----------------------
int sw; //Switch On/Off
//-----------------------motor------------------------
int yaw_tilt_IA = 90; //yawing과 tilting 모터의 Initial Angle(똑바로 서있는 상태)
int yaw_tilt_IA_p = map(yaw_tilt_IA, 0, 180, 0, 255); //위와 대응하는 pwm 값

int yaw_tilt_MA = 60; //yawing과 tilting 모터의 Motion Angle(앞뒤로 움직이는 각도): 이 경우엔 120도의 회전각을 가짐
int yaw_tilt_MA_p = map(yaw_tilt_MA, 0, 180, 0, 255); //위와 대응하는 pwm 값

int pinion_IA = 10; //pinion gear 모터의 Initial Angle (공차 조금 주기)
int pinion_FA = 170; //pinion gear 모터의 Final Angle (공차 조금 주기)
int pinion_IA_p = map(pinion_IA, 0, 180, 0, 255); //위와 대응하는 pwm 값 10*255/180
int pinion_FA_p = map(pinion_FA, 0, 180, 0, 255); //위와 대응하는 pwm 값 170*255/180=

int grip_IA = 30;  //gripper 모터의 Initial Angle(그리퍼가 쫙 펴진 상태)
int grip_IA_p = map(grip_IA, 0, 180, 0, 255); //위와 대응하는 pwm 값

int grip_FA = 150; //gripper 모터의 Final Angle(그리퍼가 최대로 움켜지는 상태)
int grip_FA_p = map(grip_FA, 0, 180, 0, 255); //위와 대응하는 pwm 값

//ser_pinion에 넣을 pwm 값 계산(얘는 위치제어가 아닌 방향과 속도제어)
int pinion_A = pinion_IA;
int pinion_p = map(pinion_A, 0, 180, 0, 255);

//-----------------------------flex sensor 관련 선언--------------------------------------
int flex_up_max = 24; //flex sensor max 값 지정
int flex_down_max = 24; 
int flex_grip_max = 40; 
int flex_up_min = 9; //flex sensor min 값 지정
int flex_down_min = 11; 
int flex_grip_min = 10;

//-----------------------------gripper 관련 선언--------------------------------------
int grip_stop;

//------------------------------ IMU Data 관련 선언 --------------------------------------------
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

//------------------------------ For Moving Average 선언--------------------------------------------
const int SAMPLING_NUM = 10;

int readings_flex [SAMPLING_NUM];
int readings_yaw [SAMPLING_NUM];
int readings_tilt [SAMPLING_NUM];
int Index = 0; 
int total_flex = 0;
int total_yaw = 0;
int total_tilt = 0;
float average_flex, average_yaw, average_tilt;
int cnt = 0;

//----vibration-----//
int vibration;


//********************************************USER FUNCTION DEFINE*************************************************
int TO_BIN(int val){
  if(val>40) return 1;
  else  return 0;
}
float MOVING_AVERAGE (int target, int indicator){
  /*if(Index==SAMPLING_NUM - 1) {
    Index=0;
    cnt++;
  }*/
  if (indicator == 1){
  total_yaw = total_yaw-readings_yaw[Index];
  readings_yaw[Index] = target;
  total_yaw = total_yaw + readings_yaw[Index];
  average_yaw = total_yaw / SAMPLING_NUM;
  if(cnt>0)  return average_yaw;
  else  return target;
  }

  else if (indicator == 2){
  total_tilt = total_tilt-readings_tilt[Index];
  readings_tilt[Index] = target;
  total_tilt = total_tilt + readings_tilt[Index];
  average_tilt = total_tilt / SAMPLING_NUM;
  if(cnt>0) return average_tilt;
  else  return target;
  }

  else if (indicator == 3){
  total_flex = total_flex-readings_flex[Index];
  readings_flex[Index] = target;
  total_flex = total_flex + readings_flex[Index];
  average_flex = total_flex / SAMPLING_NUM;
  if(cnt>0) return average_flex;
  else  return target;
  }
  
  
}

void IMU_filtering(double accX, double accY, double accZ, double tempRaw, double gyroX, double gyroY, double gyroZ, double dt){
#ifdef RESTRICT_PITCH 
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else //Use This one
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  /* Print Data */
//#if 0 // Set to 1 to activate
  /* Serial.print(accX); Serial.print("\t");
   Serial.print(accY); Serial.print("\t");
   Serial.print(accZ); Serial.print("\t");

   Serial.print(gyroX); Serial.print("\t");
   Serial.print(gyroY); Serial.print("\t");
   Serial.print(gyroZ); Serial.print("\t");
  Serial.print("\t");*/
//#endif
  // Serial.print(roll); Serial.print("\t");
  // Serial.print(gyroXangle); Serial.print("\t");
  //Serial.print("compAngX=");Serial.print(compAngleX); Serial.print("\t\t\t\t\t\t\t\t");
  // Serial.print(kalAngleX); Serial.print("\t");

  // Serial.print("\t");

  // Serial.print(pitch); Serial.print("\t");
  // Serial.print(gyroYangle); Serial.print("\t");
  //Serial.print("compAngY=");Serial.print(compAngleY); Serial.print("\n");
/*  Serial.print(kalAngleY); Serial.print("\t");
 #if 0 // Set to 1 to print the temperature
   Serial.print("\t");
   double temperature = (double)tempRaw / 340.0 + 36.53;
   Serial.print(temperature); Serial.print("\t");
 #endif
   Serial.print("\r\n");*/

  Serial.print("Swtich:\t"); Serial.println(sw);
}


//------------------------------------------Gripper Feedback Function------------------------------------------------------
int PRESS_feedback (int PRESS_data){
    if(PRESS_data > PRESS_MAX){
      
        return 1;
    }
    else{
        return 0;
    }
}

//---------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------
/*---------------------------------------------------------------------------------------
                                      Set Up
-----------------------------------------------------------------------------------------*/


void setup() {
  //----------------------------------------Initializing Moving Average func-------------------------------
  for(int i=0; i<SAMPLING_NUM; i++){
    readings_flex[i]=0;
  } 
  for(int i=0; i<SAMPLING_NUM; i++){
  readings_yaw[i]=0;
  } 
  for(int i=0; i<SAMPLING_NUM; i++){
  readings_tilt[i]=0;
  } 
  //-----------------------------------------Motor-----------------------------------------------------------
  ser_yaw1.attach(SERVO_YAWING1_PIN);
  ser_yaw2.attach(SERVO_YAWING2_PIN);
  ser_tilt.attach(SERVO_TILTING_PIN);
  ser_pinion.attach(SERVO_PINION_PIN);
  ser_grip.attach(SERVO_GRIPPING_PIN);

  pinMode(VIBRATION_PIN, OUTPUT);

  //모터 각도 initialize
 /* ser_yaw1.write(yaw_tilt_IA_p);
  ser_yaw2.write(yaw_tilt_IA_p);
  ser_tilt.write(yaw_tilt_IA_p);
  ser_pinion.write(pinion_IA_p);
  ser_grip.write(grip_IA_p);*/

  delay(100); //motor initialize 후 delay
  //------------------------------------------Switch&Gripper-----------------------------------------------------------
  //sw = 0; //switch 추가 예정
  grip_stop = 0;


  Serial.begin(115200);
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { //If Address isn't correct
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize*/

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // atan2 outputs the value of -π to π (radians)
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}


void loop() {
  //-------------------------------------------------------IMU Sensor--------------------------------------------------
    while (i2cRead(0x3B, i2cData, 14));
    accX = ((i2cData[0] << 8) | i2cData[1]);
    accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (i2cData[6] << 8) | i2cData[7];
    gyroX = (i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    gyroZ = (i2cData[12] << 8) | i2cData[13];

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    IMU_filtering(accX, accY, accZ, tempRaw, gyroX, gyroY, gyroZ, dt);
     Serial.print("X=");Serial.print(compAngleX); Serial.print("\t");
     Serial.print("Y=");Serial.print(compAngleY); Serial.print("\t\n");
    delay(2);

//-------------------------------------------------------data--------------------------------------------------
    float yaw1_p_mv,yaw2_p_mv, tilt_p_mv;
  //모터 동작 values
    int yaw_p, tilt_p, flex_down_bd, flex_up_bd, flex_grip_bd, gripper_p;
  //시리얼로 전달받은 sensor data values
    int x_angle, y_angle, flex_pinion_up, flex_pinion_down, flex_gripper;
    int x_angle_bf, y_angle_bf, flex_pinion_up_bf, flex_pinion_down_bf, flex_gripper_bf; //buffer
    int x_angle_df, y_angle_df, flex_pinion_up_df, flex_pinion_down_df, flex_gripper_df; //difference = ()buff- ()=이전값-현재값
  //압전 센서 values
    int gripper_press;
    sw = TO_BIN(analogRead(SW_PIN));
  //-----------------------------sensor data 값 설정------------------------------------
    if(sw == 0){  //switch OFF
        x_angle = 0;          //imu_compAngleX
        y_angle = 0;          //imu_compAngleY
        flex_pinion_up = 0;   //flex sensor
        flex_pinion_down = 0; //flex sensor
        flex_gripper = 0;     //flex sensor + pressure sensor
        x_angle_bf = 0;          //imu_compAngleX
        y_angle_bf = 0;          //imu_compAngleY
        flex_pinion_up_bf = 0;   //flex sensor
        flex_pinion_down_bf = 0; //flex sensor
        flex_gripper_bf = 0;     //flex sensor + pressure sensor
        x_angle_df = 0;
        y_angle_df = 0;
        flex_pinion_up_df = 0;
        flex_pinion_down_df = 0;
        flex_gripper_df = 0;
    }
    else{       //switch ON
    //급격한 움직임 방지
        if(x_angle_df > IMU_MISTAKE || y_angle_df > IMU_MISTAKE || flex_pinion_up_df > FLEX_MISTAKE || flex_pinion_down_df > FLEX_MISTAKE || flex_gripper_df> FLEX_MISTAKE){}
        else{
          x_angle = compAngleX;
          y_angle = compAngleY;
          flex_pinion_up = analogRead(FLEX1_PIN);
          flex_pinion_down = analogRead(FLEX2_PIN);
          flex_gripper = analogRead(FLEX3_PIN); 
          //Motor angle saturation setting: 최대최소 범위 넘어가는 상황 대비 
          if (x_angle > 90){
            x_angle = 90;
          }
          else if (x_angle < -90){
            x_angle = -90;
          }
          if (y_angle > 90){
            y_angle = 90;
          }
          else if (x_angle < -90){
            y_angle = -90;
          }
        }
        x_angle_df = x_angle_bf-x_angle;
        y_angle_df = y_angle_bf-y_angle;
        

        x_angle_bf = x_angle;          //imu_compAngleX buffer
        y_angle_bf = y_angle;          //imu_compAngleY buffer
        
       
        
    }
//----------------------------------------------------------------------------
  /*------------------모터 동작-------------------------*/
    //----------------yawing-------------------------------------------------
    //x_angle(-90to90) --> 모터 앵글(초기각 - motion angle ~ 초기각 + motion angle)
    yaw_p = map(x_angle, -180, 180, yaw_tilt_IA_p - yaw_tilt_MA_p, yaw_tilt_IA_p + yaw_tilt_MA_p); 
    //Serial.println(yaw_p);
    yaw1_p_mv = MOVING_AVERAGE(yaw_p, 1); 
    yaw2_p_mv = MOVING_AVERAGE(yaw_p, 1); 
    ser_yaw1.write(yaw1_p_mv);
    ser_yaw2.write(yaw2_p_mv);
    Serial.print("yaw1_p_mv \t"); Serial.print(yaw1_p_mv); Serial.print("\tyaw2_p_mv \t"); Serial.print(yaw2_p_mv); Serial.print("\t\t");
    //----------------tilting-------------------------------------------------
    //y_angle(-90to90) --> 모터 앵글(초기각 - motion angle ~ 초기각 + motion angle)
    tilt_p = map(y_angle, -180, 180, yaw_tilt_IA_p - yaw_tilt_MA_p, yaw_tilt_IA_p + yaw_tilt_MA_p); 
  
    tilt_p_mv = MOVING_AVERAGE(tilt_p, 2);
    ser_tilt.write(tilt_p_mv);
    Serial.print("tilt_p_mv \t"); Serial.println(tilt_p_mv);

   
  
    //---------------- gripper -------------------------------------------------
    float flex_gripper_mv = MOVING_AVERAGE(flex_gripper, 3);
    
   // Serial.print("Flex Gripper="); Serial.println(flex_gripper);
    //Serial.print("Felx Gripper_MV="); Serial.println(flex_gripper_mv);

    
    //Feedback 넣어서 수정 필요.
    
    if(!PRESS_feedback (gripper_press)){  
      gripper_p = map(flex_gripper_mv, flex_grip_max, flex_grip_min, grip_IA_p, grip_FA_p);
      Serial.print("Gripper_press : ");Serial.println(gripper_press);
    }
    else {//다른 반응 설정 동작 X
    ser_grip.write(gripper_p);
    }
    
    //ser_grip.write(gripper_p);
    
    if(Index==SAMPLING_NUM - 1) {
      Index=0;
      cnt++;
    }
    else  Index++;
    
    /* --------------------------------VIBRATION-----------------------------------*/
    if(gripper_press>PRESS_MAX){
      vibration = map(gripper_press, 30,150,0,255);
      analogWrite(VIBRATION_PIN, vibration);
    }
  delay(10);
}




