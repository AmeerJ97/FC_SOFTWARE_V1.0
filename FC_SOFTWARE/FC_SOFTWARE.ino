#include <Wire.h>
#include "nRF24L01.h"
#include "printf.h"
#include "RF24.h"
#include "RF24_config.h"
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <SPI.h>
#include <Servo.h>
/*------------------- General Variables-------------------*/
float rad_2_deg = 57.29577;             //Radians to degrees constant
uint32_t loopTimer;                     //loopTimer to ensure each main loop iteration is 4000microseconds                   
/*------------------- LED Variables-------------------*/
int bluePin = A0;                       //Blue LED pin
int redPin = A1;                        //Red LED pin
int yellowPin = A2;                     //Yellow LED pin
int greenPin = A3;                      //Green LED pin

/*------------------- GY-521 IMU Variables-------------------*/
const int imu_addr = 0x68;              //IMU Address for I2C communication
boolean gyro_Flag = false;              //Gyroscope flags to track startup
boolean gyro_Init = false;               
int calibrate;                          //Calibration variable on startup
//Gyro and accelerometer (X,Y,Z) arrays and temperature variable(s) 
float g_raw[4],a_raw[4], g_angle[4], a_angle[4],g_err[4], temperature; 
float alpha_gyro = 0.9;                 //complementary filter
float alpha_acc = 0.1;

/*------------------- nRF24 Transceiver Variables-------------------*/
typedef struct{                         //Structure Data type to hold rx PS3 state variables
  int Lx;                               //Left Analog stick X data
  int Ly;                               //Left Analog stick Y data
  int Rx;                               //Right Analog stick X data
  int Ry;                               //Right Analog stick Y data
  boolean xButton;                      //Cross button data
  boolean oButton;                      //Circle button data
  boolean tButton;                      //Triangle button Data
  boolean sButton;                      //Square button data
  boolean Lt;                           //Left trigger L1
  boolean Lb;                           //Left bumper L2
  boolean Rt;                           //Right trigger R1
  boolean Rb;                           //Right bumper R2
}controllerStruct;
controllerStruct controllerData;

const int pinCE = 7;                    
const int pinCSN = 8;
const int pinInt = 2;                   //Pin used for nRF24 interrupts
uint64_t channel_addr = 0xB00B1E5000LL; //Channel address
unsigned long counter;                  //Counter to track failed packets

RF24 rfRadio(pinCE,pinCSN);             //Declaring nRF24 Object

/*------------------- PID Variables-------------------*/
//P Variables
float KP_roll = 2;
float KP_pitch = 2;
float KP_yaw = 0.5;
//I Variables
float KI_roll = 0;
float KI_pitch = 0;
float KI_yaw = 0;
//D Variables
float KD_roll = 0;
float KD_pitch = 0;
float KD_yaw = 0;


float setVariables[4] = {0,0,0,0};       //roll set, pitch set, yaw set, elevation set
float fixVariables[3] = {0,0,0};         //roll fix, pitch fix, yaw pitch 
float inputVariables[3] = {0,0,0};       //roll input, pitch input, yaw input
float integralVariables[3] = {0,0,0};    //roll integral, pitch integral, yaw integral
float outputVariables[6] = {0,0,0,0,0,0};//roll output, pitch output, yaw output, last roll output, last pitch output, last yaw output
float lastDvariables[3] = {0,0,0};       //roll last D, pitch last D, yaw last D

/*------------------- ESC Variables-------------------*/
#define SPEED_MIN (1000)
#define SPEED_MAX (2000)
int motorSpeed = 0;               //Base ESC speed for all Motors
boolean motorFlag = true;         //Turn on motors on first iteration
boolean elevate = false;          //Boolean to control motorSpeed variable
boolean armMotors = false;        //Boolean to initiate arm motors method
boolean stopMotors = false;       //Boolean to stop the quadcopter
float mESC1, mESC2, mESC3, mESC4, mOffset; //ESC motor variables

Servo oESC1,oESC2,oESC3,oESC4;    //Declaring ESC Motors as objects


void setup() {
  Serial.begin(57600);
  led_Init(); 
  imu_Init();
  gyro_calibrate();
   
  printf_begin();
  radio_Init();  
  
  oESC1.attach(6);
  oESC2.attach(5);
  oESC3.attach(4);
  oESC4.attach(3);
  
  esc_Init();                      //Arm ESCs
    
  
  loopTimer = micros();
}

uint32_t configTimer =  millis();  

void loop() {
  
   if (armMotors == true && motorFlag == true ){    //Re-arming ESC if requested
    esc_Init();
    armMotors = false;
   }
  
  Serial.print("Lx: ");Serial.print(controllerData.Lx);
  Serial.print(" Ly: ");Serial.print(controllerData.Ly);
  Serial.print(" Rx: ");Serial.print(controllerData.Rx);
  Serial.print(" Ry: ");Serial.print(controllerData.Ry);
  Serial.println(" ");
  
  //Checking if Radio is still working, if not, reinitialize radio  
  if(rfRadio.failureDetected) {          
       Serial.println("Radio failure, restarting...");
       radio_Init();
   }

    //Every second turn off red led (rx) and check radio details
    if(millis() - configTimer > 1000){
      configTimer = millis();
      analogWrite(redPin,0);

      integralVariables[0] = 10;
      integralVariables[1] = 10;
      integralVariables[2] = 25;
      
        
      if (counter >= 30){
      Serial.println("30 failed packets tracked");
      counter = 0;                          //resetting packet counter
       }
      
      if(rfRadio.getDataRate() != RF24_1MBPS ){
      Serial.print("Radio configuration error detected");
    }

    }
  read_Imu();

  inputVariables[0] = (inputVariables[0] * 0.7) + ((g_raw[1] / 32.8) * 0.3);
  inputVariables[1] = (inputVariables[1] * 0.7) + ((g_raw[2] / 32.8) * 0.3);
  inputVariables[2] = (inputVariables[2] * 0.7) + ((g_raw[3] / 32.8) * 0.3);
  
  the_Gyroscope();
  pid_Init();
  pid_Controller();
  
  while (micros() - loopTimer < 4000);
  loopTimer = micros();
  e_Driver(); 
}
  /* IMU Initialization function */
void imu_Init(){
  if (gyro_Init == false){
    Wire.begin();                         //Begin the I2C link as a master
    
    //Configuring IMU Power
    Wire.beginTransmission(imu_addr);     //Start the I2C communication with the IMU
    Wire.write(0x6B);                     //Acessing Power Management register 1, 107 decimal = 6B Hex
    Wire.write(0x00);                     //Set the register as 0 to wake the IMU from sleep mode
    Wire.endTransmission();               //End transmission to IMU

    //Configuring IMU Gyroscope
    Wire.beginTransmission(imu_addr);     //Start the I2C communication with the IMU
    Wire.write(0x1B);                     //Acessing Gyroscope Configuration Register, 27 decimal = 1B
    Wire.write(0x10);                     //Set the register as 00010000 indicating readings in +/- 1000 deg/s
    Wire.endTransmission();               //End transmission to IMU

    //Configuring IMU Accelerometer
    Wire.beginTransmission(imu_addr);     //Start the I2C communication with the IMU
    Wire.write(0x1C);                     //Accessing Accelerometer Configuration Register, 28 decimal = 1C
    Wire.write(0x10);                     //Set the register as 00010000 indicating readings in +/- 8gs
    Wire.endTransmission();               //End transmission to IMU
    
    Serial.println("IMU 6050 Initialized");
    gyro_Init = true;
  }
}
  /* Radio Initialization function */
void radio_Init(){
  rfRadio.begin();                        //Initialize nRF24
  rfRadio.setPALevel(RF24_PA_LOW);        //Setting power level to low, microstrip antenna is rx only
  rfRadio.setDataRate(RF24_1MBPS);        //Setting link speed to 1MBps
  if(rfRadio.isChipConnected() == true){
    Serial.println("nRF24 Chip Connected.");
  }else{
    Serial.println("No nRF24 Chip Found.");
  }
  rfRadio.setAutoAck(1);                  //Enable auto ack
  rfRadio.enableAckPayload();             //Enable ack payload
  //rfRadio.setRetries(0, 2);             //Set Retries to 2 to avoid delay
  // rfRadio.setCRCLength(RF24_CRC_16);
  rfRadio.maskIRQ(1,1,0);                 //Mask all INT Triggers except for Rx
  rfRadio.openReadingPipe(1,channel_addr);//Open channel 1 to recieve struct type data
  rfRadio.startListening();
  pinMode(pinInt, INPUT);
  rfRadio.printDetails();
  attachInterrupt(digitalPinToInterrupt(pinInt), rf_Interrupt, CHANGE); //Create interrupt for pin 9 on falling edge
}
  /* Radio Interrupt Function */
void rf_Interrupt(){
  analogWrite(redPin,255);                //Set red LED Pin to high to indicate data rx 
  analogWrite(greenPin,0);
  bool tx,fail,rx;                        //Local Variables 
  rfRadio.whatHappened(tx,fail,rx);       
 //Read data on rx, increment packet tracker on failure
 if (rx){
   rfRadio.read(&controllerData,sizeof(controllerData));
   }
 if (tx || fail){
  counter++;
  Serial.println(tx ? F(":OK") : F(":Fail"));
 }
}

  /* IMU Reading function */
void read_Imu(){
  //Registers 59 to 64, 65 to 66, 67 to 72 are the accelerometer, 
  //temperature, and gyroscope output registers respectively
  Wire.beginTransmission(imu_addr);       //Start the I2C communication with the IMU
  Wire.write(0x3B);                       //Acessing Acc out register, 59 = 0x3B
  Wire.endTransmission();                 //End transmission to IMU
  Wire.requestFrom(imu_addr, 14);         //Request 14 bytes from IMU starting at Acc out register
  while(Wire.available() < 14);           //Waiting for the 14 bytes to be recieved

  a_raw[1] = Wire.read()<<8 | Wire.read();  //Reading 8 bytes from high and low Acc X output Register 
  a_raw[2] = Wire.read()<<8 | Wire.read();  //Reading 8 bytes from high and low Acc Y output Register 
  a_raw[3] = Wire.read()<<8 | Wire.read();  //Reading 8 bytes from high and low Acc Z output Register 
  temperature = Wire.read()<<8 | Wire.read(); //Reading 8 bytes from high and low temperature output register
  g_raw[1] = Wire.read()<<8 | Wire.read();  //Reading 8 bytes from high and low Gyro X output Register 
  g_raw[2] = Wire.read()<<8 | Wire.read();  //Reading 8 bytes from high and low Gyro Y output Register 
  g_raw[3] = Wire.read()<<8 | Wire.read();  //Reading 8 bytes from high and low Gyro Z output Register 
  
  Wire.endTransmission();
  //Directional Correction
  //a_raw[2] *= -1;
  //g_raw[2] *= -2;
  
  //Subtracting calibrated values if calibration already preformed
  if (calibrate == 3000){
    g_raw[1] -= g_err[1];
    g_raw[2] -= g_err[2];
    g_raw[3] -= g_err[3];
  }
}
  /* Gyroscope calibration function */
void gyro_calibrate(){
  Serial.print(" Starting Calibration......."); 
  
  //3000 samples to calibrate
  for (int calibration = 0; calibration < 3000 ; calibration++){     
    read_Imu();
    g_err[1] += g_raw[1];
    g_err[2] += g_raw[2];
    g_err[3] += g_raw[3];    
  }
  
  calibrate = 3000;                       //Flag for read_Imu() calibration initialization
  Serial.println(" Calibration complete!");
  
  g_err[1] /= 3000;
  g_err[2] /= 3000;
  g_err[3] /= 3000;

  Serial.println(" The Calibrated values are:  ");
  Serial.print(" Gx: ");Serial.print(g_err[1]);
  Serial.print(" Gy: "); Serial.print(g_err[2]);
  Serial.print(" Gz: "); Serial.print(g_err[3]);
  Serial.println(" ");
}
  /* Gyroscope math function to convert raw deg/s input to current angle */
void the_Gyroscope(){
  //Discrete summation multiplied by dt equivalent to continous integration
  //where dt = 1  / ( f * 32.8) where f is the frequency of the microcontroller and 32.8 is the selected sensitivity upon gyroscope configuration
  g_angle[1] += g_raw[1] * (1/(32.8 * 250));     //Roll Angle Gx
  g_angle[2] += g_raw[2] * (1/(32.8 * 250));     //Pitch Angle  Gy
  g_angle[3] += g_raw[3] * (1/(32.8 * 250));     //Yaw Angle  Gz

  g_angle[2] -= g_angle[1] * sin(g_raw[3] * (1/(32.8 * 250)) * (1 / 57.29577)); //Pitch
  g_angle[1] += g_angle[2] * sin(g_raw[3] * (1/(32.8 * 250)) * (1 / 57.29577)); //Roll

  //Calculating accelerometer vector to determine direction of acceleration
  float acc_vector = sqrt((a_raw[1] * a_raw[1]) + (a_raw[2] * a_raw[2]) + (a_raw[3] * a_raw[3]));

  if (abs(a_raw[2]) < acc_vector){
    a_angle[2] = asin((float) a_raw[2] / acc_vector) * rad_2_deg;
  }
   if (abs(a_raw[1]) < acc_vector){
    a_angle[1] = asin((float) a_raw[1] / acc_vector) * rad_2_deg;
  }

  //Hardcoded second calibration for accelerometer
  a_angle[1] += 2;                   //Ax
  a_angle[2] += 0;                  //Ay
  a_angle[3] += 0;                   //Az

  //On first run, set gyro X&Y angle values equal to acc X&Y angle values for initial stability
  //If not, implement complementary filter 
  if (gyro_Flag == false){
    g_angle[1] = a_angle[1];
    g_angle[2] = a_angle[2];
  }else {
    g_angle[1] = g_angle[1] * alpha_gyro + a_angle[1] * alpha_acc;
    g_angle[1] = g_angle[2] * alpha_gyro + a_angle[2] * alpha_acc;
  } 
//   Serial.print("Gx_angle: ");Serial.print(g_angle[1]);
//  Serial.print("  Gy_angle: ");Serial.print(g_angle[2]);
//  Serial.print("  Gz_raw: ");Serial.print(g_raw[3]);
//  Serial.print("  Ax_angle: ");Serial.print(a_angle[1]);
//  Serial.print("  Ay_angle: ");Serial.print(a_angle[2]);
//  Serial.print("  Az_raw: ");Serial.print(a_raw[3]);
//  Serial.println(" ");
}

  /* PID initialization function to input controller data */
void pid_Init(){
  
  /* Motor Startup condition via L2 and R2, R1 and L1 stop motors */
  if (controllerData.Lt == true && controllerData.Rt == true){
    motorFlag = true;
    stopMotors = false;
    armMotors = true;
    Serial.println("########### MOTOR STARTUP ############");
    controllerData.Lt = false;
    controllerData.Rt = false;
    
  } else if (controllerData.Lb == true && controllerData.Rb == true){
    motorFlag = false;
    Serial.println("########### MOTOR SHUTDOWN ############");
    controllerData.Lb = false;
    controllerData.Rb = false;
    armMotors = false;
    elevate = false;
    stopMotors = true;
   }
   
  //State control of quadcopter via PS3 buttons
  if (motorFlag == true && controllerData.xButton == true) armMotors = true;
  if (motorFlag == true && controllerData.oButton == true) elevate = true;
  if (motorFlag == true && controllerData.tButton == true) elevate = false;
  if (motorFlag == true && controllerData.sButton == true) stopMotors = true;
  if (motorFlag == true && controllerData.sButton == false) stopMotors = false;



  /* Pid setpoint controlled by PS3 controller */
  //Requires a deadband of 90 due to PS3 input fluctuations
  //Thus, maximum set roll input is 164.0 deg/s
  //Roll Set calculations
  setVariables[0] = 0;
  if (controllerData.Rx > 1590)setVariables[0] = (controllerData.Rx - 1590) ;
  else if (controllerData.Rx < 1410)setVariables[0] = (controllerData.Rx - 1410);
  setVariables[0] -= g_angle[1] * 15;          //Gyroscope roll correction
  setVariables[0] /= 3;


  //Pitch Set calculations
  setVariables[1] = 0;
  if (controllerData.Ry > 1590)setVariables[1] = controllerData.Ry - 1590;
  else if (controllerData.Ry < 1410)setVariables[1] =  controllerData.Ry - 1410;
  setVariables[1] -= g_angle[2]* 15;
  setVariables[1] /= 3;

  //Yaw Set calculations
  setVariables[2] = 0;
  if(controllerData.Lx > 1590)setVariables[2] = controllerData.Lx - 1590;
  else if(controllerData.Lx < 1410)setVariables[2] = controllerData.Lx - 1410;
  setVariables[2] /= 5;
  
  //Elevation set calculations
  setVariables[3] = 0;
  if(controllerData.Ly > 1590)setVariables[3] = controllerData.Ly - 1590;
  else if(controllerData.Ly < 1410)setVariables[3] = controllerData.Ly - 1410;
  setVariables[3] /= 2;
    Serial.print(" Set Roll: ");Serial.print(setVariables[0]);
  Serial.print(" Set Pitch: ");Serial.print(setVariables[1]);
  Serial.print(" Set Yaw: ");Serial.print(setVariables[2]);
  Serial.print(" Set Thrust: ");Serial.print(setVariables[3]);
  Serial.print(" Input roll: "); Serial.println(integralVariables[0]);
  Serial.println(" ");
}

  /* PID Controller Function */ //REMOVED I CONTROLLER
void pid_Controller(){
  //Roll Calculations
  fixVariables[0] = inputVariables[0] - setVariables[0];                //Calculating error
  integralVariables[0] += KI_roll * fixVariables[0];                    //Integral Controller
  if (integralVariables[0] > 400) integralVariables[0] = 400;           //Upper limit on integral controller
  else if (integralVariables[0] <= -400) integralVariables[0] = -400;  //Lower limit on integral controller

  outputVariables[0] = KP_roll * fixVariables[0] +integralVariables[0] + KD_roll * (fixVariables[0] - lastDvariables[0]);
  if (outputVariables[0] > 400) outputVariables[0] = 400;
  else if (outputVariables[0] < -400) outputVariables[0] = -400;
  lastDvariables[0] = fixVariables[0];

  //Pitch Calculations
  fixVariables[1] = inputVariables[1] - setVariables[1];
  integralVariables[1] += KI_pitch * fixVariables[1];
  if (integralVariables[1] > 400) integralVariables[1] = 400;
  else if (integralVariables[1] <= -400) integralVariables[1] = -400;

  outputVariables[1] = KP_pitch * fixVariables[1]  + KD_pitch * (fixVariables[1] - lastDvariables[1]);
  if (outputVariables[1] > 400) outputVariables[1] = 400;
  else if (outputVariables[1] < -400) outputVariables[1] = -400;
  lastDvariables[1] = fixVariables[1];

  //Yaw Calculations
  fixVariables[2] = inputVariables[2] - setVariables[2];
  integralVariables[2] += KI_yaw * fixVariables[2];
  if (integralVariables[2] > 400) integralVariables[2] = 400;
  else if (integralVariables[2] <= -400) integralVariables[2] = -400;

  outputVariables[2] = KP_yaw * fixVariables[2]  + KD_yaw * (fixVariables[2] - lastDvariables[2]);
  if (outputVariables[2] > 400) outputVariables[2] = 400;
  else if (outputVariables[2] < -400) outputVariables[2] = -400;
  lastDvariables[2] = fixVariables[2];

  Serial.print("Roll: ");Serial.print(outputVariables[0]);
  Serial.print("  Pitch: ");Serial.print(outputVariables[1]);
  Serial.print("  Yaw: ");Serial.print(outputVariables[2]);
  Serial.print("  Thrust: ");Serial.print(setVariables[3]);
  Serial.print("  Integral: ");Serial.print(integralVariables[0]);
  Serial.print("  Integral 2: ");Serial.print(integralVariables[1]);
  Serial.println(" ");

}

  /* ESC Initialization / Arming Function */
void esc_Init(){
  analogWrite(yellowPin, 255);                //Blue LED to indicate arming sequence
  armMotors = false;                          //Turn off arming flag
  oESC1.writeMicroseconds(2000);
  oESC2.writeMicroseconds(2000);
  oESC3.writeMicroseconds(2000);
  oESC4.writeMicroseconds(2000);
  delay(5000); 
  oESC1.writeMicroseconds(1000);
  oESC2.writeMicroseconds(1000);
  oESC3.writeMicroseconds(1000);
  oESC4.writeMicroseconds(1000);
  delay(5000);
  analogWrite(yellowPin, 0);              
}

  /* PID Controller Function */
void e_Driver(){
  analogWrite(greenPin,255);                  //Green Pin to indicate motors signal
  analogWrite(redPin,0);
  if (elevate == true){
    motorSpeed = 1450;
    mOffset = 100;
  }
  else if (elevate == false){
    motorSpeed = 1150;
    mOffset = 50;
  }

  //Deadband to counteract vibrations and other sources of error
  if(outputVariables[3] - outputVariables[0] > 20 || outputVariables[3] - outputVariables[0] < -20) outputVariables[0] = outputVariables[0];
  else if (outputVariables[3] - outputVariables[0] < 20 || outputVariables[3] - outputVariables[0] > -20)outputVariables[0] = outputVariables[3];

  if(outputVariables[4] - outputVariables[1]  > 20 || outputVariables[4] - outputVariables[1]  < -20)  outputVariables[1] = outputVariables[1];
  else if (outputVariables[4] - outputVariables[1]  < 20 || outputVariables[4] - outputVariables[1]  > - 20) outputVariables[1] = outputVariables[4];

  if(outputVariables[5] - outputVariables[2] > 10 || outputVariables[5] - outputVariables[2] < -10) outputVariables[2] = outputVariables[2];
  else if (outputVariables[5] - outputVariables[2] < 10 || outputVariables[5] - outputVariables[2] > -10)  outputVariables[2] = outputVariables[5];

//  if(pid.lastelevSet - pid.elevSet > 15  || pid.lastelevSet - pid.elevSet < -15) pid.elevSet = pid.elevSet;
//  else if (pid.lastelevSet - pid.elevSet < 15 || pid.lastelevSet - pid.elevSet > -15) pid.elevSet = pid.lastelevSet;

  //Updating last output variables
  outputVariables[3] = outputVariables[0];
  outputVariables[4] = outputVariables[1];
  outputVariables[5] = outputVariables[2];
  
  if(motorFlag == false && stopMotors == true){
   mESC1 = 0;
   mESC2 = 0; 
   mESC3 = 0;
   mESC4 = 0;
   
  } else if(motorFlag == true && stopMotors == false){
   mESC1 = motorSpeed + setVariables[3] + outputVariables[0] + outputVariables[1] + outputVariables[2];
   mESC2 = motorSpeed + setVariables[3] + outputVariables[0] - outputVariables[1] - outputVariables[2];
   mESC3 = motorSpeed + setVariables[3] - outputVariables[0] - outputVariables[1] + outputVariables[2];
   mESC4 = motorSpeed + setVariables[3] - outputVariables[0] + outputVariables[1] - outputVariables[2];
  if(mESC1 < 1050) mESC1 = 1050; if(mESC1 > 1750) mESC1 = 1750;
  if(mESC2 < 1050) mESC2 = 1050; if(mESC2 > 1750) mESC2 = 1750;
  if(mESC3 < 1050) mESC3 = 1050; if(mESC3 > 1750) mESC3 = 1750;
  if(mESC4 < 1050) mESC4 = 1050; if(mESC4 > 1750) mESC4 = 1750;  
  }

//  pid.lastoutRoll = pid.outRoll;
//  pid.lastoutPitch = pid.outPitch;
//  pid.lastoutYaw = pid.outYaw;
//  pid.lastelevSet = pid.elevSet;
//  
  
  oESC1.writeMicroseconds(mESC1);
  oESC2.writeMicroseconds(mESC2);
  oESC3.writeMicroseconds(mESC3);
  oESC4.writeMicroseconds(mESC4);
  
    Serial.print(" ESC1: ");Serial.print(mESC1);
  Serial.print(" ESC2: ");Serial.print(mESC2);
  Serial.print(" ESC3: ");Serial.print(mESC3);
  Serial.print(" ESC4: ");Serial.println(mESC4);
  
    
  
  analogWrite(greenPin,0);
}

  /* LED Initialization Function */
void led_Init(){
  //Initialize LED pins as outputs
  pinMode(bluePin, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
}
