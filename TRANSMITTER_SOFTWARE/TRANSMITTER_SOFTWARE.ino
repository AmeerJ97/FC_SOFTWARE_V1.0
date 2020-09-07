#include <PS3USB.h>
#include <Arduino.h>
#include <Wire.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "RF24_config.h"
#include <stdio.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
/*------------------- General Variables-------------------*/
typedef struct{
  int Lx;
  int Ly;
  int Rx;
  int Ry;
  boolean xButton;
  boolean oButton;
  boolean tButton;
  boolean sButton;
  boolean Lt;
  boolean Lb;
  boolean Rt;
  boolean Rb;
}controllerStruct;
controllerStruct controllerData;
const int pinCE = 7;        //Pin used to set nRF to standby 0 or active 1
const int pinCSN = 8;       //Pin used to configure SPI communication (Tx/Rx)
uint64_t channel_addr = 0xB00B1E5000LL ;   //5 Channel addresses

RF24 rfRadio(pinCE, pinCSN);            //RF24 Object
USB Usb;
PS3USB PS3(&Usb);

 /* Radio Initialization function */
void radio_Init(){
  rfRadio.begin();                         //Start RF24 object
 rfRadio.setAutoAck(1);                     // Ensure autoACK is enabled
 rfRadio.enableAckPayload();
 rfRadio.setPALevel(RF24_PA_HIGH);         //Max range
 rfRadio.setDataRate(RF24_1MBPS); 
 rfRadio.setCRCLength(RF24_CRC_16);
 if(rfRadio.isChipConnected() == true){
    Serial.println(" ");
    Serial.println("nRF24 Chip Connected.");
  }else{
    Serial.println("No nRF24 Chip Found.");
  }          
 rfRadio.openWritingPipe(channel_addr);     //Open communication channel
 rfRadio.stopListening(); 
 rfRadio.printDetails();
}
 /* USB Initialization function */
void usb_Init(){
    #if !defined(_MIPSEL_)
  while(!Serial); //wait for serial port to connect 
  #endif
  if (Usb.Init() == -1){
    Serial.print(F("\r\nOSC did not start"));
    while(1); //halot
    }
    Serial.print("\r\nPS3 USB Library Started");
  
}


void setup() {
  Serial.begin(57600);
  usb_Init();
  printf_begin();
  radio_Init();
}
uint32_t configTimer = millis();
void loop() {
  read_PS3();

  
 //Resetting all buttons pressed every second
 if(millis()- configTimer > 1000){
  configTimer = millis();
  controllerData.xButton = false;
  controllerData.oButton = false;
  controllerData.tButton = false;
  controllerData.sButton = false;
  controllerData.Lt = controllerData.Lb = controllerData.Rt = controllerData.Rb = false;
 }
 //Writing data to channel or pipe,
 if(!rfRadio.writeFast(&controllerData,sizeof(controllerData))){
  //Do nothing
 }
 //If packet transmission failed
 if(!rfRadio.txStandBy(10)){                    //Waits 0.01 second to check transmission status. Flushed FIFO Registers (contain data) on fail.
   Serial.println("####TRANSMISSION FAILED####"); //Prints to serial monitor on failure
 }
 
// if(rfRadio.failureDetected){
//  rfRadio.begin();
//  rfRadio.write(&controllerData,sizeof(controllerData));
//  rfRadio.failureDetected = 0;
//  radio_Init();
//  Serial.println("Resetting nRF24, Failure Detected");
// }
 
  
}
void read_PS3(){
  Usb.Task();
  if (PS3.PS3Connected || PS3.PS3NavigationConnected){        
    controllerData.Lx = PS3.getAnalogHat(LeftHatX);
    controllerData.Ly = PS3.getAnalogHat(LeftHatY);
    controllerData.Rx = PS3.getAnalogHat(RightHatX);
    controllerData.Ry = PS3.getAnalogHat(RightHatY);
    if(PS3.getButtonClick(CIRCLE))controllerData.oButton = true;
    if(PS3.getButtonClick(CROSS))controllerData.xButton = true;
    if(PS3.getButtonClick(SQUARE))controllerData.sButton = true;
    if(PS3.getButtonClick(TRIANGLE))controllerData.tButton = true;
    if(PS3.getButtonClick(L1))controllerData.Lb = true;
    if(PS3.getButtonClick(L2))controllerData.Lt = true;
    if(PS3.getButtonClick(R1))controllerData.Rb = true;
    if(PS3.getButtonClick(R2))controllerData.Rt = true; 
    
    //Mapping and shifting analog data such that 1500 is the center point
    controllerData.Lx = map(controllerData.Lx,0,255,1000,2000) ;
    controllerData.Rx = map(controllerData.Rx,0,255,1000,2000) ;
    controllerData.Ry = map(controllerData.Ry,255,0,1000,2000) ;
    controllerData.Ly = map(controllerData.Ly,255,0,1000,2000) ;
  
    
    //Deadband to avoid PS3 joystick fluctuations
    if((controllerData.Lx < 1750) && (controllerData.Lx > 1250)) controllerData.Lx = 1500;
    if((controllerData.Ly < 1600) && (controllerData.Ly > 1400)) controllerData.Ly = 1500;
    if((controllerData.Rx < 1600) && (controllerData.Rx > 1400)) controllerData.Rx = 1500;
    if((controllerData.Ry < 1600) && (controllerData.Ry > 1400)) controllerData.Ry = 1500;

    Serial.print("Lx: ");Serial.print(controllerData.Lx);
    Serial.print(" Ly: ");Serial.print(controllerData.Ly);
    Serial.print(" Rx: ");Serial.print(controllerData.Rx);
    Serial.print(" Ry: ");Serial.print(controllerData.Ry);
    Serial.print(" S: ");Serial.print(controllerData.sButton);
    Serial.print(" T: ");Serial.print(controllerData.tButton);
    Serial.print(" O: ");Serial.print(controllerData.oButton);
    Serial.print(" X: ");Serial.print(controllerData.xButton);
    Serial.println(" "); 
  }
}
