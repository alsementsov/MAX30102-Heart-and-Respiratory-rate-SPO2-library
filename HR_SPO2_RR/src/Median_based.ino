#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "max30102.h"
#include "my_spo2.h"

const byte oxiInt = 4; // pin connected to MAX30102 INT
uint32_t ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
uint32_t red_buffer[BUFFER_SIZE];  //red LED sensor data
bool BadContact;
int32_t temp_kfdata;
int32_t IR_norm;
struct result res;
uint32_t Nsample;
uint32_t Nbeats[HR_FIFOSIZE];
uint32_t* ptr = Nbeats;
uint32_t HR;
float spo2;
uint8_t cnt_after_badcontact;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(oxiInt, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102
  Wire.begin();
  Serial.begin(115200);
  maxim_max30102_reset(); //resets the MAX30102
  delay(1000);
  maxim_max30102_init();  //initialize the MAX30102
  BadContact=false;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
void loop() {
   //------------ Reading Sensor ---------------------
  for(uint16_t i=0;i<BUFFER_SIZE;i++)
  {
    while(digitalRead(oxiInt)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo((red_buffer+i), (ir_buffer+i));  //read from MAX30102 FIFO
    ////////////////////////////////For STATION///////////////////////////////////// 
    Nsample++;
    temp_kfdata =(int32_t)Kalman_simple_filter(ir_buffer[i],KF_Q,KF_R); 
    temp_kfdata =(int32_t)ir_buffer[i]-temp_kfdata;
    IR_norm=Median_filter_small(temp_kfdata,7);

    if (ir_buffer[i] < BAD_CONTACT_TH){
      Serial.println("Bad contact");
      ptr=&Nbeats[0];
      cnt_after_badcontact=0;}
    else {
      // Find MAX MIN
      res = MaxMin_search_stream(IR_norm,ir_buffer[i],red_buffer[i]);
      if (res.NewBeat){
        if (cnt_after_badcontact<20){
          HR = (cnt_after_badcontact*60*FS)/(Nsample-Nbeats[0]);
          cnt_after_badcontact++;
          }
        // HR avearge for 20 beats
        else {
          HR = (HR_FIFOSIZE*60*FS)/(Nsample-*(ptr));}
        Serial.print(Nsample);Serial.print(": HR=");Serial.print(HR);
        // FIFO for HR time stamp
        *(ptr)=Nsample;
        if (ptr==&Nbeats[HR_FIFOSIZE-1]){
          ptr=&Nbeats[0];}
        else
          ptr++;
        // Current spo2 from sample
        //Serial.print(" / SpO2_cur=");Serial.print(res.spo2);
        //Average spo2 for visualization 
        spo2 = Median_filter_spo2(res.spo2);
        Serial.print(" / SpO2=");Serial.println(spo2);
        if (res.error>0){
          Serial.print("Error=");Serial.println(res.error);}
      }
      if (res.error==4)
        Serial.println("Alarm! No heartbeats!");
    }
  }
}



