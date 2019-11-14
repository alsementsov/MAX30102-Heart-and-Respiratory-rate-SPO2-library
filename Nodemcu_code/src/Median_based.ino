#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "max30102.h"
#include "my_spo2.h"

const byte oxiInt = 4; // pin connected to MAX30102 INT
uint32_t ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
uint32_t red_buffer[BUFFER_SIZE];  //red LED sensor data
int32_t IR [BUFFER_SIZE];
int32_t IR_med [BUFFER_SIZE];  


float spo2;
float spo2_old;
bool flag_error;
int j;
int k;
bool FastHR;
bool BadContact;
int32_t temp_kfdata;


//////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(oxiInt, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102
  Wire.begin();
  Serial.begin(115200);
  maxim_max30102_reset(); //resets the MAX30102
  delay(1000);
  maxim_max30102_init();  //initialize the MAX30102
  //Serial.println("START");
  flag_error = false;
  j=0;
  k=0;
  FastHR=false;
  BadContact=false;

}
////////////////////////////////////////////////////////////////////////////////////////////////////
//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
void loop() {
  int32_t i;

  BadContact = false;
  //------------ Reading Sensor ---------------------
  for(i=0;i<BUFFER_SIZE;i++)
  {
    while(digitalRead(oxiInt)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo((red_buffer+i), (ir_buffer+i));  //read from MAX30102 FIFO
    temp_kfdata =(int32_t)Kalman_simple_filter(ir_buffer[i]); 
    //Serial.print(temp_kfdata);
    temp_kfdata =(int32_t)ir_buffer[i]-temp_kfdata;
    IR_med[i]=Median_filter_small(temp_kfdata,FastHR);

    if (ir_buffer[i] < BAD_CONTACT_TH)
      BadContact=true;
    //Serial.print(temp_kfdata);
    //Serial.print(" ");
    //Serial.println(IR_med[i]);
  }
  //------------ Calculation ---------------------
  struct result res = MaxMin_search(IR_med,ir_buffer,red_buffer,BUFFER_SIZE);
  int HR = (res.HR*60*FS)/BUFFER_SIZE;

  Serial.println();Serial.print("@ HR=");Serial.print(HR);
  Serial.print(" / SpO2=");Serial.println(res.spo2);

  if (HR>95)
    FastHR=true;
  else
    FastHR=false;
  if (BadContact==true)
    Serial.println("Bad contact");
  //Serial.print(red_buffer[i]);
  //Serial.print("  ");
  //Serial.println(ir_buffer[i]);
}

