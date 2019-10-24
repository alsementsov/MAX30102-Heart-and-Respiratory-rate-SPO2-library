#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "max30102.h"
#include "my_spo2.h"
#define DELAY_SIZE 44//((MEDIAN_FILTER_SIZE-1)/2)+4
#define ISTOP (BUFFER_SIZE-DELAY_SIZE)

const byte oxiInt = 4; // pin connected to MAX30102 INT
uint32_t ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
uint32_t red_buffer[BUFFER_SIZE];  //red LED sensor data
int32_t IR [BUFFER_SIZE];
int32_t IR_med [BUFFER_SIZE];  
int32_t IR_read[BUFFER_SIZE];
int32_t IR_temp[DELAY_SIZE];
int32_t Red_read[BUFFER_SIZE];
int32_t Red_temp[DELAY_SIZE];
const int sizeM = sizeof(IR_temp);

float spo2;
float spo2_old;
bool flag_error;
int j;
int k;
bool FastHR;
bool BadContact;

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
  float n_spo2,ratio,correl;  //SPO2 value
  int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate; //heart rate value
  int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
  int32_t i;
  char hr_str[10];
  //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
  //read BUFFER_SIZE samples, and determine the signal range
  BadContact = false;
  for(i=0;i<BUFFER_SIZE;i++)
  {
    while(digitalRead(oxiInt)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo((red_buffer+i), (ir_buffer+i));  //read from MAX30102 FIFO
    j = (i==(MEDIAN_FILTER_SIZE-1)/2)?0:j+1;
    k = (i==4)?0:k+1;
    IR[i] =(int32_t)ir_buffer[j]-(int32_t)Median_filter(ir_buffer[i]);
    IR_med[i]=Median_filter_small(IR[i],FastHR);
    if (i<ISTOP){
      IR_read[i+DELAY_SIZE]=ir_buffer[i];
      Red_read[i+DELAY_SIZE]=red_buffer[i];
    }
    else{
      IR_temp[i-ISTOP]=ir_buffer[i];
      Red_temp[i-ISTOP]=red_buffer[i];
    }
    if (ir_buffer[i] < BAD_CONTACT_TH)
      BadContact=true;
    //Serial.print("[");Serial.print(i); Serial.print("]= ");
    //Serial.print(ir_buffer[i]);
    //Serial.print(" | ");
    //Serial.println(red_buffer[i]);
  }
  struct result res = MaxMin_search(IR_med,IR_read,Red_read,BUFFER_SIZE);
  int HR = (res.HR*60*FS)/BUFFER_SIZE;

  Serial.println();Serial.print("@ HR=");Serial.print(HR);
  Serial.print(" / SpO2=");Serial.println(res.spo2);
  for (int i=0;i<DELAY_SIZE;i++){
    IR_read[i]=IR_temp[i];
    Red_read[i]=Red_temp[i];
  }
  if (HR>95)
    FastHR=true;
  else
    FastHR=false;
  if (BadContact==true)
    Serial.println("Bad contact");
    
}


