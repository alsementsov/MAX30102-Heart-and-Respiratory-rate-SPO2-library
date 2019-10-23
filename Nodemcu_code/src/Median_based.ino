#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "algorithm_by_RF.h"
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
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(oxiInt, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102
  Wire.begin();
  Serial.begin(115200);
  maxim_max30102_reset(); //resets the MAX30102
  delay(1000);
  maxim_max30102_init();  //initialize the MAX30102
  Serial.println("START");
  flag_error = false;
  j=0;
  k=0;
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
 

  for(i=0;i<BUFFER_SIZE;i++)
  {
    while(digitalRead(oxiInt)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo((red_buffer+i), (ir_buffer+i));  //read from MAX30102 FIFO
    j = (i==(MEDIAN_FILTER_SIZE-1)/2)?0:j+1;
    k = (i==4)?0:k+1;
 
    IR[i] =(int32_t)ir_buffer[j]-(int32_t)Median_filter(ir_buffer[i]);
    IR_med[i]=Median_filter_9(IR[i]);
    //Serial.print(IR[k]); 
    //Serial.print("\t");
    //Serial.println(IR_med[i]);

  }
  struct result res = MaxMin_search(IR_med,IR_med,IR_med,BUFFER_SIZE);
  int T_in_minute = BUFFER_SIZE/(60*FS);
  int HR = (res.HR*60*FS)/BUFFER_SIZE;
  Serial.print("Npulses=");Serial.print(res.HR);Serial.print(" / HR=");Serial.print(HR);
  Serial.print(" / SpO2=");Serial.println(res.spo2);

  //rf_heart_rate_and_oxygen_saturation(ir_buffer, BUFFER_SIZE, red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl); 

}


