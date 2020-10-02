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
int32_t temp_kfdata1;
int32_t IR_norm;
struct result res;
uint32_t Nsample;
uint32_t Nbeats[HR_FIFOSIZE];
uint32_t RR_Nbeats[RR_FIFOSIZE];
uint32_t* ptr = Nbeats;
uint32_t* RR_ptr = RR_Nbeats;
uint32_t HR;
uint32_t RR;
float spo2;
uint8_t HR_cnt;
uint8_t RR_cnt;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(oxiInt, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102
  Wire.begin();
  Serial.begin(115200);
  maxim_max30102_reset(); //resets the MAX30102
  delay(1000);
  maxim_max30102_init();  //initialize the MAX30102
  BadContact=false;
  Serial.println("START...");
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
void loop() 
{
   //------------ Reading Sensor ---------------------
  for(uint16_t i=0;i<BUFFER_SIZE;i++)
  {
    while(digitalRead(oxiInt)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo((red_buffer+i), (ir_buffer+i));  //read from MAX30102 FIFO
    ////////////////////////////////For STATION///////////////////////////////////// 
    Nsample++;
    temp_kfdata =Kalman_simple_filter1((int32_t)ir_buffer[i],KF_Q,KF_R); // Need to add double
    temp_kfdata1 = Kalman_simple_filter2(temp_kfdata,KF_Q,KF_R);;
    temp_kfdata1 =(int32_t)ir_buffer[i]-temp_kfdata1;
    IR_norm=Median_filter_small(temp_kfdata,7);
    if (ir_buffer[i] < BAD_CONTACT_TH)
    {
      //Serial.println("Bad contact");
      ptr=&Nbeats[0];
      RR_ptr=&RR_Nbeats[0];
      HR_cnt=0;
      RR_cnt=0;    
    }
    else 
    {
      // Find MAX MIN
      res = MaxMin_search_stream(IR_norm,ir_buffer[i],red_buffer[i]);
      if (res.NewBeat)
      {
        //Average spo2 for visualization 
        spo2 = Median_filter_spo2(res.spo2);
        // Starting conditions
        if (HR_cnt <= HR_FIFOSIZE)
        {
          if (HR_cnt<2)
            Nbeats[0]=Nsample;
          else
          {
            HR = ((HR_cnt-1)*60*FS)/(Nsample-Nbeats[0]);
            Serial.print("HR=");Serial.print(HR);
            Serial.print(" / SpO2=");Serial.println(floor(spo2),0);
          }
          HR_cnt++;
        }
        // HR average for 20 beats 
        else
        {
          HR = (HR_FIFOSIZE*60*FS)/(Nsample-*(ptr)); //ptr - oldest time stamp (end of circle FIFO)
          Serial.print("HR=");Serial.print(HR);
          Serial.print(" / SpO2=");Serial.println(floor(spo2),0);
        }
        // FIFO buffer for HR time stamps 
        *(ptr)=Nsample;
        if (ptr==&Nbeats[HR_FIFOSIZE-1])
          ptr=&Nbeats[0];
        else
          ptr++;
        //// Respiratory Rate calculation ////
        if (res.RR_NewBeat)
        {
          if (RR_cnt <= RR_FIFOSIZE)
          {
            if (RR_cnt<2)
              RR_Nbeats[0]=Nsample;
            else
            {
              RR = ((RR_cnt-1)*60*FS)/(Nsample-RR_Nbeats[0]);

              Serial.print("====================>> RR=");Serial.println(RR);
            }
            RR_cnt++;
          }
          // HR average for 20 beats 
          else
          {
            RR = (RR_FIFOSIZE*60*FS)/(Nsample-*(RR_ptr)); //ptr - oldest time stamp (end of circle FIFO)
            Serial.print("====================>> RR=");Serial.println(RR);
          }
          // FIFO buffer for RR time stamps
          *(RR_ptr)=Nsample;
          if (RR_ptr==&RR_Nbeats[RR_FIFOSIZE-1])
            RR_ptr=&RR_Nbeats[0];
          else
            RR_ptr++;
        }
        //ERRORS
        if (res.error>0)
        {
          Serial.print("Error=");Serial.println(res.error);
          if ((res.error==4)&&(HR_cnt >20))
            Serial.println("Alarm! No heartbeats!");
        }
      }
    }
  }
}



