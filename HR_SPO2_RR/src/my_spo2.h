#include <Arduino.h>
#define STOPPER 0 /* Smaller than any datum */
#define MEDIAN_FILTER_SIZE 81
#define FSIZE_SLOW 7
#define FSIZE_FAST 7
#define BAD_CONTACT_TH 80000
#define DELAY_SIZE 2

#define KF_Q 0.00002
#define KF_R 0.01

#define true 1
#define false 0
#define FS 25  //sampling frequency (samples/second) -> Don't change!
#define BUFFER_SIZE  (FS* 10) 
#define HR_FIFOSIZE  20
#define RR_FIFOSIZE  10

// N of samples, FS*T, T - duration of continuous buffering data packet in seconds
struct result
{
    float spo2;
    uint8_t error;
    bool NewBeat;
    bool RR_NewBeat;
};
struct element{
    uint32_t value;
    uint16_t index;};

int32_t Median_filter_small(int32_t datum,int MF_SIZE);
float Spo2_calc(int32_t IRmax,int32_t IRminl,int32_t IRminr,int32_t Rmax,int32_t Rminl,int32_t Rminr);
uint8_t CheckForErrors(uint16_t T,uint16_t Tmax,float spo2,int32_t Vl,int32_t Vc,int32_t Vr);
float StaticMedianFilter(float *array,int length);
struct result MaxMin_search_stream(int32_t irmas,uint32_t irmas_orig,uint32_t redmas_orig);
float Kalman_simple_filter(uint32_t val, float Q, float R);
float Median_filter_spo2(float datum);
bool RR_calc(int32_t maxs);







