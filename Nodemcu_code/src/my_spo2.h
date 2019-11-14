#include <Arduino.h>
#define STOPPER 0 /* Smaller than any datum */
#define MEDIAN_FILTER_SIZE 81
#define FSIZE_SLOW 7
#define FSIZE_FAST 7
#define BAD_CONTACT_TH 80000
#define DELAY_SIZE 2

#define A 105
#define B 16.5
#define true 1
#define false 0
#define FS 25  //sampling frequency (samples/second) -> Don't change!
#define BUFFER_SIZE  (FS* 10) 

// N of samples, FS*T, T - duration of continuous buffering data packet in seconds
struct result{
    float spo2;
    uint8_t error;
    uint16_t HR;};
struct element{
    uint32_t value;
    uint16_t index;};
struct errors{
    uint8_t error;
    uint16_t T;};

uint32_t Median_filter(uint32_t datum);
int32_t Median_filter_small(int32_t datum,bool FastHR);
float Spo2_calc(int32_t IRmax,int32_t IRminl,int32_t IRminr,int32_t Rmax,int32_t Rminl,int32_t Rminr,float Ak,float Bk);
struct errors CheckForErrors(uint16_t Left,uint16_t Center,uint16_t Right,uint16_t Told,float spo2,int32_t Vl,int32_t Vc,int32_t Vr);
struct element Back_to_extremum (uint32_t* ptrmas,bool up,uint32_t* startmas);
float StaticMedianFilter(float *array,int length);
struct result MaxMin_search(int32_t *irmas,uint32_t *irmas_orig,uint32_t *redmas_orig,uint16_t length_mas);
float Kalman_simple_filter(uint32_t val);









/*uch_spo2_table is approximated as  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
const float uch_spo2_table[184]=
              {94.845,95.144034,95.434056,95.715066,95.987064,96.25005,96.504024,96.748986,96.984936,97.211874,97.4298,97.638714,97.838616,98.029506,
              98.211384,98.38425,98.548104,98.702946,98.848776,98.985594,99.1134,99.232194,99.341976,99.442746,99.534504,99.61725,99.690984,99.755706,
              99.811416,99.858114,99.8958,99.924474,99.944136,99.954786,99.956424,99.94905,99.932664,99.907266,99.872856,99.829434,99.777,99.715554,
              99.645096,99.565626,99.477144,99.37965,99.273144,99.157626,99.033096,98.899554,98.757,98.605434,98.444856,98.275266,98.096664,97.90905,
              97.712424,97.506786,97.292136,97.068474,96.8358,96.594114,96.343416,96.083706,95.814984,95.53725,95.250504,94.954746,94.649976,94.336194,
              94.0134,93.681594,93.340776,92.990946,92.632104,92.26425,91.887384,91.501506,91.106616,90.702714,90.2898,89.867874,89.436936,88.996986,
              88.548024,88.09005,87.623064,87.147066,86.662056,86.168034,85.665,85.152954,84.631896,84.101826,83.562744,83.01465,82.457544,81.891426,
              81.316296,80.732154,80.139,79.536834,78.925656,78.305466,77.676264,77.03805,76.390824,75.734586,75.069336,74.395074,73.7118,73.019514,
              72.318216,71.607906,70.888584,70.16025,69.422904,68.676546,67.921176,67.156794,66.3834,65.600994,64.809576,64.009146,63.199704,62.38125,
              61.553784,60.717306,59.871816,59.017314,58.1538,57.281274,56.399736,55.509186,54.609624,53.70105,52.783464,51.856866,50.921256,49.976634,
              49.023,48.060354,47.088696,46.108026,45.118344,44.11965,43.111944,42.095226,41.069496,40.034754,38.991,37.938234,36.876456,35.805666,
              34.725864,33.63705,32.539224,31.432386,30.316536,29.191674,28.0578,26.914914,25.763016,24.602106,23.432184,22.25325,21.065304,19.868346,
              18.662376,17.447394,16.2234,14.990394,13.748376,12.497346,11.237304,9.96825,8.690184,7.403106,6.107016,4.801914,3.4878,2.164674,0.832536,
              0.0};*/