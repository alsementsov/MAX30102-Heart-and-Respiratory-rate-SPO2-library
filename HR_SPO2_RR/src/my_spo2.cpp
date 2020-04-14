#include <Arduino.h>
#include "my_spo2.h"

float Spo2_calc(int32_t IRmax,int32_t IRminl,int32_t IRminr,int32_t Rmax,int32_t Rminl,int32_t Rminr)
  {
    float Red_DC=(Rminl+Rminr)/2;
    float Red_AC = Rmax-Red_DC;
    float IR_DC=(IRminl+IRminr)/2;
    float IR_AC = IRmax-IR_DC;
    float out = (Red_AC/Red_DC)/(IR_AC/IR_DC);
    out=(-45.060*out*out) + (30.354*out) + 94.845;
    return out;
  };
/////// Errors //////////
uint8_t CheckForErrors(uint16_t T,uint16_t Tmax,float spo2,int32_t Vl,int32_t Vc,int32_t Vr)
{
    static uint16_t Told=0;
    uint8_t Error;
    Error = 0;
    if (Told!=0){
        float div=float(T)/float(Told);
        if ((div > 2.5) or (div < 0.4))
            Error=1;
    }
    if (T<=3)
        Error=2;
    if ((spo2<88)or(spo2>=101)or(Vc<=Vl)or(Vc<=Vr)) //Восстановить после отладки до 91
        Error=3;
    return Error;
};

int32_t Median_filter_small(int32_t datum,int MF_SIZE)
{
  struct pair
  {
    struct pair   *point;                              /* Pointers forming list linked in sorted order */
    int32_t  value;                                   /* Values to sort */
  };
  static struct pair buffer[7] = {0};        /* Buffer of nwidth pairs */
  static struct pair *datpoint = buffer;               /* Pointer into circular buffer of data */
  static struct pair small = {NULL, -5000};          /* Chain stopper */
  static struct pair big = {&small, 0};                /* Pointer to head (largest) of linked list.*/

  struct pair *successor;                              /* Pointer to successor of replaced data item */
  struct pair *scan;                                   /* Pointer used to scan down the sorted list */
  struct pair *scanold;                                /* Previous value of scan */
  struct pair *median;                                 /* Pointer to median */
  uint16_t i;
  if (datum == -5000)  {
    datum = -5000 + 1;                            /* No stoppers allowed. */
  }
  
 if ( (++datpoint - buffer) >= MF_SIZE)  {
    datpoint = buffer;                               /* Increment and wrap data in pointer.*/
  }
  datpoint->value = datum;                           /* Copy in new datum */
  successor = datpoint->point;                       /* Save pointer to old value's successor */
  median = &big;                                     /* Median initially to first in chain */
  scanold = NULL;                                    /* Scanold initially null. */
  scan = &big;                                       /* Points to pointer to first (largest) datum in chain */

  /* Handle chain-out of first item in chain as special case */
  if (scan->point == datpoint)  {
    scan->point = successor;}
  scanold = scan;                                     /* Save this pointer and   */
  scan = scan->point ;                                /* step down chain */
  /* Loop through the chain, normal loop exit via break. */
  for (i = 0 ; i < MF_SIZE; ++i)  {
    /* Handle odd-numbered item in chain  */
    if (scan->point == datpoint)    {
      scan->point = successor;                    /* Chain out the old datum.*/
    }
    if (scan->value < datum)                      /* If datum is larger than scanned value,*/
    {
      datpoint->point = scanold->point;             /* Chain it in here.  */
      scanold->point = datpoint;
      datum = -5000;                    /* Mark it chained in. */
    }
    /* Step median pointer down chain after doing odd-numbered element */
    median = median->point;                       /* Step median pointer.  */
    if (scan == &small){
      break;                                      /* Break at end of chain  */
    }
    scanold = scan;                               /* Save this pointer and   */
    scan = scan->point;                           /* step down chain */
    /* Handle even-numbered item in chain.  */
    if (scan->point == datpoint){
      scan->point = successor;
    }
    if (scan->value < datum)    {
      datpoint->point = scanold->point;
      scanold->point = datpoint;
      datum = -5000;
    }
    if (scan == &small)  {
      break; }
    scanold = scan;
    scan = scan->point;
  }
  return median->value;
}
float Kalman_simple_filter(uint32_t val,float Q,float R)
{
  static float Xe = 0;
  static float Xp;
  static float P = 1;
  static float Pc;
  static float K;

  if ((Xe==0)&&(P==1)){
    Xe=val;
  }
  Xp = Xe;
  Pc = P + Q;
  K = Pc/(Pc + R);
  P = (1-K)*Pc;
  Xe = K*(val-Xp)+Xp; 
  return Xe;
}

struct result MaxMin_search_stream(int32_t IRnorm,uint32_t IR,uint32_t RED)
{
  static bool searching_max = true;
  static int32_t Virmax = 0;
  static int32_t Virmin=0;
  static int32_t Vrmax=0;
  static int32_t Vrmin=0;
  static uint16_t Min_cnt=0;
  static uint16_t Max_cnt=0;
  static uint32_t HR_counter=0;
  static int32_t IRnorm_prev=0;
  static uint32_t Virmin_new=0;
  static uint32_t Vrmin_new=0;
  
  static uint8_t error;
  bool NewBeat;
  float spo2=0;
  NewBeat=false;

  // -------- Поиск максимумов и минимумов в сырых данных -----------------
  if (IR>=Virmax){
    Virmax=IR;
    Virmin_new=IR;}
  else if (IR<=Virmin_new){
    Virmin_new=IR; }
  if (RED>=Vrmax){
    Vrmax=RED;
    Vrmin_new=RED;}
  else if (RED<=Vrmin_new){
    Vrmin_new=RED; }
  // -----Поиск экстремумов в потоке и расчет HR и SPO2 на каждом найденном минимуме
  int32_t delta=abs(IRnorm-IRnorm_prev);
  if (searching_max){
      if ((IRnorm < IRnorm_prev)and(delta>=2)){ // Maximum
          searching_max = false;
          Max_cnt=0;}
  }
  else if ((IRnorm > IRnorm_prev)and(delta>=2)){// Minimum
      searching_max = true;
      NewBeat=true;
      spo2=Spo2_calc(Virmax,Virmin,Virmin_new,Vrmax,Vrmin,Vrmin_new);
      error = CheckForErrors(Min_cnt,Max_cnt,spo2,Virmin,Virmax,Virmin_new);
      if (error<=1){
          HR_counter++;
      }
      //if (error>0)
        //Serial.println(" | Error=");Serial.print(error);
      Min_cnt=0;
      Max_cnt=0;
      Virmin=Virmin_new;
      Vrmin=Vrmin_new;
      Virmax=Virmin_new;
      Vrmax=Vrmin_new;
  }
  if ((Min_cnt>62)||(Max_cnt>31)){//Завит от fps
    error=4;
    //Serial.println(" | Error=4");
  }
  IRnorm_prev=IRnorm;
  Max_cnt++;
  Min_cnt++;
  struct result out{spo2,error,HR_counter,NewBeat};
  return out;
}
float Median_filter_spo2(float datum)
{
  const uint8_t buf_size = 11;
  struct pair
  {
    struct pair   *point;                              /* Pointers forming list linked in sorted order */
    float value;                                   /* Values to sort */
  };
  static struct pair buffer[buf_size] = {0};        /* Buffer of nwidth pairs */
  static struct pair *datpoint = buffer;               /* Pointer into circular buffer of data */
  static struct pair small = {NULL, -5000};          /* Chain stopper */
  static struct pair big = {&small, 0};                /* Pointer to head (largest) of linked list.*/
  struct pair *successor;                              /* Pointer to successor of replaced data item */
  struct pair *scan;                                   /* Pointer used to scan down the sorted list */
  struct pair *scanold;                                /* Previous value of scan */
  struct pair *median;                                 /* Pointer to median */
  uint16_t i;
  if (datum == -5000)  {
    datum = -5000 + 1;                            /* No stoppers allowed. */
  }
  
 if ( (++datpoint - buffer) >= buf_size)  {
    datpoint = buffer;                               /* Increment and wrap data in pointer.*/
  }
  datpoint->value = datum;                           /* Copy in new datum */
  successor = datpoint->point;                       /* Save pointer to old value's successor */
  median = &big;                                     /* Median initially to first in chain */
  scanold = NULL;                                    /* Scanold initially null. */
  scan = &big;                                       /* Points to pointer to first (largest) datum in chain */

  /* Handle chain-out of first item in chain as special case */
  if (scan->point == datpoint)  {
    scan->point = successor;}
  scanold = scan;                                     /* Save this pointer and   */
  scan = scan->point ;                                /* step down chain */
  /* Loop through the chain, normal loop exit via break. */
  for (i = 0 ; i < buf_size; ++i)  {
    /* Handle odd-numbered item in chain  */
    if (scan->point == datpoint)    {
      scan->point = successor;                    /* Chain out the old datum.*/
    }
    if (scan->value < datum)                      /* If datum is larger than scanned value,*/
    {
      datpoint->point = scanold->point;             /* Chain it in here.  */
      scanold->point = datpoint;
      datum = -5000;                    /* Mark it chained in. */
    }
    /* Step median pointer down chain after doing odd-numbered element */
    median = median->point;                       /* Step median pointer.  */
    if (scan == &small){
      break;                                      /* Break at end of chain  */
    }
    scanold = scan;                               /* Save this pointer and   */
    scan = scan->point;                           /* step down chain */
    /* Handle even-numbered item in chain.  */
    if (scan->point == datpoint){
      scan->point = successor;
    }
    if (scan->value < datum)    {
      datpoint->point = scanold->point;
      scanold->point = datpoint;
      datum = -5000;
    }
    if (scan == &small)  {
      break; }
    scanold = scan;
    scan = scan->point;
  }
  return median->value;
}