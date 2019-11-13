#include <Arduino.h>
#include "my_spo2.h"
/////// Median filter //////////////////// 
uint32_t Median_filter(uint32_t datum)
{
  struct pair
  {
    struct pair   *point;                              /* Pointers forming list linked in sorted order */
    uint32_t  value;                                   /* Values to sort */
  };
  static struct pair buffer[MEDIAN_FILTER_SIZE] = {0}; /* Buffer of nwidth pairs */
  static struct pair *datpoint = buffer;               /* Pointer into circular buffer of data */
  static struct pair small = {NULL, STOPPER};          /* Chain stopper */
  static struct pair big = {&small, 0};                /* Pointer to head (largest) of linked list.*/

  struct pair *successor;                              /* Pointer to successor of replaced data item */
  struct pair *scan;                                   /* Pointer used to scan down the sorted list */
  struct pair *scanold;                                /* Previous value of scan */
  struct pair *median;                                 /* Pointer to median */
  uint16_t i;
  if (datum == STOPPER)  {
    datum = STOPPER + 1;                            /* No stoppers allowed. */
  }
  if ( (++datpoint - buffer) >= MEDIAN_FILTER_SIZE)  {
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
  for (i = 0 ; i < MEDIAN_FILTER_SIZE; ++i)  {
    /* Handle odd-numbered item in chain  */
    if (scan->point == datpoint)    {
      scan->point = successor;                    /* Chain out the old datum.*/
    }
    if (scan->value < datum)                      /* If datum is larger than scanned value,*/
    {
      datpoint->point = scanold->point;             /* Chain it in here.  */
      scanold->point = datpoint;                    /* Mark it chained in. */
      datum = STOPPER; }
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
      datum = STOPPER; }
    if (scan == &small)  {
      break; }
    scanold = scan;
    scan = scan->point;
  }
  return median->value;
}
float Spo2_calc(int32_t IRmax,int32_t IRminl,int32_t IRminr,int32_t Rmax,int32_t Rminl,int32_t Rminr,float Ak,float Bk)
  {
    float Red_DC=(Rminl+Rminr)/2;
    float Red_AC = Rmax-Red_DC;
    float IR_DC=(IRminl+IRminr)/2;
    float IR_AC = IRmax-IR_DC;
    //float out = Ak-(Bk*((Red_AC/Red_DC)/(IR_AC/IR_DC)));
    float out = (Red_AC/Red_DC)/(IR_AC/IR_DC);
    out=(-45.060*out*out) + (30.354*out) + 94.845;
    return out;
  };
/////// Errors //////////
struct errors CheckForErrors(uint16_t Left,uint16_t Center,uint16_t Right,uint16_t Told,float spo2,int32_t Vl,int32_t Vc,int32_t Vr)
{
    uint16_t T = Right-Left;
    uint8_t Errorval = 0;
    if (Told!=0){
        float div=float(T)/float(Told);
        //Serial.print(T);Serial.print("/");Serial.print(Told);Serial.print("/");Serial.println(div);
        if ((div > 2.5) or (div < 0.4))
            Errorval=1;
    }
    if ((Center<Left)or(Center>Right)or(Right<=Left+3))
        Errorval=2;
    if ((spo2<85)or(spo2>=102)or(Vc<=Vl)or(Vc<=Vl)) //Восстановить после отладки до 91
        Errorval=3;
    struct errors out{Errorval,T};
    return out;
};
struct element Back_to_extremum (int32_t* ptrmas,bool up,int32_t* startmas)
{
      int32_t* ptr;
      ptr = ptrmas;
      while (((*(ptr-1) >= *ptr)and(up==true))or((*(ptr-1) <= *ptr)and(up==false)))
      {
        ptr=ptr-1;
        if (ptr==startmas){break;}
      }
      struct element out {*ptr,uint16_t(ptr-startmas)};
      return out;
};

float StaticMedianFilter(float *array,int length)
{
	for (int startIndex = 0; startIndex < length/2+1; ++startIndex)
	{
		int smallestIndex = startIndex;
		for (int currentIndex = startIndex + 1; currentIndex < length; ++currentIndex)
		{
			if (array[currentIndex] < array[smallestIndex])
				smallestIndex = currentIndex;
		}
        float temp = array[startIndex];
        array[startIndex]=array[smallestIndex];
        array[smallestIndex]=temp;
	}
	return float(array[length/2]+array[(length/2)-1])/2;
}

struct result MaxMin_search(int32_t *irmas,int32_t *irmas_orig,int32_t *redmas_orig,uint16_t length_mas){

    float spo2_mas[20] = {};
    uint8_t error_mas[length_mas]={};

    int32_t Virmax = 0;
    int32_t Virmin=0;
    int32_t Vrmax=0;
    int32_t Vrmin=0;
    uint16_t left_index=0;
    uint16_t max_index=0;
    uint16_t T=0;
    uint16_t cnt_empty=0;
    uint16_t HR_counter=0;
    bool searching_max = true;
    if (irmas[0]>irmas[1])
       searching_max=false;
    bool Flag_extremum = false;
    struct element elm;
    uint16_t Told =0;
    //
    for (int i=1;i<length_mas;i++){
        //Serial.print("[");Serial.print(i);Serial.print("]=");Serial.println(irmas[i]);
        Flag_extremum=false;
        error_mas[i]=0;
        int32_t delta=abs(abs(irmas[i])-abs(irmas[i-1]));
        if (searching_max){
            if ((irmas[i] < irmas[i-1])and(delta>=5)){//Восстановить после отладки в 5
                elm = Back_to_extremum(&irmas_orig[i],true,irmas_orig);
                Virmax = elm.value;
                searching_max = false;
                Vrmax=redmas_orig[elm.index];
                max_index=elm.index;
                if (Virmin==0)
                  HR_counter++;
                //Serial.print("MAX index=");Serial.print(elm.index);Serial.print("/ IR_MAX =");Serial.print(Virmax);Serial.print("/ RED_MAX =");Serial.println(Vrmax);
            }
        }
        else if ((irmas[i] > irmas[i-1])and(delta>=1)){
            elm = Back_to_extremum(&irmas_orig[i],false,irmas_orig);
            int32_t Virmin_new = elm.value;
            searching_max = true;
            int32_t Vrmin_new=redmas_orig[elm.index];
            //Serial.print("MIN index=");Serial.print(elm.index);Serial.print("/ IR_MIN =");Serial.print(Virmin_new);Serial.print("/ RED_MIN =");Serial.println(Vrmin_new);
            //SPO2
            if (Virmax!=0){
                Flag_extremum=true;
                float spo2=Spo2_calc(Virmax,Virmin,Virmin_new,Vrmax,Vrmin,Vrmin_new,A,B);
                Told=T;
                struct errors error_res = CheckForErrors(left_index,max_index,elm.index,Told,spo2,Virmin,Virmax,Virmin_new);
                if (HR_counter<2)
                  error_mas[i]=0;
                else
                  error_mas[i]=error_res.error;
                T = error_res.T;
                if (error_res.error<=1){
                    spo2_mas[HR_counter] = spo2;
                    //Serial.print("~~~~ Heartbeat: ");Serial.print(HR_counter);Serial.print(" /  SpO2= ");Serial.println(spo2);
                    HR_counter++;
                }
            }
            left_index=elm.index;
            Virmin=Virmin_new;
            Vrmin=Vrmin_new;
            cnt_empty=0;
        }
        cnt_empty=cnt_empty+1;
        if ((Flag_extremum==false)and(T>0)){
            if (cnt_empty>62){//Завит от fps
               //Serial.print("i=");Serial.print(i);Serial.print("/T=");Serial.println(T);
               error_mas[i]=4;
            }
            else if (error_mas[i-1]<4)
                error_mas[i]=error_mas[i-1];
        }
        if (error_mas[i]>0){
          Serial.print("; ERROR[");Serial.print(i); Serial.print("]= ");Serial.print(error_mas[i]);}
     }

    struct result out{StaticMedianFilter(spo2_mas,HR_counter),error_mas[0],HR_counter};//Error_mas пока не сделан
    return out;
}

int32_t Median_filter_small(int32_t datum,bool FastHR)
{
  struct pair
  {
    struct pair   *point;                              /* Pointers forming list linked in sorted order */
    int32_t  value;                                   /* Values to sort */
  };
  static struct pair buffer[FSIZE_SLOW] = {0};        /* Buffer of nwidth pairs */
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
  int MF_SIZE;
  if (FastHR)
    MF_SIZE = FSIZE_FAST;
  else
    MF_SIZE = FSIZE_SLOW;/* code */
  
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
float Kalman_simple_filter(float val) {
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
 Xe = K*(val+Xp)+Xp; 
 return Xe;
}
