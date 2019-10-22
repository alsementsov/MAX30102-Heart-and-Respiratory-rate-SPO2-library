#include <iostream>
#include <cmath>
#define STOPPER 0                                      /* Smaller than any datum */
#define MEDIAN_FILTER_SIZE 81
using namespace std;

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

const int n = 20;
const float A = 104.5;
const float B = 16.5;

uint32_t irmas [n] = {100,50,100,200,300,400,230,70,120,140,250,360,190,90,150,160,270,37,100,180};

float Spo2_calc(uint32_t IRmax,uint32_t IRminl,uint32_t IRminr,uint32_t Rmax,uint32_t Rminl,int32_t Rminr,float A,float B){
    float Red_DC=(Rminl+Rminr)/2;
    float Red_AC = Rmax-Red_DC;
    float IR_DC=(IRminl+IRminr)/2;
    float IR_AC = IRmax-IR_DC;
    float out = A-(B*((Red_AC/Red_DC)/(IR_AC/IR_DC)));
    return out;
  };
/////// Errors //////////
struct errors CheckForErrors(uint16_t Left,uint16_t Center,uint16_t Right,uint16_t Told,float spo2,uint32_t Vl,uint32_t Vc,uint32_t Vr){
    uint16_t T = Right-Left;
    uint8_t Errorval = 0;
    if (Told!=0){
        float div=T/Told;
        if ((div > 2.5) or (div < 0.4))
            Errorval=1;
    }
    if ((Center<Left)or(Center>Right)or(Right<=Left+3))
        Errorval=2;
    if ((spo2<80)or(spo2>=102)or(Vc<=Vl)or(Vc<=Vl)) //Восстановить после отладки до 91
        Errorval=3;
    struct errors out{Errorval,T};
    return out;
};

struct element Back_to_extremum (uint32_t* ptrmas,bool up,uint32_t* startmas){
      uint32_t* ptr;
      ptr = ptrmas;
      while (((*(ptr-1) >= *ptr)and(up==true))or((*(ptr-1) <= *ptr)and(up==false)))
      {
        ptr=ptr-1;
        if (ptr==startmas){break;}
      }
      struct element out {*ptr,(ptr-startmas)};
      return out;
};

float StaticMedianFilter(float *array,int length){
	for (int startIndex = 0; startIndex < length/2+1; ++startIndex)
	{
		int smallestIndex = startIndex;
		for (int currentIndex = startIndex + 1; currentIndex < length; ++currentIndex)
		{
			if (array[currentIndex] < array[smallestIndex])
				smallestIndex = currentIndex;
		}
        int temp = array[startIndex];
        array[startIndex]=array[smallestIndex];
        array[smallestIndex]=temp;
	}
	for (int index = 0; index < length; ++index)
		cout << array[index] << ' ';
	return float(array[length/2]+array[(length/2)-1])/2;
}

struct result MaxMin_search(uint32_t *irmas,uint32_t *irmas_orig,uint32_t *redmas_orig,uint16_t length_mas){

    float spo2_mas[10] = {};
    uint8_t error_mas[length_mas]={};

    uint32_t Virmax = 0;
    uint32_t Virmin=0;
    uint32_t Vrmax=0;
    uint32_t Vrmin=0;
    uint16_t left_index=0;
    uint16_t max_index=0;
    uint16_t T=0;
    uint16_t cnt_empty=0;
    uint16_t HR_counter=0;
    uint16_t Spo2_counter =0;
    bool searching_max = true;
    if (irmas[0]>irmas[1])
       searching_max=false;
    bool Flag_extremum = false;
    struct element elm;
    uint16_t Told =0;
    //
    for (int i=1;i<length_mas;i++){
        cout <<"["<<i<<"]="<<irmas[i]<<endl;
        Flag_extremum=false;
        error_mas[i]=0;
        uint32_t delta=abs(abs(irmas[i])-abs(irmas[i-1]));
        if (searching_max){
            if ((irmas[i] < irmas[i-1])and(delta>=1)){//Восстановить после отладки в 5
                elm = Back_to_extremum(&irmas_orig[i],true,irmas_orig);
                Virmax = elm.value;
                searching_max = false;
                Vrmax=redmas_orig[elm.index];
                max_index=elm.index;
                cout <<"---------------max_index="<<elm.index<<"/ MAX ="<<Virmax <<"/Vrmax="<<Vrmax<<endl;
            }
        }
        else if ((irmas[i] > irmas[i-1])and(delta>=1)){
            elm = Back_to_extremum(&irmas_orig[i],false,irmas_orig);
            uint32_t Virmin_new = elm.value;
            searching_max = true;
            uint32_t Vrmin_new=redmas_orig[elm.index];
            cout <<"-----------------min_index="<<elm.index<<" / MIN ="<<Virmin_new<<"/Vrmin = "<<Vrmin_new<< endl;
            //SPO2
            if (Virmax!=0){
                Flag_extremum=true;
                float spo2=Spo2_calc(Virmax,Virmin,Virmin_new,Vrmax,Vrmin,Vrmin_new,A,B);
                cout<<"SPO2= "<< spo2 <<endl;
                Told=T;
                struct errors error_res = CheckForErrors(left_index,max_index,elm.index,Told,spo2,Virmin,Virmax,Virmin_new);
                error_mas[i]=error_res.error;
                cout<<"Error= "<< uint16_t(error_mas[i]) <<endl;
                T = error_res.T;
                if (error_res.error<=1){
                    spo2_mas[HR_counter] = spo2;
                    HR_counter++;
                    //cout<<"Error= "<<error_res.error<<endl;
                }
            }
            left_index=elm.index;
            Virmin=Virmin_new;
            Vrmin=Vrmin_new;
            cnt_empty=0;
        }
        cnt_empty=cnt_empty+1;
        if ((Flag_extremum==false)and(T>0)){
            if (((cnt_empty/T)>3)or(cnt_empty>62))//Завит от fps
               error_mas[i]=4;
            else
                error_mas[i]=error_mas[i-1];
        }
     }
    float spo2 = StaticMedianFilter(spo2_mas,HR_counter);
    struct result out{spo2,error_mas[0],HR_counter};//Error_mas пока не сделан
    return out;
}

uint16_t Median_filter(uint32_t datum){
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



int main()
{
    cout << "START..." << endl;
    struct result res = MaxMin_search(irmas,irmas,irmas,n);
    cout << "HR = "<< res.HR << endl;
    cout << "SpO2 = "<< res.spo2 << endl;
    cout << "Full error = "<< res.error << endl;

    return 0;
}
