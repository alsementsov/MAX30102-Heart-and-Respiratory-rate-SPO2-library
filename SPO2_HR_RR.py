import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt
import scipy.signal as sig
import pylab
from tkinter import Tk
from tkinter.filedialog import askopenfilename
############ CONSTANT
FPS = 25
A = 104.5
B = 16.5
Size_Lf = 7
Size_Lf_spo2=13
Q = 0.00002  #SKF process variance   -5 
R = 0.01  # SKF estimate of measurement variance, change to see effect
############ Functions
def KalmanFilter(mas,Qf,Rf):
    # intial parameters
    sz = size(mas) #50
    # allocate space for arrays
    x=np.zeros(sz)      # a posteri estimate of x
    P=np.zeros(sz)         # a posteri error estimate
    xhatminus=np.zeros(sz) # a priori estimate of x
    Pminus=np.zeros(sz)    # a priori error estimate
    K=np.zeros(sz)         # gain or blending factor
       
    # intial guesses
    x[0] = mas[0]
    P[0] = 1.0
    
    for k in range(1,sz):
        # time update
        xhatminus[k] = x[k-1]
        Pminus[k] = P[k-1]+Qf
        # measurement update
        K[k] = Pminus[k]/( Pminus[k]+Rf)
        x[k] = xhatminus[k]+K[k]*(mas[k]-xhatminus[k])
        P[k] = (1-K[k])*Pminus[k]
    return x
def spo2_calc(IRmax,IRminl,IRminr,Rmax,Rminl,Rminr,A,B):
    Red_DC=(Rminl+Rminr)/2
    Red_AC = Rmax-Red_DC
    IR_DC=(IRminl+IRminr)/2
    IR_AC = IRmax-IR_DC
    ratioAverage=(Red_AC/Red_DC)/(IR_AC/IR_DC)
    #return (A-(B*((Red_AC/Red_DC)/(IR_AC/IR_DC))))
    return (-45.060*ratioAverage* ratioAverage) + (30.354 *ratioAverage) + 94.845 
def CheckForErrors(Left,Center,Right,Told,spo2,Vl,Vc,Vr):
    T = Right-Left
    Error = 0
    if Told!=0:
        div=T/Told
        # Большая разница в периодах
        if (div>2.5)or(div<0.4):
            Error=1
    # Экстремумы слишком близко
    if ((Center<Left)or(Center>Right)or(Right<=Left+3)):
        Error=2
        #падение моментального SPO 
    if (spo2<91)or(spo2>=102)or(Vc<=Vl)or(Vc<=Vl):
        Error=3
    return Error,T
def back_to_extremum (mas,index,sign):
    i=index;
    while ((mas[i-1]>= mas[i])and(sign=="up"))or((mas[i-1]<= mas[i])and(sign=="down")):
        i=i-1
    return mas[i],i 
def MaxMin_search(irmas,irmas_orig,redmas_orig):
    ##################
    maxs = []
    ##################
    irmax = []
    irmin = []
    irmax_index = []
    irmin_index = []
    rmax=[]
    rmin=[]
    rmin_index=[]
    rmax_index=[]
    irmax_med=[]
    irmin_med=[]
    irmax_med_index=[]
    irmin_med_index=[]
    spo2_mas=[]
    error_mas=np.zeros(len(irmas_orig),dtype=int)
    if (irmas[0]>irmas[1]):
       searching_max=False
    else:
       searching_max = True
    sample_prev = irmas[0]
    Virmax=0
    Virmin=0
    Vrmax=0
    Vrmin=0
    left_index=0
    cnt=0;
    max_index=0;
    T=0
    cnt_empty=0
    HR_counter=0
    for sample in irmas[1:]:
        Flag_extremum=False
        delta=abs(sample-sample_prev)
        if searching_max:#Max   
            if (sample< sample_prev)and(delta>=5):               
                Virmax,i = back_to_extremum(irmas_orig,cnt,"up") 
                irmax.append(Virmax)
                irmax_index.append(i)
                searching_max = False
                irmax_med.append(sample_prev)
                irmax_med_index.append(cnt)
                Vrmax=redmas_orig[i]
                rmax.append(Vrmax)
                rmax_index.append(i)
                max_index=i
                maxs.append(sample_prev)
        elif (sample > sample_prev)and(delta>=5):
            Virmin_new,i = back_to_extremum(irmas_orig,cnt,"down")  
            irmin.append(Virmin_new)
            irmin_index.append(i)
            searching_max = True
            irmin_med.append(sample_prev)
            irmin_med_index.append(cnt)
            Vrmin_new=redmas_orig[i]
            rmin.append(Vrmin_new)
            rmin_index.append(i)
            #SPO2
            if len(rmin)>1:
                Flag_extremum=True
                spo2=spo2_calc(Virmax,Virmin,Virmin_new,Vrmax,Vrmin,Vrmin_new,A,B)
                spo2_mas.append(spo2)
                Told=T
                error,T=CheckForErrors(left_index,max_index,i,Told,spo2,Virmin,Virmax,Virmin_new)
                error_mas[cnt]=error
                if error<=1:
                    HR_counter=HR_counter+1
            left_index=i
            Virmin=Virmin_new
            Vrmin=Vrmin_new
            cnt_empty=0
        sample_prev = sample
        cnt=cnt+1
        cnt_empty=cnt_empty+1
        if (Flag_extremum==False)and(T>0):
            if ((cnt_empty/T)>3)or(cnt_empty>62):#Завит от fps
                error_mas[cnt]=4
            else:
                error_mas[cnt]=error_mas[cnt-1]        
    return irmax,irmin,irmax_index,irmin_index,rmax,rmin,rmax_index,rmin_index,irmax_med,irmin_med,irmax_med_index,irmin_med_index,spo2_mas,error_mas,HR_counter,maxs
##### OPEN
def Filename_request():
    root = Tk()
    root.withdraw()
    root.update() # we don't want a full GUI, so keep the root window from appearing
    Filename = askopenfilename(defaultextension='.csv',filetypes=[('CSV','*.csv')])
    print(Filename)
    return Filename
###############################################################################################
############### MAIN ##########################################################################
###############################################################################################
records = genfromtxt(Filename_request(), delimiter=',')
Red_read = np.array(records[2:,0],dtype=int)
IR_read = np.array(records[2:,1],dtype=int)
#Big MEDIAN = To improve performance (may be excluded)
#IR_med= sig.medfilt(IR_read,5)
IR_kalman = KalmanFilter(IR_read,Q,R)
#IR_kalman = KalmanFilter(IR_kalman,Q,R)# Smoothing


IR = IR_read - IR_kalman # without DC
# Small MEDIAN
IR_med = sig.medfilt(IR,7)

irmax,irmin,irmax_i,irmin_i,rmax,rmin,rmax_i,rmin_i,irmax_med,irmin_med,irmax_med_i,irmin_med_i,spo2,errors,HR_raw,maxs= MaxMin_search(IR_med,IR_read,Red_read)  

#### Heart rate & SpO2
spo2_med = sig.medfilt(spo2,Size_Lf_spo2)  # сглаживание spo2
Nsamples=len(Red_read)
T_in_minute = Nsamples/(60*FPS)
#HR_raw = len(irmax)
HR = int(HR_raw/T_in_minute);

############## RR ##################################

if (HR>100):
    maxs_f = KalmanFilter(maxs,0.01,0.01)
elif (HR>78):
    maxs_f = KalmanFilter(maxs,0.005,0.01)
    print('80')
else:
    maxs_f = KalmanFilter(maxs,0.001,0.01)#0.0005


Resp=[];
Resp_cnt=0;
searching_max = True
Resp_index=[]
i=0
sample_prev=maxs_f[0]

for sample in maxs_f:
   delta=abs(sample-sample_prev)
   if searching_max: 
       if (sample< sample_prev)and(delta>=5):               
            Resp.append(sample_prev)
            searching_max = False
            Resp_index.append(i-1)
            Resp_cnt=Resp_cnt+1
   elif (sample > sample_prev)and(delta>=5):
       searching_max = True
   sample_prev = sample
   i=i+1
RR = int(Resp_cnt/T_in_minute);    



#############################PLOT################################################

plt.subplot(221)
plt.grid(axis='both',linestyle = '--')
plt.plot(IR_read)
plt.plot(irmax_i,irmax,'o')
plt.plot(irmin_i,irmin,'o')
plt.plot(IR_kalman,color='purple',linewidth ='2')#errors
plt.title("IR ch. (HR= "+str(HR)+" beats/minute)")

plt.subplot(222)
plt.grid(axis='both',linestyle = '--')
plt.plot(Red_read,color='purple',linewidth ='2')
plt.plot(rmax_i,rmax,'o')
plt.plot(rmin_i,rmin,'o')
plt.title("RED ch.")

# plt.subplot(223)
# plt.grid(axis='both',linestyle = '--')
# plt.plot(IR,color='purple',linewidth ='2')
# plt.plot(IR_med,color='blue',linewidth ='2')
# plt.title("HR= "+str(HR)+" beats/minute")


plt.subplot(223)
plt.title(" === Mins ===")
plt.grid(axis='both',linestyle = '--')
plt.plot(maxs,color='red',linewidth ='2')
plt.plot(maxs_f,color='blue',linewidth ='2')
plt.plot(Resp_index,Resp,'o')
plt.title("Respiratory rate="+str(RR)+" breaths/minute")





# plt.subplot(223)
# plt.grid(axis='both',linestyle = '--')
# plt.plot(errors,color='purple',linewidth ='3')
# plt.title("IR pulses = "+str(len(irmax))+"("+str(int(len(irmax)/T_in_minute))+") /  HR= "+str(HR))
# plt.ylim((0, 4.5))

plt.subplot(224)
plt.title(" === SpO2 ===")
plt.ylim((93, 100))
pylab.yticks(range(93, 100,1))
plt.grid(axis='both',linestyle = '--')
plt.plot(spo2_med,color='red',linewidth ='2')

figManager = plt.get_current_fig_manager()
figManager.window.showMaximized()
plt.show()
           


