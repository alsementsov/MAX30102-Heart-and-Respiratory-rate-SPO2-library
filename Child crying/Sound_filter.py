import matplotlib.pyplot as plt
from scipy.io import wavfile # get the api
import numpy as np
from scipy.signal import lfilter, freqz, butter, sosfilt,sosfiltfilt

# Чтение файла WAV    
fsamples, data = wavfile.read('baby1.wav') # load the data
print('Частота дискретизации=',fsamples) #

# Переворот отрицательных значений
pdata = np.where(data<0,-1*data,data)
# Полоса пропускания фильтра
Wn = [0.1,0.9]#0.0001
#Кол-во сэмплов
N=data.size
#Расчет коэффициентов полосоовго фильтра Баттерворда (3-го порядка, sos структуры)
Ksos = butter(3,Wn,btype='band',analog=False,output='sos',fs=fsamples)
#Вывод коэффициентов в терминал - для реализации в STM нужно их скопировать
print(Ksos)

# Фильтрация с использованием встроенной функции scipy.signal.sosfilt 
#- это только для сравнения с фильтром, который ниже
fdata = sosfilt(Ksos,pdata)

# Фильтрация с расшифрованным кодом функции scipy.signal.sosfilt - использовать в STM
fself = []
[d11,d12,d21,d22,d31,d32] = [float(0),float(0),float(0),float(0),float(0),float(0)]
for x1 in pdata:
    # 1 section
    y1 = Ksos[0,0] * float(x1) + d11
    d11 = Ksos[0,1] * float(x1) - Ksos[0,4] * y1 + d12
    d12 =Ksos[0,2] * float(x1) - Ksos[0,5] * y1
    # 2 section
    y2 = Ksos[1,0] * y1 + d21
    d21 = Ksos[1,1] * y1 - Ksos[1,4] * y2 + d22
    d22 = Ksos[1,2] * y1 - Ksos[1,5] * y2
    # 3 section
    y3 = Ksos[2,0] * y2 + d31
    d31 = Ksos[2,1] * y2 - Ksos[2,4] * y3 + d32
    d32 = Ksos[2,2] * y2 - Ksos[2,5] * y3
    fself.append(y3)
   
#plt.plot(pdata)
plt.subplot(121)
plt.plot(fdata,color='purple',linewidth=2)
plt.grid(True)
plt.xticks(np.arange(0, N, step=fsamples),np.arange(0, int(N/fsamples),step=1))
plt.yticks(np.arange(0, max(data), step=500),np.arange(0, max(data), step=500))
plt.ylim(0,5000)
plt.fill_between(np.arange(0, N),fdata,where = (fdata >= 1000))

plt.subplot(122)
plt.plot(fself,color='blue',linewidth=2)
plt.grid(True)
plt.xticks(np.arange(0, N, step=fsamples),np.arange(0, int(N/fsamples),step=1))
plt.yticks(np.arange(0, max(data), step=500),np.arange(0, max(data), step=500))
plt.ylim(0,5000)
plt.fill_between(np.arange(0, N),fdata,where = (fdata >= 1000))

figManager = plt.get_current_fig_manager()
figManager.window.showMaximized()
plt.tight_layout()
plt.show()

# i=0
# outdata=[]
# for dat in fdata:
#     if (i%4000==0):
#        upd = dat
#     outdata.append(upd)
#     i=i+1  
# print(N/4000) 