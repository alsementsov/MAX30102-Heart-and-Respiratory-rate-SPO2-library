import serial
import numpy  as np
import matplotlib.pyplot as plt
import csv
import datetime

now = datetime.datetime.now()
Filename = now.strftime("%d-%m-%Y_%H-%M") # by Time 
FILENAME_CSV = Filename+'.csv'
Save_to_file = 1

####################################################
# Analyze records
####################################################
def plot_records(lrows):
    records = np.array(lrows)
    Red = np.array(records[2:,0],dtype=int)
    IR = np.array(records[2:,1],dtype=int)
    plt.grid(axis='both',linestyle = '--')
    plt.subplot(121)
    plt.plot(Red)
    plt.subplot(122)
    plt.plot(IR)
    #if (Save_to_file==1):
        #plt.savefig(Filename+'.png',quality =95,dpi=300)
    plt.show()
    
#####################################################
# Capture COM stream and save to CSV
#####################################################        
ard=serial.Serial('COM3',115200)
ard.timeout = 1 # in seconds
cnt = 0
flag= True
file = open(FILENAME_CSV, 'w')
fwriter = csv.writer(file)
list_of_rows=[]
try :
    try:
        while True:
            # Write data
            x = ard.readline()
            s = str(x)
            s = s[2:]
            lenght = len(s) - 5;
            s= s[:lenght]
            row_wr = s.split('\\t')
            print(row_wr)
            if (len(row_wr)>1):
                list_of_rows.append(row_wr)
    except KeyboardInterrupt:
        print(Filename)
        fwriter.writerows(list_of_rows)
        plot_records(list_of_rows) # Show plots
except:
    file.close()
    ard.close()
    print('Error!')
file.close()
ard.close()





