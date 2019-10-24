def spo2_calc(IRmax,IRminl,IRminr,Rmax,Rminl,Rminr,A,B):
    Red_DC=(Rminl+Rminr)/2
    Red_AC = Rmax-Red_DC
    IR_DC=(IRminl+IRminr)/2
    IR_AC = IRmax-IR_DC
    return (A-(B*((Red_AC/Red_DC)/(IR_AC/IR_DC))))

print()