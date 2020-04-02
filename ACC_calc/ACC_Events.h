#define RAD_TO_DEG 57.2957786
#define TH_dAngle 5
#define varVolt 0.008
#define varProcess  0.20
struct result_ACC
{
    bool Event_overturn;
    bool Event_movement;
};

float ACC_Kfilter(float val);
result_ACC ACC_calc(int16_t AX,int16_t AY,int16_t AZ);

const int16_t SamplesRB[16][3] = {
    {(int16_t)0xFF08, (int16_t)0x0040, (int16_t)0x03D0}, // OK
    {(int16_t)0xFCC0, (int16_t)0x01D4, (int16_t)0x00A4},
    {(int16_t)0xFE2C, (int16_t)0x01A0, (int16_t)0xFCEC},
    {(int16_t)0x0158, (int16_t)0x02C4, (int16_t)0x0278},
    {(int16_t)0xFCDC, (int16_t)0xFE04, (int16_t)0x00D4},
    {(int16_t)0xFC38, (int16_t)0xFFC4, (int16_t)0x0000},
    {(int16_t)0x0050, (int16_t)0xFF48, (int16_t)0xFC04},
    {(int16_t)0x0000, (int16_t)0x0050, (int16_t)0x03F8}, //OK
    {(int16_t)0x03E4, (int16_t)0xFFE4, (int16_t)0x0084},
    {(int16_t)0x0034, (int16_t)0x004C, (int16_t)0xFC14},
    {(int16_t)0xFFB0, (int16_t)0x03C4, (int16_t)0x0028},
    {(int16_t)0xFFDC, (int16_t)0x01C4, (int16_t)0xFC8C}, 
    {(int16_t)0xFFD0, (int16_t)0x0208, (int16_t)0x0350}, //OK
    {(int16_t)0x0000, (int16_t)0xFE18, (int16_t)0x0374},//may be OK
    {(int16_t)0x0024, (int16_t)0xFE04, (int16_t)0xFCA4},
    {(int16_t)0xFD94, (int16_t)0xFFB0, (int16_t)0xFCEC}};
