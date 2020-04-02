#include <iostream>
#include <cmath>
#include "ACC_Events.h"
using namespace std;

struct result_ACC ACC_calc(int16_t AX, int16_t AY, int16_t AZ, bool TEST)
{
    int16_t Roll;  // ACC-BLE axis -  (-180...180)
    int16_t Pitch; // ACC-SPO2 axis - (-90...90)
    int16_t Yaw;
    //FORMULA
    int16_t Raw_Roll = atan2(-AY, AZ) * RAD_TO_DEG;// ACC-BLE axis -  (-180...180)
    int16_t Raw_Pitch = atan2(AX, sqrt(AY * AY + AZ * AZ)) * RAD_TO_DEG;// ACC-SPO2 axis - (-90...90)
    int16_t Raw_Yaw = atan2(AZ, -sqrt(AY * AY + AX * AX)) * RAD_TO_DEG;
    struct result_ACC out; //Результат
    // Kalman filter - вставить для потоковых данных, c набором тестовых измерений не участвует
    if (TEST == 1)
    { //Только для тестовых данных
        Roll = Raw_Roll;
        Pitch = Raw_Pitch;
        Yaw = Raw_Yaw;
    }
    else
    {
        Roll = ACC_Kfilter(Raw_Roll);
        Pitch = ACC_Kfilter(Raw_Pitch);
        Yaw = ACC_Kfilter(Raw_Yaw);
    }
    std::cout << "Roll/Pitch/Yaw = " << (int)Roll << " / " << (int)Pitch << " / " << (int)Yaw;
    // Detect overturning
    if ((abs(Roll) < 40) && (Pitch < 75) && (Pitch > -25))
    {
        out.Event_overturn = false;
        std::cout << std::endl;
    }
    else
    {
        out.Event_overturn = true; // Возможно потребуется еще таймер отсрочки(перепроверки)!
        std::cout << "  Overturning!" << std::endl;
    }
    // Detect movements
    if ((abs(Roll - Raw_Roll) > TH_dAngle) || (abs(Pitch - Raw_Pitch) > TH_dAngle) || ((abs(Yaw - Raw_Yaw)) > TH_dAngle))
    {
        out.Event_movement = true;
        //std::cout << "  Movement!" << std::endl;
    }
    else
    {
        out.Event_movement = false;
    }
    return out;
}

float ACC_Kfilter(float val)
{
    static float Xe = 0;
    static float Xp;
    static float P = 1;
    static float Pc;
    static float K;
    if ((Xe == 0) && (P == 1))
    {
        Xe = val;
    }
    Xp = Xe;
    Pc = P + varProcess;
    K = Pc / (Pc + varVolt);
    P = (1 - K) * Pc;
    Xe = K * (val - Xp) + Xp;
    return Xe;
}

//////////ПРОВЕРКА////////////////////
int main()
{
    for (size_t i = 0; i < 16; i++)
    {
        std::cout<<i+2<<") ";
        ACC_calc(SamplesRB[i][0], SamplesRB[i][1], SamplesRB[i][2], true); //Тестовый режим
    }
    return 0;
}