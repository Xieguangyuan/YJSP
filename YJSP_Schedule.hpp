#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <fstream>
#include <functional>
#include <wiringPi.h>
#include <thread>

#include "pca9685.h"
#include "VL53L1XFuck.hpp"
#include "thirdparty/RaspberryPiRC/RPiIBus/RPiIBus.hpp"
#include "thirdparty/RaspberryPiMPU/src/MPU9250/MPU9250.hpp"

namespace YJSP_AP
{
    class YJSP
    {
    public:
        int YJSP_Init();
        void MPUThreadREG();
        void PositionThreadREG();
        void RCThreadREG();
        void ESCThreadREG();
        void DEBUGThreadREG();
        void UserInput(int Forward, int Horizontal, int Yaw, int Input_Yaw);
        void PWMUserInput(int pinBase, int on, int off);
        void Servo_ArmGrab1();
        void Servo_ArmGrab2();
        void Servo_ArmGrab3();
        void Servo_ArmPlace1();
        void Servo_ArmPlace2();
        void Servo_ArmPlace3();
        void Servo_ArmGet1();
        void Servo_ArmGet2();
        void Servo_ArmGet3();
        YJSP_AP::YJSP &&OnRCDataCome(std::function<void(int *)> call_BACK)
        {
            OnRCDataInComing = call_BACK;
        }

    private:
        std::function<void(int *)> OnRCDataInComing;

        struct Device
        {
            char RCDeviceInfo[20] = "/dev/ttyAMA0";
            RPiMPU9250 *MPUDevice;
            Ibus *myIbusDevice;
            VL53L1XFuck myVL53L1X;
            int fd;
        } DF;

        struct RCData
        {

            int IbusData[14] = {1500, 1500, 1000, 1500, 999, 999, 999, 999, 999, 999};

            bool RCARM = true;
            bool RC_Auto = false;
            float TotalForward;
            float TotalHorizontal;
            float TotalYaw;

            int RCForward;
            int RCForwardMax;
            int RCForwardMin;
            int RCForwardMiddle;
            int RCHorizontal;
            int RCHorizontalMax;
            int RCHorizontalMin;
            int RCHorizontalMiddle;
            int RCYaw;
            int RCYawMax;
            int RCYawMin;
            int RCYawMiddle;

            int Auto_Forward;
            int Auto_Horizontal;
            int Auto_Yaw;

            int InputYaw;

        } RF;

        struct PIDData
        {
            float PIDYawPGain = 0;
            float PIDYawIGain = 0;
            float PIDYawDGain = 0;
            float PIDYawLastIData = 0;
            float PIDYawLastDData = 0;

            float PIDForwardPGain = 0;
            float PIDForwardIGain = 0;
            float PIDForwardDGain = 0;
            float PIDForwardLastIData = 0;
            float PIDForwardLastDData = 0;

            float TotalYawIFilter = 0;
            float TotalYawDFilter = 0;
            float TotalForwardIFilter = 0;
            float TotalForwardDFilter = 0;
        } PF;

        struct ESCData
        {

            double SPEED_X = 0;
            double SPEED_Y = 0;
            double SpeedA1 = 0;
            double SpeedA2 = 0;
            double SpeedB1 = 0;
            double SpeedB2 = 0;
            double SpeedA1TO = 0;
            double SpeedA2TO = 0;
            double SpeedB1TO = 0;
            double SpeedB2TO = 0;
        } EF;

        struct Threading
        {
            std::thread RCThreading;
            std::thread MPUThreading;
            std::thread ESCThreading;
            std::thread PosThreading;

            int TimeStart = 0;
            int TimeEnd = 0;
            int TimeNext = 0;
            int TimeMax = 1000;

            int ClearCount = 0;

        } TF;

        struct SensorData
        {
            MPUData myData;
            int speed = 0;
            int distance = 0;
            double AccelCaliData[30];
            float Real_Yaw = 0;

            float dustVal = 0;
            double dataEx = 0;
            double dataFinal = 0;
            double dataLast = 0;
            int dataCount = 0;
            double dataExOut = 0;
            double dataVar[10] = {0};
        } SF;

        void PIDCacl(float inputDataP, float inputDataI, float inputDataD, float &outputData,
                     float &last_I_Data, float &last_D_Data,
                     float P_Gain, float I_Gain, float D_Gain, float I_Max);

        double configSettle(const char *configDir, const char *Target);

        double configWrite(const char *configDir, const char *Target, double obj);
    };
} // namespace YJSP_AP