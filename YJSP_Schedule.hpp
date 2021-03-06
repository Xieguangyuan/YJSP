#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <fstream>
#include <functional>
#include <wiringPi.h>
#include <thread>

#include "pca9685.h"
#include "VL53L1XDev.hpp"
#include "thirdparty/RaspberryPiRC/RPiIBus/RPiIBus.hpp"
#include "thirdparty/RaspberryPiMPU/src/MPU9250/MPU9250.hpp"
#include "thirdparty/KalmanIMP.hpp"

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
        void UserInput(int Forward, int Horizontal, int Yaw, int Input_Yaw, bool DisableSpeed);
        void UserLocationSet(int TargetX, int TargetY);
        void PWMUserInput(int pinBase, int on, int off);
        void Servo_ArmGrab1();
        void Servo_ArmGrab2();
        void Servo_ArmGrab3();
        void Servo_ArmGrab1_Down();
        void Servo_ArmGrab2_Down();
        void Servo_ArmGrab3_Down();
        void Servo_ArmPlace1();
        void Servo_ArmPlace2();
        void Servo_ArmPlace3();
        void Servo1_Place_Up();
        void Servo2_Place_Up();
        void Servo3_Place_Up();
        void Servo_ArmGet1();
        void Servo_ArmGet2();
        void Servo_ArmGet3();
        void Servo_Scanner();
        void Servo_Scanner_Down();
        void QRScanner();
        void Servo_Over();
        YJSP_AP::YJSP &&OnRCDataCome(std::function<void(int *)> call_BACK)
        {
            OnRCDataInComing = call_BACK;
        }

        // protected:
        std::function<void(int *)> OnRCDataInComing;

        struct Device
        {
            char RCDeviceInfo[20] = "/dev/ttyAMA0";
            TotalEKF EKFIMS;
            RPiMPU9250 *MPUDevice;
            Ibus *myIbusDevice;
            VL53L1XDevice myVL53L1X;
            VL53L1XDevice MYVL53L1X;
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
            float YawInput;
            float ForInput;
            float HorInput;

            int SPEED_X_EKF;
            int SPEED_Y_EKF;

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

            float PIDHorPGain = 0;
            float PIDHorIGain = 0;
            float PIDHorDGain = 0;
            float PIDHorLastIData = 0;
            float PIDHorLastDData = 0;

            float TotalYawIFilter = 0;
            float TotalYawDFilter = 0;
            float TotalForwardIFilter = 0;
            float TotalForwardDFilter = 0;
            float TotalHorIFilter = 0;
            float TotalHorDFilter = 0;

            bool IsPositionDisable = false;
            bool IsSpeedDisable = false;
            float UserTargetPostionX = 0;
            float UserTargetPostionY = 0;
        } PF;

        struct ESCData
        {
            double MOVE_X = 0;
            double MOVE_Y = 0;
            double SPEED_X = 0;
            double SPEED_X_EKF = 0;
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
            std::thread NewPosThreading;

            int TimeStart = 0;
            int TimeEnd = 0;
            int TimeNext = 0;
            int TimeMax = 1000;

            int ClearCount = 0;

            bool QR_Number = false;
            bool Red = false;
            bool Green = false;
            bool Blue = false;

            int H_LOW = 0, S_LOW = 0, V_LOW = 0;
            int H_HIGH = 179;
            int S_HIGH = 255;
            int V_HIGH = 255;

        } TF;

        struct SensorData
        {
            MPUData myData;
            int speed_x = 0;
            int speed_y = 0;
            int speed_y_tmp = 0;
            int distance_X = 0;
            int distance_Y = 0;
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