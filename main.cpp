#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <nlohmann/json.hpp>
#include "pca9685.h"
#include <fstream>
#include <wiringPi.h>
#include "thirdParty/RaspberryPiRC/RPiIBus/RPiIBus.hpp"
#include "thirdparty/RaspberryPiMPU/src/MPU9250/MPU9250.hpp"
#include <thread>

void configWrite(const char *configDir, const char *Target, double obj);
double configSettle(const char *configDir, const char *Target);

int main(int argc, char *argv[])
{
    wiringPiSetupSys();
    int argvs;
    int TimeStart = 0;
    int TimeEnd = 0;
    int TimeNext = 0;
    int TimeMax = 0;
    MPUData myData;
    int IbusData[14] = {1500, 1500, 1000, 1500};
    Ibus myIbusDevice("/dev/ttyAMA0");
    int fd = pca9685Setup(65, 0x40, 300);

    double SpeedA1 = 0;
    double SpeedA2 = 0;
    double SpeedB1 = 0;
    double SpeedB2 = 0;
    double SpeedA1TO = 0;
    double SpeedA2TO = 0;
    double SpeedB1TO = 0;
    double SpeedB2TO = 0;
    int RCForward, RCForwardMax, RCForwardMin, RCForwardMiddle,
        RCHorizontal, RCHorizontalMax, RCHorizontalMin, RCHorizontalMiddle,
        RCYaw, RCYawMax, RCYawMin, RCYawMiddle;

    int ClearCount = 0;

    while ((argvs = getopt(argc, argv, "RTM")) != -1)
    {
        switch (argvs)
        {
        case 'R':
        {
            int a;
            int lose;
            int Middle0, Middle1, Middle2, Middle3;
            double Max[10];
            double Min[10];
            double IbusDataTotal_0 = 0, IbusDataTotal_1 = 0, IbusDataTotal_2 = 0, IbusDataTotal_3 = 0;

            for (int i = 1; i < 500; i++)
            {
                lose = myIbusDevice.IbusRead(IbusData, 4000, 2);
                if (lose != -1)
                {
                    IbusDataTotal_0 += IbusData[0];
                    IbusDataTotal_1 += IbusData[1];
                    IbusDataTotal_2 += IbusData[2];
                    IbusDataTotal_3 += IbusData[3];
                }
                else
                    i--;
            }
            Middle0 = IbusDataTotal_0 / 500.f;
            Middle1 = IbusDataTotal_1 / 500.f;
            Middle2 = IbusDataTotal_2 / 500.f;
            Middle3 = IbusDataTotal_3 / 500.f;
            std::cout << "通道1-4的中间值校准完毕\n";
            std::cout << Middle0 << "   " << Middle1 << "   " << Middle2 << "   " << Middle3 << "\n";
            configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MIDDLE", Middle0);
            configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MIDDLE", Middle1);
            configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_3MIDDLE", Middle2);
            configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MIDDLE", Middle3);

            std::cout << "开始校准IBUS数据\n";
            delay(1000);
            std::cout << "开始校准通道1最大值，请将右遥感推至向右最大，随机输入一个数并按下回车键:"
                      << " \n";
            std::cin >> a;
            myIbusDevice.IbusRead(IbusData, 4000, 2);
            Max[0] = IbusData[0];
            std::cout << "开始校准通道1最小值，请将右遥感推至向左最大，随机输入一个数并按下回车键:"
                      << " \n";
            std::cin >> a;
            myIbusDevice.IbusRead(IbusData, 4000, 2);
            Min[0] = IbusData[0];
            //
            std::cout << "开始校准通道2最大值，请将右遥感推至向上最大，随机输入一个数并按下回车键:"
                      << " \n";
            std::cin >> a;
            myIbusDevice.IbusRead(IbusData, 4000, 2);
            Max[1] = IbusData[1];
            std::cout << "开始校准通道2最小值，请将右遥感推至向下最大，随机输入一个数并按下回车键:"
                      << " \n";
            std::cin >> a;
            myIbusDevice.IbusRead(IbusData, 4000, 2);
            Min[1] = IbusData[1];
            //
            std::cout << "开始校准通道3最大值，请将左遥感推至向上最大，随机输入一个数并按下回车键:"
                      << " \n";
            std::cin >> a;
            myIbusDevice.IbusRead(IbusData, 4000, 2);
            Max[2] = IbusData[2];
            //
            std::cout << "开始校准通道3最小值，请将左遥感推至向下最大，随机输入一个数并按下回车键:"
                      << " \n";
            std::cin >> a;
            myIbusDevice.IbusRead(IbusData, 4000, 2);
            Min[2] = IbusData[2];
            //
            std::cout << "开始校准通道4最大值，请将左遥感推至向右最大，随机输入一个数并按下回车键:"
                      << " \n";
            std::cin >> a;
            myIbusDevice.IbusRead(IbusData, 4000, 2);
            Max[3] = IbusData[3];
            std::cout << "开始校准通道4最小值，请将左遥感推至向左最大，随机输入一个数并按下回车键:"
                      << " \n";
            std::cin >> a;
            myIbusDevice.IbusRead(IbusData, 4000, 2);
            Min[3] = IbusData[3];
            //
            std::cout << "开始校准通道5最大值，请将左旋钮顺时针扭到尽头，随机输入一个数并按下回车键:"
                      << " \n";
            std::cin >> a;
            myIbusDevice.IbusRead(IbusData, 4000, 2);
            Max[4] = IbusData[4];
            std::cout << "开始校准通道5最小值，请将左旋钮逆时针扭到尽头，随机输入一个数并按下回车键:"
                      << " \n";
            std::cin >> a;
            myIbusDevice.IbusRead(IbusData, 4000, 2);
            Min[4] = IbusData[4];
            //
            std::cout << "开始校准通道6最大值，请将右旋钮顺时针扭到尽头，随机输入一个数并按下回车键:"
                      << " \n";
            std::cin >> a;
            myIbusDevice.IbusRead(IbusData, 4000, 2);
            Max[5] = IbusData[5];
            std::cout << "开始校准通道6最小值，请将右旋钮逆时针扭到尽头，随机输入一个数并按下回车键:"
                      << " \n";
            std::cin >> a;
            myIbusDevice.IbusRead(IbusData, 4000, 2);
            Min[5] = IbusData[5];

            for (int i = 0; i < 6; i++)
            {
                std::cout << Max[i] << "  ";
            }
            std::cout << "\n";
            for (int j = 0; j < 6; j++)
            {
                std::cout << Min[j] << "  ";
            }
            std::cout << "\n";

            // configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MAX", Max[0]);
            // configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MAX", Max[1]);
            // configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_3MAX", Max[2]);
            // configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MAX", Max[3]);
            // configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_5MAX", Max[4]);
            // configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_6MAX", Max[5]);

            // configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MIN", Min[0]);
            // configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MIN", Min[1]);
            // configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_3MIN", Min[2]);
            // configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MIN", Min[3]);
            // configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_5MIN", Min[4]);
            // configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_6MIN", Min[5]);

            break;
        }
        case 'M':
        {
            double AccelCaliData[30];
            TimeMax = 1000;
            std::cout << "Start MPU Monitor\n";
            std::cout << "Setting UP MPU9250 ....";
            std::cout.flush();
            std::cout << " Done!\n";
            RPiMPU9250 *myMPUTest = new RPiMPU9250(1, false, 1, 0x68, TimeMax, 0);
            //
            AccelCaliData[MPUAccelCaliX] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_X_Cali");
            AccelCaliData[MPUAccelCaliY] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Y_Cali");
            AccelCaliData[MPUAccelCaliZ] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Z_Cali");
            AccelCaliData[MPUAccelScalX] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_X_Scal");
            AccelCaliData[MPUAccelScalY] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Y_Scal");
            AccelCaliData[MPUAccelScalZ] = configSettle("../MPU9250Car.json", "_Car_MPU9250_A_Z_Scal");
            std::cout << "Calibration Gryo ......";
            std::cout.flush();
            myMPUTest->MPUCalibration(AccelCaliData);
            std::cout << " Done!\n";
            sleep(1);
            //
            myMPUTest->MPUSensorsDataGet();
            myMPUTest->ResetMPUMixAngle();
            //
            system("clear");
            RCForwardMiddle = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MIDDLE");
            RCForwardMin = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MIN");
            RCForwardMax = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MAX");
            RCHorizontalMiddle = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MIDDLE");
            RCHorizontalMin = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MIN");
            RCHorizontalMax = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MAX");
            RCYawMiddle = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MIDDLE");
            RCYawMin = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MIN");
            RCYawMax = configSettle("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MAX");

            std::thread MPUThreading = std::thread([&] {
                while (true)
                {
                    TimeStart = micros();
                    TimeNext = TimeStart - TimeEnd;
                    myData = myMPUTest->MPUSensorsDataGet();
                    TimeEnd = micros();
                    if (TimeMax < ((TimeEnd - TimeStart) + TimeNext) || (TimeNext) < 0)
                        usleep(50);
                    else
                        usleep(TimeMax - (TimeEnd - TimeStart) - TimeNext);
                    TimeEnd = micros();
                }
            });

            std::thread RCThreading = std::thread([&] {
                while (true)
                {
                    int lose = myIbusDevice.IbusRead(IbusData, 4000, 2);
                    if (IbusData[0] < RCHorizontalMiddle + 10 && IbusData[0] > RCHorizontalMiddle - 10)
                        RCHorizontal = 0;
                    else
                        RCHorizontal = IbusData[0] - RCHorizontalMiddle;

                    if (IbusData[1] < RCForwardMiddle + 10 && IbusData[1] > RCForwardMiddle - 10)
                        RCForward = 0;
                    else
                        RCForward = IbusData[1] - RCForwardMiddle;

                    if (IbusData[3] < RCYawMiddle + 10 && IbusData[3] > RCYawMiddle - 10)
                        RCYaw = 0;
                    else
                        RCYaw = IbusData[3] - RCYawMiddle;
                }
            });

            std::thread ESCThreading = std::thread([&] {
                while (true)
                {
                    SpeedA1 = RCForward - RCHorizontal + RCYaw;
                    SpeedA2 = RCForward + RCHorizontal + RCYaw;
                    SpeedB1 = RCForward + RCHorizontal - RCYaw;
                    SpeedB2 = RCForward - RCHorizontal - RCYaw;
                    SpeedA1TO = 200 + (SpeedA1 / 500.f) * (3900.f - 200.f);
                    SpeedA2TO = 200 + (SpeedA2 / 500.f) * (3900.f - 200.f);
                    SpeedB1TO = 200 + (SpeedB1 / 500.f) * (3900.f - 200.f);
                    SpeedB2TO = 200 + (SpeedB2 / 500.f) * (3900.f - 200.f);

                    SpeedA1TO = SpeedA1TO > 3900 ? 3900 : SpeedA1TO;
                    SpeedA2TO = SpeedA2TO > 3900 ? 3900 : SpeedA2TO;
                    SpeedB1TO = SpeedB1TO > 3900 ? 3900 : SpeedB1TO;
                    SpeedB2TO = SpeedB2TO > 3900 ? 3900 : SpeedB2TO;

                    if (SpeedA1 > 0)
                    {
                        pca9685PWMWrite(fd, 14, 0, (int)abs(SpeedA1TO));
                        pca9685PWMWrite(fd, 15, 0, 0);
                    }
                    else
                    {
                        pca9685PWMWrite(fd, 15, 0, (int)abs(SpeedA1TO));
                        pca9685PWMWrite(fd, 14, 0, 0);
                    }

                    if (SpeedA2 > 0)
                    {
                        pca9685PWMWrite(fd, 3, 0, (int)abs(SpeedA2TO));
                        pca9685PWMWrite(fd, 2, 0, 0);
                    }
                    else
                    {
                        pca9685PWMWrite(fd, 2, 0, (int)abs(SpeedA2TO));
                        pca9685PWMWrite(fd, 3, 0, 0);
                    }

                    if (SpeedB1 > 0)
                    {
                        pca9685PWMWrite(fd, 12, 0, (int)abs(SpeedB1TO));
                        pca9685PWMWrite(fd, 13, 0, 0);
                    }
                    else
                    {
                        pca9685PWMWrite(fd, 13, 0, (int)abs(SpeedB1TO));
                        pca9685PWMWrite(fd, 12, 0, 0);
                    }

                    if (SpeedB2 > 0)
                    {
                        pca9685PWMWrite(fd, 1, 0, (int)abs(SpeedB2TO));
                        pca9685PWMWrite(fd, 0, 0, 0);
                    }
                    else
                    {
                        pca9685PWMWrite(fd, 0, 0, (int)abs(SpeedB2TO));
                        pca9685PWMWrite(fd, 1, 0, 0);
                    }
                    usleep(4000);
                }
            });

            std::thread DEBUGThreading = std::thread([&] {
                while (true)
                {
                    ClearCount++;
                    if (ClearCount == 100)
                    {
                        system("clear");
                        ClearCount = 0;
                    }
                    std::cout << "\033[20A";
                    std::cout << "\033[K";
                    std::cout << "Accel Roll: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Accel__Roll << "|"
                              << "AccelPitch: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Accel_Pitch << "| \n";
                    std::cout << "Gryo  Roll: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo__Roll << "|"
                              << "Gryo Pitch: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo_Pitch << "|"
                              << "Gryo   Yaw: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo___Yaw << "| \n";
                    std::cout << "Real  Roll: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Real__Roll << "|"
                              << "Real Pitch: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Real_Pitch << "| \n";
                    for (int i = 0; i < 10; i++)
                    {
                        std::cout << IbusData[i] << " ";
                    }
                    std::cout << "\n";

                    std::cout << "RCForward: " << RCForward << "           \n";
                    std::cout << "RCHor: " << RCHorizontal << "             \n";
                    std::cout << "RCYaw: " << RCYaw << "                \n";
                    std::cout << "speed A1 " << SpeedA1TO << "               \n";
                    std::cout << "speed A2 " << SpeedA2TO << "               \n";
                    std::cout << "speed B1 " << SpeedB1TO << "               \n";
                    std::cout << "speed B2 " << SpeedB2TO << "               \n";
                    usleep(20000);
                }
            });

            DEBUGThreading.join();
        }
        break;
        case 'T':
        {
            while (true)
            {
                int i = 0;
                std::cin >> i;
                pca9685PWMWrite(fd, 1, 0, i);
                pca9685PWMWrite(fd, 0, 0, 0);
            }
        }
        break;
        }
    }
}

double configSettle(const char *configDir, const char *Target)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    return Configdata[Target].get<double>();
}

void configWrite(const char *configDir, const char *Target, double obj)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    Configdata[Target] = obj;
    std::ofstream configs;
    configs.open(configDir);
    configs << std::setw(4) << Configdata << std::endl;
    configs.close();
}

void PID_GryoYaw(float &inputData, float &outputData,
                 float &last_I_Data, float &last_D_Data,
                 float P_Gain, float I_Gain, float D_Gain, float I_Max)
{
    outputData = P_Gain * inputData;
    outputData += D_Gain * (inputData - last_D_Data);
    last_D_Data = inputData;
    last_I_Data += inputData * I_Gain;
    if (last_I_Data > I_Max)
        last_I_Data = I_Max;
    if (last_I_Data < I_Max * -1)
        last_I_Data = I_Max * -1;
    outputData += last_I_Data;
}
