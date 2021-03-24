#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <fstream>
#include <wiringPi.h>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "pca9685.h"
#include "lcd1602.h"
#include "thirdparty/RaspberryPiRC/RPiIBus/RPiIBus.hpp"
#include "thirdparty/RaspberryPiMPU/src/MPU9250/MPU9250.hpp"
#include "thirdparty/QRModule/src/qrscanner.hpp"
#include "thirdparty/RuModule/SRC/_VisionBase/CameraDrive/Drive_V4L2Reader.hpp"
#include "thirdparty/RuModule/SRC/_VisionBase/VisionAIDrive/Drive_OpenCVDN.hpp"
using namespace cv;
void PIDCacl(float &inputData, float &outputData,
             float &last_I_Data, float &last_D_Data,
             float P_Gain, float I_Gain, float D_Gain, float I_Max);
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
    int IbusData[14] = {1500, 1500, 1000, 1500, 0, 0, 0, 0, 1500, 0};
    Ibus myIbusDevice;
    try
    {
        myIbusDevice = Ibus("/dev/ttyAMA0");
    }
    catch (const std::string &e)
    {
        std::cout << e << "\n";
        exit(0);
    }
    // CVInferConfig InferConfigs;
    // InferConfigs.Confidence_Threshold = 0.7;
    // InferConfigs.File_args1 = "../thirdparty/RuModule/Data/vino-banketFP16/frozen_inference_graph.xml";
    // InferConfigs.File_args2 = "../thirdparty/RuModule/Data/vino-banketFP16/frozen_inference_graph.bin";
    // CVInferEngine MyEngine(InferConfigs);
    std::vector<decodedObject> decodeOB;
    int fd = pca9685Setup(65, 0x40, 50);
    int rc = lcd1602Init(1, 0x27);

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
    int RCForward, RCForwardMax, RCForwardMin, RCForwardMiddle,
        RCHorizontal, RCHorizontalMax, RCHorizontalMin, RCHorizontalMiddle,
        RCYaw, RCYawMax, RCYawMin, RCYawMiddle;
    float TotalForward;
    float TotalHorizontal;
    float TotalYaw;

    float PIDYawPGain = 1.5;
    float PIDYawIGain = 0.02;
    float PIDYawDGain = 0.1;
    float PIDYawLastIData = 0;
    float PIDYawLastDData = 0;

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
            //======================================//
            for (size_t i = 0; i < 10; i++)
            {
                Max[i] = Middle0;
            }
            for (size_t i = 0; i < 10; i++)
            {
                Min[i] = Middle0;
            }

            int RCTuneFlag = -1;
            bool RCTuning = true;
            std::thread Tuning = std::thread([&] {
                while (RCTuning)
                {
                    lose = myIbusDevice.IbusRead(IbusData, 4000, 2);
                    if (lose != -1)
                    {
                        Max[0] = Max[0] < IbusData[0] ? IbusData[0] : Max[0];
                        Max[1] = Max[1] < IbusData[1] ? IbusData[1] : Max[1];
                        Max[2] = Max[2] < IbusData[2] ? IbusData[2] : Max[2];
                        Max[3] = Max[3] < IbusData[3] ? IbusData[3] : Max[3];

                        Min[0] = Min[0] > IbusData[0] ? IbusData[0] : Min[0];
                        Min[1] = Min[1] > IbusData[1] ? IbusData[1] : Min[1];
                        Min[2] = Min[2] > IbusData[2] ? IbusData[2] : Min[2];
                        Min[3] = Min[3] > IbusData[3] ? IbusData[3] : Min[3];
                    }
                }
            });

            std::cin >> RCTuneFlag;
            RCTuning = false;
            Tuning.join();

            configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MAX", Max[0]);
            configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MAX", Max[1]);
            configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_3MAX", Max[2]);
            configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MAX", Max[3]);

            configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MIN", Min[0]);
            configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MIN", Min[1]);
            configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_3MIN", Min[2]);
            configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MIN", Min[3]);
            break;
        }
        case 'M':
        {
            TimeMax = 1000;
            RPiMPU9250 *myMPUTest = new RPiMPU9250(1, false, 1, 0x68, TimeMax, 0);
            {
                double AccelCaliData[30];
                std::cout << "Start MPU Monitor\n";
                std::cout << "Setting UP MPU9250 ....";
                std::cout.flush();
                std::cout << " Done!\n";
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
                myMPUTest->MPUSensorsDataGet();
                myMPUTest->ResetMPUMixAngle();
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
            }

            std::thread MPUThreading = std::thread([&] {
                while (true)
                {
                    TimeStart = micros();
                    TimeNext = TimeStart - TimeEnd;
                    myData = myMPUTest->MPUSensorsDataGet();
                    SPEED_X = SPEED_X + (myData._uORB_Acceleration_X * 0.001);
                    SPEED_Y = SPEED_Y + (myData._uORB_Acceleration_Y * 0.001);

                    {
                        TotalForward = RCForward;
                        TotalHorizontal = RCHorizontal;
                        TotalYaw = RCYaw;
                        // float YawInput = (RCYaw + myData._uORB_Gryo___Yaw * 5);
                        // YawInput = YawInput > 500.f ? 500.f : YawInput;
                        // PIDCacl(YawInput, TotalYaw, PIDYawLastIData,
                        //         PIDYawLastDData, PIDYawPGain, PIDYawIGain, PIDYawDGain, 80.f);
                    }

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
            std::thread RCservo = std::thread([&] {
                while (true)
                {
                    int RC_data = myIbusDevice.IbusRead(IbusData, 4000, 2);
                    if (IbusData[6] > 1900)
                    {
                        for (int i = 250; i <= 270; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 400; i >= 300; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 120; i <= 300; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 300; i >= 140; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 0; i <= 100; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 4, 0, i);
                        }

                        for (int i = 270; i <= 350; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 300; i <= 480; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 140; i <= 230; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        pca9685PWMWrite(fd, 8, 0, 390);
                        delay(1000);
                        for (int i = 100; i <= 150; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 4, 0, i);
                        }
                        for (int i = 350; i >= 270; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 230; i <= 300; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 480; i >= 300; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        pca9685PWMWrite(fd, 8, 0, 410);
                        for (int i = 300; i <= 400; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 300; i >= 120; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 270; i >= 250; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                    }
                    if (IbusData[7] > 1900)
                    {
                        for (int i = 250; i <= 270; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 400; i >= 300; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 120; i <= 300; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 300; i >= 140; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 0; i <= 100; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 4, 0, i);
                        }

                        for (int i = 270; i <= 350; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 300; i <= 400; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 140; i <= 230; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 410; i >= 360; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 8, 0, i);
                        }
                        for (int i = 230; i <= 380; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 400; i >= 90; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 350; i >= 250; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 100; i <= 150; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 4, 0, i);
                        }
                        for (int i = 250; i <= 380; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 380; i >= 300; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 90; i <= 300; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 360; i <= 410; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 8, 0, i);
                        }
                        for (int i = 300; i <= 400; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 300; i >= 120; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 270; i >= 250; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                    }
                    if (IbusData[9] > 1900)
                    {
                        for (int i = 250; i <= 270; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 400; i >= 300; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 120; i <= 300; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 300; i >= 140; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 0; i <= 100; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 4, 0, i);
                        }

                        for (int i = 300; i <= 350; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 300; i <= 400; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 140; i <= 330; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 410; i <= 490; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 8, 0, i);
                        }
                        for (int i = 400; i >= 120; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 350; i >= 250; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 100; i <= 150; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 4, 0, i);
                        }
                        for (int i = 250; i <= 270; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 330; i >= 300; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 120; i <= 300; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 490; i >= 410; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 8, 0, i);
                        }
                        for (int i = 300; i <= 400; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 300; i >= 120; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 270; i >= 250; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                    }
                    if (IbusData[4] > 1200)
                    {
                        for (int i = 410; i >= 360; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 8, 0, i);
                        }
                        for (int i = 400; i >= 370; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 120; i >= 90; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 270; i >= 250; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }

                        for (int i = 0; i <= 100; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 4, 0, i);
                        }
                        delay(1000);
                        for (int i = 90; i <= 170; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 250; i <= 270; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 360; i <= 410; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 8, 0, i);
                        }

                        for (int i = 270; i <= 360; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 170; i <= 210; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 370; i >= 130; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 100; i <= 150; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 4, 0, i);
                        }
                        for (int i = 210; i <= 400; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 130; i <= 300; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 400; i >= 300; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 360; i >= 270; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 300; i <= 400; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 300; i >= 120; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 270; i >= 250; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                    }
                    if (IbusData[5] > 1200)
                    {
                        for (int i = 410; i <= 490; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 8, 0, i);
                        }
                        for (int i = 400; i >= 320; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 0; i <= 100; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 4, 0, i);
                        }
                        delay(1000);
                        for (int i = 320; i <= 350; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 250; i <= 380; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 120; i <= 200; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 490; i >= 410; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 8, 0, i);
                        }
                        for (int i = 350; i >= 130; i--)
                        {
                            delay(10);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 100; i <= 150; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 4, 0, i);
                        }

                        for (int i = 380; i <= 400; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 200; i <= 400; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 130; i <= 300; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        delay(1000);
                        for (int i = 300; i <= 400; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 300; i >= 120; i--)
                        {
                            delay(10);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 400; i >= 250; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                    }
                    if (IbusData[2] > 1900)
                    {
                        for (int i = 410; i >= 400; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 8, 0, i);
                        }
                        for (int i = 250; i <= 270; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 400; i >= 300; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 120; i <= 300; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 300; i >= 220; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 300; i <= 500; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 270; i <= 320; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 0; i <= 100; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 4, 0, i);
                        }
                        delay(1000);
                        for (int i = 320; i >= 270; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 500; i >= 480; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 400; i <= 410; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 8, 0, i);
                        }
                        for (int i = 220; i <= 300; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 380; i >= 200; i--)
                        {
                            delay(10);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }

                        for (int i = 250; i <= 370; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 300; i >= 130; i--)
                        {
                            delay(10);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 100; i <= 150; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 4, 0, i);
                        }
                        for (int i = 370; i <= 400; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                        for (int i = 200; i <= 400; i++)
                        {
                            delay(10);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 130; i <= 300; i++)
                        {
                            delay(10);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 300; i <= 400; i++)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 7, 0, i);
                        }
                        for (int i = 300; i >= 120; i--)
                        {
                            delay(10);
                            pca9685PWMWrite(fd, 6, 0, i);
                        }
                        for (int i = 400; i >= 250; i--)
                        {
                            delay(5);
                            pca9685PWMWrite(fd, 5, 0, i);
                        }
                    }
                }
            });
            std::thread QRcamer = std::thread([&] {
                QRSCanner myScanner;
                //std::vector<decodedObject> decodeOB;
                // cv::VideoCapture cap(0);
                V4L2Tools::V4L2Drive cap("/dev/video0", {.ImgWidth = 256,
                                                         .ImgHeight = 192,
                                                         .FrameBuffer = 1,
                                                         .Is_fastMode = true,
                                                         .PixFormat = V4L2_PIX_FMT_YUYV});
                cap.V4L2Control(V4L2_CID_ZOOM_ABSOLUTE, 0x8005);
                usleep(50000);
                cap.V4L2Control(V4L2_CID_ZOOM_ABSOLUTE, 0x8802);
                cv::namedWindow("test", cv::WINDOW_NORMAL);
                cv::setWindowProperty("test", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
                unsigned char *tmpDataBGR = cap.RGB24DataInit();
                while (true)
                {
                    unsigned char *tmpData = cap.V4L2Read();
                    cap.yuyv2rgb24(tmpData, tmpDataBGR, 256, 192);
                    cv::Mat tmpMat = cv::Mat(192, 256, CV_8UC3, tmpDataBGR);
                    // cap >> tmpMat;
                    cv::resize(tmpMat, tmpMat, cv::Size(800, 600));
                    cv::rotate(tmpMat, tmpMat, cv::ROTATE_180);

                    decodeOB = myScanner.QRCodeDecoder(tmpMat);
                    tmpMat = QRSCanner::QRCodeDrawer(decodeOB, tmpMat);

                    imshow("test", tmpMat);
                    cv::waitKey(10);
                }
            });
            std::thread ESCThreading = std::thread([&] {
                while (true)
                {
                    SpeedA1 = TotalForward - TotalHorizontal - TotalYaw;
                    SpeedA2 = TotalForward + TotalHorizontal + TotalYaw;
                    SpeedB1 = TotalForward + TotalHorizontal - TotalYaw;
                    SpeedB2 = TotalForward - TotalHorizontal + TotalYaw;
                    SpeedA1TO = (SpeedA1 / 500.f) * 3900.f;
                    SpeedA2TO = (SpeedA2 / 500.f) * 3900.f;
                    SpeedB1TO = (SpeedB1 / 500.f) * 3900.f;
                    SpeedB2TO = (SpeedB2 / 500.f) * 3900.f;

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

            // std::thread CameraThreading = std::thread([&] {
            //     cv::VideoCapture cap(0);
            //     cv::namedWindow("test", cv::WINDOW_NORMAL);
            //     cv::setWindowProperty("test", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
            //     while (true)
            //     {
            //         cv::Mat TmpMat;
            //         cap.read(TmpMat);
            //         TmpMat = MyEngine.CVInferMatSync(TmpMat);
            //         cv::resize(TmpMat, TmpMat, cv::Size(800, 600));
            //         cv::rotate(TmpMat, TmpMat, cv::ROTATE_180);
            //         cv::imshow("test", TmpMat);
            //         cv::waitKey(10);
            //     }
            // });

            std::thread CamerHSV = std::thread([&] {

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
                    std::cout << "AccelX    : " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Acceleration_X << "cm/s2|"
                              << "AccelY    : " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Acceleration_Y << "cm/s2|"
                              << "AccelZ    : " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Acceleration_Z << "cm/s2| \n";
                    std::cout << "SPPED X   : " << std::setw(7) << std::setfill(' ') << SPEED_X << "cm/s|"
                              << "SPPED Y   : " << std::setw(7) << std::setfill(' ') << SPEED_Y << "cm/s|\n";

                    for (int i = 0; i < 10; i++)
                    {
                        std::cout << IbusData[i] << " ";
                    }
                    std::cout << "\n";

                    std::cout << "RCForward: " << TotalForward << "           \n";
                    std::cout << "RCHor: " << TotalHorizontal << "             \n";
                    std::cout << "RCYaw: " << TotalYaw << "                \n";
                    std::cout << "speed A1 " << SpeedA1TO << "               \n";
                    std::cout << "speed A2 " << SpeedA2TO << "               \n";
                    std::cout << "speed B1 " << SpeedB1TO << "               \n";
                    std::cout << "speed B2 " << SpeedB2TO << "               \n";
                    for (size_t i = 0; i < decodeOB.size(); i++)
                    {
                        //lcd1602WriteString((char)decodeOB[i].data);
                        std::cout << "QRSanner:" << decodeOB[i].data << "\n";
                    }
                    usleep(20000);
                }
            });

            DEBUGThreading.join();
        }
        break;
        case 'T':
        {
            VideoCapture cap(0);
            namedWindow("Control", cv::WindowFlags::WINDOW_AUTOSIZE);

            int iLowH = 0;
            int iHighH = 179;

            int iLowS = 0;
            int iHighS = 255;

            int iLowV = 0;
            int iHighV = 255;

            createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
            createTrackbar("HighH", "Control", &iHighH, 179);

            createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
            createTrackbar("HighS", "Control", &iHighS, 255);

            createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
            createTrackbar("HighV", "Control", &iHighV, 255);

            while (true)
            {
                Mat imgOriginal;

                bool bSuccess = cap.read(imgOriginal);

                if (!bSuccess)
                {
                    std::cout << "Cannot read a frame from video stream"
                              << "\n";
                    break;
                }

                Mat imgHSV;

                cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

                Mat imgThresholded;

                inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

                erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
                dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

                dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
                erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

                imshow("Thresholded Image", imgThresholded);
                imshow("Original", imgOriginal);

                if (waitKey(30) == 27)
                {
                    std::cout << "esc key is pressed by user"
                              << "\n";
                    break;
                }
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

void PIDCacl(float &inputData, float &outputData,
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
