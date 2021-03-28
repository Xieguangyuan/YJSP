#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <fstream>
#include <wiringPi.h>
#include <thread>
#include <opencv2/opencv.hpp>
#include "YJSP_Schedule.hpp"

#include "pca9685.h"
#include "lcd1602.h"

#include "thirdparty/QRModule/src/qrscanner.hpp"
#include "thirdparty/RuModule/SRC/_VisionBase/CameraDrive/Drive_V4L2Reader.hpp"
#include "thirdparty/RuModule/SRC/_VisionBase/VisionAIDrive/Drive_OpenCVDN.hpp"
using namespace cv;
void configWrite(const char *configDir, const char *Target, double obj);
double configSettle(const char *configDir, const char *Target);

int main(int argc, char *argv[])
{
    wiringPiSetup();
    // CVInferConfig InferConfigs;
    // InferConfigs.Confidence_Threshold = 0.8;
    // InferConfigs.File_args1 = "../thirdparty/RuModule/Data/vino-banketFP16/frozen_inference_graph.xml";
    // InferConfigs.File_args2 = "../thirdparty/RuModule/Data/vino-banketFP16/frozen_inference_graph.bin";
    // CVInferEngine MyEngine(InferConfigs);
    std::vector<decodedObject> decodeOB;
    int rc = lcd1602Init(1, 0x27);
    int argvs = 0;

    while ((argvs = getopt(argc, argv, "RTMF")) != -1)
    {
        switch (argvs)
        {
        case 'R':
        {
            //     int a;
            //     int lose;
            //     int Middle0, Middle1, Middle2, Middle3;
            //     double Max[10];
            //     double Min[10];
            //     double IbusDataTotal_0 = 0, IbusDataTotal_1 = 0, IbusDataTotal_2 = 0, IbusDataTotal_3 = 0;

            //     for (int i = 1; i < 500; i++)
            //     {
            //         lose = myIbusDevice.IbusRead(IbusData, 4000, 2);
            //         if (lose != -1)
            //         {
            //             IbusDataTotal_0 += IbusData[0];
            //             IbusDataTotal_1 += IbusData[1];
            //             IbusDataTotal_2 += IbusData[2];
            //             IbusDataTotal_3 += IbusData[3];
            //         }
            //         else
            //             i--;
            //     }
            //     Middle0 = IbusDataTotal_0 / 500.f;
            //     Middle1 = IbusDataTotal_1 / 500.f;
            //     Middle2 = IbusDataTotal_2 / 500.f;
            //     Middle3 = IbusDataTotal_3 / 500.f;
            //     std::cout << "通道1-4的中间值校准完毕\n";
            //     std::cout << Middle0 << "   " << Middle1 << "   " << Middle2 << "   " << Middle3 << "\n";
            //     configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MIDDLE", Middle0);
            //     configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MIDDLE", Middle1);
            //     configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_3MIDDLE", Middle2);
            //     configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MIDDLE", Middle3);
            //     //======================================//
            //     for (size_t i = 0; i < 10; i++)
            //     {
            //         Max[i] = Middle0;
            //     }
            //     for (size_t i = 0; i < 10; i++)
            //     {
            //         Min[i] = Middle0;
            //     }

            //     int RCTuneFlag = -1;
            //     bool RCTuning = true;
            //     std::thread Tuning = std::thread([&] {
            //         while (RCTuning)
            //         {
            //             lose = myIbusDevice.IbusRead(IbusData, 4000, 2);
            //             if (lose != -1)
            //             {
            //                 Max[0] = Max[0] < IbusData[0] ? IbusData[0] : Max[0];
            //                 Max[1] = Max[1] < IbusData[1] ? IbusData[1] : Max[1];
            //                 Max[2] = Max[2] < IbusData[2] ? IbusData[2] : Max[2];
            //                 Max[3] = Max[3] < IbusData[3] ? IbusData[3] : Max[3];

            //                 Min[0] = Min[0] > IbusData[0] ? IbusData[0] : Min[0];
            //                 Min[1] = Min[1] > IbusData[1] ? IbusData[1] : Min[1];
            //                 Min[2] = Min[2] > IbusData[2] ? IbusData[2] : Min[2];
            //                 Min[3] = Min[3] > IbusData[3] ? IbusData[3] : Min[3];
            //             }
            //         }
            //     });

            //     std::cin >> RCTuneFlag;
            //     RCTuning = false;
            //     Tuning.join();

            //     configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MAX", Max[0]);
            //     configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MAX", Max[1]);
            //     configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_3MAX", Max[2]);
            //     configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MAX", Max[3]);

            //     configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_1MIN", Min[0]);
            //     configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_2MIN", Min[1]);
            //     configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_3MIN", Min[2]);
            //     configWrite("../MPU9250Car.json", "_Car_IBUSDATA_Channel_4MIN", Min[3]);
            //     break;
        }
        break;
        case 'M':
        {
            YJSP_AP::YJSP SPGO;
            SPGO.YJSP_Init();
            SPGO.MPUThreadREG();
            SPGO.RCThreadREG();
            SPGO.ESCThreadREG();
            SPGO.DEBUGThreadREG();

            std::thread RCservo = std::thread([&] {
                //         while (true)
                //         {
                //             int RC_data = myIbusDevice.IbusRead(IbusData, 4000, 2);
                //             if (IbusData[6] > 1900)
                //             {
                //                 for (int i = 250; i <= 270; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 400; i >= 300; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 120; i <= 300; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 300; i >= 140; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 0; i <= 100; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }

                //                 for (int i = 270; i <= 350; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 300; i <= 480; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 140; i <= 230; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 pca9685PWMWrite(fd, 8, 0, 390);
                //                 delay(1000);
                //                 for (int i = 100; i <= 150; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }
                //                 for (int i = 350; i >= 270; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 230; i <= 300; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 480; i >= 300; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 pca9685PWMWrite(fd, 8, 0, 410);
                //                 for (int i = 300; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 300; i >= 120; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 270; i >= 250; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //             }
                //             if (IbusData[7] > 1900)
                //             {
                //                 for (int i = 250; i <= 270; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 400; i >= 300; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 120; i <= 300; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 300; i >= 140; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 0; i <= 100; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }

                //                 for (int i = 270; i <= 350; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 300; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 140; i <= 230; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 410; i >= 360; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }
                //                 for (int i = 230; i <= 380; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 400; i >= 90; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 350; i >= 250; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 100; i <= 150; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }
                //                 for (int i = 250; i <= 380; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 380; i >= 300; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 90; i <= 300; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 360; i <= 410; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }
                //                 for (int i = 300; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 300; i >= 120; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 270; i >= 250; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //             }
                //             if (IbusData[9] > 1900)
                //             {
                //                 for (int i = 250; i <= 270; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 400; i >= 300; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 120; i <= 300; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 300; i >= 140; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 0; i <= 100; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }

                //                 for (int i = 300; i <= 350; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 300; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 140; i <= 330; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 410; i <= 490; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }
                //                 for (int i = 400; i >= 120; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 350; i >= 250; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 100; i <= 150; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }
                //                 for (int i = 250; i <= 270; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 330; i >= 300; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 120; i <= 300; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 490; i >= 410; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }
                //                 for (int i = 300; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 300; i >= 120; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 270; i >= 250; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //             }
                //             if (IbusData[4] > 1200)
                //             {
                //                 for (int i = 410; i >= 360; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }
                //                 for (int i = 400; i >= 370; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 120; i >= 90; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 270; i >= 250; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }

                //                 for (int i = 0; i <= 100; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }
                //                 delay(1000);
                //                 for (int i = 90; i <= 170; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 250; i <= 270; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 360; i <= 410; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }

                //                 for (int i = 270; i <= 360; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 170; i <= 210; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 370; i >= 130; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 100; i <= 150; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }
                //                 for (int i = 210; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 130; i <= 300; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 400; i >= 300; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 360; i >= 270; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 300; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 300; i >= 120; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 270; i >= 250; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //             }
                //             if (IbusData[5] > 1200)
                //             {
                //                 for (int i = 410; i <= 490; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }
                //                 for (int i = 400; i >= 320; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 0; i <= 100; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }
                //                 delay(1000);
                //                 for (int i = 320; i <= 350; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 250; i <= 380; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 120; i <= 200; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 490; i >= 410; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }
                //                 for (int i = 350; i >= 130; i--)
                //                 {
                //                     delay(10);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 100; i <= 150; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }

                //                 for (int i = 380; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 200; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 130; i <= 300; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 delay(1000);
                //                 for (int i = 300; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 300; i >= 120; i--)
                //                 {
                //                     delay(10);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 400; i >= 250; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //             }
                //             if (IbusData[2] > 1900)
                //             {
                //                 for (int i = 410; i >= 400; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }
                //                 for (int i = 250; i <= 270; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 400; i >= 300; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 120; i <= 300; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 300; i >= 220; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 300; i <= 500; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 270; i <= 320; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 0; i <= 100; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }
                //                 delay(1000);
                //                 for (int i = 320; i >= 270; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 500; i >= 480; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 400; i <= 410; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }
                //                 for (int i = 220; i <= 300; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 380; i >= 200; i--)
                //                 {
                //                     delay(10);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }

                //                 for (int i = 250; i <= 370; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 300; i >= 130; i--)
                //                 {
                //                     delay(10);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 100; i <= 150; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }
                //                 for (int i = 370; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 200; i <= 400; i++)
                //                 {
                //                     delay(10);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 130; i <= 300; i++)
                //                 {
                //                     delay(10);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 300; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 300; i >= 120; i--)
                //                 {
                //                     delay(10);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 400; i >= 250; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //             }
                //             if (IbusData[3] > 1900)
                //             {
                //                 pca9685PWMWrite(fd, 8, 0, 410);
                //                 for (int i = 250; i <= 370; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 300; i >= 200; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 400; i >= 130; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 0; i <= 100; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }
                //                 for (int i = 370; i >= 350; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 410; i >= 400; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }
                //                 for (int i = 200; i <= 480; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 130; i <= 240; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 100; i <= 150; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }
                //                 for (int i = 350; i >= 250; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 240; i <= 300; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 480; i >= 300; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 300; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 300; i >= 120; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //             }
                //             if (IbusData[1] > 1900)
                //             {
                //                 pca9685PWMWrite(fd, 8, 0, 410);
                //                 for (int i = 250; i <= 370; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 300; i >= 200; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 400; i >= 130; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 0; i <= 100; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }
                //                 for (int i = 200; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 130; i <= 330; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 410; i <= 490; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }
                //                 for (int i = 400; i >= 130; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 370; i >= 220; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 100; i <= 150; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }
                //                 for (int i = 220; i <= 250; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 130; i <= 300; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 330; i >= 300; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 490; i >= 410; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }
                //                 for (int i = 300; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 300; i >= 120; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //             }
                //             if (IbusData[0] > 1900)
                //             {
                //                 pca9685PWMWrite(fd, 8, 0, 410);
                //                 for (int i = 250; i <= 370; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 300; i >= 200; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 400; i >= 130; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 0; i <= 100; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }
                //                 for (int i = 200; i <= 350; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 130; i <= 380; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }
                //                 for (int i = 410; i >= 355; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }
                //                 for (int i = 350; i >= 110; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //                 for (int i = 370; i >= 220; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }

                //                 for (int i = 100; i <= 150; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 4, 0, i);
                //                 }
                //                 for (int i = 220; i <= 250; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 5, 0, i);
                //                 }
                //                 for (int i = 110; i <= 300; i++)
                //                 {
                //                     delay(10);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }

                //                 for (int i = 380; i >= 300; i--)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }

                //                 for (int i = 355; i <= 410; i++)
                //                 {
                //                     delay(10);
                //                     pca9685PWMWrite(fd, 8, 0, i);
                //                 }

                //                 for (int i = 300; i <= 400; i++)
                //                 {
                //                     delay(5);
                //                     pca9685PWMWrite(fd, 7, 0, i);
                //                 }

                //                 for (int i = 300; i >= 120; i--)
                //                 {
                //                     delay(10);
                //                     pca9685PWMWrite(fd, 6, 0, i);
                //                 }
                //             }
                //         }
            });

            std::thread QRcamer = std::thread([&] {
                QRSCanner myScanner;
                cv::Mat tmpMat;
                cv::VideoCapture cap(0);
                cv::namedWindow("test", cv::WINDOW_NORMAL);
                cv::setWindowProperty("test", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

                while (true)
                {
                    cap >> tmpMat;
                    cv::resize(tmpMat, tmpMat, cv::Size(800, 600));
                    cv::rotate(tmpMat, tmpMat, cv::ROTATE_180);

                    decodeOB = myScanner.QRCodeDecoder(tmpMat);
                    tmpMat = QRSCanner::QRCodeDrawer(decodeOB, tmpMat);

                    imshow("test", tmpMat);
                    cv::waitKey(10);
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
        }
        break;
        case 'F':
        {
            V4L2Tools::V4L2Drive cap("/dev/video0", {.ImgWidth = 256,
                                                     .ImgHeight = 192,
                                                     .FrameBuffer = 1,
                                                     .Is_fastMode = true,
                                                     .PixFormat = V4L2_PIX_FMT_YUYV});
            cap.V4L2Control(V4L2_CID_ZOOM_ABSOLUTE, 0x8005);
            usleep(50000);
            cap.V4L2Control(V4L2_CID_ZOOM_ABSOLUTE, 0x8801);
            cv::namedWindow("test", cv::WINDOW_NORMAL);
            cv::setWindowProperty("test", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
            unsigned char *tmpDataBGR = cap.RGB24DataInit();
            while (true)
            {
                unsigned char *tmpData = cap.V4L2Read();
                cap.yuyv2rgb24(tmpData, tmpDataBGR, 256, 192);
                cv::Mat tmpMat = cv::Mat(192, 256, CV_8UC3, tmpDataBGR);

                cv::resize(tmpMat, tmpMat, cv::Size(800, 600));
                //cv::rotate(tmpMat, tmpMat, cv::ROTATE_180);

                imshow("test", tmpMat);
                cv::waitKey(10);
            }
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
                cv::rotate(imgOriginal, imgOriginal, cv::ROTATE_180);

                cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

                Mat imgThresholded;

                inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded);

                erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(12, 12)));  //腐蚀
                dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(18, 18))); //膨胀

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