#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <fstream>
#include <wiringPi.h>
#include <queue>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include "YJSP_Schedule.hpp"
#include "saftyMat.hpp"
#include "pca9685.h"
#include "lcd1602.h"
#include "VL53L1XDev.hpp"
#include "thirdparty/RuModule/SRC/_Excutable/Drive_Json.hpp"
#include "thirdparty/QRModule/src/qrscanner.hpp"
#include "thirdparty/RuModule/SRC/_Excutable/Drive_Json.hpp"
#include "thirdparty/RuModule/SRC/_VisionBase/CameraDrive/Drive_V4L2Reader.hpp"
#include "thirdparty/RuModule/SRC/_VisionBase/VisionAIDrive/Drive_OpenCVDN.hpp"
using namespace cv;
using json = nlohmann::json;
double MAX_Area = 0;
float YawOutput = 0;
float CXOutput = 0;
float CXOutputILast = 0;
float CXOutputDLast = 0;
double CXInput = 280;
int cx = 0;
int cy = 0;

void configWrite(const char *configDir, const char *Target, double obj);
double configSettle(const char *configDir, const char *Target);
void PIDCaclOut(float inputDataP, float inputDataI, float inputDataD, float &outputData,
                float &last_I_Data, float &last_D_Data,
                float P_Gain, float I_Gain, float D_Gain, float I_Max);

int main(int argc, char *argv[])
{
    wiringPiSetup();
    pinMode(29, OUTPUT);
    // CVInferConfig InferConfigs;
    // InferConfigs.Confidence_Threshold = 0.8;
    // InferConfigs.File_args1 = "../thirdparty/RuModule/Data/vino-banketFP16/frozen_inference_graph.xml";
    // InferConfigs.File_args2 = "../thirdparty/RuModule/Data/vino-banketFP16/frozen_inference_graph.bin";
    // CVInferEngine MyEngine(InferConfigs);

    int rc = lcd1602Init(1, 0x27);
    int argvs = 0;

    while ((argvs = getopt(argc, argv, "RTMFY")) != -1)
    {
        switch (argvs)
        {
        case 'R':
        {
            cv::VideoCapture cap(0);
            cv::Mat src;
            cv::Mat tmp;
            while (true)
            {
                cap >> src;
                resize(src, src, cv::Size(640, 480));
                rotate(src, src, cv::ROTATE_180);
                cvtColor(src, tmp, COLOR_BGR2HSV);
                inRange(tmp, Scalar(90, 160, 0), Scalar(100, 255, 255), tmp); //blue               Scalar(LOW_H,LOW_S,LOW_V),Scalar(HIGH_H,HIGH_S,HIGH_V)
                //inRange(tmp, Scalar(60, 80, 70), Scalar(90, 140, 255), tmp); //green
                //inRange(tmp, Scalar(150, 160, 0), Scalar(179, 255, 255), tmp); //red

                erode(tmp, tmp, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));    //腐蚀
                dilate(tmp, tmp, getStructuringElement(MORPH_ELLIPSE, Size(20, 20))); //膨胀

                std::vector<std::vector<cv::Point>> contours;
                findContours(tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                std::cout << contours.size() << "\n";
                MAX_Area = 0;
                int cx = 0;
                int cy = 0;
                int upY = INT_MAX, lowY = 0, upX, lowX;
                for (int i = 0; i < contours.size(); i++)
                {
                    if (contourArea(contours[i]) > 100)
                    {
                        if (contourArea(contours[i]) > MAX_Area)
                        {
                            MAX_Area = contourArea(contours[i]);
                            for (int j = 0; j < contours[i].size(); j++)
                            {
                                if (contours[i][j].y > lowY)
                                {
                                    lowY = contours[i][j].y;
                                    lowX = contours[i][j].x;
                                }
                                if (contours[i][j].y < upY)
                                {
                                    upY = contours[i][j].y;
                                    upX = contours[i][j].x;
                                }
                            }
                            std::vector<cv::Moments> mu(contours.size());
                            mu[i] = moments(contours[i], false);
                            cx = mu[i].m10 / mu[i].m00;
                            cy = mu[i].m01 / mu[i].m00;
                        }
                    }
                }

                std::cout << "low = (" << lowX << ", " << lowY << ")" << std::endl
                          << "up  = (" << upX << ", " << upY << ")" << std::endl;
                std::cout << "cx:" << cx << ","
                          << "cy" << cy << std::endl;
                circle(src, Point(cx, cy), 10, Scalar(255, 0, 255));

                imshow("Window", src);
                imshow("test", tmp);
                waitKey(10);
            }
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
            cv::Mat src;
            cv::Mat tmp;
            cv::Mat QRTmp;
            int HSVTrack = -1;
            bool MissionSet = false;
            bool StartTracking = false;
            std::queue<int> MissionQueue;
            bool MissonStartFlag = false;
            std::string dataBuffer[256];
            std::queue<std::string> dataQueue;
            FrameBuffer<cv::Mat> syncBuffer;

            std::vector<decodedObject> decodeOB;
            cv::VideoCapture cap(0);

            cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
            QRSCanner myScanner;

            cv::namedWindow("AICamer", cv::WINDOW_NORMAL);
            cv::setWindowProperty("AICamer", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

            YJSP_AP::YJSP SPGO;
            SPGO.YJSP_Init();
            SPGO.MPUThreadREG();
            SPGO.RCThreadREG();
            SPGO.ESCThreadREG();
            SPGO.PositionThreadREG();

            std::thread QRcamer = std::thread([&] {
                while (true)
                {
                    cap >> src;
                    rotate(src, src, cv::ROTATE_180);
                    syncBuffer.pushFrame(src);
                    cvtColor(src, tmp, COLOR_BGR2HSV);
                    if (StartTracking && MissionSet)
                    {
                        if (MissionQueue.front() != '+')
                        {
                            HSVTrack = MissionQueue.front();
                        }

                        MissionQueue.pop();
                        MissionSet = false;
                    }

                    if (HSVTrack == '1')
                    {
                        SPGO.TF.H_LOW = 150, SPGO.TF.S_LOW = 0, SPGO.TF.V_LOW = 0;
                        SPGO.TF.H_HIGH = 179;
                        SPGO.TF.S_HIGH = 255;
                        SPGO.TF.V_HIGH = 255;
                    }
                    else if (HSVTrack == '2')
                    {
                        SPGO.TF.H_LOW = 60, SPGO.TF.S_LOW = 10, SPGO.TF.V_LOW = 00;
                        SPGO.TF.H_HIGH = 75;
                        SPGO.TF.S_HIGH = 255;
                        SPGO.TF.V_HIGH = 255;
                    }
                    else if (HSVTrack == '3')
                    {
                        SPGO.TF.H_LOW = 100, SPGO.TF.S_LOW = 120, SPGO.TF.V_LOW = 0;
                        SPGO.TF.H_HIGH = 130;
                        SPGO.TF.S_HIGH = 210;
                        SPGO.TF.V_HIGH = 255;
                    }

                    if (HSVTrack != -1)
                    {
                        inRange(tmp, Scalar(SPGO.TF.H_LOW, SPGO.TF.S_LOW, SPGO.TF.V_LOW), Scalar(SPGO.TF.H_HIGH, SPGO.TF.S_HIGH, SPGO.TF.V_HIGH), tmp); //red //Scalar(LOW_H,LOW_S,LOW_V),Scalar(HIGH_H,HIGH_S,HIGH_V)

                        erode(tmp, tmp, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));    //腐蚀
                        dilate(tmp, tmp, getStructuringElement(MORPH_ELLIPSE, Size(20, 20))); //膨胀

                        std::vector<std::vector<cv::Point>> contours;
                        findContours(tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                        MAX_Area = 0;
                        int upY = INT_MAX, lowY = 0, upX, lowX;
                        for (int i = 0; i < contours.size(); i++)
                        {
                            if (contourArea(contours[i]) > 100)
                            {
                                if (contourArea(contours[i]) > MAX_Area)
                                {
                                    MAX_Area = contourArea(contours[i]);
                                    for (int j = 0; j < contours[i].size(); j++)
                                    {
                                        if (contours[i][j].y > lowY)
                                        {
                                            lowY = contours[i][j].y;
                                            lowX = contours[i][j].x;
                                        }
                                        if (contours[i][j].y < upY)
                                        {
                                            upY = contours[i][j].y;
                                            upX = contours[i][j].x;
                                        }
                                    }
                                    std::vector<cv::Moments> mu(contours.size());
                                    mu[i] = moments(contours[i], false);
                                    cx = mu[i].m10 / mu[i].m00;
                                    cy = mu[i].m01 / mu[i].m00;
                                }
                            }
                            //
                            CXInput = cx - 280;
                            PIDCaclOut(CXInput, CXInput, CXInput, CXOutput, CXOutputILast, CXOutputDLast, 0.5, 0, 0, 100.f);
                            SPGO.UserInput(CXOutput, 0, 0, 0, false);
                        }
                    }
                    circle(src, Point(cx, cy), 10, Scalar(0, 0, 0));
                    imshow("AICamer", src);
                    cv::waitKey(10);
                }
            });

            std::thread QRDefine = std::thread([&] {
                sleep(1);
                while (true)
                {
                    if (syncBuffer.size() > 0)
                    {
                        QRTmp = syncBuffer.getFrame();
                        if (!QRTmp.empty())
                        {
                            decodeOB = myScanner.QRCodeDecoder(QRTmp);
                            QRTmp = QRSCanner::QRCodeDrawer(decodeOB, QRTmp);
                            if (decodeOB.size() > 0)
                            {
                                char *Data = (char *)decodeOB[0].data.c_str();
                                lcd1602SetCursor(9, 0);
                                lcd1602WriteString(Data);
                                digitalWrite(29, HIGH);
                                delay(500);
                                digitalWrite(29, LOW);
                                for (int i = 0; i < decodeOB[0].data.size(); i++)
                                {
                                    std::cout << Data[i] << "\n";
                                    MissionQueue.push(Data[i]);
                                }
                            }
                        }
                    }
                    if (syncBuffer.size() > 15)
                        syncBuffer.clearBuffer();
                    usleep(200000);
                }
            });

            std::thread Mission = std::thread([&] {
                //lcd1602Clear();
                SPGO.Servo_Scanner();
                sleep(1);
                SPGO.UserInput(0, 0, 0, 0, false);
                while (!MissonStartFlag)
                    usleep(50000);
                SPGO.UserLocationSet(SPGO.EF.MOVE_X, SPGO.EF.MOVE_Y);
                // while (!(38 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 42))
                //     usleep(50000);
                SPGO.UserLocationSet(175, 30);
                SPGO.QRScanner();
                //===========================================================
                SPGO.UserLocationSet(50, 28);
                while (!(47 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 53 && 25 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 30))
                    usleep(50000);
                StartTracking = true;
                MissionSet = true;
                sleep(2);
                while (!(-50 < CXInput && CXInput < 50 && 5 > SPGO.EF.SPEED_X && SPGO.EF.SPEED_X > -5 && 25 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 30))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 0, true);
                if (HSVTrack == '1')
                {
                    HSVTrack = -1;
                    SPGO.Servo_ArmGrab2();
                }
                else if (HSVTrack == '2')
                {
                    HSVTrack = -1;
                    SPGO.Servo_ArmGrab1();
                }
                else if (HSVTrack == '3')
                {
                    HSVTrack = -1;
                    SPGO.Servo_ArmGrab3();
                }
                SPGO.UserInput(0, 0, 0, 0, false);
                //==============================================================
                SPGO.UserLocationSet(50, 28);
                while (!(47 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 53 && 25 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 30))
                    usleep(50000);
                MissionSet = true;
                sleep(1);
                while (!(-50 < CXInput && CXInput < 50 && 5 > SPGO.EF.SPEED_X && SPGO.EF.SPEED_X > -5 && 25 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 30))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 0, true);
                if (HSVTrack == '1')
                {
                    HSVTrack = -1;
                    SPGO.Servo_ArmGrab2();
                }
                else if (HSVTrack == '2')
                {
                    HSVTrack = -1;
                    SPGO.Servo_ArmGrab1();
                }
                else if (HSVTrack == '3')
                {
                    HSVTrack = -1;
                    SPGO.Servo_ArmGrab3();
                }
                SPGO.UserInput(0, 0, 0, 0, false);
                //=============================================================
                SPGO.UserLocationSet(50, 28);
                while (!(47 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 53 && 25 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 30))
                    usleep(50000);
                MissionSet = true;
                sleep(2);
                while (!(-50 < CXInput && CXInput < 50 && 5 > SPGO.EF.SPEED_X && SPGO.EF.SPEED_X > -5 && 25 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 30))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 0, true);
                if (HSVTrack == '1')
                {
                    HSVTrack = -1;
                    SPGO.Servo_ArmGrab2();
                }
                else if (HSVTrack == '2')
                {
                    HSVTrack = -1;
                    SPGO.Servo_ArmGrab1();
                }
                else if (HSVTrack == '3')
                {
                    HSVTrack = -1;
                    SPGO.Servo_ArmGrab3();
                }

                //============================================
                SPGO.UserInput(0, 0, 0, 80, true);
                while (!(78 < SPGO.SF.Real_Yaw && SPGO.SF.Real_Yaw < 82))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 80, false);
                SPGO.UserLocationSet(SPGO.EF.MOVE_X, 30);
                while (!(25 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 35))
                    usleep(50000);
                SPGO.UserLocationSet(125, 35);
                while (!(5 > SPGO.EF.SPEED_X && SPGO.EF.SPEED_X > -5 && 30 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 40 && 120 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 130))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 80, true);
                SPGO.Servo_ArmPlace2();
                SPGO.UserInput(0, 0, 0, 80, false);
                //=================================================
                SPGO.UserLocationSet(108, 35);
                while (!(5 > SPGO.EF.SPEED_X && SPGO.EF.SPEED_X > -5 && 30 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 40 && 103 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 113))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 80, true);
                SPGO.Servo_ArmPlace1();
                SPGO.UserInput(0, 0, 0, 80, false);
                //====================================
                SPGO.UserLocationSet(95, 35);
                while (!(5 > SPGO.EF.SPEED_X && SPGO.EF.SPEED_X > -5 && 30 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 40 && 90 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 100))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 80, true);
                SPGO.Servo_ArmPlace3();
                //======================================
                SPGO.Servo_ArmGet3();
                SPGO.UserInput(0, 0, 0, 80, false);
                //=======================================
                SPGO.UserLocationSet(108, 35);
                while (!(5 > SPGO.EF.SPEED_X && SPGO.EF.SPEED_X > -5 && 30 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 40 && 103 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 113))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 80, true);
                SPGO.Servo_ArmGet2();
                SPGO.UserInput(0, 0, 0, 80, false);
                //======================================
                SPGO.UserLocationSet(125, 35);
                while (!(5 > SPGO.EF.SPEED_X && SPGO.EF.SPEED_X > -5 && 30 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 40 && 120 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 130))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 80, true);
                SPGO.Servo_ArmGet1();
                //==============================================
                SPGO.UserInput(0, 0, 0, 80, false);
                SPGO.UserLocationSet(95, 35);
                while (!(5 > SPGO.EF.SPEED_X && SPGO.EF.SPEED_X > -5 && 30 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 40 && 90 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 100))
                    usleep(50000);
                SPGO.UserLocationSet(95, 135);
                while (!(5 > SPGO.EF.SPEED_X && SPGO.EF.SPEED_X > -5 && 130 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 140 && 90 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 100))
                    usleep(50000);
                SPGO.UserLocationSet(30, 140);
                while (!(5 > SPGO.EF.SPEED_X && SPGO.EF.SPEED_X > -5 && 135 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 145 && 25 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 35))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 0, true);
                while (!(-3 < SPGO.SF.Real_Yaw && SPGO.SF.Real_Yaw < 3))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 0, false);
                //=========================================
                SPGO.UserLocationSet(147, 188);
                while (!(5 > SPGO.EF.SPEED_X && SPGO.EF.SPEED_X > -5 && 183 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 193 && 142 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 152))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 0, true);
                SPGO.Servo3_Place_Up();
                SPGO.UserInput(0, 0, 0, 0, false);
                //=============================================
                SPGO.UserLocationSet(128, 188);
                while (!(5 > SPGO.EF.SPEED_X && SPGO.EF.SPEED_X > -5 && 183 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 193 && 123 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 133))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 0, true);
                SPGO.Servo2_Place_Up();
                SPGO.UserInput(0, 0, 0, 0, false);
                //=========================================
                SPGO.UserLocationSet(112, 188);
                while (!(5 > SPGO.EF.SPEED_X && SPGO.EF.SPEED_X > -5 && 183 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 193 && 107 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 117))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 0, true);
                SPGO.Servo1_Place_Up();
                SPGO.UserInput(0, 0, 0, 0, false);
                //================================================
                SPGO.UserLocationSet(235, 215);
                while (!(230 < SPGO.EF.MOVE_X && SPGO.EF.MOVE_X < 240 && 210 < SPGO.EF.MOVE_Y && SPGO.EF.MOVE_Y < 220))
                    usleep(50000);
                SPGO.UserInput(0, 0, 0, 0, true);
                sleep(3);
                SPGO.Servo_Over();
                lcd1602SetCursor(0, 1);
                lcd1602WriteString("Mission is over!");
                digitalWrite(29, HIGH);
                delay(1000);
                digitalWrite(29, LOW);
            });

            SPGO.OnRCDataCome([&](auto *RCData) {
                if (RCData[7] > 1700 && RCData[7] < 2000)
                {
                    MissonStartFlag = true;
                }
                else
                {
                    MissonStartFlag = false;
                }

                // if(RCData[9] > 1700 && RCData[9] < 2000)
            });

            SPGO.DEBUGThreadREG();
            usleep(-1);
        }
        break;
        case 'F':
        {
            YJSP_AP::YJSP SPGO;
            SPGO.YJSP_Init();
            SPGO.Servo_Over();
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
        case 'Y':
        {
            int Last = 0;
            int now = 0;
            int nowL = 0;
            double speed = 0.f;
            int stime = 0;
            int etime = 0;
            VL53L1XDevice Test;
            Test.begin("/dev/i2c-1", 0x29);
            Test.startMeasurement(0);
            Test.writeRegister(VL53L1_I2C_SLAVE__DEVICE_ADDRESS, 0x33);
            Test.~VL53L1XDevice();
            Test.begin("/dev/i2c-1", 0x33);
            sleep(2);

            while (true)
            {
                stime = micros();
                if (Test.newDataReady())
                {
                    now = (int)(Test.getDistance() / 10.f);

                    nowL = nowL * 0.7 + now * 0.3;
                    speed = (nowL - Last) / (100000.f / 1000000.f);
                    std::cout << nowL << " ";
                    std::cout << speed << " ";
                    std::cout << etime - stime << " \n";
                    Last = nowL;
                }

                etime = micros();
                if (100000.f < (etime - stime))
                    usleep(50);
                else
                    usleep(100000.f - (etime - stime));
            }
            break;
        }
        }
    }
}
void PIDCaclOut(float inputDataP, float inputDataI, float inputDataD, float &outputData,
                float &last_I_Data, float &last_D_Data,
                float P_Gain, float I_Gain, float D_Gain, float I_Max)
{
    //P caculate
    outputData = P_Gain * inputDataP;
    //D caculate
    outputData += D_Gain * (inputDataD - last_D_Data);
    last_D_Data = inputDataD;
    //I caculate
    last_I_Data += inputDataD * I_Gain;
    if (last_I_Data > I_Max)
        last_I_Data = I_Max;
    if (last_I_Data < I_Max * -1)
        last_I_Data = I_Max * -1;
    //P_I_D Mix OUTPUT
    outputData += last_I_Data;
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
