#include <iostream>
#include <unistd.h>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <fstream>
#include <wiringPi.h>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include "YJSP_Schedule.hpp"
#include "saftyMat.hpp"

#include "pca9685.h"
#include "lcd1602.h"
#include "VL53L1XFuck.hpp"
#include "thirdparty/RuModule/SRC/_Excutable/Drive_Json.hpp"
#include "thirdparty/QRModule/src/qrscanner.hpp"
#include "thirdparty/RuModule/SRC/_VisionBase/CameraDrive/Drive_V4L2Reader.hpp"
#include "thirdparty/RuModule/SRC/_VisionBase/VisionAIDrive/Drive_OpenCVDN.hpp"
using namespace cv;
double MAX_Area = 0;
float YawOutput = 0;
float CXOutput = 0;
float CXOutputILast = 0;
float CXOutputDLast = 0;

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
                resize(src, src, cv::Size(800, 600));
                rotate(src, src, cv::ROTATE_180);
                cvtColor(src, tmp, COLOR_BGR2HSV);
                //inRange(tmp, Scalar(90, 160, 0), Scalar(100, 255, 255), tmp); //blue               Scalar(LOW_H,LOW_S,LOW_V),Scalar(HIGH_H,HIGH_S,HIGH_V)
                //inRange(tmp, Scalar(60, 80, 70), Scalar(90, 140, 255), tmp); //green
                inRange(tmp, Scalar(150, 160, 0), Scalar(179, 255, 255), tmp); //red

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
            FrameBuffer<cv::Mat> syncBuffer;

            std::vector<decodedObject> decodeOB;
            cv::VideoCapture cap(0);
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
                    resize(src, src, cv::Size(800, 600));
                    rotate(src, src, cv::ROTATE_180);
                    syncBuffer.pushFrame(src);
                    cvtColor(src, tmp, CV_BGR2HSV_FULL);
                    inRange(tmp, Scalar(150, 160, 0), Scalar(179, 255, 255), tmp); //red //Scalar(LOW_H,LOW_S,LOW_V),Scalar(HIGH_H,HIGH_S,HIGH_V)

                    erode(tmp, tmp, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));    //腐蚀
                    dilate(tmp, tmp, getStructuringElement(MORPH_ELLIPSE, Size(20, 20))); //膨胀

                    std::vector<std::vector<cv::Point>> contours;
                    findContours(tmp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
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
                    //
                    double CXInput = -1 * (cx - 400);
                    // PIDCaclOut(CXInput, CXInput, CXInput, CXOutput, CXOutputILast, CXOutputDLast, 0.5, 0.07, 5, 100.f);
                    //PIDCaclOut(CXInput, CXInput, CXInput, CXOutput, CXOutputILast, CXOutputDLast, 0.1, 0, 0, 0);

                    // UserInput(CXInput, 0, 0);
                    // SPGO.PWMUserInput(8, 0, CXInput);
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
                        }
                    }
                    if (syncBuffer.size() > 15)
                        syncBuffer.clearBuffer();
                    usleep(200000);
                }
            });
            bool finish = true;
            bool finishs = true;
            SPGO.OnRCDataCome([&](auto *RCData) {
                if (RCData[7] > 1700 && RCData[7] < 2000)
                {
                    finish = false;
                }
                else
                {
                    finish = true;
                    finishs = true;
                }

                if (!finish)
                {
                    if (finishs)
                    {

                        for (int i = 400; i <= 460; i++)
                        {
                            delay(10);
                            SPGO.PWMUserInput(4, 0, i);
                        }
                        for (int i = 460; i >= 400; i--)
                        {
                            delay(10);
                            SPGO.PWMUserInput(4, 0, i);
                        }
                        for (int i = 400; i >= 350; i--)
                        {
                            delay(10);
                            SPGO.PWMUserInput(4, 0, i);
                        }
                        for (int i = 350; i <= 400; i++)
                        {
                            delay(10);
                            SPGO.PWMUserInput(4, 0, i);
                        }
                        delay(1000);
                        SPGO.UserInput(200, 0, 0, 0);
                        delay(300);
                        SPGO.UserInput(0, 0, 0, 0);

                        finishs = false;
                    }
                }
            });

            SPGO.DEBUGThreadREG();
            // QRcamer.join();

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

                erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));    //腐蚀
                dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(20, 20))); //膨胀

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
