#include <iostream>
#include "pca9685.h"
#include "thirdParty/RaspberryPiRC/RPiIBus/RPiIBus.hpp"

int main(int argc, char *argv[])
{
    int argvs;
    int IbusData[14];
    Ibus myIbusDevice("/dev/ttyAMA0");

    int LoseCount = 0;
    int fd = pca9685Setup(65, 0x40, 50);
    int ForwardBackOut;
    bool IsRecvError = false;

    while ((argvs = getopt(argc, argv, "RT")) != -1)
    {
        switch (argvs)
        {
        case 'R':
        {
            while (true)
            {
                int lose = myIbusDevice.IbusRead(IbusData, 4000, 2);
                if (lose != -1)
                {
                    if (!(1550 > IbusData[1] && IbusData[1] > 1450))
                    {
                        if (IbusData[1] > 1500)
                            ForwardBackOut = IbusData[1] - 1500;
                        if (IbusData[1] < 1500)
                            ForwardBackOut = IbusData[1] - 1500;
                    }
                    else
                    {
                        ForwardBackOut = 0;
                    }
                    for (int i = 0; i < 14; i++)
                    {
                        std::cout << IbusData[i] << " ";
                    }
                    std::cout << "\n";
                    LoseCount = 0;
                    IsRecvError = false;
                }
                else
                {
                    LoseCount++;
                }

                if (LoseCount == 200)
                {
                    LoseCount = 0;
                    IsRecvError = true;
                }

                if (IsRecvError)
                {
                    // ForwardBackOut = 0;
                }

                //===========================//
                if (ForwardBackOut > 0)
                {
                    pca9685PWMWrite(fd, 8, 0, 200 + 2100 * ((double)ForwardBackOut / 500.0));
                    pca9685PWMWrite(fd, 10, 0, 200 + 2100 * ((double)ForwardBackOut / 500.0));
                }
                else if (ForwardBackOut < 0)
                {
                    ForwardBackOut *= -1;
                    pca9685PWMWrite(fd, 9, 4000 - 2100 * ((double)ForwardBackOut / 500.0), 0);
                    pca9685PWMWrite(fd, 11, 4000 - 2100 * ((double)ForwardBackOut / 500.0), 0);
                }
                else if (ForwardBackOut == 0)
                    pca9685PWMReset(fd);
                //===========================//

                // pca9685PWMWrite(fd, 8, 0);
            }
        }
        break;

        case 'T':
        {
            while (true)
            {
                int is;
                std::cin >> is;
                pca9685PWMWrite(fd, 9, is, 0);
                pca9685PWMWrite(fd, 11, is, 0);
            }
        }
        break;
        }
    }
}