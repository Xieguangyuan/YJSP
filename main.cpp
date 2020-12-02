#include <iostream>
#include "thirdParty/RaspberryPiRC/RPiIBus/RPiIBus.hpp"

int main(int, char **)
{
    int IbusData[14];
    Ibus myIbusDevice("/dev/ttyAMA0");

    while (true)
    {
        int lose = myIbusDevice.IbusRead(IbusData, 8000, 2);
        if (lose != -1)
        {
            for (size_t i = 0; i < 14; i++)
            {
                std::cout << IbusData[i] << " ";
            }
            std::cout << "\n";
        }
    }
}
