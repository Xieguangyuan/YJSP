#define Nsta 1
#define Mobs 2
#include "EKFImp/src/TinyEKF.h"

class TotalEKF : public TinyEKF
{
public:
    TotalEKF()
    {
        this->setQ(0, 0, .0001);
        this->setR(0, 0, .5);

        this->setQ(1, 1, .01);
        this->setR(1, 1, .5);
    }

protected:
    void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta])
    {
        fx[0] = this->x[0];

        F[0][0] = 1;

        hx[0] = this->x[0];
        hx[1] = this->x[0];

        H[0][0] = 1;
        H[1][0] = 1;
    }
};