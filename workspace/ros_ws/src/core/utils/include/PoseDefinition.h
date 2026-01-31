#pragma once 

struct stPose
{   
    double xCoord{0.0};
    double yCoord{0.0};
    double zCoord{0.0};
    double yaw{0.0};
    inline void setCoord(double x, double y , double z , double yaw )
    {
        xCoord = x;
        yCoord = y;
        zCoord = z;
        yaw = yaw;
    }
};