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
    template <typename T>
    Eigen::Vector2<T> toEigenVector(const std::vector<T> &v) noexcept { return { v[0], v[1] }; }

    Eigen::Vector2<float> toEigenVector(const std::shared_ptr<stPose> &v) noexcept { return { v->xCoord, v->yCoord}; }


};