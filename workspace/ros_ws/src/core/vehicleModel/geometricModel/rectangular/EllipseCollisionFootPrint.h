#pragma once
#include "VehicleModel.h"

class EllipseCollisionFootPrint : public CollisionFootPrint
{
    public:
        EllipseCollisionFootPrint(const double majorAxisLength, const double minorAxisLength);
        ~EllipseCollisionFootPrint()=default;
        void step(std::shared_ptr<stPose>& pose) override;
        bool contains(const std::shared_ptr<stPose>& pose) const ;
        Eigen::Matrix2f getEllipseMatrix() const ;
        Eigen::Vector2f getCenter();
    private:
        std::shared_ptr<stPose> mPose{nullptr};
        double mMajorAxisLength{0.0};
        double mMinorAxisLength{0.0};
        double mSemiMajorAxisLength{0.0};
        double mSemiMinorAxisLength{0.0};

};