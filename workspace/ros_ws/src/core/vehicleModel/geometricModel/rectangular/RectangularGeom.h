
#pragma once
#include "core/CoreCollection.h"
#include "utils/UtilsCollection.h"
#include "vehicleModel/geometricModel/GeometricModel.h"
#include "EllipseCollisionFootPrint.h"
struct stVertices
{
    stPose frontLeft;
    stPose frontRight;
    stPose rearRight;
    stPose rearLeft;
};

class RectangularGeomClass : public GeometricModel
{
    public:
        RectangularGeomClass(const double length, const double width);
        virtual ~RectangularGeomClass()=default;
        void setCollisionFootPrint(ptSharedPtr<CollisionFootPrint> collisionFootPrint) override;
        std::weak_ptr<CollisionFootPrint> getCollisionFootPrint() override;
        void step(std::shared_ptr<stPose>& pose) override;
        const std::shared_ptr<stVertices> getVertices() const;
    
    private:
        void calcVertices();
    private:
        double mLength{0.0};
        double mWidth{0.0};
        ptSharedPtr<stVertices> mVertices{nullptr};
        ptSharedPtr<CollisionFootPrint> mCollisionFootprint{nullptr};
        ptSharedPtr<stPose> mPose{nullptr};
};
