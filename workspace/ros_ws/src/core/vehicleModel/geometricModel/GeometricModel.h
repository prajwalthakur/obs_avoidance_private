#pragma once
#include "core/core/CoreCollection.h"

class CollisionFootPrint
{
    public:
    CollisionFootPrint()=default;
    virtual ~CollisionFootPrint()=default;
    virtual void step(std::shared_ptr<stPose>& pose)=0;
};


class GeometricModel
{
    public:
        GeometricModel()=default;
        virtual ~GeometricModel()=default;  
        virtual void setCollisionFootPrint(const ptSharedPtr<CollisionFootPrint> collisionFootPrint)=0;
        virtual ptwkPtr<CollisionFootPrint> getCollisionFootPrint()=0;
        virtual void step(ptSharedPtr<stPose>& pose)=0 ;
};