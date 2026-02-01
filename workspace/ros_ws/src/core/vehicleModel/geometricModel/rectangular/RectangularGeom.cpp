
#include "core/vehicleModel/geometricModel/rectangular/RectangularGeom.h"

//////////////////////////////////////////////////////////////////////////

RectangularGeomClass::RectangularGeomClass(const double length, const double width)
{
    mLength = length;
    mWidth = width;
    mVertices = std::make_shared<stVertices>();
}

//////////////////////////////////////////////////////////////////////////

void RectangularGeomClass::setCollisionFootPrint( ptSharedPtr<CollisionFootPrint> collisionFootPrint)
{
    auto tempPointer =  std::dynamic_pointer_cast<EllipseCollisionFootPrint>(collisionFootPrint);
    if(!tempPointer)
        mCollisionFootprint = tempPointer;
}

//////////////////////////////////////////////////////////////////////////

std::weak_ptr<CollisionFootPrint> RectangularGeomClass::getCollisionFootPrint()
{
    return std::weak_ptr<EllipseCollisionFootPrint>(mCollisionFootprint);
}

//////////////////////////////////////////////////////////////////////////

void RectangularGeomClass::step(std::shared_ptr<stPose>& pose)
{

    mPose = pose;
    mCollisionFootprint->step(mPose);
    calcVertices();
}

//////////////////////////////////////////////////////////////////////////

void RectangularGeomClass::calcVertices()
{
    // front left
    double xFl = mPose->xCoord + (mLength/2.0)*cos(mPose->yaw) - (mWidth/2.0)*sin(mPose->yaw); 
    double yFl = mPose->yCoord + (mWidth/2.0)*cos(mPose->yaw) + (mLength/2.0)*sin(mPose->yaw);
    mVertices->frontLeft.setCoord(xFl,yFl,0.0,mPose->yaw);
    // front right
    double xFr = mPose->xCoord + (mLength/2.0)*cos(mPose->yaw) + (mWidth/2.0)*sin(mPose->yaw); 
    double yFr = mPose->yCoord - (mWidth/2.0)*cos(mPose->yaw) + (mLength/2.0)*sin(mPose->yaw);
    mVertices->frontRight.setCoord(xFr,yFr,0.0,mPose->yaw);
    // rear right
    double xRr = mPose->xCoord - (mLength/2.0)*cos(mPose->yaw) + (mWidth/2.0)*sin(mPose->yaw); 
    double yRr = mPose->yCoord - (mWidth/2.0)*cos(mPose->yaw) - (mLength/2.0)*sin(mPose->yaw);
    mVertices->rearRight.setCoord(xRr,yRr,0.0,mPose->yaw);
    // rear left
    double xRl = mPose->xCoord - (mLength/2.0)*cos(mPose->yaw) - (mWidth/2.0)*sin(mPose->yaw); 
    double yRl = mPose->yCoord + (mWidth/2.0)*cos(mPose->yaw) - (mLength/2.0)*cos(mPose->yaw);
    mVertices->rearLeft.setCoord(xRl,yRl,0.0,mPose->yaw);
     
}

//////////////////////////////////////////////////////////////////////////

const std::shared_ptr<stVertices> RectangularGeomClass::getVertices() const
{
    return mVertices;
}