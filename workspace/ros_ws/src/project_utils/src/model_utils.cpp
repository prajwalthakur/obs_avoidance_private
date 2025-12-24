#include "project_utils/model_utils.hpp"
#include "project_utils/ellipse_optimization.hpp"

//////////////////////////////////////////////////////////////////////////

namespace
{
    template <typename T>
    Eigen::Vector2<T> toEigenVector(const std::vector<T> &v) noexcept { return { v[0], v[1] }; }
    
    Eigen::Vector2<float> toEigenVector(const std::shared_ptr<stPose> &v) noexcept { return { v->xCoord, v->yCoord}; }
};

//////////////////////////////////////////////////////////////////////////

EllipseCollisionFootPrint::EllipseCollisionFootPrint(const double majorAxisLength, const double minorAxisLength)
    : mMajorAxisLength(majorAxisLength),
      mMinorAxisLength(minorAxisLength),
      mSemiMajorAxisLength(majorAxisLength/2.0),
      mSemiMinorAxisLength(minorAxisLength/2.0)
      {
        mPose  = std::make_shared<stPose>();
      }

void EllipseCollisionFootPrint::step(std::shared_ptr<stPose>& pose)
{
    mPose = pose;
}

//////////////////////////////////////////////////////////////////////////

RectangularModelClass::RectangularModelClass(const double length, const double width)
{
    mLength = length;
    mWidth = width;
    mVertices = std::make_shared<stVertices>();
}

//////////////////////////////////////////////////////////////////////////

void RectangularModelClass::setCollisionFootPrint(const std::shared_ptr<CollisionFootPrint> collisionFootPrint)
{
    auto tempPointer =  std::dynamic_pointer_cast<EllipseCollisionFootPrint>(collisionFootPrint);
    if(!tempPointer)
        mCollisionFootprint = tempPointer;
}

//////////////////////////////////////////////////////////////////////////

std::weak_ptr<CollisionFootPrint> RectangularModelClass::getCollisionFootPrint()
{
    return std::weak_ptr<EllipseCollisionFootPrint>(mCollisionFootprint);
}

//////////////////////////////////////////////////////////////////////////

void RectangularModelClass::step(std::shared_ptr<stPose>& pose)
{

    mPose = pose;
    mCollisionFootprint->step(mPose);
    calcVertices();
}

//////////////////////////////////////////////////////////////////////////

void RectangularModelClass::calcVertices()
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

const std::shared_ptr<stVertices> RectangularModelClass::getVertices() const
{
    return mVertices;
}

//////////////////////////////////////////////////////////////////////////

bool EllipseCollisionFootPrint::contains(const std::shared_ptr<stPose>& pose) const {
    // For an ellipse E and the corresponding ellipse matrix A,
    // x is inside in E if (x-x0)^T * A * (x-x0) <= 1, where x0 is the center of E.

    auto offset = toEigenVector(pose) - toEigenVector(mPose) ; // x-x0

    return offset.transpose() * getEllipseMatrix() * offset <= 1.f;
}

//////////////////////////////////////////////////////////////////////////

Eigen::Matrix2f EllipseCollisionFootPrint::getEllipseMatrix() const
{
    Eigen::Matrix2f D;
    D << 1.0f / (mSemiMajorAxisLength * mSemiMajorAxisLength), 0,
         0, 1.0f / (mSemiMinorAxisLength * mSemiMinorAxisLength);

    float c = std::cos(mPose->yaw);
    float s = std::sin(mPose->yaw);

    Eigen::Matrix2f R;
    R << c, -s,
         s,  c;

    return R * D * R.transpose();
}

//////////////////////////////////////////////////////////////////////////

Eigen::Vector2f EllipseCollisionFootPrint::getCenter()
{
    Eigen::Vector2f vec;
    vec << mPose->xCoord , mPose->yCoord;
    return vec;
}

//////////////////////////////////////////////////////////////////////////

std::pair<double,bool> EllipseCollisionDetection::detectCollision(const std::shared_ptr<CollisionFootPrint> object1, const std::shared_ptr<CollisionFootPrint> object2)
{
    auto obj1FootPrint = std::dynamic_pointer_cast<EllipseCollisionFootPrint>(object1);
    auto obj2FootPrint = std::dynamic_pointer_cast<EllipseCollisionFootPrint>(object2);

    if(!obj1FootPrint || !obj2FootPrint)
    {
        std::cerr << "ERROR: Invalid footprint type";
        return std::make_pair(-1.0,false);
    }

    Eigen::Vector2f a = obj1FootPrint->getCenter();
    Eigen::Vector2f b = obj2FootPrint->getCenter();

    Eigen::Matrix2f A = obj1FootPrint->getEllipseMatrix();
    Eigen::Matrix2f B = obj2FootPrint->getEllipseMatrix();

    Eigen::Matrix2f Ainv = A.inverse();
    Eigen::Matrix2f Binv = B.inverse();

    Eigen::Vector2f d = b - a;
    auto k_function = [&](float s)
    {
        // avoid division by zero
        s = std::clamp(s, 1e-4f, 1.0f - 1e-4f);

        Eigen::Matrix2f M =
            (1.0f / (1.0f - s)) * Ainv +
            (1.0f / s) * Binv;

        Eigen::Matrix2f Minv = M.inverse();

        float quad = d.transpose() * Minv * d;
        return 1.0f - quad;
    };

    auto [_, min_k] = optimize(0.5f, k_function);
    return std::make_pair(min_k,min_k > 0.f);
}