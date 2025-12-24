#pragma once
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <functional>
typedef Eigen::VectorXd StateVector;
typedef Eigen::VectorXd InputVector; //the size is determined at runtime rather than compile time.


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

struct stVertices
{
    stPose frontLeft;
    stPose frontRight;
    stPose rearRight;
    stPose rearLeft;
}

class CollisionFootPrint
{
    public:
    CollisionFootPrint()=default;
    virtual ~CollisionFootPrint()=default;
    virtual step(std::shared_pointer<stPose>& pose)=0;
};

class EllipseCollisionFootPrint : public CollisionFootPrint
{
    public:
        EllipseCollisionFootPrint(const double majorAxisLength, const double minorAxisLength);
        ~EllipseCollisionFootPrint()=default;
        void step(std::shared_pointer<stPose>& pose) override;
        bool contains(const std::shared_pointer<stPose>& pose) const ;
        Eigen::Matrix2f getEllipseMatrix() const ;
        Eigen::Vector2f getCenter();
    private:
        std::shared_pointer<stPose> mPose{nullptr};
        double mMajorAxisLength{0.0};
        double mMinorAxisLength{0.0};
        double mSemiMajorAxisLength{0.0};
        double mSemiMinorAxisLength{0.0};

};


class ModelClass
{
    public:
    ModelClass()=default;
    virtual ~ModelClass()=default;  
    virtual setCollisionFootPrint(const std::shared_pointer<CollisionFootPrint> collisionFootPrint)=0;
    virtual std::weak_pointer<CollisionFootPrint> getCollisionFootPrint()=0;
    virtual void step(std::shared_pointer<stPose>& pose)=0 ;
};

class RectangularModelClass : public ModelClass
{
    public:
    RectangularModelClass(const double length, const double width);
    virtual ~RectangularModelClass()=default;
    setCollisionFootPrint(const std::shared_pointer<CollisionFootPrint> collisionFootPrint) override;
    std::weak_pointer<CollisionFootPrint> getCollisionFootPrint() override;
    void step(std::shared_pointer<stPose>& pose) override;
    const std::shared_pointer<stVertices> getVertices() const;
    
    private:
        void calcVertices();
    private:
        double mLength{0.0};
        double mWidth{0.0};
        std::shared_pointer<stVertices> mVertices{nullptr};
        std::shared_pointer<EllipseCollisionFootPrint> mCollisionFootprint{nullptr};
        std::shared_pointer<stPose> mPose{nullptr};
};

class CollisionDetectionClass
{
    public:
        CollisionDetectionClass()=default;
        virtual ~CollisionDetectionClass()=default;
        virtual std::pair<double,bool>  detectCollision(const std::shared_pointer<CollisionFootPrint> object1, const std::shared_pointer<CollisionFootPrint> object2)=0;

};

class EllipseCollisionDetection: public CollisionDetectionClass
{
    public :
    RectCollisionDetectionClass()=default;
    ~RectCollisionDetectionClass()=default;
    std::pair<double,bool>  detectCollision(const std::shared_pointer<CollisionFootPrint> object1, const std::shared_pointer<CollisionFootPrint> object2) override;
};

