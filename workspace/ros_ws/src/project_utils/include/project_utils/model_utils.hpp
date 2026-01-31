#pragma once
#include <memory>
#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <iostream>
typedef Eigen::VectorXd StateVector;
typedef Eigen::VectorXd InputVector; //the size is determined at runtime rather than compile time.















class CollisionDetectionClass
{
    public:
        CollisionDetectionClass()=default;
        virtual ~CollisionDetectionClass()=default;
        virtual std::pair<double,bool>  detectCollision(const std::shared_ptr<CollisionFootPrint> object1, const std::shared_ptr<CollisionFootPrint> object2)=0;

};

class EllipseCollisionDetection: public CollisionDetectionClass
{
    public :
    EllipseCollisionDetection()=default;
    ~EllipseCollisionDetection()=default;
    std::pair<double,bool>  detectCollision(const std::shared_ptr<CollisionFootPrint> object1, const std::shared_ptr<CollisionFootPrint> object2) override;
};

