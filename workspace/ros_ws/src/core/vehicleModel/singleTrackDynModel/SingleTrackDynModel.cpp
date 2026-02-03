#include "SingleTrackDynModel.h"

////////////////////////////////////////////////////////////////////////////////

SingleTrackDynModel::SingleTrackDynModel(const std::string& vehParamConfig)
{
    YAML::Node config = YAML::LoadFile(vehParamConfig);
    YAML::Node veh_param = config["vehicle_param"];
    if(isStringEqual(veh_param["geomModelType"].as<std::string>(),"rectangular"))
        mGeomModel  = std::make_shared<RectangularGeomClass>(veh_param["vehLength"].as<double>(),veh_param["vehWidth"].as<double>());
    if(isStringEqual(veh_param["collFootPrintType"].as<std::string>(),"ellipse"))
    {
        double majorAxis = veh_param["collisionFootPrint"]["majorAxis"].as<double>();
        double minorAxis = veh_param["collisionFootPrint"]["minorAxis"].as<double>();
        ptSharedPtr<CollisionFootPrint> collFootPrint = std::make_shared<EllipseCollisionFootPrint>(majorAxis,minorAxis);
        mGeomModel->setCollisionFootPrint(collFootPrint);
    }

    mStateModel = std::make_shared<SingleTrackDynStateModel>(veh_param);
    mId = Uuid("vehicle");
}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynModel::step()
{
    mStateModel->step();
    StateVector st  = mStateModel->getState();
    double x = st(0);
    double y = st(1);
    double yaw = st(2);
    // double vx = st(3);
    // double sf = st(4);
    ptSharedPtr<stPose> pose  = std::make_shared<stPose>(x,y,0.0,yaw);
    mGeomModel->step(pose);
}

StateVector SingleTrackDynModel::getState() const
{
    return mStateModel->getState();
}