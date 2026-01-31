#include "core/vehicleModel/singleTrackDynModel/SingleTrackDynModel.h"

////////////////////////////////////////////////////////////////////////////////

SingleTrackDynModel::SingleTrackDynModel(const std::string& vehParam)
{
    YAML::Node config = YAML::LoadFile(vehParam);
    YAML::Node veh_param = config["vehicle_param"];
    if(vehParam["geomModelType"] == "rectangular")
        mGeomModel  = std::make_shared<RectangularGeomClass>(vehParam["vehLength"].as<double>(),vehParam["vehWidth"].as<double>());
    if(vehParam["collFootPrintType"]=="ellipse")
    {
        double majorAxis = vehParam["collisionFootPrint"]["majorAxis"].as<double>();
        double minorAxis = vehParam["collisionFootPrint"]["minorAxis"].as<double>();
        ptSharedPtr<CollisionFootPrint> collFootPrint = std::make_shared<EllipseCollisionFootPrint>(majorAxis,minorAxis);
        mGeomModel->setCollisionFootPrint(collFootPrint);
    }

    mStateModel = std::make_shared<SingleTrackDynStateModel>(const std::string& vehParam)
}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynModel::step()
{
    mStateModel->step()
}