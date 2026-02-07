#pragma once
#include "core/CoreCollection.h"
#include "utils/UtilsCollection.h"
#include "geometricModel/GeometricModelCollection.h"
#include  "stateModel/StateModelCollection.h"
class VehicleModel
{
    public:
        inline VehicleModel(YAML::Node& simConfig, YAML::Node& vehConfig):
            mSimConfig(simConfig), mVehConfig(vehConfig){};
        ~VehicleModel()=default;
        inline const Uuid& id() const { return mId; }
        virtual void step() = 0;
        virtual StateVector getState() const = 0;
        inline void updateCommandedControl( const InputVector& u )
        {
            mStateModel->updateCommandedControl(u);
        }
    protected:
        ptSharedPtr<GeometricModel> mGeomModel{nullptr}; // Geometric Model, contains geometric shape and footprint
        ptSharedPtr<StateModel> mStateModel{nullptr}; // state space model of vehicle
        Uuid mId;
        YAML::Node mSimConfig;
        YAML::Node mVehConfig;

};// VehicleModel