#pragma once
#include "core/core/CoreCollection.h"
#include "core/vehicleModel/geometricModel/GeometricModelCollection.h"
#include  "core/vehicleModel/stateModel/StateModelCollection.h"
class VehicleModelClass
{
    public:
        VehicleModelClass()=default;
        ~VehicleModelClass()=default;
        virtual void step() = 0;
    protected:
        ptSharedPtr<GeometricModel> mGeomModel{nullptr};
        ptSharedPtr<StateModel> mStateModel{nullptr};
};// VehicleModelClass