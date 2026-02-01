#pragma once
#include "core/core/CoreCollection.h"
#include "core/utils/include/UtilsCollection.h"
#include "core/vehicleModel/geometricModel/GeometricModelCollection.h"
#include  "core/vehicleModel/stateModel/StateModelCollection.h"
class VehicleModel
{
    public:
        VehicleModel()=default;
        ~VehicleModel()=default;
        inline const Uuid& id() const
        {
            return mId;
        }
        virtual void step() = 0;
    protected:
        ptSharedPtr<GeometricModel> mGeomModel{nullptr}; // Geometric Model, contains geometric shape and footprint
        ptSharedPtr<StateModel> mStateModel{nullptr}; // state space model of vehicle
        InputVector mCommandedControl;
        Uuid mId;
};// VehicleModel