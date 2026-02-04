#pragma once
#include "core/CoreCollection.h"
#include "utils/UtilsCollection.h"
#include "geometricModel/GeometricModelCollection.h"
#include  "stateModel/StateModelCollection.h"
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
        virtual StateVector getState() const = 0;
    protected:
        ptSharedPtr<GeometricModel> mGeomModel{nullptr}; // Geometric Model, contains geometric shape and footprint
        ptSharedPtr<StateModel> mStateModel{nullptr}; // state space model of vehicle
        InputVector mCommandedControl;
        Uuid mId;
};// VehicleModel