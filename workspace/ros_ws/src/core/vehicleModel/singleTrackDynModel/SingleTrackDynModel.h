#pragma once
#include "vehicleModel/VehicleModel.h"
#include <yaml-cpp/yaml.h>
class SingleTrackDynModel : public VehicleModel
{
    using BaseType = VehicleModel;
    public:
        // Constructor
        SingleTrackDynModel(YAML::Node& simConfig, YAML::Node& vehConfig);
        // Destructor
        ~SingleTrackDynModel()=default;
        // Step function
        void step() override;
        StateVector getState() const override;
};