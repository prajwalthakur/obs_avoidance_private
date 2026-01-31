#pragma once
#include "core/vehicleModel/VehicleModel.h"
#include <yaml-cpp/yaml.h>
class SingleTrackDynModel : public VehicleModelClass
{
    public:
        // Constructor
        SingleTrackDynModel(const std::string& vehicleParamFilePath);
        // Destructor
        ~SingleTrackDynModel()=default;
        // Step function
        void step() override;


};