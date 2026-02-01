#pragma once
#include "VehicleModel.h"
#include <yaml-cpp/yaml.h>
class SingleTrackDynModel : public VehicleModel
{
    public:
        // Constructor
        SingleTrackDynModel(const std::string& vehicleParamFilePath);
        // Destructor
        ~SingleTrackDynModel()=default;
        // Step function
        void step() override;


};