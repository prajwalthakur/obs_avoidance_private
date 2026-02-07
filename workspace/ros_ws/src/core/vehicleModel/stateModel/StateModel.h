#pragma once
#include "core/CoreCollection.h"
#include "utils/UtilsCollection.h"
class StateModel
{
    public:
        StateModel(YAML::Node& simConfig, YAML::Node& vehConfig, const Uuid& id):
            mSimConfig(simConfig),mVehConfig(vehConfig),mId(id){};
        virtual ~StateModel()=default;  
        virtual void updateCommandedControl( const InputVector& u ) = 0 ; //ptSharedPtr<stPose>& pose)=0 ;
        virtual void step()=0;
        virtual void createIntegrator()=0;
        virtual const StateVector& getState() const = 0;
    protected:
        ptSharedPtr<IntegratorClass> mIntegrator{nullptr};
        YAML::Node mSimConfig;
        YAML::Node mVehConfig;
        InputVector mCommandedControl;
        Uuid mId;
        
};