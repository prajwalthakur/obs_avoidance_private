#pragma once
#include "core/core/CoreCollection.h"
#include "core/utils/include/UtilsCollection.h"
class StateModel
{
    public:
        StateModel()=default;
        virtual ~StateModel()=default;  
        virtual void updateCommandedControl( const InputVector& u ) = 0 ; //ptSharedPtr<stPose>& pose)=0 ;
        virtual void step(double simTimeStep)=0;
    protected:
        ptSharedPtr<IntegratorClass> mIntegrator{nullptr};
        
};