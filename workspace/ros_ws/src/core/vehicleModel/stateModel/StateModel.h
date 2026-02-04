#pragma once
#include "core/CoreCollection.h"
#include "utils/UtilsCollection.h"
class StateModel
{
    public:
        StateModel()=default;
        virtual ~StateModel()=default;  
        virtual void updateCommandedControl( const InputVector& u ) = 0 ; //ptSharedPtr<stPose>& pose)=0 ;
        virtual void step()=0;
        virtual const StateVector& getState() const = 0;
    protected:
        ptSharedPtr<IntegratorClass> mIntegrator{nullptr};
        
};