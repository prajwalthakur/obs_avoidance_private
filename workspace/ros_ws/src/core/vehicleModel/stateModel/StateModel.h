#pragma once
#include "core/core/CoreCollection.h"
#include "core/utils/include/UtilsCollection.h"
class StateModel
{
    public:
        StateModel()=default;
        virtual ~StateModel()=default;  
        virtual void step( const InputVector& u ) = 0 ; //ptSharedPtr<stPose>& pose)=0 ;
    protected:
        ptSharedPtr<IntegratorClass> mIntegrator{nullptr};
        
};