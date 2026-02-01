#pragma once
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include "core/core/CoreCollection.h"

class IntegratorClass
{
    public:
        explicit IntegratorClass(std::function<StateVector(const StateVector&, const InputVector&)> dynamics , 
                std::function<const StateVector&()> getState,
                std::function<void(const StateVector&)> setState,
                std::function<void(const InputVector&)> setInput,double fine_time_step) 
            : dynamics_(dynamics),
             setState_(setState), 
             setInput_(setInput), 
             getState_(getState), 
             mIntegrationStepSize(fine_time_step){}
        
        explicit IntegratorClass(std::function<StateVector(const StateVector&, const InputVector&)> dynamics , 
            std::function<const StateVector&()> getState,
            std::function<void(const StateVector&)> setState,
            std::function<void(const InputVector&)> setInput,double integrationTimeStep, double simTimeStep) 
            : dynamics_(dynamics) , 
            setState_(setState) , 
            setInput_(setInput), 
            getState_(getState), 
            mIntegrationStepSize(integrationTimeStep), 
            mSimStepSize(simTimeStep){}

        StateVector rk4Integrator(const StateVector& , const InputVector& ,double )const;
        StateVector efIntegrator(const StateVector& , const InputVector& ,double )const;
        void simNextState(const InputVector& ,double )const;
        void simNextState(const InputVector&) const;
    private:
        const double mIntegrationStepSize;   
        const double mSimStepSize{-1.0};   
        std::function<StateVector(const StateVector&, const InputVector&)> dynamics_;  
        std::function<void(const StateVector&)> setState_;
        std::function<void(const InputVector&)> setInput_;
        std::function<const StateVector&()> getState_;
};
