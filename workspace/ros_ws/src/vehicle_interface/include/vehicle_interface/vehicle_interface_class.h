#pragma once
#include <rclcpp/rclcpp.hpp>
#include <project_utils/msg/eigen_vector.hpp>
#include <chrono>
#include "core/CoreCollection.h"
#include "utils/UtilsCollection.h"
#include "vehicleModel/VehicleModelCollection.h"

class VehicleInterface: public rclcpp::Node
{
    public:
        VehicleInterface();
        void on_activate();
    private:
        void addVehicles();
    private: 
        int mNumVehicles{2};
        ptUnorderedMap<Uuid,ptSharedPtr<VehicleModel>> mVehicleCollection;
        ptUnorderedMap<long int , Uuid> mVehKeyCollection;
        ptUnorderedMap<Uuid,InputVector> mCommandedControlMap;
        ptUnorderedMap<Uuid, rclcpp::Publisher<project_utils::msg::EigenVector>::SharedPtr> mStatePublisher;
        ptUnorderedMap<Uuid, rclcpp::Subscription<project_utils::msg::EigenVector>::SharedPtr> mControlSubscriber;

        rclcpp::TimerBase::SharedPtr mStateUpdateTimer;
        rclcpp::TimerBase::SharedPtr mStatePubTimer;

        void state_pub_timer_callback(); 
        void state_update_timer_callback();
        void control_sub_callback(const project_utils::msg::EigenVector::SharedPtr& msg, const Uuid& id);
        
        double mSimTimeStep;
        double mStatePublisherTimeStep;
};
