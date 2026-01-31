#pragma once
#include <rclcpp/rclcpp.hpp>
#include <project_utils/vehicle_class.hpp>
#include <project_utils/integrator_class.hpp>
#include <project_utils/msg/eigen_vector.hpp>
#include <chrono>

#include "core/vehicleModel/VehicleModelCollection.h"
#include "core/core/CoreCollection.h"
class VehicleInterface: public rclcpp::Node{
    public:
        VehicleInterface();
        void on_activate();
    private:
        void addVehicles();
    private: 
        ptUnorderedMap<Uuid,ptSharedPtr<VehicleModel>> mVehicleCollection;
        rclcpp::Publisher<project_utils::msg::EigenVector>::SharedPtr m_state_publisher;
        rclcpp::Subscription<project_utils::msg::EigenVector>::SharedPtr m_control_subscriber;
        rclcpp::TimerBase::SharedPtr m_state_update_timer;
        rclcpp::TimerBase::SharedPtr m_state_pub_timer;
        InputVector m_control_ref;
        void state_pub_timer_callback(); 
        void state_update_timer_callback();
        void control_sub_callback(const project_utils::msg::EigenVector::SharedPtr& msg);
        int nx;
        int nu;
        double m_sim_delta_t;
        double m_state_publish_delta_t;
};
