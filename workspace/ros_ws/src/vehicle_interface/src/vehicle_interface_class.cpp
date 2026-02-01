#include "vehicle_interface/vehicle_interface_class.h"

////////////////////////////////////////////////////////////////////////////////

VehicleInterface::VehicleInterface():Node("vehicle_interface_node",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) 
{

    RCLCPP_INFO(this->get_logger(),"vehicle interface node started");
    this->declare_parameter<double>("simulation.simTimeStep", 0.01);
    this->declare_parameter<double>("simulation.statePublisherTimeStep", 0.05);
    
    mSimTimeStep = this->get_parameter("simulation.simTimeStep").as_double();
    mStatePublisherTimeStep = this->get_parameter("simulation.statePublisherTimeStep").as_double();
}

////////////////////////////////////////////////////////////////////////////////

void VehicleInterface::state_pub_timer_callback() 
{
    // Timer callback logic here
    RCLCPP_INFO(this->get_logger(), " state Publisher Timer triggered");
    for(auto& [key, veh] : mVehicleCollection)
    {
        project_utils::msg::EigenVector msg;
        StateVector state = veh->getState(); 
        msg.data = std::vector<double>(state.data(), state.data() + state.size());
        mStatePublisher[key]->publish(msg);
    }
}

////////////////////////////////////////////////////////////////////////////////

void VehicleInterface::state_update_timer_callback()
{
    RCLCPP_INFO(this->get_logger(), " State Update Timer triggered");
    for(auto& [key, veh] : mVehicleCollection)
    {
        if(mCommandedControlMap.find(key) != mCommandedControlMap.end())
            veh->step();
    }
}

////////////////////////////////////////////////////////////////////////////////

void VehicleInterface::control_sub_callback(
    const project_utils::msg::EigenVector::SharedPtr& msg,
    const Uuid& id)
{
    //int idx = msg->data.idx();
    //if(mVehKeyCollection.find(idx)!=mVehKeyCollection.end())
    mCommandedControlMap[id] = Eigen::Map<Eigen::VectorXd>(msg->data.data(), msg->data.size());    
}

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief VehicleInterface activation function to initialize 
 * m_vehicle, m_integrator, publisher and subscriber
 */
void VehicleInterface::on_activate() 
{
    addVehicles();

    for(int idx=1;idx<=mNumVehicles;++idx)
    {
            mStatePublisher[mVehKeyCollection.at(idx)] = create_publisher<project_utils::msg::EigenVector>("/" + "vehicle_" + std::to_string(idx) + "/cmd_vel", 10);
            mControlSubscriber[mVehKeyCollection.at(idx)] = create_subscription<project_utils::msg::EigenVector>("/" + "vehicle_" + std::to_string(idx) + "/control_command",10,
                            [this](const project_utils::msg::EigenVector::SharedPtr msg ){this->control_sub_callback(msg,mVehKeyCollection.at(idx));})
    }
    

    mStateUpdateTimer = this->create_wall_timer(std::chrono::duration<double>(mSimTimeStep),[this](){this->state_update_timer_callback();});
    mStatePubTimer = this->create_wall_timer(std::chrono::duration<double>(mStatePublisherTimeStep),[this](){this->state_pub_timer_callback();});
    
}

////////////////////////////////////////////////////////////////////////////////

void VehicleInterface::addVehicles()
{
    for(int i=0;i<mNumVehicles;++i)
    {
        std::string vehConfig = "/home/prajwal/projects/obs_avoidance_private/workspace/ros_ws/src/core/config" + "/Vehicle" + std::to_string(i) + "_Config.yaml";
        ptSharedPtr<VehicleModel> veh  = std::make_shared<SingleTrackDynModel>(vehConfig)
        mVehicleCollection[veh->id()] = veh;
        mVehKeyCollection[veh->id().value()] = veh->id();
    }
}