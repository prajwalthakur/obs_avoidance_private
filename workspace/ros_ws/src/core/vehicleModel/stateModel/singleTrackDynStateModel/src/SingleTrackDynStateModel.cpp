#include "core/vehicleModel/stateModel/singleTrackDynStateModel/include/SingleTrackDynStateModel.h"
SingleTrackDynStateModel::SingleTrackDynStateModel(const std::string& vehConfig)
{
    // node->declare_parameter("NX", 5);
    // node->declare_parameter("NU", 2);
    // node->declare_parameter("mInitXPose", 0.0);
    // node->declare_parameter("default_y_pos", 0.0);
    // node->declare_parameter("mInitYaw", 0.0);
    // node->declare_parameter("default_vx", 0.0);
    // node->declare_parameter("default_sf", 0.0);
    // node->declare_parameter("default_acc",0.0);
    // node->declare_parameter("default_sv",0.0);
    // node->declare_parameter("wheelbase",0.0);

    // NX = node->get_parameter("NX").as_int();
    // NU = node->get_parameter("NU").as_int();
    // mInitXPose = node->get_parameter("mInitXPose").as_double();
    // default_y_pos = node->get_parameter("default_y_pos").as_double();
    // mInitYaw = node->get_parameter("mInitYaw").as_double();
    // default_vx = node->get_parameter("default_vx").as_double();
    // default_sf = node->get_parameter("default_sf").as_double();
    // default_acc = node->get_parameter("default_acc").as_double();
    // default_sv = node->get_parameter("default_sv").as_double();
    // veh_wheelbase = node->get_parameter("veh_wheelbase").as_double();

    YAML::Node config = YAML::LoadFile(vehConfig);
    YAML::Node vehParam = config["vehicle_param"];
    NX = vehParam["NX"].as<double>();
    NU = vehParam["NU"].as<double>();
    mInitXPose = vehParam["initXPose"].as<double>();
    mInitYPose = vehParam["initYPose"].as<double>();
    mInitYaw = vehParam["initYaw"].as<double>();

    mInitVx = vehParam["initVx"].as<double>();
    mInitSf = vehParam["initSf"].as<double>();
    mInitAcc = vehParam["initAcc"].as<double>();
    mInitSv = vehParam["initSv"].as<double>();
    mVehWheelBase = vehParam["vehWheelBase"].as<double>();
    setIntegrationStepSize(vehParam);
    createIntegrator(vehParam);
    reset();

}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::createIntegrator(const YAML::Node& vehYamlConfig)
{
    double intTimeStep = vehYamlConfig["integration"]["integrationStepSize"].as<double>();
    auto tmpPointer = shared_from_this();
    mIntegrator = std::make_shared<IntegratorClass>
                (
                    [this](const StateVector& state, const InputVector& input)->StateVector
                    {
                        return xdot(state,input);
                    },
                    [this]() const StateVector&
                    {
                        return getState();
                    },
                    [this](const StateVector& state) -> void
                    {
                        return setState(state);
                    },
                    [this](const InputVector& input)-> void
                    {
                        return setInput(input);
                    },
                    intTimeStep;
                );
}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::reset(){

    mState.x = mInitXPose;
    mState.y = default_y_pos;
    mState.yaw = mInitYaw;
    mState.vx = default_vx;
    mState.sf = default_sf;
    statevector.resize(NX);
    statevector(0) = state.x;
    statevector(1) = state.y;
    statevector(2) = state.yaw;
    statevector(3) = state.vx;
    statevector(4) = state.sf;

}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::setState(const StateVector & statevector){
    this->statevector = statevector; 
    this->state.x = this->statevector(0);
    this->state.y = this->statevector(1);
    this->state.yaw = this->statevector(2);
    this->state.vx = this->statevector(3);
    this->state.sf = this->statevector(4);
}

////////////////////////////////////////////////////////////////////////////////

void SingleTrackDynStateModel::setInput(const InputVector & input_vector){
    this->inputvector = input_vector;
    this->input.acc = this->inputvector(0);
}

////////////////////////////////////////////////////////////////////////////////

const StateVector& SingleTrackDynStateModel::getState() const {
    return this->statevector;
}

////////////////////////////////////////////////////////////////////////////////

const StateVector& SingleTrackDynStateModel::getInput() const {
    return this->inputvector;
}

////////////////////////////////////////////////////////////////////////////////

StateVector SingleTrackDynStateModel::StateToVector(const StateStruct & state_struct) const{
    StateVector state_vector;
    state_vector(0) = state_struct.x;
    state_vector(1) = state_struct.y;
    state_vector(2) = state_struct.yaw;
    state_vector(3) = state_struct.vx;
    state_vector(4) = state_struct.sf;
    return state_vector;
}

////////////////////////////////////////////////////////////////////////////////

InputVector SingleTrackDynStateModel::InputToVector(const InputStruct& input_struct) const{
    InputVector input_vector;
    input_vector(0) = input_struct.sv;
    input_vector(1) = input_struct.acc;
    return input_vector;

}

////////////////////////////////////////////////////////////////////////////////

StateStruct SingleTrackDynStateModel::VectorToState(const StateVector& statevector) const{
    StateStruct st;
    st.x = statevector(0);
    st.y = statevector(1);
    st.yaw = statevector(2);
    st.vx = statevector(3);
    st.sf = statevector(4);
    return st;
}

////////////////////////////////////////////////////////////////////////////////

InputStruct SingleTrackDynStateModel::VectorToInput(const InputVector& inputvector) const{
    InputStruct input;
    input.acc = inputvector(0);
    input.sv = inputvector(1);
    return input;
}

////////////////////////////////////////////////////////////////////////////////

StateVector SingleTrackDynStateModel::xdot(const StateVector& statevector, const InputVector& inputvector) const
{
    StateVector statevector_dot;
    statevector_dot.resize(NX);

    auto xk = this->VectorToState(statevector);
    auto uk = this->VectorToInput(inputvector);
    //xdot,ydot,yawdot,vfodt,sfdot
    statevector_dot(0) = xk.vx*std::cos(xk.yaw);
    statevector_dot(1) = xk.vx*std::sin(xk.yaw);
    statevector_dot(2) = xk.vx*std::tan(xk.sf)/veh_wheelbase;
    statevector_dot(3) = uk.acc;
    statevector_dot(4) = uk.sv;
    return statevector_dot;
}

////////////////////////////////////////////////////////////////////////////////

