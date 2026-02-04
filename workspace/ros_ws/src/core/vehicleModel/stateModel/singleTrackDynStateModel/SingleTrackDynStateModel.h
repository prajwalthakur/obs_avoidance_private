
#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "vehicleModel/stateModel/StateModel.h"

//////////////////////////////////////////////////////////////////////////

struct StateStruct{
    double x;
    double y;
    double yaw;
    double vx;
    double sf;
};

struct InputStruct
{
        double sv;
        double acc;
};


class SingleTrackDynStateModel : public StateModel, public std::enable_shared_from_this<SingleTrackDynStateModel>
{
    using BaseType = StateModel;
    public:
        explicit SingleTrackDynStateModel(const YAML::Node& vehParam);
        void updateCommandedControl(const InputVector& u ) override;
        void step() override;
        const StateVector& getState() const override;
        const StateVector& getInput() const;
        void reset();
        void setState(const StateVector &);
        void setInput(const InputVector &);
        StateVector StateToVector(const StateStruct & ) const;
        StateStruct VectorToState(const StateVector &) const;
        InputVector InputToVector(const InputStruct &) const;
        InputStruct VectorToInput(const InputVector &) const;
        StateVector xdot(const StateVector & , const InputVector &) const;
    private:
        void createIntegrator(const YAML::Node& vehYamlConfig);
        void updateCommandedControl();
    private:
        int NX;
        int NU;
        float T_fwd;
        double mInitXPose;
        double mInitYPose;
        double mInitYaw;
        double mInitVx;
        double mInitSf;
        double mInitSv;
        double mInitAcc;
        double mVehWheelBase;
        InputStruct mInputStruct;
        StateStruct mStateStruct;
        InputVector mInputVector;
        StateVector mStateVector;
        InputVector mCommandedControl;
};





