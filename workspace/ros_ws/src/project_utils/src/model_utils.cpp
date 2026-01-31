#include "project_utils/model_utils.hpp"
#include "project_utils/ellipse_optimization.hpp"

//////////////////////////////////////////////////////////////////////////

namespace
{
    template <typename T>
    Eigen::Vector2<T> toEigenVector(const std::vector<T> &v) noexcept { return { v[0], v[1] }; }
    
    Eigen::Vector2<float> toEigenVector(const std::shared_ptr<stPose> &v) noexcept { return { v->xCoord, v->yCoord}; }
};

