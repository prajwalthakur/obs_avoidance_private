/**
 * @file Memory.hpp
 * @author Prajwal Thakur (you@domain.com)
 * @brief  header file containing definations for typedefs.
 * */


#pragma once
#include <Eigen/Dense>
#include <vector>
#include <optional>
#include <vector>
#include <unordered_set>
#include <memory>


//typedef Eigen::Matrix<double,NX,1> StateVector;
using InputVector =  Eigen::VectorXd ;
using StateVector = Eigen::VectorXd ;

template <typename T>
using ptStlCollection = std::vector<T>;

template <typename T>
using ptStl2dCollection = std::vector<std::vector<T>>;

template <typename T>
using ptCollection = Eigen::Matrix<T, Eigen::Dynamic, 1>;

template <typename T> 
using ptUnorderedSet = std::unordered_set<T>;

template <typename T, typename Q>
using ptUnorderedMap = std::unordered_map<T,Q>;

template <typename T>
using ptUniquePtr = std::unique_ptr<T>;

template <typename T>
using ptSharedPtr = std::shared_ptr<T>;

template <typename T>
using ptwkPtr = std::weak_ptr<T>;

template <typename T>
using ptOpt = std::optional<T>;


