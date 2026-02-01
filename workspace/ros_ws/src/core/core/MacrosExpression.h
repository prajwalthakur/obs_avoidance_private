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


using  MapArrayXfRow = Eigen::Map<Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> ;

using MapMatrixfRow =  Eigen::Map<Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> ;

using MatrixXfRow =  Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> ;

using AXXf = Eigen::ArrayXXf;

using AXf = Eigen::ArrayXf;       // column vector

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


void load_map(std::string map_path, std::vector<float>& values_buf, MapArrayXfRow & mat_map_out)
{
    int numCOl =3;
    std::ifstream file(map_path); //map_path is  a string to the .csv file containing
    std::string line;
    //std::vector<float> values_vec;
    while(std::getline(file,line)){
        std::stringstream ss(line);
        std::string word; 
        while(std::getline(ss,word,','))
        {
            values_buf.push_back(std::stof(word)); // creating a row of 3x1
        }
    }
    int rows = values_buf.size()/numCOl;
    new (&mat_map_out) MapArrayXfRow(values_buf.data(), rows, numCOl);
    return;
}
