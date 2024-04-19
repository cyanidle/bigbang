#pragma once
#include "describe/describe.hpp"
#include <ros/ros.h>

template<typename T>
void DeserializeFromRos(T& out, ros::NodeHandle& nh) {

}

template<typename T>
void DeserializeFromRos(std::vector<T>& out, ros::NodeHandle& nh) {
    
}