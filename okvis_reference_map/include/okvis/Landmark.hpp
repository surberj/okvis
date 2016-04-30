/**
 * @file Keyframe.hpp
 * @brief Header file for the Landmark class.
 * @author Julian Surber
 */

#ifndef INCLUDE_OKVIS_LANDMARK_HPP
#define INCLUDE_OKVIS_LANDMARK_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>

/// \brief Main namespace of this package.
namespace okvis {

class Landmark
{
	/**
   	* @brief Constructor.
   	* @param id        ID of the point. E.g. landmark ID.
   	* @param point     Homogeneous coordinate of the point.
  	*/
  	Landmark(uint64_t id, const Eigen::Vector4d & point);

  	/**
   	* @brief Destructor.
  	*/
  	~Landmark();


private:
	uint64_t id_;            ///< ID of the point. E.g. landmark ID.
  	Eigen::Vector4d point_;  ///< Homogeneous coordinate of the point.
  	
};



}  // namespace okvis

#endif /* INCLUDE_OKVIS_LANDMARK_HPP */