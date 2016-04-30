/**
 * @file Keyframe.cpp
 * @brief Source file for the Keyframe class.
 * @author Marlin Strub
 */

#include <okvis/Landmark.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// Constructor
Landmark::Landmark(uint64_t id, const Eigen::Vector4d & point)
: id_(id), point_(point) {
};

// Destructor
Landmark::~Landmark() {
}



} // namespace okvis