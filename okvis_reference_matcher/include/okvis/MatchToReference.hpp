/**
 * @file MatchToReference.hpp
 * @brief Header file for the MatchToReference class.
 * @author Julian Surber
 */

#ifndef INCLUDE_OKVIS_MATCHTOREFERENCE_HPP_
#define INCLUDE_OKVIS_MATCHTOREFERENCE_HPP_

#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <string>

#include <glog/logging.h>
#include <okvis/Keyframe.hpp>
#include <okvis/KeyframeDatabase.hpp>
#include <okvis/FrameTypedefs.hpp>

namespace okvis {

class MatchToReference
{
public:
	
	/**
	 * @brief      Constructor of the MatchToReference class
	 */
	MatchToReference();

	/**
	 * @brief      Destructor of the MatchToReference class
	 */
	~MatchToReference();

	void match();

	void runP3P();

	void get_current_T_WC(okvis::kinematics::Transformation & T_WC);
	
private:
	std::shared_ptr<okvis::KeyframeDatabase> databasePtr_;
	okvis::MultiFrame currentFrame_;
	
};

}

#endif /* INCLUDE_OKVIS_MATCHTOREFERENCE_HPP_ */