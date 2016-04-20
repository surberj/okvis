#include <okvis/MatchToReference.hpp>

namespace okvis {

// Constructor
MatchToReference::MatchToReference(){	
}

// Destructor
MatchToReference::~MatchToReference(){
}



void MatchToReference::match() {
	LOG(WARNING) << "MatchToReference::match() not implemented!";
	// based on the prior pose and uncertainty get all possible landmark candidates
	// from the reference map


	// perform matching between all keypoints of the frame and the landmark candidates


	// perform ransac PnP to filter outliers and get a pose measurement of the frame 
	// with respect to the global frame
}

void MatchToReference::runP3P() {
	LOG(WARNING) << "MatchToReference::runP3P() not implemented!";
	// perform ransac PnP to filter outliers and get a pose measurement of the frame 
	// with respect to the global frame
}

void MatchToReference::get_current_T_WC(okvis::kinematics::Transformation & T_WC) {
	LOG(WARNING) << "MatchToReference::get_current_T_WC() not implemented!";
}

} // namespace okvis