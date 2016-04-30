/**
 * @file Keyframe.cpp
 * @brief Source file for the Keyframe class.
 * @author Marlin Strub
 */

#include <okvis/Keyframe.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// Constructor
Keyframe::Keyframe(MultiFrame mF, kinematics::Transformation T)
	: multiFrame_(mF), T_WS_(T), parent_(NULL), firstConnection_(true) {
};

// Destructor
Keyframe::~Keyframe(){
}

// Getter functions
Time Keyframe::getTimestamp() const {
	return multiFrame_.timestamp();
}

std::shared_ptr<okvis::MultiFrame> Keyframe::multiFramePtr() {
	//std::shared_ptr<okvis::MultiFrame> ptr(*multiFrame_);
	return nullptr;
}

kinematics::Transformation Keyframe::getPose() const {
	return T_WS_;
}

Eigen::Map<Eigen::Vector3d> Keyframe::getPosition() const {
	return T_WS_.r();
}

uint64_t Keyframe::getId() const {
	return multiFrame_.id();
}

std::set<uint64_t> Keyframe::getLandmarkIds() {
	return multiFrame_.getAllLandmarkIds();
}

bool Keyframe::isFirstConnection() {
	return firstConnection_;
}

void Keyframe::setFirstConnection(bool firstConnectionFlag) {
	firstConnection_ = firstConnectionFlag;
}


// Spanning Tree
void Keyframe::addChild(Keyframe* child){
	children_.insert(child);
}

void Keyframe::eraseChild(Keyframe* child){
	children_.erase(child);
}

void Keyframe::setParent(Keyframe* parent) {
	parent_ = parent;
}

const Keyframe* const Keyframe::getParent() {
  return parent_;
}

uint16_t Keyframe::getNumberOfChildren() {
	return children_.size();
}

// Covisibility Graph

void Keyframe::setConnectedKeyframesAndWeights(std::map<Keyframe*, size_t> connectedKeyframesAndWeights) {
	connectedKeyframesAndWeights_ = connectedKeyframesAndWeights;
}

void Keyframe::setOrderedConnectedKeyframes(std::vector<Keyframe*> orderedConnectedKeyframes) {
	orderedConnectedKeyframes_ = orderedConnectedKeyframes;
}

void Keyframe::setOrderedWeights(std::vector<size_t> orderedWeights) {
	orderedWeights_ = orderedWeights;
}

void Keyframe::addConnection(Keyframe* connectedKeyframe, size_t & weight) {
	if(!connectedKeyframesAndWeights_.count(connectedKeyframe)) {
		connectedKeyframesAndWeights_[connectedKeyframe] = weight;
	} else if (connectedKeyframesAndWeights_[connectedKeyframe] != weight) {
		connectedKeyframesAndWeights_[connectedKeyframe] = weight;
	} else {
		return;
	}

	updateBestCovisibles();
}

void Keyframe::updateBestCovisibles() {
	std::vector<std::pair<size_t, Keyframe*>> weightAndKeyframePairs;
	weightAndKeyframePairs.reserve(connectedKeyframesAndWeights_.size());
	for (std::map<Keyframe*, size_t>::iterator it = connectedKeyframesAndWeights_.begin(); it != connectedKeyframesAndWeights_.end(); ++it) {
		weightAndKeyframePairs.push_back(std::make_pair(it->second,it->first));
	}
	sort(weightAndKeyframePairs.begin(), weightAndKeyframePairs.end());
	std::vector<Keyframe*> orderedConnectedKeyframes;
  orderedConnectedKeyframes.reserve(weightAndKeyframePairs.size());
  std::vector<size_t> orderedWeights;
  orderedWeights.reserve(weightAndKeyframePairs.size());

  for (std::vector<std::pair<size_t, Keyframe*>>::reverse_iterator rit = weightAndKeyframePairs.rbegin();
    rit != weightAndKeyframePairs.rend(); ++rit) {
    orderedConnectedKeyframes.push_back(rit->second);
    orderedWeights.push_back(rit->first);
  }

  setOrderedConnectedKeyframes(orderedConnectedKeyframes);
  setOrderedWeights(orderedWeights);
}

std::vector<Keyframe*> Keyframe::getCovisiblesByMinimumWeight(const size_t & benchmarkWeight) {
	if(orderedConnectedKeyframes_.empty()) {
		return std::vector<Keyframe*>();
	}

	// TODO Remark: Held closely to ORB-SLAM here, using upper_bound instead of lower_bound
	// seems weird and needs the somewhat backward carriesMoreWeight comparison
	std::vector<size_t>::iterator it = std::upper_bound(orderedWeights_.begin(),
		orderedWeights_.end(), benchmarkWeight, Keyframe::carriesMoreWeight);

	// Handle case where no weight is greater than benchmark
	if (it == orderedWeights_.end()){
		return std::vector<Keyframe*>(); 
	} else {
		int n = it - orderedWeights_.begin();
		return std::vector<Keyframe*>(orderedConnectedKeyframes_.begin(), orderedConnectedKeyframes_.begin()+n);
	}
}

uint16_t Keyframe::getNumberOfConnections() {
	return orderedConnectedKeyframes_.size();
}

} // namespace okvis