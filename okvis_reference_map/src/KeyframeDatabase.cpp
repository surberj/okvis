#include <okvis/KeyframeDatabase.hpp>

namespace okvis {

// Constructor
KeyframeDatabase::KeyframeDatabase(){	
}

// Destructor
KeyframeDatabase::~KeyframeDatabase(){
  // Free memory of keyframes
  for (std::map<uint64_t, Keyframe*>::iterator it = keyframes_.begin(); it != keyframes_.end(); ++it) {
    eraseKeyframe(it->second->getId());
  }
  // Free memory of landmarks
  for (std::map<uint64_t, MapPoint*>::iterator it = landmarks_.begin(); it != landmarks_.end(); ++it) {
    eraseLandmark(it->second->id);
  }
}

// Getter functions
std::map<uint64_t, Keyframe*> KeyframeDatabase::getKeyframes(){
  return keyframes_;
}

Keyframe* KeyframeDatabase::getKeyframe(uint64_t keyframeId) {
  return keyframes_.at(keyframeId);
}

uint64_t KeyframeDatabase::getNumberOfKeyframes() {
  return keyframes_.size();
}

uint64_t KeyframeDatabase::getNumberOfLandmarks() {
  return landmarks_.size();
}

bool KeyframeDatabase::isKeyframe(uint64_t keyframeId) {
  return keyframes_.count(keyframeId);
}

// Covisibility graph / spanning tree specific functions
bool KeyframeDatabase::insertKeyframe(std::shared_ptr<MultiFrame> multiFramePtr, kinematics::Transformation T_WS){
	Keyframe* newKeyframe = new Keyframe(*multiFramePtr,T_WS);
  keyframes_.insert(std::pair<uint64_t, Keyframe*>(newKeyframe->getId(), newKeyframe));
  return true;
}

bool KeyframeDatabase::updateConnections(uint64_t keyframeId) {
  // Get all the landmarks observed by this keyframe
  Keyframe* currentKeyframe = getKeyframe(keyframeId);
  assert(currentKeyframe);
  std::vector<MapPoint*> observedLandmarks;
  observedLandmarks.reserve(currentKeyframe->getLandmarkIds().size());

  for (std::set<uint64_t>::iterator it = currentKeyframe->getLandmarkIds().begin();
    it != currentKeyframe->getLandmarkIds().end(); ++it) {
    if (isLandmark(*it)){
      observedLandmarks.push_back(landmarks_.at(*it));
    }
  }

  // For all landmarks observed by this keyframe, check in which other keyframes they are
  // seen and increase the corresponding counter

  std::map<Keyframe*, size_t> commonCounter;

  for (std::vector<MapPoint*>::iterator it = observedLandmarks.begin(); it != observedLandmarks.end(); ++it) {

    for (std::map<KeypointIdentifier, uint64_t>::iterator mit = (*it)->observations.begin();
      mit != (*it)->observations.end(); ++mit) {
      if(mit->first.frameId == currentKeyframe->getId() || !isKeyframe(mit->first.frameId))
        continue;
      commonCounter[getKeyframe(mit->first.frameId)]++;
    }
  }

  if(commonCounter.empty())
    return true;

  // If the counter is greater than a threshold, add a connection
  // In case no keyframe counter is over the threshold, add the one with maximum counter
  
  size_t max = 0;
  Keyframe* maxCovisibleKeyframe = NULL;
  size_t threshold = 15;

  std::vector<std::pair<size_t, Keyframe*>> weightAndKeyframePairs;
  weightAndKeyframePairs.reserve(commonCounter.size());

  for (std::map<Keyframe*, size_t>::iterator it = commonCounter.begin(); it != commonCounter.end(); ++it) {
    // Check if this is the max covisible keyframe
    if (it->second > max) {
      max = it->second;
      maxCovisibleKeyframe = it->first;
    }
    // If the keyframe has sufficiently many common landmarks, add a connection
    if (it->second >= threshold) {
      weightAndKeyframePairs.push_back(std::make_pair(it->second,it->first));
      // Add the connection also to the other keyframe
      (it->first)->addConnection(currentKeyframe,it->second);
    }
  }

  if (weightAndKeyframePairs.empty()) {
    weightAndKeyframePairs.push_back(std::make_pair(max,maxCovisibleKeyframe));
    maxCovisibleKeyframe->addConnection(currentKeyframe,max);
  }

  std::sort(weightAndKeyframePairs.begin(), weightAndKeyframePairs.end());
  std::vector<Keyframe*> orderedConnectedKeyframes;
  orderedConnectedKeyframes.reserve(weightAndKeyframePairs.size());
  std::vector<size_t> orderedWeights;
  orderedWeights.reserve(weightAndKeyframePairs.size());

  for (std::vector<std::pair<size_t, Keyframe*>>::reverse_iterator rit = weightAndKeyframePairs.rbegin();
    rit != weightAndKeyframePairs.rend(); ++rit) {
    orderedConnectedKeyframes.push_back(rit->second);
    orderedWeights.push_back(rit->first);
  }

  currentKeyframe->setConnectedKeyframesAndWeights(commonCounter);
  currentKeyframe->setOrderedConnectedKeyframes(orderedConnectedKeyframes);
  currentKeyframe->setOrderedWeights(orderedWeights);

  if (currentKeyframe->isFirstConnection() && currentKeyframe->getId() != 0){
    currentKeyframe->setParent(orderedConnectedKeyframes.front());
    orderedConnectedKeyframes.front()->addChild(currentKeyframe);
    currentKeyframe->setFirstConnection(false);
  }

  return true;
}

bool KeyframeDatabase::insertLandmark(MapPoint landmark){
  MapPoint* newMapPoint = new MapPoint(landmark);
  landmarks_.insert(std::pair<uint64_t, MapPoint*>(newMapPoint->id, newMapPoint));
  return true;
}

void KeyframeDatabase::eraseKeyframe(uint64_t keyframeId) {
  Keyframe* keyframeToBeErased = getKeyframe(keyframeId);
  keyframes_.erase(keyframeId);
  delete keyframeToBeErased;
}

MapPoint* KeyframeDatabase::getLandmark(uint64_t landmarkId) {
  return landmarks_.at(landmarkId);
}

void KeyframeDatabase::eraseLandmark(uint64_t landmarkId) {
  MapPoint* landmarkToBeErased = getLandmark(landmarkId);
  landmarks_.erase(landmarkId);
  delete landmarkToBeErased;
}

bool KeyframeDatabase::isLandmark(uint64_t mapPointId) {
  return landmarks_.count(mapPointId);
}

void KeyframeDatabase::writeKeyframePosesToTxtFile(){
  std::ofstream txt;
  txt.open("/home/marlin/catkin_okvis/src/okvis_ros/okvis/okvis_loop_closing/spanning_tree_poses.txt");
  for (std::map<uint64_t, Keyframe*>::iterator it = keyframes_.begin(); it != keyframes_.end(); ++it) {
    txt << it->second->getTimestamp() << "\n";
    txt << it->second->getPose().T() << "\n";
  }
  txt.close();
}

void KeyframeDatabase::writeCurrentDatabaseToTxtFile(std::string bagname) {
  std::ofstream txt;
  std::string location = "/home/marlin/catkin_okvis/src/okvis_ros/okvis/okvis_loop_closing/";
  std::size_t found = bagname.find_last_of("/\\");
  std::string filename = bagname.substr(found+1);
  txt.open(location + filename + "_database.txt");
  txt << "=========================="
      << "\nDATABASE"
      << "\n=========================="
      << "\nDataset: " << filename
      << "\nNumber of keyframes: " << getNumberOfKeyframes() 
      << "\nNumber of landmarks: " << getNumberOfLandmarks()
      << "\n=========================="
      << "\nKEYFRAMES"
      << "\n==========================";
  int entry = 0;
  for (std::map<uint64_t, Keyframe*>::iterator it = keyframes_.begin(); it != keyframes_.end(); ++it) {
    txt << "\nEntry: " << entry
        << "\n--------------------------"
        << "\nAddress: " << it->second
        << "\nId: " << it->first << " (" << it->second->getId() << ")"
        << "\nConnected: " << !it->second->isFirstConnection()
        << "\nParent: " << it->second->getParent()
        << "\nNr of Children: " << it->second->getNumberOfChildren()
        << "\nNr of Connections: " << it->second->getNumberOfConnections()
        << "\n--------------------------";
    entry++;
  } 
}

} // namespace okvis