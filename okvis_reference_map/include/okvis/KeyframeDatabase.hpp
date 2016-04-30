/**
 * @file KeyframeDatabase.hpp
 * @brief Header file for the KeyframeDatabase class.
 * @author Marlin Strub
 */

#ifndef INCLUDE_OKVIS_KEYFRAMEDATABASE_HPP_
#define INCLUDE_OKVIS_KEYFRAMEDATABASE_HPP_

#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <string>

#include <okvis/Keyframe.hpp>
#include <okvis/FrameTypedefs.hpp>

namespace okvis {

class KeyframeDatabase
{
public:
	
	/**
	 * @brief      Constructor of the KeyframeDatabase class
	 */
	KeyframeDatabase();

	/**
	 * @brief      Destructor of the KeyframeDatabase class, deallocate all
	 *             memory from the keyframes and the landmarks
	 */
	~KeyframeDatabase();
	
	/**
	 * @brief      Get the keyframe with a certain id
	 *
	 * @return     Pointer to the keyframe with a certain id
	 */
	Keyframe* getKeyframe(uint64_t keyframeId);

	/**
	 * @brief      Get a pointer on the spanning tree
	 *
	 * @return     Pointer on the spanning tree
	 */
	std::map<uint64_t, Keyframe*> getKeyframes();

	/**
	 * @brief      Get the number of keyframes
	 *
	 * @return     The number of keyframes
	 */
	uint64_t getNumberOfKeyframes();

	/**
	 * @brief      Determine if a frame with a given id is a keyframe
	 *
	 * @param[in]  id    The id of the (multi-)frame
	 *
	 * @return     Boolean wether this frame is in the keyframe database
	 */
	bool isKeyframe(uint64_t keyframeId);

	/**
	 * @brief      Inserts a keyframe in the spanning tree
	 *
	 * @param[in]  multiFramePtr  Pointer to the associated multiframe
	 * @param[in]  T_WS           Associated transformation
	 *
	 * @return     Boolean whether insertion was successful or not
	 */
	bool insertKeyframe(std::shared_ptr<MultiFrame> multiFramePtr, kinematics::Transformation T_WS);

	/**
	 * @brief      Erase the keyframe with the given id (not only from the
	 *             database, but actually deallocating its memory)
	 *
	 * @param[in]  keyframeId  Id of the keyframe to be erased
	 */
	void eraseKeyframe(uint64_t keyframeId);

	/**
	 * @brief      Update the connections of a given keyframe
	 *
	 * @param[in]  keyframeId  The id of the keyframe whose connections should be updated
	 *
	 * @return     Boolean value whether or not the update was successful
	 */
	bool updateConnections(uint64_t keyframeId);

	/**
	 * @brief      Determine if the map point with the given id is an instantiated landmark
	 *
	 * @param[in]  landmarkId  the id of the map point
	 *
	 * @return     Boolean whether the map point is an instantiated landmark or not
	 */
	bool isLandmark(uint64_t mapPointId);

	/**
	 * @brief      Insert a landmark in the landmark database
	 *
	 * @param[in]  landmark  The landmark to be inserted
	 *
	 * @return     Boolean whether the insertion was successful
	 */
	bool insertLandmark(MapPoint landmark);

	/**
	 * @brief      Erase the landmark with the given id, not only from the
	 *             database, but actually deallocate its memory
	 *
	 * @param[in]  landmarkId  The id of the landmark to be erased
	 */
	void eraseLandmark(uint64_t landmarkId);

	/**
	 * @brief      Get a pointer to the landmark corresponding to the given id
	 *
	 * @param[in]  landmarkId  The id of the requested landmark
	 *
	 * @return     A pointer to the landmark corresponding to the given id
	 */
	MapPoint* getLandmark(uint64_t landmarkId);

	/**
	 * @brief      Get the number of landmarks
	 *
	 * @return     The number of landmarks
	 */
	uint64_t getNumberOfLandmarks();

	/**
	 * @brief      Write the timestamps and poses of all keyframes to a txt file
	 */
	void writeKeyframePosesToTxtFile();

	/**
	 * @brief      Write the current database content to a text file
	 *
	 * @param[in]  bagname  The name of the bagfile, passed by main
	 */
	void writeCurrentDatabaseToTxtFile(std::string bagname);

	/**
	 * @brief      Get the multiframe which has most similar pose as T_WS_des
	 *
	 * @param[in]  T_WS_des  the desired transformation
	 */
	//std::shared_ptr<okvis::MultiFrame> getNearestMultiframe(okvis::kinematics::Transforamtion T_WS_in);

private:
	std::map<uint64_t, Keyframe*> keyframes_; ///< Map holding all keyframes
	std::map<uint64_t, MapPoint*> landmarks_; ///< Map holding all landmarks
};

}

#endif /* INCLUDE_OKVIS_KEYFRAMEDATABASE_HPP_ */