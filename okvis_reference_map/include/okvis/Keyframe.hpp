/**
 * @file Keyframe.hpp
 * @brief Header file for the Keyframe class.
 * @author Marlin Strub
 */

#ifndef INCLUDE_OKVIS_KEYFRAME_HPP
#define INCLUDE_OKVIS_KEYFRAME_HPP

#include <set>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <okvis/MultiFrame.hpp>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/Time.hpp>
// #include <okvis/Estimator.hpp>
// #include </home/marlin/catkin_okvis/src/okvis_ros/okvis/okvis_ceres/include/okvis/Estimator.hpp>

/// \brief Main namespace of this package.
namespace okvis {

/**
 * \brief
 * This class adds keyframe specific functionality to the MultiFrame class
 */
class Keyframe
{
public:
	
	/**
	 * @brief      Constructor for the Keyframe class
	 *
	 * @param[in]  mF    Associated multiframe
	 * @param[in]  T     Associated transformation (T_WS)
	 */
	Keyframe(MultiFrame mF, kinematics::Transformation T);

	/**
	 * @brief      Default destructor for the Keyframe class
	 */
	~Keyframe();

	/**
	 * @brief      Get a pointer to the multiframe of the keyframe
	 *
	 * @return     Pointer to the multiframe
	 */
	std::shared_ptr<okvis::MultiFrame> multiFramePtr();

	/**
	 * @brief      Get the timestamp of the keyframe
	 *
	 * @return     The time this keyframe was recorded
	 */
	Time getTimestamp() const;

	/**
	 * @brief      Get the id of this keyframe (same as for the underlying multiframe)
	 *
	 * @return     Id of the underlying multiframe
	 */
	uint64_t getId() const;

	/**
	 * @brief      Get the estimated pose of the keyframe
	 *
	 * @return     The estimated pose of the keyframe
	 */
	kinematics::Transformation getPose() const;

	/**
	 * @brief      Get the estimated position of the keyframe
	 *
	 * @return     The estimated position of the keyframe
	 */
	Eigen::Map<Eigen::Vector3d> getPosition() const;

	/**
	 * @brief      Get the landmark ids of the underlying multiframe
	 *
	 * @return     Set of the landmark ids seen by this underlying multiframe
	 */
	std::set<uint64_t> getLandmarkIds();

	/**
	 * @brief      Determine if the calling keyframe has a connection or if its
	 *             the first
	 *
	 * @return     Boolean that is true if the calling keyframe is not yet
	 *             connected
	 */
	bool isFirstConnection();

	/**
	 * @brief      Access the first connection flag
	 *
	 * @param[in]  firstConnectionFlag  Boolean that is true if the calling
	 *                                  keyframe is not yet connected
	 */
	void setFirstConnection(bool firstConnectionFlag);

	// Spanning Tree
	
	/**
	 * @brief      Add a child to this keyframe
	 *
	 * @param      child  Pointer to the child of this keyframe
	 */
	void addChild(Keyframe* child);

	/**
	 * @brief      Erase the connection from this keyframe to a child
	 *
	 * @param      child  The child which is to be erased as a child
	 */
	void eraseChild(Keyframe* child);

	/**
	 * @brief      Set the parent of the calling keyframe in the spanning tree
	 *
	 * @param      parent  Pointer to the parent of the calling keyframe
	 */
	void setParent(Keyframe* parent);

	/**
	 * @brief      Get a pointer to the parent of this keyframe
	 *
	 * @return     A pointer to the parent of this keyframe
	 */
	const Keyframe* const getParent();

	/**
	 * @brief      Get the number of children of this keyframe
	 *
	 * @return     Number of children of this keyframe
	 */
	uint16_t getNumberOfChildren();


	// Covisibility Graph
	
	void setConnectedKeyframesAndWeights(std::map<Keyframe*, size_t> connectedKeyframesAndWeights);

	/**
	 * @brief      Set the ordered connected keyframes
	 *
	 * @param[in]  orderedConnectedKeyframes  Vector of ordered keyframes, ordered
	 *                                        by decreasing edge weight
	 */
	void setOrderedConnectedKeyframes(std::vector<Keyframe*> orderedConnectedKeyframes);

	/**
	 * @brief      Set the ordered weights
	 *
	 * @param[in]  orderedWeights  Vector of ordered unsigned integers,
	 *                             representing the weights
	 */
	void setOrderedWeights(std::vector<size_t> orderedWeights);

	/**
	 * @brief      Add a connection of a specified weight between the calling
	 *             keyframe and the connected keyframe
	 *
	 * @param      connectedKeyframe  The keyframe which is connected
	 * @param      weight             The weight of the edge
	 */
	void addConnection(Keyframe* connectedKeyframe, size_t & weight);

	/**
	 * @brief      Updates the ordered connected keyframes and the ordered weights
	 */
	void updateBestCovisibles();

	/**
	 * @brief      Helper function for comparison of covisibility graph edge
	 *             weights
	 *
	 * @param[in]  benchmarkWeight  The weight to be compare against
	 * @param[in]  testedWeight     The weight being compared
	 *
	 * @return     Boolean wether the tested weight has more strength than the
	 *             benchmark weight (NOTE: The function name is misleading TODO:
	 *             use std::lower_bound instead of std::upper_bound in
	 *             Keyframe::getCovisiblesByMinimumWeight to fix this)
	 */
	static bool carriesMoreWeight(size_t benchmarkWeight, size_t testedWeight) {
		return benchmarkWeight > testedWeight;
	}

	/**
	 * @brief      Get all keyframes connected to this keyframe with an edge that
	 *             has at least the specified weight
	 *
	 * @param[in]  weight  The minimum weight of the edge for a connected keyframe
	 *                     to be returned
	 *
	 * @return     A vector containing all the keyframes connected to the calling
	 *             keyframe by an edge with at least specified weight
	 */
	std::vector<Keyframe*> getCovisiblesByMinimumWeight(const size_t & weight);

	uint16_t getNumberOfConnections();

private:
	MultiFrame multiFrame_; ///< The multiframe of the keyframe
	kinematics::Transformation T_WS_; ///< The pose of the keyframe wrt the world frame

	// Spanning Tree
  bool firstConnection_;
  Keyframe* parent_;
  std::set<Keyframe*> children_;
 
 	// Covisibility Graph
 	std::map<Keyframe*, size_t> connectedKeyframesAndWeights_;
 	std::vector<Keyframe*> orderedConnectedKeyframes_;
 	std::vector<size_t> orderedWeights_;

	
};

}  // namespace okvis

#endif /* INCLUDE_OKVIS_KEYFRAME_HPP */