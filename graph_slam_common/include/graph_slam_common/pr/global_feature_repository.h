#ifndef GLOBAL_FEATURE_REPOSITORY_H
#define GLOBAL_FEATURE_REPOSITORY_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <graph_slam_tools/lsh.h>
#include <graph_slam_msgs/FeatureType.h>
#include <eigen3/Eigen/Dense>
#include <flann/flann.h>
#include <unordered_map>

typedef flann::Hamming<unsigned char> FlannDistance;
typedef FlannDistance::ElementType FlannElement;
typedef FlannDistance::ResultType FlannDistanceType;

class GlobalFeatureRepository {
public:
    GlobalFeatureRepository();
    virtual ~GlobalFeatureRepository();

    void clear();

	/**
      * @brief: Adds a matrix of feature descriptors of one node to the global feature repository
      * @param: newDesc - Matrix containing the feature descriptors
      * @param: nodeNumber - The number of the node in which the feature descriptors were detected
      * @param: newDescposition - corresponds to single feature descriptors (rows) of newDesc indicating
      * 			the row of the descriptor in the Descriptor matrix stored in the node
      */
	void addDescriptors(cv::Mat newDesc, int nodeNumber, std::vector<int> newDescPosition);
	/**
      * @brief: same as above, but enumerating the newDescPosition in ascending order beginning by zero
      */
	void addDescriptors(cv::Mat newDesc, int nodeNumber);
	/**
      * @brief: Linking existing features in the global feature repository with an existing node
      * @param: featIndex - index of the feature in global feature rep
      * @param nodeNumber - number of the node containing the feature
      * @param: descPosition - the row index of the feature in nodes desc matrix
      */
	void addLinks(std::vector<int> featIndex, int nodeNumber, std::vector<int> descPosition);

	/**
      * @brief: matching given descriptors with the global feature repository to find nodes with a high feature matching
      * @param: currDesc - the descriptors of the current node to match with the repository
      * @param: matches - returns the index of the feature if found in the feature rep or -1 otherwise
      * @param: matchingNodes - number of the relevant nodes corresponding to bestMatches
      * @return: true if nodes with a suitable amount of mathing features is found
      * false otherwise
      */
    bool match(cv::Mat currDesc, Eigen::VectorXi &matches, std::vector<int> &matchingNodes, int type, int min_matches = 1);

	/**
      * @brief: Adds a single descriptor to the feature rep
      * @param: desc - the descriptor
      * @param: nodeNumber - number of the node containing the descriptor
      * @param: descPosition - the position of the descriptor in the desc matrix of the node
      */
	void addDescriptor(cv::Mat desc, int nodeNumber, int descPosition);

	/**
      * @brief: Adds a single link to the feature rep. Linking an existing feature to a new node
      * @param: featIndex - index of the feature in global feature rep
      * @param: nodeNumber - the number of the node containing the feature
      * @param: descPosition - the position of the descriptor in the desc matrix of the node
      */
	void addLink(int featIndex, int nodeNumber, int descPosition);

	/**
      * @brief rebuilds the matcher using currently stored data
      */
	void rebuildMatcher();

private:

	/**
      * @brief: Searches for knn matching in the big k-d tree given a descriptor set
      * @param: desc - descriptors to mach with the global feature rep.
      * @param: dmatch - matching features found using knn search
      */
	void knnFeatureSearch(cv::Mat desc, std::vector< cv::DMatch > &dmatch);

    flann::LshIndexParams flann_params_;
    flann::Index<FlannDistance> flann_matcher_;
    std::unordered_map<int,int> flann_desc_map_;
    int flann_count_;

//    cv::Ptr<cv::DescriptorMatcher> matcher;
	std::vector<cv::Mat> descriptors;
    std::vector<std::vector<int> > links;
	cv::Ptr<cv::flann::IndexParams> iParam;
	cv::Ptr<cv::flann::SearchParams> sParam;

protected:
        int descriptor_;
        bool binary_;

};

#endif /* FEATUREREP_H_ */
