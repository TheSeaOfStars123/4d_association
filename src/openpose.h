#pragma once
#include <Eigen/Core>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include "skel.h"
#include <string>
#include <sstream>
#include <iostream>


struct OpenposeDetection
{
	OpenposeDetection() { type = SkelType::SKEL_TYPE_NONE; }
	OpenposeDetection(const SkelType& _type);
	OpenposeDetection Mapping(const SkelType& tarType);
	std::vector<Eigen::Matrix3Xf> Associate(const int& jcntThresh = 5);

	SkelType type;
	std::vector<Eigen::Matrix3Xf> joints;
	std::vector<Eigen::MatrixXf> pafs;
};
//void ParseDetectionsByFrames(char* ss, std::vector<std::vector<OpenposeDetection>>& seqDetections, const SkelType& type, cv::Size imgSize);
void ParseDetectionsByFrames(const std::string& filename, std::vector<std::vector<OpenposeDetection>>& seqdetections, const SkelType& type, cv::Size imgsize);
std::vector<OpenposeDetection> ParseDetections(const std::string& filename);
void SerializeDetections(const std::vector<OpenposeDetection>& detections, const std::string& filename);

