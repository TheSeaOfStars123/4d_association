#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <filesystem>
#include "skel_driver.h"
#include "math_util.h"
#include <json/json.h>


std::vector<std::map<int, Eigen::Matrix4Xf>> ParseSkels(const std::string& filename)
{
	std::ifstream fs(filename);
	if (!fs.is_open()) {
		std::cerr << "file not exist: " << filename << std::endl;
		std::abort();
	}

	int jointSize, frameSize, personSize, identity;
	fs >> jointSize >> frameSize;

	std::vector<std::map<int, Eigen::Matrix4Xf>> skels(frameSize);
	for (int frameIdx = 0; frameIdx < frameSize; frameIdx++) {
		fs >> personSize;
		for (int pIdx = 0; pIdx < personSize; pIdx++) {
			fs >> identity;
			Eigen::Matrix4Xf skel(4, jointSize);
			for (int i = 0; i < 4; i++)
				for (int jIdx = 0; jIdx < jointSize; jIdx++)
					fs >> skel(i, jIdx);
			skels[frameIdx].insert(std::make_pair(identity, skel));
		}
	}
	fs.close();
	return skels;
}


void SerializeSkels(const std::vector<std::map<int, Eigen::Matrix4Xf>>& skels, const std::string& filename)
{
	const int jointSize = [&]() {
		for (const auto& _skels : skels)
			if (!_skels.empty())
				return _skels.begin()->second.cols();
		std::cerr << "Empty skels" << std::endl;
		std::abort();
	}();

	std::ofstream fs(filename);
	fs << jointSize << "\t" << skels.size() << std::endl;

	for (const auto& _skels : skels){
		fs << _skels.size() << std::endl;
		for (const auto& _skel : _skels)
			fs << _skel.first << std::endl << _skel.second << std::endl;
	}

	fs.close();
}

void WriteSkelToJson(const std::map<int, Eigen::Matrix4Xf >& skel, const std::string& filename)
{
	//const SkelDef& def = GetSkelDef(SKEL19);
	Json::Value vec(Json::arrayValue);
	Json::Value item;

	for (const auto& _skel : skel) {
		item["id"] = _skel.first;
		Json::Value kp3d(Json::arrayValue);
		//std::cout <<"_skel的大小为："<< _skel.second.cols() << std::endl;
		for (int jIdx = 0; jIdx < _skel.second.cols(); jIdx++) {
			//std::cout << _skel.second.col(jIdx) << std::endl;
			const float* point = _skel.second.col(jIdx).data();
			Json::Value kp3d_joint(Json::arrayValue);
			for (int i = 0; i < 4; i++) {
				kp3d_joint.append(Json::Value(point[i]));
			}
			kp3d.append(kp3d_joint);
			kp3d_joint.clear();
		}
		item["keypoints3d"] = kp3d;
		vec.append(item);
		kp3d.clear();
	}
	vec.toStyledString();
	std::string out = vec.toStyledString();
	std::ofstream fs(filename);
	fs << out;
	fs.close();
	//std::cout << out << std::endl;

}

SkelDriver::SkelDriver(const SkelType& _type, const std::string& modelPath)
{
	m_type = _type;
	const SkelDef& def = GetSkelDef(m_type);
	
	m_joints = MathUtil::LoadMat<float>((std::filesystem::path(modelPath) / std::filesystem::path("joints.txt")).string()).transpose();
	m_jShapeBlend = MathUtil::LoadMat<float>((std::filesystem::path(modelPath) / std::filesystem::path("jshape_blend.txt")).string());

	assert(m_joints.cols() ==def.jointSize
		&& m_jShapeBlend.rows() == 3 *def.jointSize
		&& m_jShapeBlend.cols() ==def.shapeSize);
}


Eigen::Matrix3Xf SkelDriver::CalcJBlend(const SkelParam& param) const
{
	const Eigen::VectorXf jOffset = m_jShapeBlend * param.GetShape();
	const Eigen::Matrix3Xf jBlend = m_joints + Eigen::Map<const Eigen::Matrix3Xf>(jOffset.data(), 3, m_joints.cols());
	return jBlend;
}


Eigen::Matrix4Xf SkelDriver::CalcNodeWarps(const SkelParam& param, const Eigen::Matrix3Xf& jBlend) const
{
	const SkelDef& def = GetSkelDef(m_type);
	Eigen::Matrix4Xf nodeWarps = Eigen::Matrix4Xf(4, jBlend.cols() * 4);
	for (int jIdx = 0; jIdx < jBlend.cols(); jIdx++) {
		Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
		if (jIdx == 0)
			matrix.topRightCorner(3, 1) = jBlend.col(jIdx) + param.GetTrans();
		else
			matrix.topRightCorner(3, 1) = jBlend.col(jIdx) - jBlend.col(def.parent[jIdx]);

		matrix.topLeftCorner(3, 3) = MathUtil::Rodrigues<float>(param.GetPose().segment<3>(3*jIdx));
		nodeWarps.block<4, 4>(0, 4 * jIdx) = matrix;
	}
	return nodeWarps;
}


Eigen::Matrix4Xf SkelDriver::CalcChainWarps(const Eigen::Matrix4Xf& nodeWarps) const
{
	const SkelDef& def = GetSkelDef(m_type);
	Eigen::Matrix4Xf chainWarps(4, nodeWarps.cols());
	for (int jIdx = 0; jIdx < nodeWarps.cols() / 4; jIdx++) 
		if (jIdx == 0)
			chainWarps.middleCols(jIdx * 4, 4) = nodeWarps.middleCols(jIdx * 4, 4);
		else
			chainWarps.middleCols(jIdx * 4, 4) = chainWarps.middleCols(def.parent[jIdx] * 4, 4) * nodeWarps.middleCols(jIdx * 4, 4);
	return chainWarps;
}


Eigen::Matrix3Xf SkelDriver::CalcJFinal(const Eigen::Matrix4Xf& chainWarps) const
{
	Eigen::Matrix3Xf jFinal(3, chainWarps.cols() / 4);
	for (int jIdx = 0; jIdx < jFinal.cols(); jIdx++)
		jFinal.col(jIdx) = chainWarps.block<3, 1>(0, 4 * jIdx + 3);
	return jFinal;
}


Eigen::Matrix3Xf SkelDriver::CalcJFinal(const SkelParam& param, const int& _jCut) const
{
	const int jCut = _jCut > 0 ? _jCut : m_joints.cols();
	const Eigen::Matrix3Xf jBlend = CalcJBlend(param);
	return CalcJFinal(CalcChainWarps(CalcNodeWarps(param, jBlend.leftCols(jCut))));
}

