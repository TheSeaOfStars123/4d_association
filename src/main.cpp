#include "kruskal_associater.h"
#include "skel_updater.h"
#include "skel_painter.h"
#include "openpose.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <json/json.h>
#include <Eigen/Dense>
#include <filesystem>
#include <stdio.h>
#include <vector> 
#include "BaseSocket.h"

void replaceAll(std::string& str, const std::string& from, const std::string& to) {
	if (from.empty())
		return;
	size_t start_pos = 0;
	while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
		str.replace(start_pos, from.length(), to);
		start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
	}
}
char* read_keypoints3d_encode(std::string jsonFile) {
	Json::Value json;
	std::ifstream fs(jsonFile);
	if (!fs.is_open()) {
		std::cerr << "json file not exist: " << jsonFile << std::endl;
		std::abort();
	}
	std::string errs;
	Json::parseFromStream(Json::CharReaderBuilder(), fs, &json, &errs);
	fs.close();

	if (errs != "") {
		std::cerr << "json read file error: " << errs << std::endl;
		std::abort();
	}
	std::map<int, Eigen::MatrixX4f> keypoints3d;
	std::string data = json.toStyledString();
	replaceAll(data, "\r", "");
	replaceAll(data, "\n", "");
	replaceAll(data, "\t", "");
	replaceAll(data, " ", "");
	
	std::cout << data << std::endl;
	std::cout << data.c_str() << std::endl;
	char *p = (char*)data.c_str();
	std::cout << p << std::endl;
	return p;
}

int main(int argc, char* argv[])
{
	const std::string path = argv[1];
	const std::string skel_path = argv[2];
	const std::string output_path = argv[3];
	const std::string ext = argv[4];
	std::map<std::string, Camera> cameras = ParseCameras(path + "/calibration.json");
	Eigen::Matrix3Xf projs(3, cameras.size() * 4);
	std::vector<cv::Mat> rawImgs(cameras.size());
	std::vector<cv::VideoCapture> videos(cameras.size());
	std::vector<std::vector<OpenposeDetection>> seqDetections(cameras.size());
	const SkelDef& skelDef = GetSkelDef(SKEL19);
	std::vector<std::map<int, Eigen::Matrix4Xf>> skels;
	cv::Size imgSize;
	//#pragma omp parallel for
	// 第一步，读取视频到videos
	for (int i = 0; i < cameras.size(); i++) {
		auto iter = std::next(cameras.begin(), i);
		videos[i] = cv::VideoCapture(path + "/video/" + iter->first + ext);
		videos[i].set(cv::CAP_PROP_POS_FRAMES, 0);
		projs.middleCols(4 * i, 4) = iter->second.eiProj;
		imgSize.width = int(videos[i].get(cv::CAP_PROP_FRAME_WIDTH));
		imgSize.height = int(videos[i].get(cv::CAP_PROP_FRAME_HEIGHT));
		rawImgs[i].create(imgSize, CV_8UC3);
	}
	// 第二步，实例化Associater类
	KruskalAssociater associater(SKEL19, cameras);
	associater.SetMaxTempDist(0.3f);
	associater.SetMaxEpiDist(0.15f);
	associater.SetEpiWeight(1.f);
	associater.SetTempWeight(2.f);
	associater.SetViewWeight(1.f);
	associater.SetPafWeight(2.f);
	associater.SetHierWeight(1.f);
	associater.SetViewCntWelsh(1.0);
	associater.SetMinCheckCnt(10);
	associater.SetNodeMultiplex(true);
	associater.SetNormalizeEdge(true);			// new feature

	SkelPainter skelPainter(SKEL19);
	skelPainter.rate = 512.f / float(cameras.begin()->second.imgSize.width);
	SkelFittingUpdater skelUpdater(SKEL19, skel_path);

	skelUpdater.SetTemporalTransTerm(1e-1f / std::powf(skelPainter.rate, 2));
	skelUpdater.SetTemporalPoseTerm(1e-1f / std::powf(skelPainter.rate, 2));
	cv::Mat detectImg, assocImg, reprojImg;
	cv::Mat resizeImg;

	// 第一个socket通信：python(客户端)->c++(服务端) 端口号是8010
	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);
	BaseSocket sockServer("", 8010);
	// 第二个socket通信：c++(客户端)->python(服务端) 端口号是：9999
	WSADATA wsaDataClient;
	SOCKET sockClient;//客户端Socket
	SOCKADDR_IN addrServerClient;//服务端地址
	WSAStartup(MAKEWORD(2, 2), &wsaDataClient);
	//新建客户端socket
	sockClient = socket(AF_INET, SOCK_STREAM, 0);
	//定义要连接的服务端地址
	addrServerClient.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");//目标IP
	addrServerClient.sin_family = AF_INET;
	addrServerClient.sin_port = htons(9999);//连接端口
	//连接到服务端
	connect(sockClient, (SOCKADDR*)&addrServerClient, sizeof(SOCKADDR));

	// 第三步，对每一帧的多视图进行openpose+4d_association_graph+输出3D json
	for (int frameIdx = -1; ;) {
		bool flag = false;
		// 用返回的套接字和客户端进行通信(send()/recv())；
		std::string txt_name;
		if (!sockServer.getQueue().empty()) {
			txt_name = sockServer.getQueue().front();
			printf("txt_name:%s\n", txt_name);
			if (strcmp(txt_name.c_str(), "Q") == 0)
				break;
			sockServer.getQueue().pop();
			flag = true;
			frameIdx++;
		}

		if (!flag)
			continue;
		std::string filename = path + "/detections/" + txt_name + ".txt";
		ParseDetectionsByFrames(filename, seqDetections, BODY25, imgSize);
		//ParseDetectionsByFrames(sockServer.getData(), seqDetections, BODY25, imgSize);
		for (int view = 0; view < cameras.size(); view++) {
			auto iter = std::next(cameras.begin(), view);
			associater.SetDetection(view, seqDetections[view][frameIdx].Mapping(SKEL19));
		}
		associater.SetSkels3dPrev(skelUpdater.GetSkel3d());
		associater.Associate();
		skelUpdater.Update(associater.GetSkels2d(), projs);
		skels.emplace_back(skelUpdater.GetSkel3d());
		std::cout << "Finished: " << std::to_string(frameIdx) <<" frame."<< std::endl;
		// 先保存为json文件
		int n_zero = 6;//总共多少位
		auto s_frameIdx = std::to_string(frameIdx);
		auto new_str = std::string(n_zero - min(n_zero, s_frameIdx.length()), '0') + s_frameIdx;
		std::string json_path = output_path + "/keypoints3d/" + new_str + ".json";
		WriteSkelToJson(skelUpdater.GetSkel3d(), json_path);
		// 根据path读取json文件
		//char* data_encode = read_keypoints3d_encode(json_path);
		Json::Value json;
		std::ifstream fs(json_path);
		if (!fs.is_open()) {
			std::cerr << "json file not exist: " << json_path << std::endl;
			std::abort();
		}
		std::string errs;
		Json::parseFromStream(Json::CharReaderBuilder(), fs, &json, &errs);
		fs.close();

		if (errs != "") {
			std::cerr << "json read file error: " << errs << std::endl;
			std::abort();
		}
		std::map<int, Eigen::MatrixX4f> keypoints3d;
		std::string data = json.toStyledString();
		replaceAll(data, "\r", "");
		replaceAll(data, "\n", "");
		replaceAll(data, "\t", "");
		replaceAll(data, " ", "");
		//std::cout << data.c_str() << std::endl;
		char *data_encode = (char*)data.c_str();
		// 发送skels数据
		int data_encode_len = strlen(data_encode);
		std::string s_data_encode_len = std::to_string(data_encode_len) + "\n";
		//std::cout << s_data_encode_len << std::endl;
		send(sockClient, s_data_encode_len.c_str(), strlen(s_data_encode_len.c_str()), 0);
		send(sockClient, data_encode, data_encode_len, 0);
#ifdef SAVE_RESULT		
		// save
		const int layoutCols = 3;
		std::vector<cv::Rect> rois = SkelPainter::MergeImgs(rawImgs, detectImg, layoutCols,
			{ rawImgs.begin()->cols, rawImgs.begin()->rows });
		detectImg.copyTo(assocImg);
		detectImg.copyTo(reprojImg);

		//#pragma omp parallel for
		for (int view = 0; view < cameras.size(); view++) {
			const OpenposeDetection detection = seqDetections[view][frameIdx].Mapping(SKEL19);
			skelPainter.DrawDetect(detection.joints, detection.pafs, detectImg(rois[view]));
			for (const auto& skel2d : associater.GetSkels2d())
				skelPainter.DrawAssoc(skel2d.second.middleCols(view * skelDef.jointSize, skelDef.jointSize), assocImg(rois[view]), skel2d.first);

			for (const auto& skel3d : skelUpdater.GetSkel3d())
				skelPainter.DrawReproj(skel3d.second, projs.middleCols(4 * view, 4), reprojImg(rois[view]), skel3d.first);
		}

		cv::imwrite("../../4d_association_output/" + dataset + "/detect/" + std::to_string(frameIdx) + ".jpg", detectImg);
		cv::imwrite("../../4d_association_output/" + dataset + "/assoc/" + std::to_string(frameIdx) + ".jpg", assocImg);
		cv::imwrite("../../4d_association_output/" + dataset + "/reproj/" + std::to_string(frameIdx) + ".jpg", reprojImg);
#endif
	}
	// 关闭socket1中的进程
	sockServer.detachThread();
	// 关闭socket2
	closesocket(sockClient);
	WSACleanup();

	//if (!skels.empty()) {
	//	SerializeSkels(skels, output_path + "/skel_mocap.txt");
	//}
	return 0;
}