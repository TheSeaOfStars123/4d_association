#pragma once
#include <Winsock2.h>
#include <windows.h>
#include <thread>
#pragma comment(lib,"ws2_32.lib")
// Queue in Standard Template Library (STL)
#include <queue>

class BaseSocket;
class BaseSocket
{
public:
	BaseSocket(const std::string& host, const int& port);
	friend void run(BaseSocket* bs);
	std::queue<std::string>& getQueue() { return queue; }
	bool getDisConnect() { return disconnect; }
	char* getData() { return data; }
	void detachThread() { t.detach(); }
protected:
	std::string host;
	int port = 8010;
	SOCKET sockServer;
	SOCKADDR_IN addrServer;

	SOCKET conn;
	SOCKADDR_IN addr;
	std::queue<std::string> queue;
	char* data;
	std::thread t;
	bool disconnect = FALSE;


};
struct sockLine {
	bool flag;
	std::string result;
};

sockLine recvLine(SOCKET& sock);
//std::vector<byte> recvAll(SOCKET& sock, int& limit);
char* recvAll(SOCKET& sock, const char* recvBuf);