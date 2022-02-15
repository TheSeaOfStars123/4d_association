#include "BaseSocket.h"

BaseSocket::BaseSocket(const std::string & host, const int & port)
{
	printf("[Info] server start\n");
	// ����Socket�������׽��ֿ⣬�����׽���(WSAStartup()/socket())�� 
	// Creating socket file descriptor
	if ((sockServer = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
		perror("socket failed");
		exit(EXIT_FAILURE);
	}
	//׼��ͨ�ŵ�ַ  
	addrServer.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	addrServer.sin_family = AF_INET;
	addrServer.sin_port = htons(port);
	//�󶨣����׽��ֵ�һ��IP��ַ��һ���˿���(bind())��
	 // Forcefully attaching socket to the port 8080
	if (bind(sockServer, (SOCKADDR*)&addrServer, sizeof(SOCKADDR)) < 0)
	{
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	//���������׽�������Ϊ����ģʽ�ȴ���������(listen())����
	if (listen(sockServer, 5) < 0)
	{
		perror("listen");
		exit(EXIT_FAILURE);
	}
	t = std::thread(run, this);
	//t.join();
	disconnect = FALSE;
}

void run(BaseSocket* bs) {

	printf("Friend function from thread\n");
	int len = sizeof(SOCKADDR);
	while (TRUE) {
		if ((bs->conn = accept(bs->sockServer, (SOCKADDR*)&bs->addr, &len)) < 0)
		{
			perror("accept");
			exit(EXIT_FAILURE);
		}
		printf("[Info] Connect:\n", bs->conn);
		bs->disconnect = FALSE;
		while (TRUE) {
			sockLine sL = recvLine(bs->conn);
			// ȥ��ĩβ�Ļ����ַ�
			bs->queue.push(sL.result.substr(0, sL.result.length() - 1));
			if (!sL.flag) {
				printf("[Info] Disonnect\n");
				bs->disconnect = TRUE;
				break;
			}
			//bs->data = recvAll(bs->conn, bs->queue.front().c_str());
		}
		closesocket(bs->conn);
	}
}
char* recvAll(SOCKET& sock, const char* recvBuf) {
	std::vector<char> data;
	char* dynData;
	int dynData_len = atoi(recvBuf);
	dynData = new char[dynData_len];
	recv(sock, dynData, dynData_len, 0);
	return dynData;

}
sockLine recvLine(SOCKET& sock)
{
	sockLine sL = sockLine();
	sL.flag = TRUE;
	char recvBuf_1[1];
	//while (!hasEnding(sL.result, "\n")) {
	while (sL.result.length() == 0 || strcmp(&sL.result[sL.result.length() - 1], "\n")) {
		recv(sock, recvBuf_1, 1, 0);
		if(recvBuf_1[0] == 'Q')
		{
			sL.flag = FALSE;
			sL.result = "Q\n";
			break;
		}
		else {
			sL.result += recvBuf_1[0];
			// strcmp���Ϊ0 ����Ϊ1
			bool isEuqal = strcmp(&sL.result[sL.result.length() - 1], "\n");
		}
	}
	return sL;
}

