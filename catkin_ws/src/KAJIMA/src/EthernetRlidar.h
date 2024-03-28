#pragma once
#ifndef __ETHERNET_RLIDAR_COMMUN__
#define __ETHERNET_RLIDAR_COMMUN__

#include <stdio.h>
#include <sys/types.h>
#include <iostream>
#include <vector>
#include <string> 
#include <fstream>

#include <time.h>

//#include "ConvenientTool.h"



#ifdef _WIN32
//#include <ws2tcpip.h>
#include <time.h>
#include <windows.h>
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "User32.lib")
#pragma comment(lib,"Winmm.lib")
//#define close(A) closesocket(A) 
typedef int socklen_t;
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#define closesocket(A) close(A) 
#define Sleep(A) usleep(1000 *A) 

#endif

#define ETH_RLIDAR_SERVER_PORT 8856
#define ETH_RLIDAR_BUFF_LEN 24
#define RECV_RLIDAR_LIST_MAX 128
#define PEAR_RLIDAR_SIZE 240


class Remote_Rlidar_Server {
public:
	Remote_Rlidar_Server() {
#ifdef _WIN32
		WSAData wsaData;
		WORD  DLLVSERION = 0x102; // MAKEWORD(2, 1);0x102
		WSAStartup_res = WSAStartup(DLLVSERION, &wsaData);
#endif
	}
	~Remote_Rlidar_Server() {

	}
//parameter
public:
	std::vector<int> LidarDis;
	std::vector<int> LidarTh;
	bool Ether_Work = true;
private:
	int Pos[3] = {0};
	char buf[ETH_RLIDAR_BUFF_LEN] = { 0 };
	char buf_GetSize[4] = { 0 };
	char buf_Replay[ETH_RLIDAR_BUFF_LEN] = { 0 };
	int returnType = 0;
	int recvRlidarPtr = 0;
	int sListen, ret = 0;
	int sockfd = 0;
	int forClientSockfd = 0;
	struct sockaddr_in serverInfo;
	socklen_t addrlen = sizeof(serverInfo);
	int AutoKillCount = 0;
	int AutoKillCountMax = 499;
	int recv_function_Fail = 0;
	fd_set rfd;
	int selectReturn;
	timeval timeout = { 0,300 };
	bool EthGetData = false;
	int WSAStartup_res;
	bool ClientAskReturn = false;
	bool MacroDateAskReturn = false;
	bool SensorAskReturn = false;
	int ClientAskReturn_Dir;
	int MacroDateAsReturn_Dir;
	int CmdListDir_Save = 0;
	int CmdListDir_Now = 0;
	int recvSize = 0;
	//ConvenientTool Tool;
//function
public:
	void Socket_init() {
		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0)
		{
			printf("create socket fail!\n");
			system("pause");
			return;
		}
		memset(&serverInfo, 0, sizeof(serverInfo));

		serverInfo.sin_family = AF_INET;
		serverInfo.sin_port = htons(ETH_RLIDAR_SERVER_PORT);

		ret = ::bind(sockfd, (struct sockaddr*)&serverInfo, sizeof(serverInfo));
		int option = 1;
		setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const char*)&option, sizeof(option));

		if (ret < 0)
		{
			perror("bind");
			printf("socket bind fail!\n");
			return;
		}
		if (listen(sockfd, 5) == -1) {
			perror("listen");
			printf("socket listen fail!\n");
			return;

		}
		printf("recieve start! \n");
	}

	int ether_main(void)
	{
		while (Ether_Work) {
			forClientSockfd = accept(sockfd, (struct sockaddr*)&serverInfo, &addrlen);
			printf("Server %d accept %d \n", sockfd, forClientSockfd);
			if (forClientSockfd != -1){
				printf("forClientSockfd %d, sockfd %d \n", forClientSockfd, sockfd);
			}
			else {

			}
			handle_udp_msg(forClientSockfd);
			printf("handle_udp_msg End %d, sockfd %d \n", forClientSockfd, sockfd);
			closesocket(forClientSockfd);
			printf("closesocket End %d, sockfd %d \n", forClientSockfd, sockfd);

			closesocket(sockfd);
			Socket_init();
			Sleep(1000);
		}

		closesocket(sockfd);
		return 1;
	}

	void UpdatePos(double GetPos_x, double GetPos_y, double GetPos_d) {
		Pos[0] = GetPos_x * 10000;
		Pos[1] = GetPos_y * 10000;
		Pos[2] = GetPos_d * 10000;
	}

private:
	int CombineCharToInt(char* tmp, int count) {
		int tmpV = 0;
		for (int i = 0; i < count; i++) {
			tmpV = tmpV | ((tmp[i] & 0x0ff) << ((count - i - 1) * 8));
		}
		return tmpV;
	}
	void cutIntToChar(char* tmpc, int input, int count) {
		for (int i = count - 1; i >= 0; i--) {
			tmpc[i] = input & 0x0ff;
			input = input >> 8;
		}
	}
	void handle_udp_msg(int fd)
	{
		socklen_t len;
		int count;
		int coooo = 0;

		printf( "recieve start! \n");
		while (1) {

			coooo++;
			//if (EthGetData) {
			//	printf("==========[ %d ]=========== \n", coooo);
			//}

			memset(buf, 0, ETH_RLIDAR_BUFF_LEN);
			
			EthGetData = false;

			////time out setting
			//FD_ZERO(&rfd);
			//FD_SET(forClientSockfd, &rfd);
			//selectReturn = select(forClientSockfd + 1, &rfd, NULL, NULL, &timeout);
			//if (selectReturn < 0) {
			//	printf( "Error connecting %d\n", errno);
			//}
			//else if (selectReturn > 0) {
			//	printf( "selectReturn Get %d\n", selectReturn);
			//	int recv_function_res = recv_function(buf);
			//	if (recv_function_res == (-1)) {
			//		printf( "recv_function Get %d, \n", recv_function_res);
			//		break;
			//	}
			//	int send_function_res = sent_function(buf_Replay);
			//}
			//else { //timeout
			//	//AutoKillCount++;
			//	//if ((AutoKillCount % 100) == 0) {
			//	//	printf( "AutoKillCount = %d\n", AutoKillCount);
			//	//}
			//	//if (AutoKillCount > AutoKillCountMax) {
			//	//	printf( "AutoKillCount Get Max %d, \n", AutoKillCount);
			//	//	break;
			//	//}
			//}
			int recv_function_res = recv_function(buf);
			if (recv_function_res == (-1)) {
				printf("recv_function Get %d, \n", recv_function_res);
				break;
			}
			int send_function_res = sent_function();

		}
	}
	int sent_function(void) {
		
		int send_count = 0;
		if (returnType == 1 || returnType == '1') {
			SendSizeChar(ETH_RLIDAR_BUFF_LEN);
			send_count = SelfDateSend();
		}
		else if (returnType == 2) {			
			std::vector<char> maptmp;
			int sizetmp = 8 * LidarDis.size() ;
			SendSizeChar(sizetmp);
			send_count = RlidarData_T1_Send(sizetmp);
		}
		else if (returnType == 3) {
			send_count = SelfDateSend();
		}
		else if (returnType == 4) {
			send_count = RlidarData_T2_Send(recvRlidarPtr);
		}
		else {
			SendSizeChar(ETH_RLIDAR_BUFF_LEN);
			memset(buf_Replay, 0, ETH_RLIDAR_BUFF_LEN);
			//cutIntToChar(mat, ETH_RLIDAR_BUFF_LEN, 2);
			send_count = send(forClientSockfd, buf_Replay, sizeof(buf_Replay), 0);
		}
		return send_count;
	}
	int  SelfDateSend() {
		int sizetmp = LidarDis.size();
		memset(buf_Replay, 0, ETH_RLIDAR_BUFF_LEN);
		//cutIntToChar(mat, ETH_RLIDAR_BUFF_LEN, 2);
		cutIntToChar(buf_Replay, Pos[0], 4);
		cutIntToChar(buf_Replay + 4, Pos[1], 4);
		cutIntToChar(buf_Replay + 8, Pos[2], 4);
		cutIntToChar(buf_Replay + 12, sizetmp, 4);
		return send(forClientSockfd, buf_Replay, sizeof(buf_Replay), 0);
	}
	int RlidarData_T1_Send(int sizetmp) {
		std::vector<char> maptmp;
		int tmpptr;
		maptmp.assign(sizetmp, 0);
		for (int i = 0; i < LidarDis.size(); i++) {
			tmpptr = i * 8;
			cutIntToChar(&(maptmp[0]) + tmpptr, LidarDis.at(i), 4);
			cutIntToChar(&(maptmp[0]) + tmpptr + 4, LidarTh.at(i), 4);
		}
		//send(forClientSockfd, &maptmp[0], 2, 0);
		return send(forClientSockfd, &maptmp[0], sizetmp, 0) + 2;
	}
	int RlidarData_T2_Send(int ptr) {
		printf("RlidarData_T2_Send [%d] \n", ptr);

		char maptmp[1920];
		memset(maptmp, 0, 1920);
		int tmpptr;
		int sizetmp;
		int ptrmax;
		ptr = ptr * PEAR_RLIDAR_SIZE;
		if (ptr >= LidarDis.size() ) {
			ptr = 0;			
		}
		ptrmax = ptr + PEAR_RLIDAR_SIZE;
		if (ptrmax >= LidarDis.size()) {
			ptrmax = LidarDis.size();
		}
		printf("RlidarData_T2_Send [%d]_[%d] / %d \n", ptr, ptrmax, LidarDis.size());

		for (int i = ptr, matptr = 0; i < ptrmax; i++) {
			cutIntToChar(maptmp + matptr, LidarDis.at(i), 4);
			cutIntToChar(maptmp + matptr + 4, LidarTh.at(i), 4);
			matptr+=8;
		}


		return send(forClientSockfd, maptmp, 1920, 0) ;

	}
	int recv_function(char* mat) {
		int recv_count = recv(forClientSockfd, mat, sizeof(buf), 0);
		if (recv_count <= 0) {
			printf("recieve data fail! count = %d\n", recv_count);
			recv_function_Fail = 1;
			return -1;
		}
		else {
			printf("recieve recvfrom! count = %d \n", recv_count);
			EthGetData = true;
			AutoKillCount = 0;
			recv_function_Fail = 0;
			int tmp_cmdValue = 0;
			returnType = 0;
			if (buf[0]) {

				returnType = buf[0];
				if (returnType == 4) {
					recvRlidarPtr = buf[1];
				}
				printf("Cmd [%d]\n", returnType);
			}
		}
		return recv_count;
	}
	void SendSizeChar(int size) {
		cutIntToChar(buf_GetSize, size, 4);
		int tmp = send(forClientSockfd, buf_GetSize, sizeof(buf_GetSize), 0);
		//printf("[%d], [%d]\n", 0x0ff & buf_GetSize[2], 0x0ff & buf_GetSize[3]);
		int tmp2 = recv(forClientSockfd, buf_GetSize, sizeof(buf_GetSize), 0);
		//printf("size1 %d, size2 %d\n",tmp, tmp2);
		recvSize = size;
	}
};

class Remote_Rlidar_Client {
public:
	struct DataStruct
	{
	public:
		uchar		flag = 0;
		double	raw_distance_mm = 0;
		double	angle_deg = -1;
		double	angle_th = 0;
		uchar		quality = 0;
		double	x = 0;
		double	y = 0;
	};
	Remote_Rlidar_Client() {
#ifdef _WIN32
		WSAData wsaData;
		WORD  DLLVSERION = 0x102; // MAKEWORD(2, 1);0x102
		WSAStartup_res = WSAStartup(DLLVSERION, &wsaData);
#endif
	}
	~Remote_Rlidar_Client() {

	}
	//parameter
public:

	bool Ether_Work = true;
	int RPlidar_data_Size=0;
	double Recv_Pos[3] = { 0.0 };
	std::vector<DataStruct> RPlidar_data;
	double MaxRawDistance = 0;
private:
	const DataStruct Vector_0_Reset;
	std::vector<char> LidarrDate;
	std::vector<double> LidarDis_float;
	std::vector<double> LidarTh_float;
	char buf[ETH_RLIDAR_BUFF_LEN] = { 0 };
	char buf_Replay[ETH_RLIDAR_BUFF_LEN] = { 0 };
	char buf_GetSize[4] = { 0 };
	int sListen, ret;
	int sockfd = 0;
	int forClientSockfd = 0;
	int AutoKillCount = 0;
	int AutoKillCountMax = 149;
	int recv_function_Fail = 0;
	fd_set rfd;
	int selectReturn;
	timeval timeout = { 0,300 };
	bool EthGetData = false;
	std::string SERVER_IP = "127.0.0.1";
	int SERVER_PORT;
	int WSAStartup_res;
	bool Client_WantDateSend = false;
	//ConvenientTool Tool;
	int buf_Write_dir_Now = 0;
	int recvSize;
	int ServerSayRlidarSize =0;

public:
	void Socket_init(std::string IP, int PORT) {
		SERVER_PORT = PORT;
		SERVER_IP = IP;

		sockfd = socket(AF_INET, SOCK_STREAM, 0);
		if (sockfd < 0)
		{
			printf("create socket fail!\n");
			system("pause");
			return;
		}

		struct sockaddr_in clientInfo;
		memset(&clientInfo, 0, sizeof(clientInfo));
		clientInfo.sin_family = AF_INET;
		clientInfo.sin_port = htons(SERVER_PORT);
		clientInfo.sin_addr.s_addr = inet_addr(SERVER_IP.c_str());

		int err = connect(sockfd, (struct sockaddr*)&clientInfo, sizeof(clientInfo));
		if (err == -1) {
			printf("Connection error");
		}
		printf("Connect start! \n");
	}
	
	int ether_send(int type = 0, int ptr = 0) {
		int GetSendCount = 0;
		int GetRecvCount = 0;
		if (type == 1) {
			buf[0] = type;
			GetSendCount = sent_function(buf);
			GetSizeChar();
			GetRecvCount = GetRecvPos();
		}
		else if (type == 2) {
			buf[0] = type;
			GetSendCount = sent_function(buf);
			GetSizeChar();
			GetRecvCount = GetRecvRlidar();
		}
		if (type == 3) {
			buf[0] = type;
			GetSendCount = sent_function(buf);
			GetRecvCount = GetRecvPos();
		}
		else if (type == 4) {
			buf[0] = type;
			buf[1] = ptr;
			GetSendCount = sent_function(buf);
			GetRecvCount = GetRecvRlidar_2(ptr);
		}
		else {
			memset(buf, 0, ETH_RLIDAR_BUFF_LEN);
			GetSendCount = sent_function(buf);
			GetSizeChar();
			GetRecvCount = recv_function(buf_Replay);
		}
		//printf("GetSendCount %d, GetRecvCount %d \n",GetSendCount, GetRecvCount);

		return GetRecvCount;
	}

private:
	int CombineCharToInt(char* tmp, int count) {
		int tmpV = 0;
		for (int i = 0; i < count; i++) {
			tmpV = tmpV | ((tmp[i] & 0x0ff) << ((count - i - 1) * 8));
		}
		return tmpV;
	}
	void cutIntToChar(char* tmpc, int input, int count) {
		for (int i = count - 1; i >= 0; i--) {
			tmpc[i] = input & 0x0ff;
			input = input >> 8;
		}
	}
	int GetRecvPos() {
		int GetRecvCount = recv_function(buf_Replay);
		Recv_Pos[0] = CombineCharToInt(buf_Replay , 4) / 10000.0;
		Recv_Pos[1] = CombineCharToInt(buf_Replay + 4, 4) / 10000.0;
		Recv_Pos[2] = CombineCharToInt(buf_Replay + 8, 4) / 10000.0;
		int ServerSayRlidarSize_tmp = CombineCharToInt(buf_Replay + 12, 4);
		if (ServerSayRlidarSize_tmp) {
			ServerSayRlidarSize = ServerSayRlidarSize_tmp;
		}
		//printf("Recv_Pos %.4f,%.4f,%.4f\n", Recv_Pos[0], Recv_Pos[1], Recv_Pos[2]);
		return GetRecvCount;
	}
	int GetRecvRlidar() {
		//char GetSizeChar[2];
		int tmpptr;

		int GetSize = recvSize;

		int getcount;

		if (GetSize > 8000) {
			char tmpddd[8000];
			for (int i = 0; (i < 3) || (getcount > 0); i++) {
				getcount = recv(sockfd, tmpddd, 8000, 0);
				printf("ERROR getcount = %d\n", getcount);
			}
			return getcount;
		}
		else {
			if (LidarrDate.size() < GetSize) {
				LidarrDate.clear();
				LidarrDate.assign(GetSize, 0);
			}
			getcount = recv(sockfd, &LidarrDate[0], GetSize, 0);
		}

		if (getcount != GetSize) {
			//printf("getcount %d, GetSize %d \n", getcount, GetSize);
			//GetSize = getcount;
			int getcount_2 = recv(sockfd, &LidarrDate[getcount-1], GetSize - getcount, 0);
			//printf("getcount_2 %d, GetSize %d \n", getcount_2, GetSize);
			if ((getcount + getcount_2) != GetSize)
				printf("(getcount %d + getcount_2 %d ) != GetSize %d \n", getcount, getcount_2, GetSize);
		}
		GetSize = GetSize / 8;
		if (LidarDis_float.size() < GetSize ) {
			LidarDis_float.clear();
			LidarTh_float.clear();
			LidarDis_float.assign(GetSize, 0.0);
			LidarTh_float.assign(GetSize, 0.0);
		}

		for (int i = 0; i < GetSize;i++) {
			tmpptr = i * 8;
			LidarDis_float.at(i) = CombineCharToInt(&LidarrDate[tmpptr], 4) / 10000.0;
			tmpptr += 4;
			LidarTh_float.at(i) = CombineCharToInt(&LidarrDate[tmpptr], 4) / 10000.0;
		}
		RPlidar_dataUpdate(GetSize);
		return GetSize * 8;
		//printf("getcount %d \n", getcount);
	}
	int GetRecvRlidar_2(int ptr) {

		if (LidarDis_float.size() < ServerSayRlidarSize) {
			LidarDis_float.clear();
			LidarTh_float.clear();
			LidarDis_float.assign(ServerSayRlidarSize, 0.0);
			LidarTh_float.assign(ServerSayRlidarSize, 0.0);
				LidarrDate.clear();
			LidarrDate.assign(ServerSayRlidarSize * 8, 0);
			}

		char recvTmp[1920];
		memset(recvTmp, 0, 1920);
		int getcount = recv(sockfd, recvTmp, 1920, 0);

			ptr = ptr * PEAR_RLIDAR_SIZE;
		if (ptr > ServerSayRlidarSize) {
			printf("ptr (%d) > ServerSayRlidarSize (%d) \n", ptr, ServerSayRlidarSize);
				ptr = 0;
			}
		int ptrmax = ptr + 240;
		if (ptrmax > ServerSayRlidarSize) {
			ptrmax = ServerSayRlidarSize;
			}
		int tmpptr;
		for (int i = ptr, iptr = 0 ; i < ptrmax; i++, iptr+=8) {
			tmpptr = i * 8;
			LidarDis_float.at(i) = CombineCharToInt(recvTmp + iptr, 4) / 10000.0;
			tmpptr += 4;
			LidarTh_float.at(i) = CombineCharToInt(recvTmp + iptr + 4, 4) / 10000.0;
		}


		RPlidar_dataUpdate(ServerSayRlidarSize);
		return ptrmax - ptr;
	}
	void RPlidar_dataUpdate(int datasize) {
		int count_int = datasize;
		int pos = 0;
		double Postmp[2];
		double angle_th, angle_deg, rawData;
		double MaxDis = 0;
		if (RPlidar_data.size() < count_int) {
			RPlidar_data.clear();
			RPlidar_data.assign(count_int, Vector_0_Reset);
		}

		for (pos = 0; pos < count_int; pos++) {
			angle_th = LidarTh_float.at(pos);
			angle_deg = (((angle_th / 3.14159265358979323846) * 180 ) + 360);
			
			rawData = LidarDis_float.at(pos);
			Postmp[0] = (rawData * cos(angle_th));
			Postmp[1] = (rawData * sin(angle_th));

			RPlidar_data[pos].angle_deg = angle_deg;
			RPlidar_data[pos].angle_th = angle_th;
			RPlidar_data[pos].raw_distance_mm = rawData;
			//RPlidar_data[pos].flag = RPlidar_nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT;
			RPlidar_data[pos].x = Postmp[0];
			RPlidar_data[pos].y = Postmp[1];

			if (MaxDis < rawData) {
				MaxDis = rawData;
			}
		}
		RPlidar_data_Size = count_int;
		MaxRawDistance = MaxDis;
	}
	int sent_function(char* mat) {
		int send_count = send(sockfd, mat, sizeof(buf), 0);
		buf_Write_dir_Now = 0;
		return send_count;
	}
	int recv_function(char* mat) {
		memset(mat, 0, ETH_RLIDAR_BUFF_LEN);
		int recv_count = recv(sockfd, mat, sizeof(buf_Replay), 0);
		if (recv_count <= 0) {
			printf("recieve data fail! count = %d\n", recv_count);
			recv_function_Fail = 1;
			return -1;
		}
		else {
			//printf("recieve recvfrom! count = %d \n", recv_count);
			EthGetData = true;
			AutoKillCount = 0;
			recv_function_Fail = 0;					
		}
		return recv_count;
	}
	int GetSizeChar() {
		recv(sockfd, buf_GetSize, sizeof(buf_GetSize), 0);

		int GetSize = CombineCharToInt(buf_GetSize,4);
		//printf("buf_GetSize[%d][%d] %d\n", 0x0ff & buf_GetSize[2], 0x0ff & buf_GetSize[3], GetSize);
		cutIntToChar(buf_GetSize, GetSize, 4);
		send(sockfd, buf_GetSize, sizeof(buf_GetSize), 0);
		
		return recvSize = GetSize;
	}
};

#endif