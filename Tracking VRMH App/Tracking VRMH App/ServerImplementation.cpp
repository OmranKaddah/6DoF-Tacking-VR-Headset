#include"ServerHeader.h"
using namespace std;
server::server(cv::VideoCapture & _cam, cv::Mat  & _cameraMat, cv::Mat & _distCoefficient, int _portNumbe) :
	camera(&_cam), cameraMat(&_cameraMat), distCoefficient(&_distCoefficient), portNumber(_portNumbe)
{}


int server::run() {
	cv::Mat frame;
	string outputs;
	if (!camera->isOpened())
	{
		return -1;
	}

	////Initialize winsock
	WSAData wsData;
	WORD ver = MAKEWORD(2, 2);

	int wsOk = WSAStartup(ver, &wsData);

	if (wsOk != 0)
	{
		cerr << "Can't initialize winsock! Quitting" << endl;
		return 0;
	}

	//Creat a socket

	SOCKET listening = socket(AF_INET, SOCK_STREAM, 0);
	if (listening == INVALID_SOCKET)
	{
		cerr << "Can't create a socket! Quitting" << endl;
		return 0;
	}
	//Bind the sockt to an  IP address and port
	sockaddr_in hint;
	hint.sin_family = AF_INET;
	hint.sin_port = htons(54001);
	hint.sin_addr.S_un.S_addr = INADDR_ANY;// inet_pton

	bind(listening, (sockaddr*)&hint, sizeof(hint));
	// Tell winsock that socket is for listening

	listen(listening, SOMAXCONN);

	// Wait for connection

	sockaddr_in client;
	int clientSize = sizeof(client);
	SOCKET clientSocket = accept(listening, (sockaddr*)&client, &clientSize);
	if (clientSocket == INVALID_SOCKET)
	{
		cerr << "this is invalid socket" << endl;
	}

	char host[NI_MAXHOST]; // Client's remoste name
	char service[NI_MAXSERV];//  Service (i.e port) the client is connect on

	ZeroMemory(host, NI_MAXHOST);// same as memset(host, 0 ,NI_MAXHOST);
	ZeroMemory(service, NI_MAXSERV);
	if (getnameinfo((sockaddr*)&client, sizeof(client), host, NI_MAXHOST, service, NI_MAXSERV, 0) == 0)
	{
		cout << host << "connected on port " << service << endl;

	}
	else
	{
		inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
		cout << host << "connect on port " << ntohs(client.sin_port)
			<< endl;

	}
	//Close listening socket
	closesocket(listening);

	//	While loop: accept and echo message back to the client
	char buf[4096];
	fillin();
	while (true)
	{
		if (!camera->read(frame))
			break;

		ZeroMemory(buf, 4096);

		if (camMonitoring(frame, *cameraMat, *distCoefficient, rotationVectors, transaltionVectors) == 1)
		{


			outputs = converIntoString(transaltionVectors, rotationVectors);

			/*	char* pBuffer = new char[sizeof(int) + outputs.size()];

			int iSize = outputs.size();
			memcpy(pBuffer, &iSize, sizeof(int));
			memcpy(pBuffer + sizeof(int), outputs.c_str(), outputs.size());*/

			if (outputs.size() > 0) {
				//send(clientSocket, pBuffer, sizeof(int) + iSize, 0);
				send(clientSocket, outputs.data(), outputs.size(), 0);
			}

		}
		cv::imshow("MyCam", frame);
		if (cv::waitKey(30) >= 0) break;

	}
	closesocket(clientSocket);


	return 1;
}
string server::converIntoString(cv::Mat m, cv::Mat d) {

	string p = "";

	for (int index = 0; index < m.rows; index++)
	{
		for (int col = 0; col < m.cols; col++)
			p += std::to_string(m.at<double>(index, col)) + ";";

	}


	for (int index = 0; index < d.rows; index++)
	{
		for (int col = 0; col < d.cols; col++)
			p += std::to_string(d.at<double>(index, col)) + ";";

	}
	p.pop_back();

	p += "\n";

	return p;
}
