/*/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/
#include"ServerHeader.h"
#include"CamCalibrationHeader.h"
using namespace std;
//void calibrate()
//{}
bool loadCameraCalibration(std::string fileName, cv::Mat & cameraMat, cv::Mat & distanceCoefficient)
{
	cv::FileStorage in(fileName, cv::FileStorage::READ);
	in["cameraMat"] >> cameraMat;
	in["dist"] >> distanceCoefficient;
	return true;

}
//string  converIntoString(cv::Mat m, cv::Mat d) {
//	
//
//	string p = "";
//
//	for (int index = 0; index < m.rows; index++)
//	{
//		for (int col = 0; col < m.cols; col++)
//			p += std::to_string(m.at<double>(index, col)) + ";";
//
//	}
//
//
//	for (int index = 0; index < d.rows; index++)
//	{
//		for (int col = 0; col < d.cols; col++)
//			p += std::to_string(d.at<double>(index, col)) + ";";
//
//	}
//	
//	p += "\n";
//
//	return p;
//}
int main()
{
	//cv::Mat rotationVectors, transaltionVectors;
	//cv::Mat frame;
	cv::Mat  _cameraMat; cv::Mat  _distCoefficient;
	bool found = 0;

	found = loadCameraCalibration("CamAndDistMAT.txt", _cameraMat, _distCoefficient);
	if (!found)
	{
		cout << "CamAndDistMAT.txt was not found, you must calibrate the camera to continue." << endl;
		cout << "if you want to calibrate camera press Enter or any Key to exist" << endl;
		char in;
		cin >> in;
		if (in == 13) {
			int outCali = runCalibratorProgram();
			switch (outCali)
			{
			case 0:
				cout << "The video camera coudn't be opened. Please set your first camera and check that your camera is connected" << endl;
				break;
			case 1:
				cout << "You camera is successfully calibrated, you find the camlibration out out in CamAndDistMAT.txt" << endl;
				break;
			case 3:
				cout << "You've exited the calibration! Press any key to exit the whole program" << endl;
				cin >> outCali;
				return 0;

			default:
				cout << "Error occurred during video monitoring, please check the camera is connected ! Press any key to exit the whole program" << endl;
				cin >> outCali;
				return 0;
				break;
			}
		}

		else return 0;
	}
	printf("done loading the camera Matrix and Distance Coefficient\n");

	//


	cv::VideoCapture video(0);
	
	server camMonitoringServer(video, _cameraMat, _distCoefficient);


	camMonitoringServer.run();

	return 0;


	//	if (!video.isOpened())
	//	{
	//		return -1;
	//	}
	//
	//	////Initialize winsock
	//	WSAData wsData;
	//	WORD ver = MAKEWORD(2, 2);
	//
	//	int wsOk = WSAStartup(ver, &wsData);
	//
	//	if (wsOk != 0)
	//	{
	//		cerr << "Can't initialize winsock! Quitting" << endl;
	//		return 0;
	//	}
	//
	//	//Creat a socket
	//
	//	SOCKET listening = socket(AF_INET, SOCK_STREAM, 0);
	//	if (listening == INVALID_SOCKET)
	//	{
	//		cerr << "Can't create a socket! Quitting" << endl;
	//		return 0;
	//	}
	//	//Bind the sockt to an  IP address and port
	//	sockaddr_in hint;
	//	hint.sin_family = AF_INET;
	//	hint.sin_port = htons(54000);
	//	hint.sin_addr.S_un.S_addr = INADDR_ANY;// inet_pton
	//
	//	bind(listening, (sockaddr*)&hint, sizeof(hint));
	//	// Tell winsock that socket is for listening
	//
	//	listen(listening, SOMAXCONN);
	//
	//	// Wait for connection
	//
	//	sockaddr_in client;
	//	int clientSize = sizeof(client);
	//	SOCKET clientSocket = accept(listening, (sockaddr*)&client, &clientSize);
	//	if (clientSocket == INVALID_SOCKET)
	//	{
	//		cerr << "this is invalid socket" << endl;
	//	}
	//
	//	char host[NI_MAXHOST]; // Client's remoste name
	//	char service[NI_MAXSERV];//  Service (i.e port) the client is connect on
	//
	//	ZeroMemory(host, NI_MAXHOST);// same as memset(host, 0 ,NI_MAXHOST);
	//	ZeroMemory(service, NI_MAXSERV);
	//	if (getnameinfo((sockaddr*)&client, sizeof(client), host, NI_MAXHOST, service, NI_MAXSERV, 0) == 0)
	//	{
	//		cout << host << "connected on port " << service << endl;
	//
	//	}
	//	else
	//	{
	//		inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
	//		cout << host << "connect on port " << ntohs(client.sin_port)
	//			<< endl;
	//
	//	}
	//	//Close listening socket
	//	closesocket(listening);
	//
	////	While loop: accept and echo message back to the client
	//	char buf[4096];
	//	
	//	while (true)
	//	{
	//		if (!video.read(frame))
	//			break;
	//
	//		ZeroMemory(buf, 4096);
	//	//	 wait for client to sned data
	//	/*	int bytesReceived = recv(clientSocket, buf, 4096, 0);
	//		if (bytesReceived == SOCKET_ERROR)
	//		{
	//			cerr << "Error in recv(). Quitting" << endl;
	//			break;
	//		}
	//*/
	//		//if (bytesReceived == 0)
	//		//{
	//		//	cout << "Client disconnected" << endl;
	//		//	break;
	//		//}
	//		//Echo message back to client
	//		//
	//		if (camMonitoring(frame, cameraMat, distCoefficient, rotationVectors, transaltionVectors) == 1) 
	//		{
	//			cout << "Rotatisyon" << endl;
	//
	//
	//			cout << "-------------" << endl;
	//			//cout << transaltionVectors << endl;
	//			//send(clientSocket, buf, bytesReceived + 1, 0);
	//
	//			/*int  Size = rotationVectors.total()*rotationVectors.elemSize();
	//			send(clientSocket, (char *)rotationVectors.data, Size,0);
	//			  Size = transaltionVectors.total()*transaltionVectors.elemSize();*/
	//			outputs = converIntoString(rotationVectors, transaltionVectors);
	//			//outputs = to_string(rotationVectors.at<double>(0, 0));
	//			//*(double*)buf = rotationVectors.at<double>(0,0);
	//			char* pBuffer = new char[sizeof(int) + outputs.size()];
	//		//	ZeroMemory(pBuffer, outputs.size());
	//			int iSize = outputs.size();
	//			memcpy(pBuffer, &iSize, sizeof(int));
	//			memcpy(pBuffer + sizeof(int), outputs.c_str(), outputs.size());
	//			
	//			if (outputs.size() > 0) {
	//				send(clientSocket, pBuffer, sizeof(int) + iSize , 0);
	//			}
	//		
	//		}
	//		cv::imshow("MyCam", frame);
	//		if (cv::waitKey(30) >= 0) break;
	//		
	//	}
	//
	//	//close the socket
	//	//closesocket(clientSocket);
	//
	//	//Shutdown Socket
	//	//WSACleanup();
	//







	return 0;
}