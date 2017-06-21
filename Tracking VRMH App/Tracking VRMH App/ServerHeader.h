
#ifndef SERVERHEADER_H
#define SERVERHEADER_H
#include"TrackingSystemComponents.h"
#include<fstream>
#include<string>
#include<WS2tcpip.h>
#include<iostream>
#pragma comment(lib,"ws2_32.lib")

class server {

private:
	int portNumber;
	cv::VideoCapture *camera;
	cv::Mat frame;
	cv::Mat  *cameraMat; cv::Mat * distCoefficient;
	cv::Mat rotationVectors, transaltionVectors;
public:
	server(cv::VideoCapture & _cam, cv::Mat  & _cameraMat, cv::Mat &  _distCoefficient, int _portNumber = 54000);
	int run();

	std::string  converIntoString(cv::Mat m, cv::Mat d);



};

#endif // !SERVERHEADER_H

#pragma once