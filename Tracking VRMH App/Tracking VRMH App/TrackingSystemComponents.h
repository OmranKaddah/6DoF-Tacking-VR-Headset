
#pragma once
#ifndef TRACKINGSYSTEMCOMPONENTS_H
#define TRACKINGSYSTEMCOMPONENTS_H
#include"DetectionFunction.h"
#include<opencv2\core.hpp>
#include<opencv2\imgcodecs.hpp>
#include<opencv2\imgproc.hpp>
#include<opencv2\aruco.hpp>
#include<opencv2\highgui.hpp>
#include<opencv2\calib3d.hpp>

#include<iostream>

#include<vector>
//#include"Cornerscoor1dinates.h"



static std::vector<std::vector<cv::Point3f >> coor1;
inline void fillin() {
	std::vector<cv::Point3f> temp;
	temp.push_back({ 0.00615f, 0.02505f, -0.00088f });
	temp.push_back({ 0.06385f, 0.02505f, -0.01736f });
	temp.push_back({ 0.06385f, 0.08505f, -0.01736f });
	temp.push_back({ 0.00615f, 0.08505f, -0.00088f });
	coor1.push_back(temp);
	temp.clear();
	//
	temp.push_back({ 0.07500f, 0.02505f, -0.01912f });
	temp.push_back({ 0.13500f, 0.02505f, -0.01912f });
	temp.push_back( { 0.13500f, 0.08505f, -0.01912f });
	temp.push_back({ 0.07500f, 0.08505f, -0.01912f });
	coor1.push_back(temp);
	temp.clear();
	//
	temp.push_back({ 0.14615f, 0.02505f, -0.01736f });
	temp.push_back({ 0.20385f, 0.02505f, -0.00088f });
	temp.push_back({ 0.20385f, 0.08505f, -0.00088f });
	temp.push_back({ 0.14615f, 0.08505f, -0.01736f });
	coor1.push_back(temp);
	temp.clear();
	//
	/*  */
	temp.push_back({ 0.00000f, -0.02995f, 0.07588f });
	temp.push_back({ 0.06000f, -0.02995f, 0.07588f });
	temp.push_back({ 0.06000f, -0.00766f, 0.02017f });
	temp.push_back({ 0.00000f, -0.00766f, 0.02017f });
	coor1.push_back(temp);
	temp.clear();
	//
	temp.push_back({ 0.07500f, -0.02995f, 0.07588f });
	temp.push_back({ 0.13500f, -0.02995f, 0.07588f });
	temp.push_back({ 0.13500f, -0.00766f, 0.02017f });
	temp.push_back({ 0.07500f, -0.00766f, 0.02017f });
	coor1.push_back(temp);
	temp.clear();
	//
	temp.push_back({ 0.15000f, -0.02995f, 0.07588f });
	temp.push_back({ 0.21000f, -0.02995f, 0.07588f });
	temp.push_back({ 0.21000f, -0.00766f, 0.02017f });
	temp.push_back({ 0.15000f, -0.00766f, 0.02017f });
	coor1.push_back(temp);
	temp.clear();
	//
	/**/
	temp.push_back({ 0.00000f, 0.11777f, 0.02017f });
	temp.push_back({ 0.06000f, 0.11777f, 0.02017f });
	temp.push_back({ 0.06000f, 0.14005f, 0.07588f });
	temp.push_back({ 0.00000f, 0.14005f, 0.07588f });
	coor1.push_back(temp);
	temp.clear();
	temp.push_back({ 0.07500f, 0.11777f, 0.02017f }); 
	temp.push_back({ 0.13500f, 0.11777f, 0.02017f }); 	
	temp.push_back({ 0.13500f, 0.14005f, 0.07588f });
	temp.push_back({ 0.07500f, 0.14005f, 0.07588f });
	coor1.push_back(temp);
	temp.clear();
	temp.push_back({ 0.15000f, 0.11777f, 0.02017f });
	temp.push_back({ 0.21000f, 0.11777f, 0.02017f });
	temp.push_back({ 0.21000f, 0.14005f, 0.07588f });
	temp.push_back({ 0.15000f, 0.14005f, 0.07588f });
	coor1.push_back(temp);
	temp.clear();
	//
	/**/
	temp.push_back({ -0.03000f, 0.02505f, 0.07588f });
		temp.push_back({ -0.00772f, 0.02505f, 0.02017f });
		temp.push_back({ -0.00772f, 0.08505f, 0.02017f });
		temp.push_back({ -0.03000f, 0.08505f, 0.07588f });
		coor1.push_back(temp);
	temp.clear();
	temp.push_back({ 0.21772f, 0.02505f, 0.02017f });
	temp.push_back({ 0.24000f, 0.02505f, 0.07588f }); 
	temp.push_back({ 0.24000f, 0.08505f, 0.07588f });
	temp.push_back({ 0.21772f, 0.08505f, 0.02017f });

	coor1.push_back(temp);
	temp.clear();
}
		
		
	














inline void drawAxis(cv::Mat &_image, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
	cv::InputArray _rvec, cv::InputArray _tvec, float length) {
	// project axis points
	std::vector< cv::Point3f > axisPoints;
	axisPoints.push_back(cv::Point3f(0, 0, 0));
	axisPoints.push_back(cv::Point3f(length, 0, 0));
	axisPoints.push_back(cv::Point3f(0, length, 0));
	axisPoints.push_back(cv::Point3f(0, 0, length));
	std::vector< cv::Point2f > imagePoints;
	cv::projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);

	// draw axis lines
	cv::line(_image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3);
	cv::line(_image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 3);
	cv::line(_image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);
}

inline bool objectWorldsCordinates(std::vector<int>MarkersID, std::vector<cv::Point3f>& RealWorldCoo, std::vector<int> & toBeDiscarded)
{
	bool flag = false;
	for (std::vector<int>::iterator ita = MarkersID.begin(); ita != MarkersID.end(); ita++) 
	{

		if (*ita > 11 || *ita == 0) {
			int index = std::distance(MarkersID.begin(), ita);
			toBeDiscarded.push_back(index);
			flag = true;
			continue;
		}
			
		std::cout << "this is for ID" << *ita << endl;
		RealWorldCoo.push_back(coor1.at(*ita - 1).at(0));

		RealWorldCoo.push_back(coor1.at(*ita - 1).at(1));
		RealWorldCoo.push_back(coor1.at(*ita - 1).at(2));
		RealWorldCoo.push_back(coor1.at(*ita - 1).at(3));

		std::cout << "first : " << coor1.at(*ita - 1).at(0) << "second :  " << coor1.at(*ita - 1).at(1) << "third :  "<< coor1.at(*ita - 1).at(2)<< "fourth :  "<< coor1.at(*ita - 1).at(3) << endl;
	}
	return flag;
}
inline void discardTheNoneExistentMarkers(std::vector<int> & DiscardedIDs, std::vector < cv::Point2f> & twoDCorners) {
	for (std::vector<int>::iterator ita = DiscardedIDs.begin(); ita != DiscardedIDs.end(); ita++) {
		twoDCorners.erase(twoDCorners.begin() + (*ita));
		twoDCorners.erase(twoDCorners.begin() + (*ita +1));
		twoDCorners.erase(twoDCorners.begin() + (*ita + 2));
		twoDCorners.erase(twoDCorners.begin() + (*ita + 3));
	}
		
}
inline void spreadedCornerPoints(std::vector<std::vector<cv::Point2f>>corners, std::vector<cv::Point2f> & spreaded, cv::Mat &frame)
{
	for (int oind = 0; oind<corners.size(); oind++)
	{
		for (int iind = 0; iind < corners[oind].size(); iind++)
		{
			cv::circle(frame, corners[oind][iind], 2, cv::Scalar(50, 205, 50), 4);
			spreaded.push_back(corners[oind][iind]);
		}

	}


}


inline int camMonitoring(cv::Mat & frame, cv::Mat cameraMat, cv::Mat distanceCoefficient, cv::Mat & rotationVectors, cv::Mat & transaltionVectors)
{

	std::vector<std::vector<cv::Point2f>> markerCorners;

	std::vector<int> markersID;
	std::vector<int> DiscarededIDs;

	cv::Ptr<cv::aruco::Dictionary> Dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

	cv::namedWindow("MyCam", CV_WINDOW_AUTOSIZE);

	std::vector<cv::Point2f> spreadedCorners;

	std::vector<cv::Point3f> realWorld;



	cv::aruco::DetectorParameters _para;



	cv::aruco::detectMarkers(frame, Dictionary, markerCorners, markersID);


	spreadedCornerPoints(markerCorners, spreadedCorners, frame);
	
	cv::aruco::drawDetectedMarkers(frame, markerCorners, markersID);


	if (markersID.size() > 0)
	{
		if(objectWorldsCordinates(markersID, realWorld, DiscarededIDs))
		     discardTheNoneExistentMarkers(DiscarededIDs, spreadedCorners);
		
	
		if (markersID.size() <= 3) {
			if (cv::solvePnP(realWorld, spreadedCorners, cameraMat, distanceCoefficient, rotationVectors, transaltionVectors, false, cv::SOLVEPNP_ITERATIVE)) {
				drawAxis(frame, cameraMat, distanceCoefficient, rotationVectors, transaltionVectors, 0.1f);
				//	realWorld.clear();
				return 1;

			}
		}
		else if (markersID.size() > 3) {
			if (cv::solvePnP(realWorld, spreadedCorners, cameraMat, distanceCoefficient, rotationVectors, transaltionVectors, false, cv::SOLVEPNP_EPNP)) {
				drawAxis(frame, cameraMat, distanceCoefficient, rotationVectors, transaltionVectors, 0.1f);
				//	realWorld.clear();
				return 1;

			}
		}
		
	}





	return 0;



}




#endif // !1
