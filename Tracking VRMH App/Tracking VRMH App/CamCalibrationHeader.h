
#pragma once

#ifndef CAMCALIBRATIONHEADER_H
#define CAMCALIBRATIONHEADER_H

#include<opencv2\core.hpp>
#include<opencv2\imgcodecs.hpp>
#include<opencv2\imgproc.hpp>
#include<opencv2\aruco.hpp>
#include<opencv2\highgui.hpp>
#include<opencv2\calib3d.hpp>

#include<vector>
#include<sstream>
#include<fstream>


/*
In this source file Camera Calibration is Defined.
Credite goes to a toutrial from George Leacakes on YouTube.

*/

// In calibration Board measures in Meter

const float LenghtOFSquaresEdge = 0.02086f;

const cv::Size BoardSize = cv::Size(9, 6);
//__________________________________________
//__________________________________________


// this function is for adding the corners at 3D location 
void PostionOfcornersOnTheBoard(float LenghtOFSquaresEdge, cv::Size BoardSize, std::vector<cv::Point3f> & corners);

// retrieves all points, corners, in te chessboard 
void getChessBoardCorners(std::vector<cv::Mat> imgs, std::vector<std::vector<cv::Point2f>> & FoundCorners, bool showResults = false);

//calculating camera's calibration, this funciton is depended on PostionOFcornersOnTheBoard and getChessBoardCorners
void cameraCalibration(std::vector<cv::Mat> calibrationImages, cv::Size boardSize, float LenghtOFSquaresEdge, cv::Mat & cameraMat, cv::Mat & distanceCoieffcients);

bool saveCameraCalibration(std::string nameOfFile, cv::Mat cameraMatrix, cv::Mat distancecoeffiecients);

int runCalibratorProgram();

#endif // CAMCALIBRATIONHEADER_H


