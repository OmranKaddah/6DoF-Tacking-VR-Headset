#include"CamCalibrationHeader.h"
void PostionOfcornersOnTheBoard(float LenghtOFSquaresEdge, cv::Size BoardSize, std::vector<cv::Point3f> & corners) {

	for (int h = 0; h< BoardSize.height; h++)
	{
		for (int w = 0; w<BoardSize.width; w++)
		{
			corners.push_back(cv::Point3f(h * LenghtOFSquaresEdge, w * LenghtOFSquaresEdge, 0.0f));
		}
	}

}

// retrieves all points, corners, in te checkers board 
void getChessBoardCorners(std::vector<cv::Mat> imgs, std::vector<std::vector<cv::Point2f>> & FoundCorners, bool showResults)
{
	for (std::vector<cv::Mat>::iterator it = imgs.begin(); it != imgs.end(); it++)
	{
		std::vector<cv::Point2f> bufferP;// buffer for points
		bool found = cv::findChessboardCorners(*it, BoardSize, bufferP, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
		if (found)
		{

			FoundCorners.push_back(bufferP);
		}

		if (showResults)
		{
			cv::drawChessboardCorners(*it, BoardSize, bufferP, found);
			cv::imshow("Showing corners", *it);
			cv::waitKey();

		}


	}

}

//calculating camera's calibration, this funciton is depended on PostionOFcornersOnTheBoard and getChessBoardCorners

void cameraCalibration(std::vector<cv::Mat> calibrationImages, cv::Size boardSize, float LenghtOFSquaresEdge, cv::Mat & cameraMat, cv::Mat & distCoefficients)
{
	std::vector<std::vector<cv::Point2f>> chessBoardImageSpacePoints;//these are the points that are being detected on calibration Image, 2D image
	getChessBoardCorners(calibrationImages, chessBoardImageSpacePoints, false);

	//this vector is for points of corners in real world
	std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints(1);

	PostionOfcornersOnTheBoard(LenghtOFSquaresEdge, boardSize, worldSpaceCornerPoints[0]);
	// resize to the number of 2D we have and fill the element [0]
	// Reduplicate over and overagine, so there will be a relationship beteen whatever the chessboard stuff was 2D points and 3D points that are expected
	worldSpaceCornerPoints.resize(chessBoardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

	//radial vector , tangential vector
	std::vector<cv::Mat> rVectors, tVectors;

	distCoefficients = cv::Mat::zeros(8, 1, CV_64F);
	cv::calibrateCamera(
		worldSpaceCornerPoints,
		chessBoardImageSpacePoints,
		boardSize,
		cameraMat,
		distCoefficients,
		rVectors,
		tVectors
	);



}

//
bool saveCameraCalibration(std::string nameOfFile, cv::Mat cameraMatrix, cv::Mat distancecoeffiecients)
{
	bool falg = false;
	cv::FileStorage fout(nameOfFile, cv::FileStorage::WRITE);

	fout << "cameraMat" << cameraMatrix;

	fout << "dist" << distancecoeffiecients;
	return true;

}

int runCalibratorProgram() {

	cv::Mat frame;
	cv::Mat drawToFrame;

	cv::Mat camMat = cv::Mat::eye(3, 3, CV_64F);

	cv::Mat distCoefficients;

	std::vector<cv::Mat> savedImages; // If a good calibration is found, the image is going tobe stored here.

	std::vector<std::vector<cv::Point2f>> markerCorner,//these are the points, markers, found on the corners in the board.
		rejectedCandidtes;//these are the rejected points, markers, found on the corners in the board.

	cv::VideoCapture video(0);
	if (!video.isOpened())
	{

		return 0;
	}

	int fps = 20;

	cv::namedWindow("myWebCam", CV_WINDOW_AUTOSIZE);
	while (1) {

		if (!video.read(frame)) {
			video.release();
			return 2;
			break;

		}
		std::vector<cv::Vec2f> foundPoints;
		bool found = false;
		found = cv::findChessboardCorners(frame, BoardSize, foundPoints, CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_ADAPTIVE_THRESH);
		frame.copyTo(drawToFrame);
		cv::drawChessboardCorners(drawToFrame, BoardSize, foundPoints, found);
		if (found)
			cv::imshow("myWebCam", drawToFrame);
		else
			cv::imshow("myWebCam", frame);

		char c = cv::waitKey(1000 / fps);

		switch (c)
		{
			//save image
		case ' ':
			if (found) {
				cv::Mat temp;
				frame.copyTo(temp);
				savedImages.push_back(temp);
				printf("Image saved\n");
			}

			break;
			//start calibration
		case 13:
			if (savedImages.size() > 15)
			{
				printf("strating the calibration proccess\n");
				cameraCalibration(savedImages, BoardSize, LenghtOFSquaresEdge, camMat, distCoefficients);
				printf("end the calibration proccess\n");
				saveCameraCalibration("CamAndDistMAT.txt", camMat, distCoefficients);
				video.release();
				return 1;

			}
			else
			{
				printf("less than 15 images!");
			}
			break;
		case 27:
			//exist
			video.release();

			return 3;
			break;

		}

	}

	return 0;


}
