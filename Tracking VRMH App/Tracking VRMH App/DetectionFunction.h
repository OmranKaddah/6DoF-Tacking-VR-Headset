#pragma once
#ifndef DETECTIONFUNCTION_H
#define DETECTIONFUNCTION_H


#include<opencv2\core.hpp>
#include<opencv2\imgcodecs.hpp>
#include<opencv2\imgproc.hpp>
#include<opencv2\aruco.hpp>
#include<opencv2\highgui.hpp>
#include<opencv2\calib3d.hpp>



#include<vector>
using namespace std;



static void _convertToGrey1(cv::InputArray _in, cv::OutputArray _out) {

	CV_Assert(_in.getMat().channels() == 1 || _in.getMat().channels() == 3);

	_out.create(_in.getMat().size(), CV_8UC1);
	if (_in.getMat().type() == CV_8UC3)
		cvtColor(_in.getMat(), _out.getMat(), cv::COLOR_BGR2GRAY);
	else
		_in.getMat().copyTo(_out);
}


/**
* @brief Threshold input image using adaptive thresholding
*/
static void _threshold1(cv::InputArray _in, cv::OutputArray _out, int winSize, double constant) {

	CV_Assert(winSize >= 3);
	if (winSize % 2 == 0) winSize++; // win size must be odd
	adaptiveThreshold(_in, _out, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, winSize, constant);
}


/**
* @brief Given a tresholded image, find the contours, calculate their polygonal approximation
* and take those that accomplish some conditions
*/
static void _findMarkerContours1(cv::InputArray _in, vector< vector< cv::Point2f > > &candidates,
	vector< vector< cv::Point > > &contoursOut, double minPerimeterRate,
	double maxPerimeterRate, double accuracyRate,
	double minCornerDistanceRate, int minDistanceToBorder) {

	CV_Assert(minPerimeterRate > 0 && maxPerimeterRate > 0 && accuracyRate > 0 &&
		minCornerDistanceRate >= 0 && minDistanceToBorder >= 0);

	// calculate maximum and minimum sizes in pixels
	unsigned int minPerimeterPixels =
		(unsigned int)(minPerimeterRate * max(_in.getMat().cols, _in.getMat().rows));
	unsigned int maxPerimeterPixels =
		(unsigned int)(maxPerimeterRate * max(_in.getMat().cols, _in.getMat().rows));

	cv::Mat contoursImg;
	_in.getMat().copyTo(contoursImg);
	vector< vector< cv::Point > > contours;
	findContours(contoursImg, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
	// now filter list of contours
	for (unsigned int i = 0; i < contours.size(); i++) {
		// check perimeter
		if (contours[i].size() < minPerimeterPixels || contours[i].size() > maxPerimeterPixels)
			continue;

		// check is square and is convex
		vector< cv::Point > approxCurve;
		approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * accuracyRate, true);
		if (approxCurve.size() != 4 || !isContourConvex(approxCurve)) continue;

		// check min distance between corners
		double minDistSq =
			max(contoursImg.cols, contoursImg.rows) * max(contoursImg.cols, contoursImg.rows);
		for (int j = 0; j < 4; j++) {
			double d = (double)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) *
				(double)(approxCurve[j].x - approxCurve[(j + 1) % 4].x) +
				(double)(approxCurve[j].y - approxCurve[(j + 1) % 4].y) *
				(double)(approxCurve[j].y - approxCurve[(j + 1) % 4].y);
			minDistSq = min(minDistSq, d);
		}
		double minCornerDistancePixels = double(contours[i].size()) * minCornerDistanceRate;
		if (minDistSq < minCornerDistancePixels * minCornerDistancePixels) continue;

		// check if it is too near to the image border
		bool tooNearBorder = false;
		for (int j = 0; j < 4; j++) {
			if (approxCurve[j].x < minDistanceToBorder || approxCurve[j].y < minDistanceToBorder ||
				approxCurve[j].x > contoursImg.cols - 1 - minDistanceToBorder ||
				approxCurve[j].y > contoursImg.rows - 1 - minDistanceToBorder)
				tooNearBorder = true;
		}
		if (tooNearBorder) continue;

		// if it passes all the test, add to candidates vector
		vector< cv::Point2f > currentCandidate;
		currentCandidate.resize(4);
		for (int j = 0; j < 4; j++) {
			currentCandidate[j] = cv::Point2f((float)approxCurve[j].x, (float)approxCurve[j].y);
		}
		candidates.push_back(currentCandidate);
		contoursOut.push_back(contours[i]);
	}
}


/**
* @brief Assure order of candidate corners is clockwise direction
*/
static void _reorderCandidatesCorners1(vector< vector< cv::Point2f > > &candidates) {

	for (unsigned int i = 0; i < candidates.size(); i++) {
		double dx1 = candidates[i][1].x - candidates[i][0].x;
		double dy1 = candidates[i][1].y - candidates[i][0].y;
		double dx2 = candidates[i][2].x - candidates[i][0].x;
		double dy2 = candidates[i][2].y - candidates[i][0].y;
		double crossProduct = (dx1 * dy2) - (dy1 * dx2);

		if (crossProduct < 0.0) { // not clockwise direction
			swap(candidates[i][1], candidates[i][3]);
		}
	}
}


/**
* @brief Check candidates that are too close to each other and remove the smaller one
*/
static void _filterTooCloseCandidates1(const vector< vector< cv::Point2f > > &candidatesIn,
	vector< vector< cv::Point2f > > &candidatesOut,
	const vector< vector<cv::Point > > &contoursIn,
	vector< vector< cv::Point > > &contoursOut,
	double minMarkerDistanceRate) {

	CV_Assert(minMarkerDistanceRate >= 0);

	vector< pair< int, int > > nearCandidates;
	for (unsigned int i = 0; i < candidatesIn.size(); i++) {
		for (unsigned int j = i + 1; j < candidatesIn.size(); j++) {

			int minimumPerimeter = min((int)contoursIn[i].size(), (int)contoursIn[j].size());

			// fc is the first corner considered on one of the markers, 4 combinations are possible
			for (int fc = 0; fc < 4; fc++) {
				double distSq = 0;
				for (int c = 0; c < 4; c++) {
					// modC is the corner considering first corner is fc
					int modC = (c + fc) % 4;
					distSq += (candidatesIn[i][modC].x - candidatesIn[j][c].x) *
						(candidatesIn[i][modC].x - candidatesIn[j][c].x) +
						(candidatesIn[i][modC].y - candidatesIn[j][c].y) *
						(candidatesIn[i][modC].y - candidatesIn[j][c].y);
				}
				distSq /= 4.;

				// if mean square distance is too low, remove the smaller one of the two markers
				double minMarkerDistancePixels = double(minimumPerimeter) * minMarkerDistanceRate;
				if (distSq < minMarkerDistancePixels * minMarkerDistancePixels) {
					nearCandidates.push_back(pair< int, int >(i, j));
					break;
				}
			}
		}
	}

	// mark smaller one in pairs to remove
	vector< bool > toRemove(candidatesIn.size(), false);
	for (unsigned int i = 0; i < nearCandidates.size(); i++) {
		// if one of the marker has been already markerd to removed, dont need to do anything
		if (toRemove[nearCandidates[i].first] || toRemove[nearCandidates[i].second]) continue;
		size_t perimeter1 = contoursIn[nearCandidates[i].first].size();
		size_t perimeter2 = contoursIn[nearCandidates[i].second].size();
		if (perimeter1 > perimeter2)
			toRemove[nearCandidates[i].second] = true;
		else
			toRemove[nearCandidates[i].first] = true;
	}

	// remove extra candidates
	candidatesOut.clear();
	unsigned long totalRemaining = 0;
	for (unsigned int i = 0; i < toRemove.size(); i++)
		if (!toRemove[i]) totalRemaining++;
	candidatesOut.resize(totalRemaining);
	contoursOut.resize(totalRemaining);
	for (unsigned int i = 0, currIdx = 0; i < candidatesIn.size(); i++) {
		if (toRemove[i]) continue;
		candidatesOut[currIdx] = candidatesIn[i];
		contoursOut[currIdx] = contoursIn[i];
		currIdx++;
	}
}


/**
* ParallelLoopBody class for the parallelization of the basic candidate detections using
* different threhold window sizes. Called from function _detectInitialCandidates()
*/

struct  DetectorParameters1 {

	DetectorParameters1();

	static cv::Ptr<DetectorParameters1> create();

	int adaptiveThreshWinSizeMin;
	int adaptiveThreshWinSizeMax;
	int adaptiveThreshWinSizeStep;
	double adaptiveThreshConstant;
	double minMarkerPerimeterRate;
	double maxMarkerPerimeterRate;
	double polygonalApproxAccuracyRate;
	double minCornerDistanceRate;
	int minDistanceToBorder;
	double minMarkerDistanceRate;
	bool doCornerRefinement;
	int cornerRefinementWinSize;
	int cornerRefinementMaxIterations;
	double cornerRefinementMinAccuracy;
	int markerBorderBits;
	int perspectiveRemovePixelPerCell;
	double perspectiveRemoveIgnoredMarginPerCell;
	double maxErroneousBitsInBorderRate;
	double minOtsuStdDev;
	double errorCorrectionRate;
};
inline DetectorParameters1::DetectorParameters1()
	: adaptiveThreshWinSizeMin(3),
	adaptiveThreshWinSizeMax(23),
	adaptiveThreshWinSizeStep(10),
	adaptiveThreshConstant(7),
	minMarkerPerimeterRate(0.03),
	maxMarkerPerimeterRate(4.),
	polygonalApproxAccuracyRate(0.03),
	minCornerDistanceRate(0.05),
	minDistanceToBorder(3),
	minMarkerDistanceRate(0.05),
	doCornerRefinement(false),
	cornerRefinementWinSize(5),
	cornerRefinementMaxIterations(30),
	cornerRefinementMinAccuracy(0.1),
	markerBorderBits(1),
	perspectiveRemovePixelPerCell(4),
	perspectiveRemoveIgnoredMarginPerCell(0.13),
	maxErroneousBitsInBorderRate(0.35),
	minOtsuStdDev(5.0),
	errorCorrectionRate(0.6) {}


/**
* @brief Create a new set of DetectorParameters with default values.
*/
inline cv::Ptr<DetectorParameters1> DetectorParameters1::create() {
	cv::Ptr<DetectorParameters1> params = cv::makePtr<DetectorParameters1>();
	return params;
}





class DetectInitialCandidatesParallel1 : public cv::ParallelLoopBody {
public:
	DetectInitialCandidatesParallel1(const cv::Mat *_grey,
		vector< vector< vector< cv::Point2f > > > *_candidatesArrays,
		vector< vector< vector< cv::Point > > > *_contoursArrays,
		const cv::Ptr<DetectorParameters1> &_params)
		: grey(_grey), candidatesArrays(_candidatesArrays), contoursArrays(_contoursArrays),
		params(_params) {}

	void operator()(const cv::Range &range) const {
		const int begin = range.start;
		const int end = range.end;

		for (int i = begin; i < end; i++) {
			int currScale =
				params->adaptiveThreshWinSizeMin + i * params->adaptiveThreshWinSizeStep;
			// threshold
			cv::Mat thresh;
			_threshold1(*grey, thresh, currScale, params->adaptiveThreshConstant);

			// detect rectangles
			_findMarkerContours1(thresh, (*candidatesArrays)[i], (*contoursArrays)[i],
				params->minMarkerPerimeterRate, params->maxMarkerPerimeterRate,
				params->polygonalApproxAccuracyRate, params->minCornerDistanceRate,
				params->minDistanceToBorder);
		}
	}

private:
	DetectInitialCandidatesParallel1 &operator=(const DetectInitialCandidatesParallel1 &);

	const cv::Mat *grey;
	vector< vector< vector< cv::Point2f > > > *candidatesArrays;
	vector< vector< vector< cv::Point > > > *contoursArrays;
	const cv::Ptr<DetectorParameters1> &params;
};


/**
* @brief Initial steps on finding square candidates
*/
static void _detectInitialCandidates1(const cv::Mat &grey, vector< vector< cv::Point2f > > &candidates,
	vector< vector< cv::Point > > &contours,
	const cv::Ptr<DetectorParameters1> &params) {

	CV_Assert(params->adaptiveThreshWinSizeMin >= 3 && params->adaptiveThreshWinSizeMax >= 3);
	CV_Assert(params->adaptiveThreshWinSizeMax >= params->adaptiveThreshWinSizeMin);
	CV_Assert(params->adaptiveThreshWinSizeStep > 0);

	// number of window sizes (scales) to apply adaptive thresholding
	int nScales = (params->adaptiveThreshWinSizeMax - params->adaptiveThreshWinSizeMin) /
		params->adaptiveThreshWinSizeStep + 1;

	vector< vector< vector< cv::Point2f > > > candidatesArrays((size_t)nScales);
	vector< vector< vector< cv::Point > > > contoursArrays((size_t)nScales);

	////for each value in the interval of thresholding window sizes
	// for(int i = 0; i < nScales; i++) {
	//    int currScale = params.adaptiveThreshWinSizeMin + i*params.adaptiveThreshWinSizeStep;
	//    // treshold
	//    Mat thresh;
	//    _threshold(grey, thresh, currScale, params.adaptiveThreshConstant);
	//    // detect rectangles
	//    _findMarkerContours(thresh, candidatesArrays[i], contoursArrays[i],
	// params.minMarkerPerimeterRate,
	//                        params.maxMarkerPerimeterRate, params.polygonalApproxAccuracyRate,
	//                        params.minCornerDistance, params.minDistanceToBorder);
	//}

	// this is the parallel call for the previous commented loop (result is equivalent)
	parallel_for_(cv::Range(0, nScales), DetectInitialCandidatesParallel1(&grey, &candidatesArrays,
		&contoursArrays, params));

	// join candidates
	for (int i = 0; i < nScales; i++) {
		for (unsigned int j = 0; j < candidatesArrays[i].size(); j++) {
			candidates.push_back(candidatesArrays[i][j]);
			contours.push_back(contoursArrays[i][j]);
		}
	}
}


/**
* @brief Detect square candidates in the input image
*/
static void _detectCandidates1(cv::InputArray _image, vector< vector< cv::Point2f > >& candidatesOut,
	vector< vector< cv::Point > >& contoursOut, const cv::Ptr<DetectorParameters1> &_params) {

	cv::Mat image = _image.getMat();
	CV_Assert(image.total() != 0);

	/// 1. CONVERT TO GRAY
	cv::Mat grey;
	_convertToGrey1(image, grey);

	vector< vector< cv::Point2f > > candidates;
	vector< vector< cv::Point > > contours;
	/// 2. DETECT FIRST SET OF CANDIDATES
	_detectInitialCandidates1(grey, candidates, contours, _params);

	/// 3. SORT CORNERS
	_reorderCandidatesCorners1(candidates);

	/// 4. FILTER OUT NEAR CANDIDATE PAIRS
	_filterTooCloseCandidates1(candidates, candidatesOut, contours, contoursOut,
		_params->minMarkerDistanceRate);
}


/**
* @brief Given an input image and candidate corners, extract the bits of the candidate, including
* the border bits
*/
static cv::Mat _extractBits1(cv::InputArray _image, cv::InputArray _corners, int markerSize,
	int markerBorderBits, int cellSize, double cellMarginRate,
	double minStdDevOtsu) {

	CV_Assert(_image.getMat().channels() == 1);
	CV_Assert(_corners.total() == 4);
	CV_Assert(markerBorderBits > 0 && cellSize > 0 && cellMarginRate >= 0 && cellMarginRate <= 1);
	CV_Assert(minStdDevOtsu >= 0);

	// number of bits in the marker
	int markerSizeWithBorders = markerSize + 2 * markerBorderBits;
	int cellMarginPixels = int(cellMarginRate * cellSize);

	cv::Mat resultImg; // marker image after removing perspective
	int resultImgSize = markerSizeWithBorders * cellSize;
	cv::Mat resultImgCorners(4, 1, CV_32FC2);
	resultImgCorners.ptr< cv::Point2f >(0)[0] = cv::Point2f(0, 0);
	resultImgCorners.ptr< cv::Point2f >(0)[1] = cv::Point2f((float)resultImgSize - 1, 0);
	resultImgCorners.ptr< cv::Point2f >(0)[2] =
		cv::Point2f((float)resultImgSize - 1, (float)resultImgSize - 1);
	resultImgCorners.ptr< cv::Point2f >(0)[3] = cv::Point2f(0, (float)resultImgSize - 1);

	// remove perspective
	cv::Mat transformation = getPerspectiveTransform(_corners, resultImgCorners);
	warpPerspective(_image, resultImg, transformation, cv::Size(resultImgSize, resultImgSize),
		cv::INTER_NEAREST);

	// output image containing the bits
	cv::Mat bits(markerSizeWithBorders, markerSizeWithBorders, CV_8UC1, cv::Scalar::all(0));

	// check if standard deviation is enough to apply Otsu
	// if not enough, it probably means all bits are the same color (black or white)
	cv::Mat mean, stddev;
	// Remove some border just to avoid border noise from perspective transformation
	cv::Mat innerRegion = resultImg.colRange(cellSize / 2, resultImg.cols - cellSize / 2)
		.rowRange(cellSize / 2, resultImg.rows - cellSize / 2);
	meanStdDev(innerRegion, mean, stddev);
	if (stddev.ptr< double >(0)[0] < minStdDevOtsu) {
		// all black or all white, depending on mean value
		if (mean.ptr< double >(0)[0] > 127)
			bits.setTo(1);
		else
			bits.setTo(0);
		return bits;
	}

	// now extract code, first threshold using Otsu
	threshold(resultImg, resultImg, 125, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

	// for each cell
	for (int y = 0; y < markerSizeWithBorders; y++) {
		for (int x = 0; x < markerSizeWithBorders; x++) {
			int Xstart = x * (cellSize)+cellMarginPixels;
			int Ystart = y * (cellSize)+cellMarginPixels;
			cv::Mat square = resultImg(cv::Rect(Xstart, Ystart, cellSize - 2 * cellMarginPixels,
				cellSize - 2 * cellMarginPixels));
			// count white pixels on each cell to assign its value
			size_t nZ = (size_t)countNonZero(square);
			if (nZ > square.total() / 2) bits.at< unsigned char >(y, x) = 1;
		}
	}

	return bits;
}



/**
* @brief Return number of erroneous bits in border, i.e. number of white bits in border.
*/
static int _getBorderErrors1(const cv::Mat &bits, int markerSize, int borderSize) {

	int sizeWithBorders = markerSize + 2 * borderSize;

	CV_Assert(markerSize > 0 && bits.cols == sizeWithBorders && bits.rows == sizeWithBorders);

	int totalErrors = 0;
	for (int y = 0; y < sizeWithBorders; y++) {
		for (int k = 0; k < borderSize; k++) {
			if (bits.ptr< unsigned char >(y)[k] != 0) totalErrors++;
			if (bits.ptr< unsigned char >(y)[sizeWithBorders - 1 - k] != 0) totalErrors++;
		}
	}
	for (int x = borderSize; x < sizeWithBorders - borderSize; x++) {
		for (int k = 0; k < borderSize; k++) {
			if (bits.ptr< unsigned char >(k)[x] != 0) totalErrors++;
			if (bits.ptr< unsigned char >(sizeWithBorders - 1 - k)[x] != 0) totalErrors++;
		}
	}
	return totalErrors;
}


/**
* @brief Tries to identify one candidate given the dictionary
*/
static bool _identifyOneCandidate(const cv::Ptr<cv::aruco::Dictionary> &dictionary, cv::InputArray _image,
	cv::InputOutputArray _corners, int &idx, const cv::Ptr<DetectorParameters1> &params) {

	CV_Assert(_corners.total() == 4);
	CV_Assert(_image.getMat().total() != 0);
	CV_Assert(params->markerBorderBits > 0);

	// get bits
	cv::Mat candidateBits =
		_extractBits1(_image, _corners, dictionary->markerSize, params->markerBorderBits,
			params->perspectiveRemovePixelPerCell,
			params->perspectiveRemoveIgnoredMarginPerCell, params->minOtsuStdDev);

	// analyze border bits
	int maximumErrorsInBorder =
		int(dictionary->markerSize * dictionary->markerSize * params->maxErroneousBitsInBorderRate);
	int borderErrors =
		_getBorderErrors1(candidateBits, dictionary->markerSize, params->markerBorderBits);
	if (borderErrors > maximumErrorsInBorder) return false; // border is wrong

															// take only inner bits
	cv::Mat onlyBits =
		candidateBits.rowRange(params->markerBorderBits,
			candidateBits.rows - params->markerBorderBits)
		.colRange(params->markerBorderBits, candidateBits.rows - params->markerBorderBits);

	// try to indentify the marker
	int rotation;
	if (!dictionary->identify(onlyBits, idx, rotation, params->errorCorrectionRate))
		return false;
	else {
		// shift corner positions to the correct rotation
		if (rotation != 0) {
			cv::Mat copyPoints = _corners.getMat().clone();
			for (int j = 0; j < 4; j++)
				_corners.getMat().ptr< cv::Point2f >(0)[j] =
				copyPoints.ptr< cv::Point2f >(0)[(j + 4 - rotation) % 4];
		}
		return true;
	}
}


/**
* ParallelLoopBody class for the parallelization of the marker identification step
* Called from function _identifyCandidates()
*/
class IdentifyCandidatesParallel1 : public cv::ParallelLoopBody {
public:
	IdentifyCandidatesParallel1(const cv::Mat& _grey, cv::InputArrayOfArrays _candidates,
		cv::InputArrayOfArrays _contours, const cv::Ptr<cv::aruco::Dictionary> &_dictionary,
		vector< int >& _idsTmp, vector< char >& _validCandidates,
		const cv::Ptr<DetectorParameters1> &_params)
		: grey(_grey), candidates(_candidates), contours(_contours), dictionary(_dictionary),
		idsTmp(_idsTmp), validCandidates(_validCandidates), params(_params) {}

	void operator()(const cv::Range &range) const {
		const int begin = range.start;
		const int end = range.end;

		for (int i = begin; i < end; i++) {
			int currId;
			cv::Mat currentCandidate = candidates.getMat(i);
			if (_identifyOneCandidate(dictionary, grey, currentCandidate, currId, params)) {
				validCandidates[i] = 1;
				idsTmp[i] = currId;
			}
		}
	}

private:
	IdentifyCandidatesParallel1 &operator=(const IdentifyCandidatesParallel1 &); // to quiet MSVC

	const cv::Mat &grey;
	cv::InputArrayOfArrays candidates, contours;
	const cv::Ptr<cv::aruco::Dictionary> &dictionary;
	vector< int > &idsTmp;
	vector< char > &validCandidates;
	const cv::Ptr<DetectorParameters1> &params;
};



/**
* @brief Copy the contents of a corners vector to an OutputArray, settings its size.
*/
static void _copyVector2Output1(vector< vector< cv::Point2f > > &vec, cv::OutputArrayOfArrays out) {
	out.create((int)vec.size() * 4, 1, CV_32FC2);


	//for (int oind = 0; oind< (int)vec.size(); oind++)
	//{
	//	for (int iind = 0; iind <(int) vec[oind].size(); iind++)
	//	{
	//		//cv::circle(frame, corners[oind][iind], 2, cv::Scalar(50, 205, 50), 4);
	//		out.push_back(vec[oind][iind]);
	//	}

	//}


	if (out.isMatVector()) {
		for (unsigned int i = 0; i < vec.size(); i++) {
			out.create(4, 1, CV_32FC2, i);
			cv::Mat &m = out.getMatRef(i);
			cv::Mat(cv::Mat(vec[i]).t()).copyTo(m);

		}
	}
	else if (out.isUMatVector()) {
		for (unsigned int i = 0; i < vec.size(); i++) {
			out.create(4, 1, CV_32FC2, i);
			cv::UMat &m = out.getUMatRef(i);
			cv::Mat(cv::Mat(vec[i]).t()).copyTo(m);
		}
	}
	else if (out.kind() == cv::_OutputArray::STD_VECTOR) {

		for (unsigned int i = 0; i < vec.size(); i++) {
			out.create(4, 1, CV_32FC2, i);
			cv::Mat m = out.getMat(i);
			cv::Mat(cv::Mat(vec[i]).t()).copyTo(m);

		}
	}
	else {
		CV_Error(cv::Error::StsNotImplemented,
			"Only Mat vector, UMat vector, and vector<vector> OutputArrays are currently supported.");
	}
}



/**
* @brief Identify square candidates according to a marker dictionary
*/
static void _identifyCandidates1(cv::InputArray _image, vector< vector< cv::Point2f > >& _candidates,
	cv::InputArrayOfArrays _contours, const cv::Ptr<cv::aruco::Dictionary> &_dictionary,
	vector< vector< cv::Point2f > >& _accepted, vector< int >& ids,
	const cv::Ptr<DetectorParameters1> &params,
	cv::OutputArrayOfArrays _rejected = cv::noArray()) {

	int ncandidates = (int)_candidates.size();

	vector< vector< cv::Point2f > > accepted;
	vector< vector< cv::Point2f > > rejected;

	CV_Assert(_image.getMat().total() != 0);

	cv::Mat grey;
	_convertToGrey1(_image.getMat(), grey);

	vector< int > idsTmp(ncandidates, -1);
	vector< char > validCandidates(ncandidates, 0);

	//// Analyze each of the candidates
	// for (int i = 0; i < ncandidates; i++) {
	//    int currId = i;
	//    Mat currentCandidate = _candidates.getMat(i);
	//    if (_identifyOneCandidate(dictionary, grey, currentCandidate, currId, params)) {
	//        validCandidates[i] = 1;
	//        idsTmp[i] = currId;
	//    }
	//}

	// this is the parallel call for the previous commented loop (result is equivalent)
	parallel_for_(cv::Range(0, ncandidates),
		IdentifyCandidatesParallel1(grey, _candidates, _contours, _dictionary, idsTmp,
			validCandidates, params));

	for (int i = 0; i < ncandidates; i++) {
		if (validCandidates[i] == 1) {
			accepted.push_back(_candidates[i]);
			ids.push_back(idsTmp[i]);
		}
		else {
			rejected.push_back(_candidates[i]);
		}
	}

	// parse output
	_accepted = accepted;

	//if (_rejected.needed()) {
	//	_copyVector2Output1(rejected, _rejected);
	//}
}


/**
* @brief Final filter of markers after its identification
*/
static void _filterDetectedMarkers1(vector< vector< cv::Point2f > >& _corners, vector< int >& _ids) {

	CV_Assert(_corners.size() == _ids.size());
	if (_corners.empty()) return;

	// mark markers that will be removed
	vector< bool > toRemove(_corners.size(), false);
	bool atLeastOneRemove = false;

	// remove repeated markers with same id, if one contains the other (doble border bug)
	for (unsigned int i = 0; i < _corners.size() - 1; i++) {
		for (unsigned int j = i + 1; j < _corners.size(); j++) {
			if (_ids[i] != _ids[j]) continue;

			// check if first marker is inside second
			bool inside = true;
			for (unsigned int p = 0; p < 4; p++) {
				cv::Point2f point = _corners[j][p];
				if (pointPolygonTest(_corners[i], point, false) < 0) {
					inside = false;
					break;
				}
			}
			if (inside) {
				toRemove[j] = true;
				atLeastOneRemove = true;
				continue;
			}

			// check the second marker
			inside = true;
			for (unsigned int p = 0; p < 4; p++) {
				cv::Point2f point = _corners[i][p];
				if (pointPolygonTest(_corners[j], point, false) < 0) {
					inside = false;
					break;
				}
			}
			if (inside) {
				toRemove[i] = true;
				atLeastOneRemove = true;
				continue;
			}
		}
	}

	// parse output
	if (atLeastOneRemove) {
		vector< vector< cv::Point2f > >::iterator filteredCorners = _corners.begin();
		vector< int >::iterator filteredIds = _ids.begin();

		for (unsigned int i = 0; i < toRemove.size(); i++) {
			if (!toRemove[i]) {
				*filteredCorners++ = _corners[i];
				*filteredIds++ = _ids[i];
			}
		}

		_ids.erase(filteredIds, _ids.end());
		_corners.erase(filteredCorners, _corners.end());
	}
}



/**
* @brief Return object points for the system centered in a single marker, given the marker length
*/
static void _getSingleMarkerObjectPoints1(float markerLength, cv::OutputArray _objPoints) {

	CV_Assert(markerLength > 0);

	_objPoints.create(4, 1, CV_32FC3);
	cv::Mat objPoints = _objPoints.getMat();
	// set coordinate system in the middle of the marker, with Z pointing out
	objPoints.ptr< cv::Vec3f >(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
	objPoints.ptr< cv::Vec3f >(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
	objPoints.ptr< cv::Vec3f >(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
	objPoints.ptr< cv::Vec3f >(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
}




/**
* ParallelLoopBody class for the parallelization of the marker corner subpixel refinement
* Called from function detectMarkers()
*/
class MarkerSubpixelParallel : public cv::ParallelLoopBody {
public:
	MarkerSubpixelParallel(const cv::Mat *_grey, cv::OutputArrayOfArrays _corners,
		const cv::Ptr<DetectorParameters1> &_params)
		: grey(_grey), corners(_corners), params(_params) {}

	void operator()(const cv::Range &range) const {
		const int begin = range.start;
		const int end = range.end;

		for (int i = begin; i < end; i++) {
			cv::cornerSubPix(*grey, corners.getMat(i),
				cv::Size(params->cornerRefinementWinSize, params->cornerRefinementWinSize),
				cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
					params->cornerRefinementMaxIterations,
					params->cornerRefinementMinAccuracy));
		}
	}

private:
	MarkerSubpixelParallel &operator=(const MarkerSubpixelParallel &); // to quiet MSVC

	const cv::Mat *grey;
	cv::OutputArrayOfArrays corners;
	const cv::Ptr<DetectorParameters1> &params;
};



/**
*/


inline void detectMarkers1(cv::InputArray _image, const cv::Ptr<cv::aruco::Dictionary> &_dictionary, cv::OutputArrayOfArrays  _corners,
	cv::OutputArray & _ids, const cv::Ptr<DetectorParameters1> &_params = DetectorParameters1::create(), cv::OutputArrayOfArrays _rejectedImgPoints = cv::noArray()) {

	CV_Assert(!_image.empty());

	cv::Mat grey;
	//const cv::Ptr<DetectorParameters1> &_params();

	_convertToGrey1(_image.getMat(), grey);

	/// STEP 1: Detect marker candidates
	vector< vector< cv::Point2f > > candidates;
	vector< vector< cv::Point > > contours;
	vector< int > ids;
	_detectCandidates1(grey, candidates, contours, _params);

	/// STEP 2: Check candidate codification (identify markers)
	_identifyCandidates1(grey, candidates, contours, _dictionary, candidates, ids, _params,
		_rejectedImgPoints);

	/// STEP 3: Filter detected markers;
	_filterDetectedMarkers1(candidates, ids);

	// copy to output arrays
	//_corners.clear();
	//for (int oind = 0; oind< (int)candidates.size(); oind++)
	//{
	//	for (int iind = 0; iind <(int) candidates[oind].size(); iind++)
	//	{
	//	//	cv::circle(frame, corners[oind][iind], 2, cv::Scalar(50, 205, 50), 4);
	//		_corners.push_back(candidates[oind][iind]);
	//	}

	//}


	//_copyVector2Output1(candidates, _corners);
	//candidates.clear();

	cv::Mat(ids).copyTo(_ids);

	/// STEP 4: Corner refinement
	//if (_params->doCornerRefinement) {
	//	CV_Assert(_params->cornerRefinementWinSize > 0 && _params->cornerRefinementMaxIterations > 0 &&
	//		_params->cornerRefinementMinAccuracy > 0);

	//	//// do corner refinement for each of the detected markers
	//	// for (unsigned int i = 0; i < _corners.cols(); i++) {
	//	//    cornerSubPix(grey, _corners.getMat(i),
	//	//                 Size(params.cornerRefinementWinSize, params.cornerRefinementWinSize),
	//	//                 Size(-1, -1), TermCriteria(TermCriteria::MAX_ITER | TermCriteria::EPS,
	//	//                                            params.cornerRefinementMaxIterations,
	//	//                                            params.cornerRefinementMinAccuracy));
	//	//}

	//	// this is the parallel call for the previous commented loop (result is equivalent)
	//	parallel_for_(cv::Range(0, _corners.cols()),
	//		MarkerSubpixelParallel(&grey, _corners, _params));
	//}
}
#endif // !DETECTIONFUNCTION_H