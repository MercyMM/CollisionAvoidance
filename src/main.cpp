/*
 *  main.cpp
 *  COLA
 * @Authors: PGalvin, KSebesta
 *
 */

#include <opencv2/opencv.hpp>
#include <brisk/brisk.h>
#include <OF.hpp>
#include <KeyFeature.hpp>
#include <GraphUtils.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <list>
#include <string>
#include <vector>
#include <deque>
#include <stdlib.h>
#include "openCVhelpers.h"
#include "ui.h"


using namespace std;

typedef vector <double> record_t;
typedef vector <record_t> data_t;

void generateMovie(string outputMovieName, float outputVideoFPS, cv::Mat outimgScaled, cv::VideoCapture cap, cv::VideoWriter outputVideo, unsigned long loopCount);

//standard BRISK configuration for the case of no file given //GIVEN BY STEFAN LEUTENEGGER
const int n=12;
const float r=2.5; // found 8-9-11, r=3.6, exponent 1.5

enum KeyframeState {
	EXPIRED, UNDETECTED, UNEXTRACTED, PROCESSED
};

#ifdef __APPLE__
#define LEFT_ARROW	63234
#define UP_ARROW	63232
#define RIGHT_ARROW	63235
#define DOWN_ARROW	63233
#elif defined __linux__
#define LEFT_ARROW	'Q'
#define UP_ARROW	'R'
#define RIGHT_ARROW	'S'
#define DOWN_ARROW	'T'
#elif defined _WIN32 || defined _WIN64
#else
#error "unknown platform"
#endif

#define MIN_KEYPOINT_THRESH 20
#define MAX_KEYPOINT_THRESH 50


/*
 * Main
 */
int main (int argc, char * const argv[]) {

	/**************************************
	 *            Declare variables       *
	 **************************************/
//	unsigned long startFrame=0;
//	double inputScale=1, outputScale=1;
	double tickFrequency;
//	double FPS=30;
//	int captureType=-2;
//	float rotationAngle =0;
//	int detectorType=NO_ALGO;
//	int descriptorType=NO_ALGO;
//	int decimateFrame=1;
	
	
	// names of the image files
	cv::Mat imgTmp;
	cv::Mat roiTmp;
	cv::Mat imgKeyframeRGB;
	cv::Mat imgKeyframeGray;
	cv::Mat imgSampleRGB;
	cv::Mat imgSampleGray;
	cv::Mat imgRGB2;
	cv::Mat imgRGB3;
	cv::Mat imgOriginalInputCF;
	cv::Mat imgGrayKF;
//	cv::Mat imgGrayCF;
	cv::Mat img_old_grey;
	cv::Mat outimg, outimgScaled;
	cv::Mat quadMask;
	cv::Mat quadMaskFullScale;
	cv::Mat imgCIELab;
	std::vector<cv::Mat> imgCIELabChannels;
	
	
	//Create GUI window
	cv::namedWindow("Matches");
	cv::namedWindow("dummy", CV_WINDOW_AUTOSIZE);	
	//	cv::namedWindow("L", CV_WINDOW_AUTOSIZE);	
	//	cv::namedWindow("a", CV_WINDOW_AUTOSIZE);	
	//	cv::namedWindow("b", CV_WINDOW_AUTOSIZE);	
    
	// standard file extensions for opening images
	std::vector<std::string> fextensions=generateImageFileExtList();

	//Undistort variables
	int top, bottom, left, right;
	cv::Mat imgPadded;	
	cv::Mat imageUndistorted;
	cv::Scalar value = cv::Scalar( 0,0,0 );
//	bool enableUndistort=0;
	
	//Mask related variables
//	bool enableMask=0;
	
	//Graph related variables
//	bool enableGraph=0;
	
	//Polar vision variables
	int numBlocks;
//	bool enableBlurredOutput=0;
//	double ofBlurring=1;
	
	//Optical flow variables
	vector<cv::KeyPoint> keyPoints_LK;  
	vector < vector<cv::KeyPoint> > nextPoints_LK;  
	vector<cv::Point2f> points_flowVector_LK;
	vector < vector <double> > ttc;
	cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);  //Set max number of iterations and minimum movement amount
//	bool enableOF=true;
	int ofPointsGeneration=0;
	
	//Detector related variables
	std::vector<cv::KeyPoint> keypointsKF, keypointsCF;
	cv::Ptr<cv::FeatureDetector> detector;
//	int featureThreshold=0;
	
	// Init descriptor related variables
	int descriptorFilePresent=0;
	string desc1, desc2;
	cv::Mat descriptorsKF, descriptorsCF;
	vector< vector<cv::DMatch> > matches;
	cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
	
	// Init external IMU variables
	data_t attitudeData; 	// Here is the csv data we want.
	int attitudeIndex=0;
	double attitudeTime=0;
	deque <double> fltAtt(200, 0);
//	double imuStartTime=0;
	bool imuCsvDataPresent=false;
	
	// Init loop variables
	double imageDelT=1.0/50.0; //The time period of the image sequence
	int loopCount=0;	
	double absoluteTime=0;
	tickFrequency=cv::getTickFrequency();
	long long int old_TickCount=0, currentTickCount, imageAnalysis_TickCount;	
	int keyPress=0;
	char imgStateFlagKF=EXPIRED;

	
	//Video output variables
	cv::VideoWriter outputVideo;
	float outputVideoFPS=30;
	
	/**************************************
	 * process command line args		  *
	 **************************************/
	InputParams inputParams=parseArguments(argc, argv);
	
	if(argc>1){
		//Ahhh, maybe the user wants a little more out of the demo. Let's give him/her some options.
		for (int i=1; i<argc; i++){
			if (strncmp("-", argv[i], 1)==0 ){				
				if(strncmp("-descriptor_file", argv[i]+1, 15)==0){ //DO NOT MOVE RELATIVE TO descriptor SINCE OTHERWISE THE CONDITION WILL NEVER BE TESTED
					desc1 = std::string(argv[i+1]);
					desc2 = std::string(argv[i+2]);
					
					descriptorFilePresent=1;
					i++;
				}
			}
		}
	}	
	
	//===================================================
	// create the detector
	detector=createDetector( inputParams);
	
	// now the extractor:	
	cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
	bool hamming=true;
	
	createExtractor(inputParams, &descriptorExtractor, &descriptorMatcher, &hamming);	

	// Load the descriptor file, if present	
	if (descriptorFilePresent==1){
		// try to read descriptor files
		std::ifstream descf1(desc1.c_str());
		if(!descf1.good()){
			std::cout<<"Descriptor file not found at " << desc1<<std::endl;
			return 3;
		}
		std::ifstream descf2(desc2.c_str());
		if(!descf2.good()){
			std::cout<<"Descriptor file not found at " << desc2<<std::endl;
			return 3;
		}
		
		// fill the keypoints
		std::string str1;
		std::stringstream strstrm1;
		std::getline(descf1,str1);
		std::getline(descf1,str1);
		while(!descf1.eof()){
			std::getline(descf1,str1);
			float x,y,a;
			strstrm1.str(str1);
			strstrm1>>x;
			strstrm1>>y;
			strstrm1>>a;
			float r=sqrt(1.0/a);
			keypointsKF.push_back(cv::KeyPoint(x, y, 4.0*r));
		}
		std::string str2;
		std::stringstream strstrm2;
		std::getline(descf2,str2);
		std::getline(descf2,str2);
		while(!descf2.eof()){
			std::getline(descf2,str2);
			float x,y,a;
			strstrm2.str(str2);
			strstrm2>>x;
			strstrm2>>y;
			strstrm2>>a;
			float r=sqrt(1.0/a);
			keypointsCF.push_back(cv::KeyPoint(x, y, 4.0*r));
		}
		
		// clean up
		descf1.close();
		descf2.close();
	}
	
	//===================================================
	
	//========================//
	// Read csv attitude data //
	//========================//
	if (inputParams.csvInputFile.length()){
		// Here is the file containing the data. Read it into attitudeData.
		ifstream infile( inputParams.csvInputFile.c_str());
//		ifstream infile( "../../Attitude_2011-10-22_23-03-25.csv" );
		infile >> attitudeData;
		
		// Complain if something went wrong.
		if (!infile.eof())
		{
			cout << "CSV read failed.\n";
			return 1;
		}
		else{
			infile.close();
		}
		
		imuCsvDataPresent=true;
	}
	
	//========================//
	// Initialize video input //
	//========================//
	
	//Initialize video capture as camera
	cv::VideoCapture cap;
	
	if (inputParams.captureType!=-2){
		//The user has selected non-static images as the image source
		
		//In case we want to open a video file...
		if (inputParams.captureType!=-1){
			cap.open(inputParams.vidFileName);
			cap.set(CV_CAP_PROP_FPS,3.0); //WEIRD, WHAT DOES THIS LINE DO???
			
			if (cap.get(CV_CAP_PROP_FRAME_COUNT) >= inputParams.startFrame){
				//Potentially jump past first few frames because they're boring
				cap.set(CV_CAP_PROP_POS_FRAMES, inputParams.startFrame);
			}
			else {
				std::cout<<"Requested start_frame: " << inputParams.startFrame<<", but file only has "<< cap.get(CV_CAP_PROP_FRAME_COUNT) << " frames" << std::endl;
				return -1;
			}
		}
		else{
			//...or else we open the camera
			//	std::cout<<"Shouldn't be doing this until CAM is fixed." << std::endl;
			cap.open(0);
		}
		
		
		if(!cap.isOpened()){
			return -1;
		}
		
		inputParams.captureType=cap.get(CV_CAP_PROP_MODE); //WEIRD, WHAT DOES THIS LINE DO???
	}
	else{
		cap.open(inputParams.imageKeyFrameName);
	}
	
	//Read reference image files
	if(inputParams.imageKeyFrameName.length()){
		imgKeyframeRGB=cv::imread(inputParams.imageKeyFrameName);
		cv::cvtColor(imgKeyframeRGB, imgKeyframeGray, CV_BGR2GRAY);
		imgStateFlagKF= UNEXTRACTED;
	}
	else{
		cap>>imgKeyframeRGB;
	}
	if(inputParams.imageCurrentFrameName.length() && inputParams.captureType==-2){
		imgSampleRGB=cv::imread(inputParams.imageCurrentFrameName);
	}
	
	
	//=============//
	// Create mask //
	//=============//
	//Create full scale mask
	quadMaskFullScale.create(cap.get(CV_CAP_PROP_FRAME_HEIGHT), cap.get(CV_CAP_PROP_FRAME_WIDTH), CV_8UC1);
	
	//Fill mask with 255
	quadMaskFullScale=(cv::Scalar(255));
	
	if (inputParams.enableMask){
		
		
		cv::Rect bladeLeft, bladeRight;
		//Define ROI for left and right blades
		if (0) { //Flying GoPro Hero2
			bladeLeft=cv::Rect(25, quadMaskFullScale.rows-57, 32, 57);
			bladeRight=cv::Rect(quadMaskFullScale.cols-40, quadMaskFullScale.rows-52, 32, 52);			
		}
		else if(1){ //Flying GoPro Hero
			bladeLeft=cv::Rect(70, quadMaskFullScale.rows-116, 38, 116);
			bladeRight=cv::Rect(quadMaskFullScale.cols-72, quadMaskFullScale.rows-120, 37, 120);
		}
		else if (1){ //Driving GoPro Hero
			bladeLeft=cv::Rect(110, quadMaskFullScale.rows-84, 460, 84);
		}
		
		//Create ROI for left blade
		roiTmp=quadMaskFullScale(bladeLeft);
		roiTmp=(cv::Scalar(0));
		
		//Create ROI for right blade
		roiTmp=quadMaskFullScale(bladeRight);
		roiTmp=(cv::Scalar(0));
		
	}
	//Scale mask
	if (inputParams.inputScale!=1.0) {
		cv::resize(quadMaskFullScale, quadMask, cv::Size(), inputParams.inputScale, inputParams.inputScale, cv::INTER_NEAREST);
	}
	else{
		quadMask=quadMaskFullScale;
	}
	
	// convert first image to grayscale
	cv::cvtColor(imgKeyframeRGB, imgGrayKF, CV_BGR2GRAY);
	
	
	
	//================================//
	// Create blocks for polar vision //
	//================================//
	
	vector<cv::Rect> polarBlocks;
	int numBlocksW=3, numBlocksH=3, overlap=0;
	
	//Pull a funky shenanigan so that the blocks are to the right scale
	cv::Mat img;
	if(inputParams.imageCurrentFrameName.length() && inputParams.captureType==-2){
		imgTmp =cv::imread(inputParams.imageCurrentFrameName);
	}
	else{
		cap>>imgTmp;
	}
	
	cv::resize(imgTmp, img, cv::Size(), inputParams.inputScale, inputParams.inputScale, cv::INTER_NEAREST);
	
	//Define some local variables
	numBlocks=numBlocksW*numBlocksH;
	const int blockWidth=ceil(img.cols/numBlocksW*(1+overlap/100.0));
	const int blockHeight=ceil(img.rows/numBlocksH*(1+overlap/100.0));
	
	double overlapW, overlapH;
	
	if (numBlocks>1){
		if (numBlocksW > 1) {
			overlapW=((numBlocksW*blockWidth-(img.cols-1))/2.0)/(numBlocksW-1);			
		}
		else{
			overlapW=0;
		}
		if (numBlocksH > 1) {
			overlapH=((numBlocksH*blockHeight-(img.rows-1))/2.0)/(numBlocksH-1);
		}
		else {
			overlapH=0;			
		}
		
		for (int i=0; i<numBlocksH; i++){
			for (int j=0; j<numBlocksW; j++){
				polarBlocks.push_back(cv::Rect(floor((j)*(blockWidth-overlapW*2)+0.5), floor((i)*(blockHeight-overlapH*2)+0.5), blockWidth, blockHeight));
			}
		}
	}
	else{
		polarBlocks.push_back(cv::Rect(0,0,blockWidth, blockHeight));
	}
	//	for (int i=0; i < numBlocks; i++) {
	//		cout << polarBlocks[i].x << "\t" << polarBlocks[i].y << "\t" << polarBlocks[i].width << "\t" << polarBlocks[i].height << "\t" << endl;
	//	}
	
	
	
	
	//============================//
	// Create movie output module //
	//============================//
	string outputMovieName;
	if (inputParams.videoOutEnable) {
		string::size_type pAt = inputParams.videoOutFile.find_last_of('.');   // Find extension point
		outputMovieName = inputParams.videoOutFile.substr(0, pAt);   // Form the new name with container
	}
	
	
	
	
	
	
	
	
	
	
	
#if 0
	
	// Specify the number of squares along each dimension of the board.
	// This is actually the number of "inside corners" (where a black square meets a white square).
	// That is, if the board is composed of n x m squares, you would use (n-1, m-1) as the arguments.
	// For example, for a standard checkerboard (7x10 squares), you would use cv::Size boardSize(7-1,10-1);
	cv::Size boardSize(inputParams.FPS, inputParams.outputScale);
	
	float squareSize = 1.f; // This is "1 arbitrary unit"
	
	//Create image and fill with video capture
	cv:: Mat image;
	imgKeyframeRGB.copyTo(image);
	
	if(image.empty())
    {
		std::cerr << "Image not read correctly!" << std::endl;
		exit(-1);
    }
	
	cv::namedWindow( "Image View", 1 );
	
	cv::Size imageSize = image.size();
	
	//Create corner vectors
	std::vector<std::vector<cv::Point2f> > imagePoints;
	std::vector <cv::Point2f> corners;
	
	int successes=0;
	int key=0;
	int numBoards=1;
	
	// Open the output files
	ofstream cornersX("cToMatlab/cornersX.txt");
	ofstream cornersY("cToMatlab/cornersY.txt");
	ofstream cornerInfo("cToMatlab/cornerInfo.txt");
	
	while(key!=27){
//		cap >> image;
		
		// Find the chessboard corners
		bool found = cv::findChessboardCorners(image, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		
        if(found)
        {
			cv::Mat gray_image;
			cv::cvtColor(image, gray_image, CV_BGR2GRAY);
            cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			
			//Advance video so we have different viewpoints
			cap.set(CV_CAP_PROP_POS_FRAMES, cap.get(CV_CAP_PROP_POS_FRAMES)+60);
			
        }
		
		
		drawChessboardCorners(image, boardSize, cv::Mat(corners), found );
		cv::imshow("Image View", image);
		key = cv::waitKey(2);
		if(found){
			
            imagePoints.push_back(corners);
			std::cout << successes << " Snap stored!\n";
            successes++;
			
			for (int i=0; i< 54; i++) {
				cornersX << corners[i].x;
				cornersX << " ";
				cornersY << corners[i].y;
				cornersY << " ";
			}
			
			// Write to the corner matrix size info file
			cornerInfo << boardSize.width<< " " << boardSize.height << endl;
			
			
            if(successes>=numBoards){
                break;
			}			
		}
	}
	
	
	// Close the output files
	cornersX.close(); 
	cornersY.close();
	cornerInfo.close();
	
	exit(1);
	
	std::vector<std::vector<cv::Point3f> > objectPoints;
	for (int i=0; i<successes; i++) {
		objectPoints.push_back(Create3DChessboardCorners(boardSize, squareSize));		
	}
	
	std::vector<cv::Mat> rotationVectors;
	std::vector<cv::Mat> translationVectors;
	
	cv::Mat distortionCoefficients = cv::Mat::zeros(8, 1, CV_64F); // There are 8 distortion coefficients
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	
	
	cameraMatrix.at<double>(0,2)=640;
	cameraMatrix.at<double>(1,2)=360;
	
	std::cout << cameraMatrix << std::endl;
	
	int flags = 0;
	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
								 distortionCoefficients, rotationVectors, translationVectors, flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5|CV_CALIB_FIX_PRINCIPAL_POINT);
	
	std::cout << "RMS: " << rms << std::endl;
	
	std::cout << "Camera matrix: " << cameraMatrix << std::endl;
	std::cout << "Distortion _coefficients: " << distortionCoefficients << std::endl;
	
	
	imageUndistorted;
    while(1)
    {
        cap >> image;
		cv::undistort(image, imageUndistorted, cameraMatrix, distortionCoefficients);
		
		cv::imshow("Image View", image);
        cv::imshow("L", imageUndistorted);
		
        cv::waitKey(2);
    }
	
	
	return 0;
	
#elif 1
	cv::Mat distortionCoefficients = cv::Mat::zeros(8, 1, CV_64F); // There are 8 distortion coefficients
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat image;
	
	cameraMatrix.at<double>(0,2)=640;
	cameraMatrix.at<double>(1,2)=360;
	
	
	cameraMatrix.at<double>(0,0)=541.7546930137235;
	cameraMatrix.at<double>(0,1)=0;
	cameraMatrix.at<double>(0,2)=639.5;
	cameraMatrix.at<double>(1,0)=0;
	cameraMatrix.at<double>(1,1)=543.625380610144;
	cameraMatrix.at<double>(1,2)=359.5;
	cameraMatrix.at<double>(2,0)=0;
	cameraMatrix.at<double>(2,1)=0;
	cameraMatrix.at<double>(2,2)=1;
	distortionCoefficients.at<double>(0)=-0.2315011758929961;
	distortionCoefficients.at<double>(1)=0.05007781689737046;
	distortionCoefficients.at<double>(2)=-0.001114796513427666;
	distortionCoefficients.at<double>(3)=-0.0003610047760401707;
	distortionCoefficients.at<double>(4)=-0.004593828627337842;
	
	
	cap >> image;
	
	top = (int) (0.1*image.rows); bottom = (int) (0.1*image.rows);
	left = (int) (0.1*image.cols); right = (int) (0.1*image.cols);
	
	copyMakeBorder( image, imgPadded, top, bottom, left, right, cv::BORDER_CONSTANT, value );
	cameraMatrix.at<double>(0,2)=imgPadded.cols/2-.5;
	cameraMatrix.at<double>(1,2)=imgPadded.rows/2-.5;	
	
    while(0)
    {
        cap >> image;
		copyMakeBorder( image, imgPadded, top, bottom, left, right, cv::BORDER_CONSTANT, value );
		cv::undistort(imgPadded, imageUndistorted, cameraMatrix, distortionCoefficients);
		
		cv::imshow("Image View", imgPadded);
        cv::imshow("L", imageUndistorted);
		
        cv::waitKey(2);
    }	
#endif	
	
	
	
	
	
	nextPoints_LK.resize(numBlocks);
	
	vector <char> needToInitOF(numBlocks, true);
	vector <double> ofScale(numBlocks, inputParams.ofBlurring);
	ofScale[4]=1; //Set middle square not to blur
		
	
	/*************************************************************
	 * START MAIN LOOP
	 *************************************************************/	
	while(keyPress!=0x1B) //Press 'Esc' to exit
	{
		
		//If we're reading from the video, accept key inputs for advancing and rewinding the movie
		if (inputParams.captureType==0){
			//Read input cursors to advance or rewind movie
			if(keyPress>0){
				int oldPos=cap.get(CV_CAP_PROP_POS_FRAMES);
				switch (keyPress) {
					case LEFT_ARROW:
						cap.set(CV_CAP_PROP_POS_FRAMES, oldPos-30);
						break;
					case UP_ARROW:
						cap.set(CV_CAP_PROP_POS_FRAMES, oldPos+30*5);
						break;
					case RIGHT_ARROW:
						cap.set(CV_CAP_PROP_POS_FRAMES, oldPos+30);
						break;
					case DOWN_ARROW:
						cap.set(CV_CAP_PROP_POS_FRAMES, oldPos-30*5);
						break;
					case 'p':
						cvWaitKey(0);
						break;
					default:
						std::cout<<"key press: " << keyPress << " not known\n" << std::endl;
						break;
				}
				
				//Don't forget to also advance the absolute time clock
				absoluteTime+=(imageDelT*(cap.get(CV_CAP_PROP_POS_FRAMES)-oldPos));
			}
		}
		
		//Handle more key inputs
		if(keyPress>0){
			switch (keyPress) {
				case 'k':
					inputParams.enableFD^=1;
					cout << "Toggling feature detection " << (inputParams.enableFD ? "on.":"off.") << endl;
					break;

				case 'o':
					needToInitOF=vector <char>(numBlocks, true);
					if (inputParams.enableOF) {
						ofPointsGeneration=(ofPointsGeneration+1)%2;
					}
					inputParams.enableOF^=1;
					cout << "Toggling optical flow " << (inputParams.enableOF ? "on.":"off.") << endl;
					break;
				case 'u':
					inputParams.enableUndistort^=1;
					cout << "Toggling undistort " << (inputParams.enableUndistort ? "on.":"off.") << endl;
					break;
				case 'm':
					inputParams.enableMask^=1;
					cout << "Toggling mask " << (inputParams.enableMask ? "on.":"off.") << endl;
					break;
				case 'g':
					inputParams.enableGraph^=1;
					cout << "Toggling graph " << (inputParams.enableGraph ? "on.":"off.") << endl;
					break;
				case 'b':
					inputParams.enableBlurredOutput^=1;
					cout << "Toggling blurred OF output " << (inputParams.enableBlurredOutput ? "on.":"off.") << endl;
					break;
				case ' ': //Space key (0x20)
					imgStateFlagKF=EXPIRED; //Space bar input resets the keyframe
					break;
				default:
					std::cout<<"key press: " << keyPress << " not known\n" << std::endl;
					break;
			}
		}
		
		//Fill attitude vector with graph data if CSV is present
		if(imuCsvDataPresent){
			fltAtt.push_back(attitudeData[attitudeIndex][7]);		
			if (fltAtt.size() > 200) { 
				//Rolling buffer. Pop the front value in order to make space for the back one
				fltAtt.pop_front();
			}
		}
		
		
		// Burn cycles if going too quickly. HAH, as if!
		do{
			//If there's CSV data present, need to make sure that it advances with the real time
			if(imuCsvDataPresent){
				while(attitudeTime-inputParams.imuStartTime < absoluteTime){
					attitudeTime=attitudeData[++attitudeIndex][2];
				}
			}
			currentTickCount=cv::getTickCount();
			
			//If the program will have to wait more than 10ms, use a cv::waitKey for part of that time
			if (tickFrequency/(double)(currentTickCount-old_TickCount) > 1/.010) {
				cv::waitKey(1);
			}
		} while(tickFrequency/(double)(currentTickCount-old_TickCount) > inputParams.FPS);
		
		//Start timer for measuring total loop cost
		old_TickCount=currentTickCount;
		
		
		//Read images. Skip images-- or read multiple times-- if decimating
		if (inputParams.captureType==-1){ 
			//Grabbing from the camera...
			for (int i=0; i<inputParams.decimateFrame; i++) {
				cap >> imgTmp;
			}
		}
		else if (inputParams.captureType==0){
			//...or else, reading from the video	
			if (inputParams.decimateFrame > 1){
				cap.set(CV_CAP_PROP_POS_FRAMES, cap.get(CV_CAP_PROP_POS_FRAMES)+inputParams.decimateFrame);					
			}
			
			cap >> imgTmp;
		}

		//Resize image if it's too large.
		if (inputParams.inputScale!=1.0) {
			cv::resize(imgTmp, imgSampleRGB, cv::Size(), inputParams.inputScale, inputParams.inputScale);
			imgTmp.release(); //Release memory. This keeps us from accidentally using this matrix later on.
		}
		else{
			imgSampleRGB=imgTmp; //Assign pointers
		}
		
		//Start timer for measuring image analysis processing cost
		imageAnalysis_TickCount=cv::getTickCount();
		

		//Undistort image. Useful for lenses like on the GoPro
		cv::Mat imgUndistortedRGB;
		if (inputParams.enableUndistort) {
			top = (int) (0.05*imgSampleRGB.rows); bottom = (int) (0.05*imgSampleRGB.rows);
			left = (int) (0.05*imgSampleRGB.cols); right = (int) (0.05*imgSampleRGB.cols);
			
			copyMakeBorder( imgSampleRGB, imgPadded, top*0, bottom*0, left*0, right*0, cv::BORDER_CONSTANT, value ); //*0 in order to turn off the padded borders
			cameraMatrix.at<double>(0,2)=imgPadded.cols/2-.5;
			cameraMatrix.at<double>(1,2)=imgPadded.rows/2-.5;	
			
			cv::undistort(imgPadded, imgUndistortedRGB, cameraMatrix, distortionCoefficients);
			imgSampleRGB.release(); //Release memory. This keeps us from accidentally using this matrix later on.
		}
		else {
			imgUndistortedRGB=imgSampleRGB;
		}

		//Convert to CIELab
//		cv::cvtColor(imgUndistortedRGB, imgCIELab, CV_RGB2Lab);
//		cv::split(imgCIELab, imgCIELabChannels);
//		imgCIELab[0].copyTo(imgUndistortedGray, quadMask);

//		//Apply mask
//		if (enableMask){
//			imgGray2.copyTo(imgTmp, quadMask);
//			imgTmp.copyTo(imgUndistortedGray);
//		}
		
		
		//Make a backup copy of RGB for Continuous Frame. This must be done before rotating it.
		imgUndistortedRGB.copyTo(imgOriginalInputCF);
		
		
		//Rotate image
		cv::Mat imgRotatedRGB;
		if(inputParams.rotationAngle != 0){
			cv::Mat tmpRotImage;
			cv::Point2f src_center(imgUndistortedRGB.cols/2.0F, imgUndistortedRGB.rows/2.0F);
			cv::Mat rot_mat = getRotationMatrix2D(src_center, inputParams.rotationAngle, 1.0);
			cv::Mat dst;
			cv::warpAffine(imgUndistortedRGB, imgRotatedRGB, rot_mat, imgUndistortedRGB.size());			
		}
		else {
			imgRotatedRGB=imgUndistortedRGB;
		}

		
		//Convert to greyscale
		cv::Mat imgRotatedGrayCF;
		cv::cvtColor(imgRotatedRGB, imgRotatedGrayCF, CV_BGR2GRAY);
//		imgRotatedRGB.release(); //Don't release memory. This image is used later on for drawing purposes
		
		//If this is the first time through the loop, copy image 
		if (loopCount==0){
			imgRotatedGrayCF.copyTo(img_old_grey);
		}
		
		if (inputParams.enableFD) {
			
			//Strive to maintain a constant number of keypoints 
			if ((matches.size() <MIN_KEYPOINT_THRESH) || (matches.size() > MAX_KEYPOINT_THRESH)){
				
				imgStateFlagKF=EXPIRED;
				
				//If there are too few matches, reduce threshold... 
				if (matches.size() <MIN_KEYPOINT_THRESH){
					inputParams.featureThreshold-=10;
				}
				else{
					//...if there are too many matches, increase threshold.
					inputParams.featureThreshold+=10;
				}
				
				//Cycle detector by releasing it and creating new one with new threshold.
				detector.release();
				detector = new cv::BriskFeatureDetector(inputParams.featureThreshold,4);
			}
			
			//Update keyframe image
			if (imgStateFlagKF==EXPIRED){
				if(!inputParams.imageKeyFrameName.length()){
					cv::cvtColor(imgOriginalInputCF, imgGrayKF, CV_BGR2GRAY);
					imgOriginalInputCF.copyTo(imgKeyframeRGB);
					
					if (inputParams.enableMask) { 
						imgGrayKF.copyTo(imgKeyframeGray, quadMask);
					}
					else{
						imgGrayKF.copyTo(imgKeyframeGray);
					}
				}
				imgStateFlagKF=UNDETECTED;
			}
			
			// run the detector:
			if (descriptorFilePresent==0){
				if (imgStateFlagKF==UNDETECTED){ //If the detector has not yet run on the keyframe, find keypoints
					if (inputParams.enableMask) {
						detector->detect(imgGrayKF, keypointsKF, quadMask);
					}
					else {
						detector->detect(imgGrayKF, keypointsKF);
					}
					for( size_t i = 0; i < matches.size(); i++ )
					{
						for( size_t j = 0; j < matches[i].size(); j++ )
						{
							keypointsKF.push_back(keypointsCF[matches[i][j].trainIdx]);
						}
					}
					imgStateFlagKF=UNEXTRACTED;
				}
				
				if (inputParams.enableMask) {
					detector->detect(imgRotatedGrayCF, keypointsCF, quadMask);
				}
				else {
					detector->detect(imgRotatedGrayCF, keypointsCF);
				}
			}
			
			// get the descriptors...
			std::vector<cv::DMatch> indices;
			
			// ... for the new image...
			descriptorExtractor->compute(imgRotatedGrayCF, keypointsCF, descriptorsCF);
			
			if (imgStateFlagKF==UNEXTRACTED || loopCount==0){
				// ...and for the old one
				descriptorExtractor->compute(imgGrayKF, keypointsKF, descriptorsKF);
				imgStateFlagKF=PROCESSED;
			}
			
			// Matching
			if(hamming){
				descriptorMatcher->radiusMatch(descriptorsCF, descriptorsKF, matches, 100.0);
			}
			else{
				descriptorMatcher->radiusMatch(descriptorsCF, descriptorsKF, matches, 0.21);
			}
			
			// Draw keyfeature matches
			cv::drawMatches(imgRotatedRGB, keypointsCF, imgKeyframeGray, keypointsKF, matches, outimg,
							cv::Scalar(0,255,0), cv::Scalar(0,0,255),
							std::vector<std::vector<char> >(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
		}
		else{
			//Output image since we are no longer drawing the keyfeature matches
			cv::Mat roiImgResult_Left  = outimg(cv::Rect(0,0,imgRotatedRGB.cols,imgRotatedRGB.rows)); //Img1 will be on the left part
			cv::Mat roiImgResult_Right = outimg(cv::Rect(imgRotatedRGB.cols,0,imgKeyframeGray.cols,imgKeyframeGray.rows)); //Img2 will be on the right part, we shift the roi of img1.cols on the right
																	
			
			imgRotatedRGB.copyTo(roiImgResult_Left); //Img1 will be on the left of imgResult
			imgKeyframeGray.copyTo(roiImgResult_Right); //Img2 will be on the right of imgResult
			
		}

		//========================//
		// Calculate optical flow //
		//========================//
		if (inputParams.enableOF){
			for (int i=0; i < numBlocks; i++){
				
				vector<cv::Point2f> points_keyPoints_LK;
				
				//Set ROI for subsquare
				cv::Mat roiSubOF;
				cv::Mat roi_old;
				if (ofScale[i]==1.0){
					roiSubOF=imgRotatedGrayCF(polarBlocks[i]);
					roi_old=img_old_grey(polarBlocks[i]);
				}
				else{
					imgTmp=imgRotatedGrayCF(polarBlocks[i]);
					cv::resize(imgTmp, roiSubOF, cv::Size(), ofScale[i], ofScale[i]);
					
					imgTmp=img_old_grey(polarBlocks[i]);
					cv::resize(imgTmp, roi_old, cv::Size(), ofScale[i], ofScale[i]);
				}
				
				roiTmp=imgRotatedGrayCF(polarBlocks[i]); //Create ROI for the subblock
				
				if(needToInitOF[i]) //Initialize keypoints for optical flow
				{
					initOF( &(needToInitOF[i]), &points_keyPoints_LK, &(nextPoints_LK[i]), roiSubOF, polarBlocks[i], ofScale[i], numBlocks, ofPointsGeneration, termcrit);
				}
				
				// convert vector of keypoints to vector of points	
				cv::KeyPoint::convert(nextPoints_LK[i], points_keyPoints_LK);  
				
				if (points_keyPoints_LK.size()){
					cv::vector<uchar> status;  
					cv::vector<float> err;  
					//Compute the optical flow
					cv::calcOpticalFlowPyrLK(roi_old, roiSubOF, points_keyPoints_LK, points_flowVector_LK, status, err, cv::Size(15,15), 3, termcrit, 0);
				}
			
				
				// Show blurred image, if enabled
				if (inputParams.enableBlurredOutput) {
					imgTmp=outimg(polarBlocks[i]);
					resize(imgTmp, roiSubOF, cv::Size(), ofScale[i], ofScale[i]);
					
					resize(roiSubOF, imgTmp, cv::Size(), 1.0/ofScale[i], 1.0/ofScale[i]);
					
					imgTmp=outimg(cv::Rect(polarBlocks[i].x, polarBlocks[i].y, imgTmp.cols, imgTmp.rows));
					
					resize(roiSubOF, imgTmp, cv::Size(), 1.0/ofScale[i], 1.0/ofScale[i]);					
				}
				
				//Plot the optical flow, only in the proper ROI
//				if (points_keyPoints_LK.size() && points_flowVector_LK.size()){
//					plotOF(outimg(polarBlocks[i]), points_keyPoints_LK, points_flowVector_LK, 1.0/ofScale[i]);
//				}				
				if (points_keyPoints_LK.size() && points_flowVector_LK.size()){
					plotOF2(outimg, polarBlocks[i], points_keyPoints_LK, points_flowVector_LK, 1.0/ofScale[i]);
				}				
			}
			//flow.updateFlowData(grayImage.getCvImage());  
			imgRotatedGrayCF.copyTo(img_old_grey);
			
		}
		
		// Change output size in case the user wants to see a larger version
		if (inputParams.outputScale!=1.0) {
			cv::resize(outimg, outimgScaled, cv::Size(), inputParams.outputScale, inputParams.outputScale);
		}
		else{
			outimgScaled=outimg;
		}
		
		// Add yaw graph, from CSV file
		if(inputParams.enableGraph){
			if(imuCsvDataPresent){
				roiTmp=outimgScaled(cv::Rect(30,outimgScaled.rows-250,1100,200));
				roiTmp=drawFloatGraph2(fltAtt, 0, roiTmp, -181, 181, roiTmp.cols, roiTmp.rows);
			}
		}
		
		// Draw blocks		
		if (inputParams.enableOF && 0){
			for (int i=0; i < numBlocks; i++) {
				//First scale the blocks...
				cv::Rect tmpBlock(polarBlocks[i].x*inputParams.outputScale, polarBlocks[i].y*inputParams.outputScale, polarBlocks[i].width*inputParams.outputScale, polarBlocks[i].height*inputParams.outputScale);
				//...then draw them
				rectangle(outimgScaled, tmpBlock, cv::Scalar(0,255,255), 0);
			}		
		}
		
		//Add text
		cv::Point string_pt(outimgScaled.cols/2+50, 40);
		cv::putText(outimgScaled, cv::format("total time: %f ms", ((double) cv::getTickCount()-old_TickCount)/tickFrequency*1000), string_pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1);
		string_pt.y+=20;
		cv::putText(outimgScaled, cv::format("comp. time: %f ms", ((double) cv::getTickCount()-imageAnalysis_TickCount)/tickFrequency*1000), string_pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1);
		string_pt.y= outimgScaled.rows-40;
		cv::putText(outimgScaled, cv::format("frame #: %i", (int) cap.get(CV_CAP_PROP_POS_FRAMES)), string_pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1);
		string_pt.y= outimgScaled.rows-20;
		cv::putText(outimgScaled, cv::format("OP time #: %f", attitudeTime), string_pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1);
		
		
		//Display image
		cv::imshow("Matches", outimgScaled);
		//		cv::imshow("gray", imgGray2);
		//		cv::imshow("L", imgCIELabChannels[0]);
		//		cv::imshow("a", imgCIELabChannels[1]);
		//		cv::imshow("b", imgCIELabChannels[2]);
		
		//Output video file
		if (inputParams.videoOutEnable) {
			generateMovie(outputMovieName, outputVideoFPS, outimgScaled, cap, outputVideo, loopCount);
		}
		
		//Wait at end of loop
		if (inputParams.captureType==-2) {
			// If we are viewing static images, wait until user hits a key before proceeding
			keyPress=cv::waitKey(0);
		}
		else{
			// Else wait a couple ms for image to properly display.
			keyPress=cv::waitKey(2);
		}
		
		// Update loop loopCount
		loopCount++;
		absoluteTime+=(imageDelT*inputParams.decimateFrame);
		
		if (cap.get(CV_CAP_PROP_POS_FRAMES)==1372) {
			cv::waitKey(0);
		}
		
	}

	return 0;
}


/*
 * Movie generator
 */
void generateMovie(string outputMovieName, float outputVideoFPS, cv::Mat outimgScaled, cv::VideoCapture cap, cv::VideoWriter outputVideo, unsigned long loopCount){
	
	//============================//
	// Create movie output module //
	//============================//
	
	if (loopCount==0) {
		//				string::size_type pAt = videoOutFile.find_last_of('.');   // Find extension point
		//				const string NAME = videoOutFile.substr(0, pAt) + ".avi";   // Form the new name with container		
		int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));     // Get Codec Type- Int form	}
		
		union { int v; char c[5];} uEx ;
		uEx.v = ex;                              // From Int to char via union
		uEx.c[4]='\0';
		
		cv::Size S = cv::Size((int) outimgScaled.rows,    //Acquire input size
									 (int) outimgScaled.cols);
		
		outputVideo.open(outputMovieName , CV_FOURCC('I', 'Y', 'U', 'V'), outputVideoFPS, S, true);
		
		if (!outputVideo.isOpened()){
			cout << "Could not open video output file " << outputMovieName << endl;
			exit(0);
		}				
	}
	
#ifdef __APPLE__
	//HAVE TO DO THIS HACK BECAUSE OPENCV DOESN'T OUTPUT VIDEO ON OSX
	std::vector<int> qualityType;
	qualityType.push_back(CV_IMWRITE_JPEG_QUALITY);
	qualityType.push_back(100);
	
	std::string s;
	std::stringstream out;
	out << "./imgOutput/" << outputMovieName << "-" << setfill('0') << setw(4) << loopCount << ".jpg";
	s = out.str();
	
	imwrite( s, outimgScaled, qualityType);
#else
	//DOESN'T WORK ON MAC OSX
	outputVideo << outimgScaled;
#endif
	
	return;
}