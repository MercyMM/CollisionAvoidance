/*
 *  ui.cpp
 *  COLA
 *
 *  @Authors: PGalvin, KSebesta
 *
 */

#include "ui.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string.h>

using namespace std;


// Check for the existence of a file
bool fexists(string filename)
{
	ifstream ifile( filename.c_str() ); //Try opening it. On a failure to open, ifile = 0
	return ifile;
}

void parseKeyboardInput(InputParams inputParams, int keyPress){
//	//If we're reading from the video, accept key inputs for advancing and rewinding the movie
//	if (inputParams.captureType==0){
//		//Read input cursors to advance or rewind movie
//		if(keyPress>0){
//			int oldPos=cap.get(CV_CAP_PROP_POS_FRAMES);
//			switch (keyPress) {
//				case LEFT_ARROW:
//					cap.set(CV_CAP_PROP_POS_FRAMES, oldPos-30);
//					break;
//				case UP_ARROW:
//					cap.set(CV_CAP_PROP_POS_FRAMES, oldPos+30*5);
//					break;
//				case RIGHT_ARROW:
//					cap.set(CV_CAP_PROP_POS_FRAMES, oldPos+30);
//					break;
//				case DOWN_ARROW:
//					cap.set(CV_CAP_PROP_POS_FRAMES, oldPos-30*5);
//					break;
//				case 'p':
//					cvWaitKey(0);
//					break;
//				default:
//					std::cout<<"key press: " << keyPress << " not known\n" << std::endl;
//					break;
//			}
//			
//			//Don't forget to also advance the absolute time clock
//			absoluteTime+=(imageDelT*(cap.get(CV_CAP_PROP_POS_FRAMES)-oldPos));
//		}
//	}
//	
//	//Handle more key inputs
//	if(keyPress>0){
//		switch (keyPress) {
//			case 'o':
//				needToInitOF=vector <char>(numBlocks, true);
//				if (enableOF) {
//					ofPointsGeneration=(ofPointsGeneration+1)%2;
//				}
//				enableOF^=1;
//				cout << "Toggling optical flow " << (enableOF ? "on.":"off.") << endl;
//				break;
//			case 'u':
//				enableUndistort^=1;
//				cout << "Toggling undistort " << (enableUndistort ? "on.":"off.") << endl;
//				break;
//			case 'm':
//				enableMask^=1;
//				cout << "Toggling mask " << (enableMask ? "on.":"off.") << endl;
//				break;
//			case 'g':
//				enableGraph^=1;
//				cout << "Toggling graph " << (enableGraph ? "on.":"off.") << endl;
//				break;
//			case 'b':
//				enableBlurredOutput^=1;
//				cout << "Toggling blurred OF output " << (enableBlurredOutput ? "on.":"off.") << endl;
//				break;
//			default:
//				std::cout<<"key press: " << keyPress << " not known\n" << std::endl;
//				break;
//		}
//	}
	
}

InputParams parseArguments(int argc, char * const argv[])
{
	InputParams inputParams;

	//Set default values
	inputParams.captureType=-2;
	inputParams.rotationAngle=0;
	inputParams.startFrame=0;
	inputParams.inputScale=1;
	inputParams.outputScale=1;
	inputParams.FPS=30;
	inputParams.enableMask=0;
	inputParams.enableFD=1;
	inputParams.enableOF=1;
	inputParams.enableGraph=0;
	inputParams.enableUndistort=0;
	inputParams.enableBlurredOutput=0;
	inputParams.decimateFrame=1;
	inputParams.imuStartTime=0;
	inputParams.ofBlurring=0.5;	
	
	inputParams.detectorType=NO_ALGO;
	inputParams.descriptorType=NO_ALGO;
	inputParams.featureThreshold=0;

	inputParams.imageKeyFrameName="";
	inputParams.imageCurrentFrameName="";
	inputParams.vidFileName="";
	
	inputParams.videoOutEnable=0;
	inputParams.videoOutFile="";

	vector<string> fextensions=generateImageFileExtList();
	
	if(argc==1){
		//Simple demo, loads two images so the user doesn't have to learn lots in order to get the program running
		string fname1;
		string fname2;
		bool fid;
		
		//Check for existence of file
		fname1 = "../../images/img1.ppm";
		fid=fexists(fname1);
		if (!fid){
			cout<<"image "<<fname1<<" not found." << endl;
			exit(2);
		}
		
		//Check for existence of file
		fid=false;
		fname2 = "../../images/img2.ppm";
		fid=fexists(fname2);
		if (!fid){
			cout<<"image "<<fname2<<" not found." << endl;
			exit(2);
		}

		inputParams.imageKeyFrameName=fname1;
		inputParams.imageCurrentFrameName=fname2;
		
		inputParams.detectorType=BRISK;
		inputParams.descriptorType=BRISK;
		inputParams.featureThreshold = 100;
		
	}
	else{
		//Ahhh, maybe the user wants a little more out of the demo. Let's give him/her some options.
		for (int i=1; i<argc; i++){
			if (strncmp("-", argv[i], 1)==0 ){
				if(strncmp("h", argv[1]+1, 1)==0 || strncmp("-help", argv[1]+1, 5)==0){
					//Help
					help();
					exit (1);
				}				
				else if(strncmp("-video", argv[i]+1, 6)==0){
					//Set video type
					if(strncmp("cam", argv[i+1], 3)==0){
						inputParams.captureType=-1;
					}
					else{
						inputParams.captureType=0;
						inputParams.vidFileName=argv[i+1];
					}
					
					i++;
				}
				else if(strncmp("-image", argv[i]+1, 6)==0){
					int j=0;
					int fextensions_size=fextensions.size();
					string fname=argv[i+1];
					//First try reading whole file name. This is in case the call includes the extensions, such as `--image foo.jpg`
					bool fid=fexists(fname);
					while(!fid){						
						//Else, perhaps the call does not include the extensions, such as `--image foo`, and so now we try to find the full extension
						fname = argv[i+1]+fextensions[j];
						fid=fexists(fname);
						j++;
						if(j>=fextensions_size) break;
					}
					
					if (!fid)
					{
						cout<<"image "<<fname<<" not found." << endl;
						exit(2);
					}
					
					if(strncmp("KF", argv[i]+7, 1)==0){
						inputParams.imageKeyFrameName=fname;
					}
					else if(strncmp("CF", argv[i]+7, 1)==0 && inputParams.captureType==-2){
						inputParams.imageCurrentFrameName=fname;
					}
					else{
						if (inputParams.captureType!=-2 && strncmp("CF", argv[i]+7, 1)==0) {
							cout<<"Improper call for input image " << argv[i]+7 <<". Second image and movie input cannot both be selected at same time." << endl;
						}
						else{
							cout<<"Improper call for input image " << argv[i]+7 <<"." << endl;
						}
						exit(2);
					}
					
					i++;
				}
				else if(strncmp("-csv", argv[i]+1, 4)==0){
					inputParams.csvInputFile=argv[i+1];
					i++;
				}
				else if(strncmp("-rot", argv[i]+1, 4)==0){
					inputParams.rotationAngle=atof(argv[i]+6);					
				}
				else if(strncmp("-fps", argv[i]+1, 4)==0){
					//Set frames per second
					inputParams.FPS=atof(argv[i]+6);
				}
				else if(strncmp("-input_scale", argv[i]+1, 12)==0){
					//Set video input scaling
					inputParams.inputScale=atof(argv[i]+14);
				}
				else if(strncmp("-output_scale", argv[i]+1, 13)==0){
					//Set video input scaling
					inputParams.outputScale=atof(argv[i]+15);
				}
				else if(strncmp("-start_frame", argv[i]+1, 12)==0){
					//Set video output scaling
					inputParams.startFrame=atoi(argv[i]+14);
				}
				else if(strncmp("-start_time", argv[i]+1, 11)==0){
					//Set IMU data start time
					inputParams.imuStartTime=atof(argv[i]+13);
				}
				else if(strncmp("-decimate", argv[i]+1, 9)==0){
					//Set video sequence decimation
					inputParams.decimateFrame=atoi(argv[i]+11);
				}
				else if(strncmp("-mask", argv[i]+1, 5)==0){
					//enable image masking
					inputParams.enableMask=atoi(argv[i]+7);
				}
				else if(strncmp("-OF_blurring", argv[i]+1, 12)==0){
					//enable optical flow
					inputParams.ofBlurring=atof(argv[i]+14);
					inputParams.enableBlurredOutput=1;
				}				
				else if(strncmp("-OF", argv[i]+1, 3)==0){
					//enable optical flow
					inputParams.enableOF=atoi(argv[i]+5);
				}
				else if(strncmp("-output", argv[i]+1, 7)==0){
					//Write video to ouput file
					inputParams.videoOutFile=argv[i+1];
					inputParams.videoOutEnable=1;
					i++;
				}
				else if(strncmp("-detector", argv[i]+1, 9)==0){
					if (strncmp("FAST", argv[i+1], 4)==0) {
						inputParams.detectorType=FAST;
						inputParams.featureThreshold = atoi(argv[i+1]+5);
					}
					else if (strncmp("AGAST", argv[i+1], 5)==0) {
						inputParams.detectorType=AGAST;
						inputParams.featureThreshold = atoi(argv[i+1]+6);
					}
					else if (strncmp("BRISK", argv[i+1], 5)==0) {
						inputParams.detectorType=BRISK;
						inputParams.featureThreshold = atoi(argv[i+1]+6);
					}
					else if (strncmp("SURF", argv[i+1], 4)==0) {
						inputParams.detectorType=SURF;
						inputParams.featureThreshold = atoi(argv[i+1]+5);
					}
					else if (strncmp("SIFT", argv[i+1], 4)==0) {
						inputParams.detectorType=SIFT;
						inputParams.featureThreshold = atoi(argv[i+1]+5);
					}
					else{
						//Bad detector input
						cout << "Detector " << argv[i+1] << " not recognized. Check spelling!" << endl;
						exit(1);
					}
					i++;
				}				
				else if(strncmp("-descriptor_file", argv[i]+1, 15)==0){ //DO NOT MOVE RELATIVE TO --descriptor SINCE OTHERWISE THE CONDITION WILL NEVER BE TESTED
//					desc1 = string(argv[i+1]);
//					desc2 = string(argv[i+2]);
//					
//					descriptorFilePresent=1;
//					i++;
				}
				else if(strncmp("-descriptor", argv[i]+1, 10)==0){
					if (strncmp("BRISK", argv[i+1], 4)==0) {
						inputParams.descriptorType=BRISK;
					}
					else if (strncmp("U-BRISK", argv[i+1], 5)==0) {
						inputParams.descriptorType=U_BRISK;
					}
					else if (strncmp("SU-BRISK", argv[i+1], 5)==0) {
						inputParams.descriptorType=SU_BRISK;
					}
					else if (strncmp("S-BRISK", argv[i+1], 5)==0) {
						inputParams.descriptorType=S_BRISK;
					}
					else if (strncmp("BRIEF", argv[i+1], 5)==0) {
						inputParams.descriptorType=BRIEF;
					}
					else if (strncmp("CALONDER", argv[i+1], 5)==0) {
						inputParams.descriptorType=CALONDER;
					}
					else if (strncmp("SURF", argv[i+1], 4)==0) {
						inputParams.descriptorType=SURF;
					}
					else if (strncmp("SIFT", argv[i+1], 4)==0) {
						inputParams.descriptorType=SIFT;
					}
					else{
						//Bad extractor input
						cout << "Descriptor " << argv[i+1] << " not recognized. Check spelling!" << endl;
						exit(1);
					}
					i++;
				}
				else{
					help();
					exit(1);
				}
			}
		}
	}
	
	return inputParams;	
}

vector<string> generateImageFileExtList()
{
	
	// standard file extensions for opening images
	vector<string> fextensions;
	fextensions.push_back(".bmp");
	fextensions.push_back(".jpeg");
	fextensions.push_back(".jpg");
	fextensions.push_back(".jpe");
	fextensions.push_back(".jp2");
	fextensions.push_back(".png");
	fextensions.push_back(".pgm");
	fextensions.push_back(".ppm");
	fextensions.push_back(".sr");
	fextensions.push_back(".ras");
	fextensions.push_back(".tiff");
	fextensions.push_back(".tif");
	
	return fextensions;
}


void help()
{
	cout << "This command line tool lets you evaluate different keypoint "
	<< "detectors, descriptors and matchers." << endl
	<< "Usage:" "demo [IMAGE] [--detector] [--descriptor] [--descriptor_file] [--fps] [--scale] [--start_frame] [--rot]" << endl
	<< " " << "IMAGE:                        Input images to process. Images must be of type *.ppm "<< endl
	<< " " << "   --imageKF [FILEPATH]       file path to keyframe image image." << endl
	<< " " << "   --imageCF [FILEPATH]       Chooses current frame input as static. Requires file path to second image." << endl
	<< " " << "                                           or"<< endl
	<< " " << "   --video		              Chooses current frame input as video. Requires defining video type."<< endl
	<< " " << "     cam                          defines video type as live camera." << endl
	<< " " << "                                           or"<< endl
	<< " " << "     [FILEPATH]                   defines video type as movie. Requires file path to movie." << endl
	<< " " << "--detector[=THRESHOLD]:       Feature detector: FAST, AGAST, BRISK, SURF, or SIFT. You can add the "<< endl
	<< " " << "                                 threshold, e.g. BRISK=80 or SURF=2000"<< endl
	<< " " << "--descriptor:                 Feature descriptor: SIFT, SURF, CALONDER, BRIEF, BRISK, S-BRISK, SU-BRISK or U-BRISK."<< endl
	<< " " << "--descriptor_file [FILEPATH]: Optional: files with descriptors to act as detected points."<< endl
	<< " " << "--csv:                        Optional: input csv file with IMU data."<< endl
	<< " " << "--fps [FPS]:                  Optional: maximum image processing speed per second."<< endl
	<< " " << "--input_scale=[SCALE]:        Optional: scale input images."<< endl
	<< " " << "--output_scale=[SCALE]:       Optional: scale output images."<< endl
	<< " " << "--start_frame=[FRAME #]:      Optional: start processing movie files at the requested frame."<< endl
	<< " " << "--rot=[ANGLE]:                Optional: rotation in degrees of the 1st image." << endl
	<< " " << "--decimate=[N]:               Optional: use only 1 out of N images." << endl
	<< " " << "--output [FILEPATH]:          Optional: write video to file." << endl
	<< " " <<  endl
	<< " " << "example usage: demo --video ~/Movies/foo.MP4  --detector BRISK100 --descriptor BRISK --fps 30 --imageKF ../../images/img1.ppm" << endl
	<< " " << "             : demo --video cam --detector SURF2000 --descriptor BRISK --scale 0.5 --fps 15 --imageKF ../../images/img1.ppm" << endl
	<< " " << "             : demo --detector FAST100 --descriptor BRIEF --imageKF ../../images/img1.ppm --imageCF ../../images/img2.ppm --rot 10" << endl
	<< " " << "             : demo --video ~/Movies/foo.MP4 --detector AGAST100 --descriptor BRIEF --imageKF ../../images/img1.ppm --start_frame=1234" << endl;
}
