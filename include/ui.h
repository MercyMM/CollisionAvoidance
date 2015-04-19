/*
 *  ui.h
 *  COLA
 *
 *  @Authors: PGalvin, KSebesta
 *
 */

#ifndef __UI
#define __UI

#include <vector>
#include <string>

using namespace std;

enum AlgoType {
	FAST, AGAST, BRISK, SURF, SIFT, U_BRISK, SU_BRISK, S_BRISK, BRIEF, CALONDER, NO_ALGO
};


struct InputParams {
	int captureType;
	string vidFileName;
	float rotationAngle;
	unsigned long startFrame;
	float inputScale;
	float outputScale;
	float FPS;
	bool enableMask;
	bool enableGraph;
	bool enableBlurredOutput;
	bool enableFD;
	bool enableOF;
	bool enableUndistort;
	int decimateFrame;
	double imuStartTime;
	double ofBlurring;
	int detectorType;
	int descriptorType;
	int featureThreshold;
	string imageKeyFrameName;
	string imageCurrentFrameName;
	bool videoOutEnable;
	string videoOutFile;
	string csvInputFile;
};


void help();
InputParams parseArguments(int argc, char * const argv[]);

vector <string> generateImageFileExtList();


#endif
