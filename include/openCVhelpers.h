/*
 *  openCVhelpers.h
 *  COLA
 *
 *  @Authors: PGalvin, KSebesta
 *
 */


#ifndef __OPENCVHELPERS
#define __OPENCVHELPERS

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

using namespace std;

typedef vector <double> record_t;
typedef vector <record_t> data_t;


istream& operator >> ( istream& ins, record_t& record );
istream& operator >> ( istream& ins, data_t& data );
std::vector<cv::Point3f> Create3DChessboardCorners(cv::Size boardSize, float squareSize);


#endif

