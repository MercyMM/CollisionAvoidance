/*
    BRISK - Binary Robust Invariant Scalable Keypoints
    Reference implementation of
    [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
    	Binary Robust Invariant Scalable Keypoints, in Proceedings of
    	the IEEE International Conference on Computer Vision (ICCV2011).

    Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
    Stefan Leutenegger and Margarita Chli.

    This file is part of BRISK.

    BRISK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    BRISK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with BRISK.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OF_HPP_
#define OF_HPP_
#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

void plotOF(cv::Mat outimgROI, std::vector<cv::Point2f> points_keyPoints_LK, std::vector<cv::Point2f> points_nextPoints_LK, double scale);
void plotOF2(cv::Mat outimg, cv::Rect polarBlocks, std::vector<cv::Point2f> points_keyPoints_LK, std::vector<cv::Point2f> points_nextPoints_LK, double scale);
void initOF(char *needToInitOF, vector<cv::Point2f> *points_keyPoints_LK, vector<cv::KeyPoint> *nextPoint_LK, cv::Mat roiSubOF, cv::Rect polarBlock, double ofScale, int numBlocks, int ofPointsGeneration, cv::TermCriteria termcrit);

#endif /* OF_HPP_ */
