#include <fstream>
#include <iostream>
#include <list>
#include <opencv2/opencv.hpp>

#include "../include/OF.hpp"


void calculateOF(	int numBlocks, int ofPointsGeneration){
//	
//	cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03); 
//	cv::Mat imgTmp;
//	cv::Mat roiTmp;
//	
//	for (int i=0; i < numBlocks; i++){
//		
//		vector<cv::Point2f> points_keyPoints_LK;
//		
//		//Set ROI for subsquare
//		cv::Mat roiSubOF;
//		cv::Mat roi_old;
//		if (ofScale[i]==1.0){
//			roiSubOF=imgRotatedGrayCF(polarBlocks[i]);
//			roi_old=img_old_grey(polarBlocks[i]);
//		}
//		else{
//			imgTmp=imgRotatedGrayCF(polarBlocks[i]);
//			cv::resize(imgTmp, roiSubOF, cv::Size(), ofScale[i], ofScale[i]);
//			
//			imgTmp=img_old_grey(polarBlocks[i]);
//			cv::resize(imgTmp, roi_old, cv::Size(), ofScale[i], ofScale[i]);
//		}
//		
//		roiTmp=imgRotatedGrayCF(polarBlocks[i]);
//		
//		
//		bool addRemovePt = false;
//		
//		if( needToInit[i])
//		{
//			int numPointsH=5;
//			int numPointsW=numPointsH*1280/720;
//			
//			cv::Size subPixWinSize(10,10);
//			const int MAX_COUNT = 500/numBlocks;
//			
//			switch (ofPointsGeneration) {
//				case 0:
//					for (int j=0; j<numPointsH; j++) {
//						for (int k=0; k<numPointsW; k++) {
//							//Points are referenced from within the ROI
//							points_keyPoints_LK.push_back(cv::Point2f((polarBlocks[i].width/(2*numPointsW)+polarBlocks[i].width*k/numPointsW)*ofScale[i], (polarBlocks[i].height/(2*numPointsH)+polarBlocks[i].height*j/numPointsH)*ofScale[i]));
//						}
//					}
//					
//					// convert vector of points to vector of keypoints
//					cv::KeyPoint::convert(points_keyPoints_LK, nextPoints_LK[i]);
//					
//					break;
//				case 1:
//					//Save old keypoints
//					cv::KeyPoint::convert(nextPoints_LK[i], points_keyPoints_LK);
//					
//					// automatic initialization
//					//Choose most interesting points for optical flow
//					//							cv::FAST(roiSubOF, nextPoints_LK[i], 40, true);  
//					goodFeaturesToTrack(roiSubOF, points_keyPoints_LK, MAX_COUNT, 0.01, 10, cv::Mat(), 3, 0, 0.04);
//					cornerSubPix(roiSubOF, points_keyPoints_LK, subPixWinSize, cv::Size(-1,-1), termcrit);
//					addRemovePt = false;						
//					
//					// convert vector of points to vector of keypoints
//					cv::KeyPoint::convert(points_keyPoints_LK, nextPoints_LK[i]);
//					
//					break;
//				default:
//					break;
//			} 
//			
//			needToInit[i] = false;
//		}
//		
//		// convert vector of keypoints to vector of points	
//		cv::KeyPoint::convert(nextPoints_LK[i], points_keyPoints_LK);  
//		
//		if (points_keyPoints_LK.size()){
//			cv::vector<uchar> status;  
//			cv::vector<float> err;  
//			//Compute the optical flow
//			cv::calcOpticalFlowPyrLK(roi_old, roiSubOF, points_keyPoints_LK, points_nextPoints_LK, status, err, cv::Size(15,15), 3, termcrit, 0);  
//		}
//		
//		
//		// Show blurred image, if enabled
//		if (enableBlurredOutput) {
//			imgTmp=outimg(polarBlocks[i]);
//			resize(imgTmp, roiSubOF, cv::Size(), ofScale[i], ofScale[i]);
//			
//			resize(roiSubOF, imgTmp, cv::Size(), 1.0/ofScale[i], 1.0/ofScale[i]);
//			
//			imgTmp=outimg(cv::Rect(polarBlocks[i].x, polarBlocks[i].y, imgTmp.cols, imgTmp.rows));
//			
//			resize(roiSubOF, imgTmp, cv::Size(), 1.0/ofScale[i], 1.0/ofScale[i]);					
//		}
//		
//		//Plot the optical flow, only in the proper ROI
//		if (points_keyPoints_LK.size() && points_nextPoints_LK.size()){
//			plotOF(outimg(polarBlocks[i]), points_keyPoints_LK, points_nextPoints_LK, 1.0/ofScale[i]);
//		}
//		
//		
//	}
//	//flow.updateFlowData(grayImage.getCvImage());  
//	imgRotatedGrayCF.copyTo(img_old_grey);
}



void initOF(char *needToInitOF, vector<cv::Point2f> *points_keyPoints_LK, vector<cv::KeyPoint> *nextPoint_LK, cv::Mat roiSubOF, cv::Rect polarBlock, double ofScale, int numBlocks, int ofPointsGeneration, cv::TermCriteria termcrit){

	int numPointsH=5;
	int numPointsW=numPointsH*1280/720;
	cv::Size subPixWinSize(10,10);
	const int MAX_COUNT = 500/numBlocks;
	
	switch (ofPointsGeneration) {
		case 0: //Use uniform grid of keypoints

			for (int j=0; j<numPointsH; j++) {
				for (int k=0; k<numPointsW; k++) {
					//Points are referenced from within the ROI
					points_keyPoints_LK->push_back(cv::Point2f((polarBlock.width/(2*numPointsW)+polarBlock.width*k/numPointsW)*ofScale, (polarBlock.height/(2*numPointsH)+polarBlock.height*j/numPointsH)*ofScale));
				}
			}
			
			// convert vector of points to vector of keypoints
			cv::KeyPoint::convert(*points_keyPoints_LK, *nextPoint_LK);
			
			break;
		case 1: //Use "interesting" keypoints
			//Save old keypoints
			cv::KeyPoint::convert(*nextPoint_LK, *points_keyPoints_LK);
			
			// automatic initialization
			//Choose most interesting points for optical flow
			//							cv::FAST(roiSubOF, nextPoints_LK[i], 40, true);  
			goodFeaturesToTrack(roiSubOF, *points_keyPoints_LK, MAX_COUNT, 0.01, 10, cv::Mat(), 3, 0, 0.04);
			cornerSubPix(roiSubOF, *points_keyPoints_LK, subPixWinSize, cv::Size(-1,-1), termcrit);
//			addRemovePt = false;						
			
			// convert vector of points to vector of keypoints
			cv::KeyPoint::convert(*points_keyPoints_LK, *nextPoint_LK);
			
			break;
		default:
			break;
	} 
	
	*needToInitOF = false;
}




void plotOF(cv::Mat outimgROI, std::vector<cv::Point2f> arrowTail, std::vector<cv::Point2f> arrowHead, double scale){
	
	/* For fun (and debugging :)), let's draw the flow field. */
	for( int i = 0; i < (int)arrowHead.size(); i++)
	{
		/* Set quiver line thickness */
		int line_thickness;				line_thickness = 3;
		
		/* CV_RGB(red, green, blue) is the red, green, and blue components
		 * of the color you want, each out of 255.
		 */	
		CvScalar line_color;			line_color = CV_RGB(255,255,0);
		
		/* Let's make the flow field look nice with arrows. */
		/* The arrows will be a bit too short for a nice visualization because of the high framerate
		 * (ie: there's not much motion between the frames).  So let's lengthen them by a factor of 3.
		 */
		CvPoint p,q;
		p.x = (int) arrowTail[i].x*scale;
		p.y = (int) arrowTail[i].y*scale;
		q.x = (int) arrowHead[i].x*scale;
		q.y = (int) arrowHead[i].y*scale;
		double hypotenuse;	hypotenuse = sqrt( (p.y - q.y)*(p.y - q.y) + (p.x - q.x)*(p.x - q.x) );
		
		if (hypotenuse > 0){
			
			double angle;		angle = atan2( (double) p.y - q.y, (double) p.x - q.x );
			
			/* Here we lengthen the arrow by a factor of three. */
			q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
			q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
			
			/* Now we draw the main line of the arrow. */
			/* "frame1" is the frame to draw on.
			 * "p" is the point where the line begins.
			 * "q" is the point where the line stops.
			 * "CV_AA" means antialiased drawing.
			 * "0" means no fractional bits in the center cooridinate or radius.
			 */
			cv::line( outimgROI, p, q, line_color, line_thickness, CV_AA, 0 );
			
			/* Now draw the tips of the arrow.  I do some scaling so that the
			 * tips look proportional to the main line of the arrow.
			 */			
			double pi=3.141529654;
			p.x = (int) (q.x + 9*2/3 * cos(angle + pi / 4));
			p.y = (int) (q.y + 9*2/3 * sin(angle + pi / 4));
			cv::line( outimgROI, p, q, line_color, line_thickness, CV_AA, 0 );
			p.x = (int) (q.x + 9*2/3 * cos(angle - pi / 4));
			p.y = (int) (q.y + 9*2/3 * sin(angle - pi / 4));
			cv::line( outimgROI, p, q, line_color, line_thickness, CV_AA, 0 );
		}
	}	
}

void plotOF2(cv::Mat outimg, cv::Rect polarBlocks, std::vector<cv::Point2f> arrowTail, std::vector<cv::Point2f> arrowHead, double scale){
	
	/* For fun (and debugging :)), let's draw the flow field. */
	for( int i = 0; i < (int)arrowHead.size(); i++)
	{
		/* Set quiver line thickness */
		int line_thickness;				line_thickness = 1;
		
		/* CV_RGB(red, green, blue) is the red, green, and blue components
		 * of the color you want, each out of 255.
		 */	
		CvScalar line_color;			line_color = CV_RGB(255,255,0);
		
		/* Let's make the flow field look nice with arrows. */
		/* The arrows will be a bit too short for a nice visualization because of the high framerate
		 * (ie: there's not much motion between the frames).  So let's lengthen them by a factor of 3.
		 */
		CvPoint p0,q0;
		p0.x = (int) arrowTail[i].x*scale+polarBlocks.x;
		p0.y = (int) arrowTail[i].y*scale+polarBlocks.y;
		q0.x = (int) arrowHead[i].x*scale+polarBlocks.x;
		q0.y = (int) arrowHead[i].y*scale+polarBlocks.y;
		double hypotenuse;	hypotenuse = sqrt( (p0.y - q0.y)*(p0.y - q0.y) + (p0.x - q0.x)*(p0.x - q0.x) );
		
		if (hypotenuse > 0 && 1){
			CvPoint p,q;
			
			double angle;		
			angle = atan2( (double) p0.y - q0.y, (double) p0.x - q0.x );
			
			/* Here we lengthen the arrow by a factor of three. */
			q.x = (int) (p0.x - 3 * hypotenuse * cos(angle));
			q.y = (int) (p0.y - 3 * hypotenuse * sin(angle));
			
			/* Now we draw the main line of the arrow. */
			/* "frame1" is the frame to draw on.
			 * "p" is the point where the line begins.
			 * "q" is the point where the line stops.
			 * "CV_AA" means antialiased drawing.
			 * "0" means no fractional bits in the center cooridinate or radius.
			 */
			cv::line( outimg, p0, q, line_color, line_thickness, CV_AA, 0 );
			
			/* Now draw the tips of the arrow.  I do some scaling so that the
			 * tips look proportional to the main line of the arrow.
			 */			
			double pi=3.141529654;
			p.x = (int) (q.x + 9*2/3 * cos(angle + pi / 4));
			p.y = (int) (q.y + 9*2/3 * sin(angle + pi / 4));
			cv::line( outimg, p, q, line_color, line_thickness, CV_AA, 0 );
			p.x = (int) (q.x + 9*2/3 * cos(angle - pi / 4));
			p.y = (int) (q.y + 9*2/3 * sin(angle - pi / 4));
			cv::line( outimg, p, q, line_color, line_thickness, CV_AA, 0 );
		}
		if (hypotenuse > 0 && 0 ){
			double dx, dy, dx_dot, dy_dot, tau;
			dx=p0.x-639.5;
			dy=p0.y-359.5;
			dx_dot=(arrowHead[i].x-arrowTail[i].x)*50;
			dy_dot=(arrowHead[i].y-arrowTail[i].y)*50;
			
			tau=sqrt((dx*dx+dy*dy)/(dx_dot*dx_dot+dy_dot*dy_dot));
			tau=dx/dx_dot;
			
			if (fabs(tau) <300 && tau > 0.75 ){
				cv::Point string_pt(p0.x-30, p0.y);
				if (tau < 1.0 ) {
					cv::putText(outimg, cv::format("%3.2f", tau), string_pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 2);					
				}else {
//					cv::putText(outimg, cv::format("%3.2f", tau), string_pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 2);
				}
			}			
		}
	}	
}