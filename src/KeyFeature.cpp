#include <fstream>
#include <iostream>
#include <list>

#include "../include/KeyFeature.hpp"

void keyFeatureExtraction(cv::Mat imgGray2, std::vector<cv::Point2f> points_keyPoints_LK, std::vector<cv::Point2f> points_nextPoints_LK){
//	
//	// run the detector:
//	if(argc == 7){
//		// try to read descriptor files
//		std::string desc1 = std::string(argv[5]);
//		std::string desc2 = std::string(argv[6]);
//		std::ifstream descf1(desc1.c_str());
//		if(!descf1.good()){
//			std::cout<<"Descriptor file not found at " << desc1<<std::endl;
//			return 3;
//		}
//		std::ifstream descf2(desc2.c_str());
//		if(!descf2.good()){
//			std::cout<<"Descriptor file not found at " << desc2<<std::endl;
//			return 3;
//		}
//		
//		// fill the keypoints
//		std::string str1;
//		std::stringstream strstrm1;
//		std::getline(descf1,str1);
//		std::getline(descf1,str1);
//		while(!descf1.eof()){
//			std::getline(descf1,str1);
//			float x,y,a;
//			strstrm1.str(str1);
//			strstrm1>>x;
//			strstrm1>>y;
//			strstrm1>>a;
//			float r=sqrt(1.0/a);
//			keypoints.push_back(cv::KeyPoint(x, y, 4.0*r));
//		}
//		std::string str2;
//		std::stringstream strstrm2;
//		std::getline(descf2,str2);
//		std::getline(descf2,str2);
//		while(!descf2.eof()){
//			std::getline(descf2,str2);
//			float x,y,a;
//			strstrm2.str(str2);
//			strstrm2>>x;
//			strstrm2>>y;
//			strstrm2>>a;
//			float r=sqrt(1.0/a);
//			keypoints2.push_back(cv::KeyPoint(x, y, 4.0*r));
//		}
//		
//		// clean up
//		descf1.close();
//		descf2.close();
//	}
//	else{
//		if (newImageFlag==1 || counter==0){
//			detector->detect(imgGray1,keypoints);
//		}
//		detector->detect(imgGray2,keypoints2);
//	}
//	
//	// get the descriptors
//	std::vector<cv::DMatch> indices;
//	
//	// new image
//	descriptorExtractor->compute(imgGray2,keypoints2,descriptors2);
//	
//	if (newImageFlag==1 || counter==0){
//		// and the old one
//		descriptorExtractor->compute(imgGray1,keypoints,descriptors);
//	}
//	
//	// matching
//	
//	if(hamming){
//		descriptorMatcher->radiusMatch(descriptors2,descriptors,matches,100.0);
//	}
//	else{
//		descriptorMatcher->radiusMatch(descriptors2,descriptors,matches,0.21);
//	}	
}


//Generate the detector
cv::Ptr<cv::FeatureDetector> createDetector(InputParams inputParams){
	cv::Ptr<cv::FeatureDetector> detector;

	switch (inputParams.detectorType) {
			float thresh;
			float edgeThreshold;
			
		case FAST:
			if(inputParams.featureThreshold==0)
				inputParams.featureThreshold = 30;
			detector = new cv::FastFeatureDetector(inputParams.featureThreshold,true);
			break;
		case AGAST:
			if(inputParams.featureThreshold==0)
				inputParams.featureThreshold = 30;
			detector = new cv::BriskFeatureDetector(inputParams.featureThreshold,0);				
			break;
		case BRISK:
			if(inputParams.featureThreshold==0)
				inputParams.featureThreshold = 30;
			detector = new cv::BriskFeatureDetector(inputParams.featureThreshold,4);		
			break;
		case SURF:
			if(inputParams.featureThreshold==0)
				inputParams.featureThreshold = 400;
			detector = new cv::SurfFeatureDetector(inputParams.featureThreshold);
			break;
		case SIFT:
			thresh = 0.04 / cv::SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS / 2.0;
			edgeThreshold=(float) inputParams.featureThreshold;
			if(edgeThreshold==0)
				thresh = 10.0;
			detector = new cv::SiftFeatureDetector(thresh,edgeThreshold);
			break;
		default:
			std::cout << "Detector " << inputParams.detectorType << " not specified. Must specify a detector!" << std::endl;
			exit(3);
	}
	
	if (detector.empty()){
		std::cout << "Detector " << inputParams.detectorType << " not recognized. Check spelling!" << std::endl;
		exit(3);
	}
	
	return detector;
	
}

// Generate the extractor
void createExtractor(InputParams inputParams, cv::Ptr<cv::DescriptorExtractor> *descriptorExtractor, cv::Ptr<cv::DescriptorMatcher> *descriptorMatcher, bool *hamming){
	
	switch (inputParams.descriptorType) {
		case BRISK:
			*descriptorExtractor = new cv::BriskDescriptorExtractor();
		case U_BRISK:
			*descriptorExtractor = new cv::BriskDescriptorExtractor(false);
			break;
		case SU_BRISK:
			*descriptorExtractor = new cv::BriskDescriptorExtractor(false,false);
			break;
		case S_BRISK:
			*descriptorExtractor = new cv::BriskDescriptorExtractor(true,false);
			break;
		case BRIEF:				
			*descriptorExtractor = new cv::BriefDescriptorExtractor(64);
			break;
		case CALONDER:
			*descriptorExtractor = new cv::CalonderDescriptorExtractor<float>("current.rtc");
			*hamming=false;				
			break;
		case SURF:
			*descriptorExtractor = new cv::SurfDescriptorExtractor();
			*hamming=false;				
			break;
		case SIFT:
			*descriptorExtractor = new cv::SiftDescriptorExtractor();
			*hamming=false;
			break;
//		default:
//			descriptorExtractor = cv::DescriptorExtractor::create( argv[4] );
//			break;
	}
	
	if (descriptorExtractor->empty()){
		std::cout << "Descriptor not assigned. Check spelling!" << std::endl;
		exit(4);
	}
	
	if(*hamming){
		*descriptorMatcher = new cv::BruteForceMatcher<cv::HammingSse>();
	}
	else{
		*descriptorMatcher = new cv::BruteForceMatcher<cv::L2<float> >();
	}
	
}