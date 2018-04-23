#ifndef _Overlap_h
#define _Overlap_h

#include "border.h"	
#include <fstream>
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <algorithm>    // std::min

using namespace cv;
class Overlap
{
	public: 
		Overlap();
		cv::Mat* getImage();
		void Initialise();
        void Start();
        void Stop();
		void Finalise();
        void set_images(cv::Mat&, cv::Mat&);
		
	private:
        cv::Mat img;  //first image Under Process: 
        cv::Mat img2; //second Image Under Process: 
		cv::Matx33f find_transform(bool if_draw_feature_matches=false, bool if_write_to_disk=false, std::string filename="match");
		bool is_transform_good();
		void plot_border();

		border<int> v4c_pic;    			 // four corners of picture:
		border<int> v4c_pic_H;			     // transformed corners of a picture (using feature matching):
			
		//~ Ptr<FeatureDetector> detector = ORB::create();
		//~ Ptr<DescriptorExtractor> extractor = ORB::create();
        cv::OrbFeatureDetector detector;
        cv::OrbDescriptorExtractor extractor;
		
		std::vector<cv::KeyPoint> img1_keypoints, img2_keypoints;    // Output keypoints from the feature detection algorithms
		cv::Mat img1_descriptors, img2_descriptors;				     // Output descriptors from feature detection 
		bool if_draw_feature_matches;								 // flag to draw matches between current and previous images  	
		cv::Mat img_matches;										 // container matrix for the image in which the matching between two images are depicted
		
		cv::Matx33f H;												 // The Homography transform matrix 
		
	   	cv::Scalar blue=cv::Scalar(255,0,0);
		cv::Scalar red=cv::Scalar(0,0,255);
};

#endif //_Overlap_h
