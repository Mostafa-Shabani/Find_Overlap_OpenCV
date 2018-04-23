#include "Overlap.h"
#include <opencv2/features2d/features2d.hpp>


cv::Mat* Overlap::getImage() {return &img;}

void Overlap::set_images(cv::Mat& _img1, cv::Mat& _img2)
{
    img =_img1;
    img2=_img2;
}


Overlap::Overlap()
{}
void Overlap::Initialise() 
{
    // check if images are already set and non empty
    try
    {
        if(img.empty())
            {
                std::cout<<"Image 1 is empty"<<std::endl;
                throw;
            }
         if(img2.empty())
            {
                std::cout<<"Image 2 is empty"<<std::endl;
                throw;
            }   
            
    }
    catch (...)
    {
        std::cout<<"Images not initialised or they are empty"<<std::endl;
        return;    
    }
    
    v4c_pic.SetPolygonToImgBorder(img.size());
    
    detector.detect(img, img1_keypoints);
    extractor.compute(img, img1_keypoints, img1_descriptors);
    
    detector.detect(img2, img2_keypoints);
    extractor.compute(img2, img2_keypoints, img2_descriptors);
    
    
}
void Overlap::Finalise()
{}


void Overlap::Stop()
{}

void Overlap::Start()
{


		 
		 // perform feature detection to find a transform between the two images: 
		 try
		 {
			 H=find_transform(true, true, "./data/matches.jpg");
		     v4c_pic_H=v4c_pic.transform(H);		 
			 if(cv::Mat(H).total()== 0 || cv::Mat(H).data == NULL) 
				{
					throw;
				}
		 }
		 catch(...)
			{
                std::cout<<"No Transform was found"<<std::endl;
			}
	
	plot_border();

}

void Overlap::plot_border()
{		
	
    v4c_pic_H.plot_polygon(img2, red);
		
}


 
cv::Matx33f Overlap::find_transform(bool if_draw_feature_matches, bool if_write_to_disk, std::string filename)
{
		cv::BFMatcher matcher;
		std::vector<cv::DMatch> matches;
		unsigned int N_top_matches=50;				// number of top matches that would be used to calculate the homography transform
		std::vector<cv::DMatch> Selected_matches;	// top matches that would be used to calculate the homography transform
		std::vector<cv::Point2f> img1_points, img2_points;		 // 2D points in the pictures corresponding to the location of the extracted (and sorted and filteres) keypoints	 
																 // it is used to find the homography transform of the pairs of images
		std::vector<cv::KeyPoint> img1_keypoints_s, img2_keypoints_s;// sorted (and filtered keypoints)
		
 			    
		matches.clear();
		matcher.match(img2_descriptors, img1_descriptors, matches);
    	std::cout<<"Size of matches: "<<matches.size()<<std::endl;

		// sort matches based on distance:
		std::sort(matches.begin(), matches.end(), [](cv::DMatch a, cv::DMatch b){ return a.distance< b.distance;});
			
		// extract the first N_top_matches matches
		Selected_matches.clear();
		for( size_t count=0; count<std::min((int) N_top_matches, (int) matches.size()); ++count) Selected_matches.push_back(matches[count]);
			

		//~ img2_points.clear();
		//~ img1_points.clear();
		
		//~ img1_keypoints_s.clear();
		//~ img2_keypoints_s.clear();
		
		for (unsigned int i=0; i< Selected_matches.size(); i++)
			{
				  //~ std::cout<<"Size of img2_keypoints "<<img2_keypoints.size()<<std::endl;
				  //~ std::cout<<"Size of Selected_matches "<<Selected_matches.size()<<std::endl;
  				  //~ std::cout<<"i="<<i<<std::endl;

				  //~ std::cout<<" calling index number "<< Selected_matches[i].queryIdx <<std::endl;
				  //~ std::cout<<img2_keypoints[Selected_matches[i].queryIdx].pt.x<<" "<<img2_keypoints[Selected_matches[i].queryIdx].pt.y<<std::endl;
				  
				  img2_points.push_back(img2_keypoints[Selected_matches[i].queryIdx].pt);
				  img1_points.push_back(img1_keypoints[Selected_matches[i].trainIdx].pt);
				  
				  img2_keypoints_s.push_back(img2_keypoints[Selected_matches[i].queryIdx]);
				  Selected_matches[i].queryIdx=i;
			  	  img1_keypoints_s.push_back(img1_keypoints[Selected_matches[i].trainIdx]);
				  Selected_matches[i].trainIdx=i;
				 //cout<< Selected_matches[i].distance <<endl;
				 
				  
			}
		// draw features in a seperate window
			if(if_draw_feature_matches)
			{
				cv::Scalar white(255,255,255);
				cv::Scalar black(0,0,0);
				cv::Mat img_matches;
				
				cv::drawMatches(img, img1_keypoints_s, img2, img2_keypoints_s, Selected_matches, img_matches, black, white, std::vector<char> (), 0);//cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);	
				
				cv::namedWindow("img_matches", cv::WINDOW_NORMAL);
				cv::imshow("img_matches",img_matches);
				//cv::resizeWindow("img_matches", 600, 400);
				if(if_write_to_disk)
				{
					std::cout << "writing to disk:" << filename <<std::endl;
					cv::imwrite(filename,img_matches);
					//cv::waitKey();
				}
			}
			  
		  //calculate the transform from prev image to current image
		  //~ std::cout<<"Size of img1_points "<<img1_points.size()<<std::endl;
  		  //~ std::cout<<"Size of img2_points "<<img2_points.size()<<std::endl;

		  return(cv::findHomography (img1_points, img2_points,  CV_RANSAC));
}


bool Overlap::is_transform_good()
{
	unsigned int Np(6), Rtest=(unsigned int) std::min(img.size().width/4, img.size().height/4);
	border<int> Test_Circ, Trans_Test_Circ;
	cv::Point2i ImgCentre((int) img.size().width/2, (int) img.size().height/2);
	cv::Point2i Trans_ImgCentre;
	
	
	 //Does the tranform maintains the order of N points on a circle with Radius Rtest?
		
		// create a "border" object : a circle made of Np points
		
		Test_Circ.SetPolygonToCircle(ImgCentre, Rtest, Np);
		// transform the circle and the centre point using H
		Trans_Test_Circ=Test_Circ.transform(H);
		
		cv::Vec3f p = H * cv::Vec3f(ImgCentre.x, ImgCentre.y, 1);
		Trans_ImgCentre=cv::Point2i((int)(p[0] / p[2]), (int)(p[1] / p[2]) );
		
		// check if the transformed polygon has still the same order of vertices
		bool if_monotonic=Trans_Test_Circ.is_monotonic_angles( Trans_ImgCentre );
		return(if_monotonic);
	
}

