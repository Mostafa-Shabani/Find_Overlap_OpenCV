#ifndef _border_h
#define _border_h
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>       /* cos */
#include <opencv2/features2d/features2d.hpp>
/**
 * @file  border.h
 * 
 * @brief a class for image borders used for stitching aerial images
 * 
 * 
 */
template <class T> 
class border
{
private:
	std::vector<cv::Point_<T> > polygon;

public:
	border();
	border(cv::Size);
	border(const std::vector< cv::Point_<T> >& );
	border<T> transform(const cv::Matx33f&); 
	void plot_polygon(cv::Mat&, const cv::Scalar&);
	void DefinePolygon(const std::vector< cv::Point_<T> >&);
	void SetPolygonToImgBorder(cv::Size);
	void SetPolygonToCircle(const cv::Point_<T>&, const T&, const unsigned int&);
	bool is_monotonic_angles( const cv::Point_<T>& );
	void filter_keypoints(std::vector<cv::KeyPoint>& );

};
/**
 * @brief Default constructor that assigns the border of a 600x400 image as the polygon  
 * 	
 */
template <class T>
border<T>::border()
{	
	polygon.clear();
	polygon.push_back(cv::Point_<T>(0.0, 0.0));
	polygon.push_back(cv::Point_<T>(599, 0.0));
	polygon.push_back(cv::Point_<T>(599,399));
	polygon.push_back(cv::Point_<T>(0.0,399));
}

/**
 * @brief Constructing a border object by assigning the outer border of an image as the polygon  
 * 	
 * @param imSize a cv::Size object that represents the size of the image under study
 *  
 * 
  */
template <class T>
border<T>::border(cv::Size imSize)
{
	polygon.clear();
	polygon.push_back(cv::Point_<T>(0.0,             0.0));
	polygon.push_back(cv::Point_<T>(imSize.width-1,0.0));
	polygon.push_back(cv::Point_<T>(imSize.width-1,imSize.height-1));
	polygon.push_back(cv::Point_<T>(0.0,             imSize.height-1));
}

/**
 * @brief Constructing a border object by passing a vector of cv::Points to initialise the polygon object  
 * 	
 * @param b a vector of cv::Points representing the vertices of a polygon
 *  
 * 
  */
template <class T>
border<T>::border(const std::vector< cv::Point_<T> >& b) : polygon(b)
{
}

template <class T>
void border<T>::DefinePolygon(const std::vector< cv::Point_<T> >& b)
{
	polygon=b;
}

/**
 * @brief assigns the outer border of an image as the polygon  
 * 	
 * @param imSize a cv::Size object that represents the size of the image under study
 *  
 * 
  */
template <class T>
void border<T>::SetPolygonToImgBorder(cv::Size imSize)
{
	polygon.clear();
	polygon.push_back(cv::Point_<T>(0.0,           0.0));
	polygon.push_back(cv::Point_<T>(imSize.width-1,0.0));
	polygon.push_back(cv::Point_<T>(imSize.width-1,imSize.height-1));
	polygon.push_back(cv::Point_<T>(0.0,           imSize.height-1));
}





/**
 * @brief transforms a polygon according to the input matrix 
 * 	
 * @param H could be a transform found by feature matching between two consecutive images. 
 *  
 * @return a border object that contains the result of tranforming the polygon
 * 
  */
template <class T>
border<T> border<T>::transform(const cv::Matx33f& H)
{	
	std::vector< cv::Point_<T> > temp;
	for_each(begin(polygon), end(polygon), [&](cv::Point_<T> P)
	{
		cv::Vec3d p = H * cv::Vec3f(P.x, P.y, 1);
		
		temp.push_back(cv::Point_<T>((T)(p[0] / p[2]), (T)(p[1] / p[2]) ));
		
		//~ cout<<P<<" is transformed to "<<cv::Point(p[0] / p[2], p[1] / p[2])<<endl;
		//~ cout<< " *********************** " <<endl;
	});
	
	border<T> transformed_border(temp) ;
	return(transformed_border);
	
}

/**
 * @brief plots the polygon as an overlay on the given cv::Mat object using the input color 
 * 	
 * @param img the cv::Mat container for the image on which the polygon is overlayed on
 * @param color the cv::Scalar object used as a color for plotting the polygon  
 *  
 * 
  */
template <class T>
void border<T>::plot_polygon(cv::Mat& img, const cv::Scalar& color)
{	
	size_t L=polygon.size();
	for (size_t i = 0; i < L; ++i)
			{
				cv::Point2f P = polygon[1];
				//if(P.x>=0 && P.x<(img.size().width-1) && P.y>=0 && P.y<(img.size().height-1)) 
				//{
					cv::line(img, P, polygon[0], color);
					cv::line(img, P, polygon[2], color);
				//}
				
				std::rotate( begin(polygon),next(begin(polygon)) , end(polygon));
				
			}
}

/**
 * @brief assigns points on circumference of a circle as the polygon  
 * 	
 * @param Centre centre point of the circle
 * @param Radius radius of the circle
 * @param Npoints Number of points on the circle's circumference  
 *  
 * 
  */
template <class T>
void border<T>::SetPolygonToCircle(const cv::Point_<T>& Centre , const T& Radius, const unsigned int& Npoints)
{
	#define PI 3.14159256
	double alpha;
	alpha=2*PI/Npoints;
	polygon.clear();
	for(unsigned int n=0; n<Npoints; n++)
	{
		T Px=Centre.x + (T) Radius * cos(n*alpha);
		T Py=Centre.y + (T) Radius * sin(n*alpha);
		polygon.push_back(cv::Point_<T>(Px, Py));
	}
	#undef PI
}

/**
 * @brief checks if the angles between the input point and the vertices of the polygon are monotonically increasing/decreasing  
 * 	
 * This function is helpful in determining if a transformation has maintained the order of the vertices  
 * 	
 * @param P0 The point to be tested (usually a point in the middle of the expected polygon)
 *  
 * 
  */
template <class T>
bool border<T>::is_monotonic_angles( const cv::Point_<T>& P0 )
{
	#define PI 3.14159256
	if(polygon.empty()) return(true);
	if(polygon.size() == 1) return(true);
	std::vector<cv::Point_<T>> Vecs_from_P0, Vecs_from_P0_rotated;
	std::vector<double> cross_product;
	Vecs_from_P0.resize(polygon.size());
	cross_product.resize(polygon.size(),0); 
	std::transform (begin(polygon), end(polygon), begin(Vecs_from_P0), [&] (cv::Point_<T> P) -> cv::Point_<T> {return(P-P0);});
	Vecs_from_P0_rotated=Vecs_from_P0;
	std::rotate(begin(Vecs_from_P0_rotated),begin(Vecs_from_P0_rotated)+1,end(Vecs_from_P0_rotated));
	
	std::transform (begin(Vecs_from_P0), end(Vecs_from_P0), begin(Vecs_from_P0_rotated),begin(cross_product), [] 
	(cv::Point_<T> V1,cv::Point_<T> V2) -> double {return( V1.cross(V2));});
	
	if     (all_of(begin(cross_product), end(cross_product),[](double a){return a>0;} )) return(true);
	else if(all_of(begin(cross_product), end(cross_product),[](double a){return a<0;} )) return(true);	
	else return(false);
	#undef PI
}


/**
 * @brief removes points inside a polygon from a list of input points  
 * 	
 * This function is useful for removing previously counted objects, when a the borders of the previous image is found inside the current image
 * 
 * @param keypoints a vector of keypoints (usually some detected objects in the image) 
 * 
 *  
 * 
  */
template <class T>
void border<T>::filter_keypoints(std::vector<cv::KeyPoint>& keypoints)
{
	keypoints.erase(std::remove_if(begin(keypoints), end(keypoints), [&](cv::KeyPoint Kt)
	{	
		bool is_pt_in_polygon= cv::pointPolygonTest(polygon, Kt.pt, false) >0;
		return(is_pt_in_polygon);
	}), end(keypoints));
}


#endif

