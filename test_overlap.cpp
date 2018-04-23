#include "Overlap.h"
int main(int argc, const char *argv[])
{
    cv::Mat img1= cv::imread("./data/Img_00231.jpg");
    cv::Mat img2= cv::imread("./data/Img_00235.jpg");
    cv::imshow("image 1", img1);
    cv::imshow("image 2", img2);
    cv::waitKey();

    Overlap calc_overlap;
    calc_overlap.set_images(img1, img2);
    calc_overlap.Initialise();
    calc_overlap.Start();
    cv::imshow("image 1", img1);
    cv::imshow("image 2", img2);
    cv::waitKey();

}
