#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Int32MultiArray.h"

using namespace cv;

//Parameters
//============================================
//============================================
#define SAMPLE_COEF_EXP		2
#define PREPROCESS		1

//============================================
//============================================

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
static const char WINDOWRAW[] = "Image 1";
 
image_transport::Publisher pub;
cv_bridge::CvImagePtr oldImage=NULL;
 

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    	cv_bridge::CvImagePtr cv_ptr;
	
    	try
    	{
        	cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    	}
    	catch (cv_bridge::Exception& e)
    	{
        	//if there is an error during conversion, display it
        	ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        	return;
    	}

	if(oldImage!=NULL)
	{
		//Simulate dvs
		cv_bridge::CvImagePtr difference;
		cv::absdiff(oldImage->image, cv_ptr->image, difference->image);	
		pub.publish(difference->toImageMsg());
	}
	oldImage=cv_ptr
}


int main(int argc, char **argv)
{
   
    ros::init(argc, argv, "DVS");

    ros::NodeHandle nh;

    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);

    cv::namedWindow(WINDOWRAW, CV_WINDOW_AUTOSIZE);


    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1,   imageCallback);


    cv::destroyWindow(WINDOWRAW);

    pub = it.advertise("camera/dvs_image", 100);


    ros::spin();
 
}

