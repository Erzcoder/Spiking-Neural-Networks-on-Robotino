#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Int32MultiArray.h"

//=========================================================================================================
//This node is the main preprocessing node which results can later be used as rate coded input to the S.N.N
//=========================================================================================================

//Parameters
//============================================
//============================================

#define SAMPLE_COEF_EXP		2
#define DEVELOP_MODE		1
#define ALPHA			2.2
#define BETA			0


//============================================
//============================================

//ROI params
//============================================
//============================================

int WIDTH_START_NOM 	=1;
int WIDTH_START_DENOM 	=6;

int HEIGHT_START_NOM 	=1;
int HEIGHT_START_DENOM 	=2;

int WIDTH_NOM 			=2;
int WIDTH_DENOM 		=3;

int HEIGHT_NOM 			=1;
int HEIGHT_DENOM 		=2;
//============================================
//============================================

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";
static const char WINDOWRAW[] = "Image raw";
static const char WINDOWTEST[] = "Image test";
 
//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
 
//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
	
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("Image preprocessing node::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    

 

//////////////////// Do the image processing ///////////////////////////
	
	int threshold_value=50;
	int max_binary_value=255;
	int threshold_type=0;

	cv::Size s = cv_ptr->image.size();
	int height = s.height;
	int width  = s.width;
	

    cv::imshow(WINDOWRAW, cv_ptr->image);

	cv::Rect myROI((width*WIDTH_START_NOM)/WIDTH_START_DENOM, (height*HEIGHT_START_NOM)/HEIGHT_START_DENOM, (width*WIDTH_NOM)/WIDTH_DENOM, (height*HEIGHT_NOM)/HEIGHT_DENOM);

	// Crop the full image to that image contained by the rectangle myROI
	cv_ptr->image=cv_ptr->image(myROI);
	// subsampling the image by a coefiecient of pow(2,SAMPLE_COEF_EXP)
	for(int sample=0; sample<SAMPLE_COEF_EXP; sample++)
	{
	    pyrDown( cv_ptr->image, cv_ptr->image, cv::Size( cv_ptr->image.cols/2, cv_ptr->image.rows/2 ) );
	}
	// smoothe the image to reduce noise
	cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2GRAY);

	//GaussianBlur(processedImage, processedImage,cv::Size(7,7),0,0);
	// transform the image to gray scale

	//image histogram equalisation
	equalizeHist(cv_ptr->image,cv_ptr->image);
	// threshold the image (could alternatively use adaptivThreshold() ) 
	threshold(cv_ptr->image,cv_ptr->image,threshold_value,max_binary_value,threshold_type);

//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
 
    //Display the image using OpenCV
    cv::imshow(WINDOW, cv_ptr->image);
    

    if(DEVELOP_MODE)
   	cv::imshow(WINDOWTEST, cv_ptr->image);
     
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);
    /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor in main().
    */
    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.

    pub.publish(cv_ptr->toImageMsg());
}

void paramsCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    WIDTH_START_NOM 	=msg->data[0];
	WIDTH_START_DENOM 	=msg->data[1];

	HEIGHT_START_NOM 	=msg->data[2];
	HEIGHT_START_DENOM 	=msg->data[3];

	WIDTH_NOM 			=msg->data[4];
	WIDTH_DENOM 		=msg->data[5];

	HEIGHT_NOM 			=msg->data[6];
	HEIGHT_DENOM 		=msg->data[7];
}
 
int main(int argc, char **argv)
{
    ROS_DEBUG("in main");
   
    ros::init(argc, argv, "image_processor");

    ros::NodeHandle nh;

    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);

    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOWRAW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOWTEST, CV_WINDOW_AUTOSIZE);

    ros::NodeHandle nparams;
    ros::Subscriber paramsSub = nparams.subscribe("ROI_params", 1000, paramsCallback);

    //subscribing to the camera image topic
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1,   imageCallback);

    //OpenCV HighGUI call to destroy a display window on shut-down.
    cv::destroyWindow(WINDOW);
    cv::destroyWindow(WINDOWRAW);

    
    
    pub = it.advertise("camera/image_processed", 1);

    ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("image preprocessing node::main.cpp::No error.");
 
}