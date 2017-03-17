//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Int32MultiArray.h"

//Parameters
//============================================
//============================================
#define SAMPLE_COEF_EXP		3
#define PREPROCESS		1

//============================================
//============================================

//ROI params
//============================================
//============================================

int WIDTH_START_NOM     =1;
int WIDTH_START_DENOM   =6;

int HEIGHT_START_NOM    =1;
int HEIGHT_START_DENOM  =2;

int WIDTH_NOM           =2;
int WIDTH_DENOM         =3;

int HEIGHT_NOM          =1;
int HEIGHT_DENOM        =2;
//============================================
//============================================

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOWRAW[] = "Image raw";

image_transport::Publisher pub;;

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
        	ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        	return;
    	}


//////////////////// Do the image processing ///////////////////////////

	int threshold_value=50;
	int max_binary_value=1;
	int threshold_type=0;


	//cv::Mat rimage=cv_ptr->image;
	// transform the image to gray scale
	cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2GRAY);

	if(PREPROCESS){
    	cv::Size s = cv_ptr->image.size();
    	int height = s.height;
    	int width  = s.width;

    	cv::Rect myROI((width*WIDTH_START_NOM)/WIDTH_START_DENOM, (height*HEIGHT_START_NOM)/HEIGHT_START_DENOM, (width*WIDTH_NOM)/WIDTH_DENOM, (height*HEIGHT_NOM)/HEIGHT_DENOM);

    	// Crop the full image to that image contained by the rectangle myROI
    	cv_ptr->image=cv_ptr->image(myROI);
    	// subsampling the image by a coefiecient of pow(2,SAMPLE_COEF_EXP)
    	for(int sample=0; sample<SAMPLE_COEF_EXP; sample++)
    	{
    	    pyrDown( cv_ptr->image, cv_ptr->image, cv::Size( cv_ptr->image.cols/2, cv_ptr->image.rows/2 ) );
    	}
	}

	// threshold the image (could alternatively use adaptivThreshold() )
	threshold(cv_ptr->image,cv_ptr->image,threshold_value,max_binary_value,threshold_type);

	cv::imshow(WINDOWRAW, cv_ptr->image);
    cv::waitKey(10);

	//Calculate column summation
	//int imgStepSize = rimage->widthStep;
	//uchar* data = (uchar*)rimage->imageData;
	/*td_msgs::Int32MultiArray csumVector;
	uint tmpSumVector[rimage.cols];
	memset(tmpSumVector, 0, sizeof(uchar) * rimage.cols);
	for (int col = 0; col < rimage.cols; col++) {
  		for (int row = 0; row < rimage.rows; row++) {
			ROS_INFO("i=%d, j=%d value=%d",row,col,rimage.at<int>(row,col));
    			tmpSumVector[col] += rimage.at<int>(row,col);
  		}
		csumVector.data.push_back(tmpSumVector[col]);
	}*/

    	//Display the image using OpenCV in this case just to check that we are getting input
   	// cv::imshow(WINDOW, processedImage);
    	cv::reduce(cv_ptr->image, cv_ptr->image, 0, CV_REDUCE_SUM, CV_32S);


    	//Add some delay in miliseconds.
    	//cv::waitKey(3);

    	//Convert the CvImage to a ROS image message and publish it on the "camera/image_comcessed" topic.
        pub.publish(cv_ptr->toImageMsg());
}

void paramsCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    WIDTH_START_NOM     =msg->data[0];
    WIDTH_START_DENOM   =msg->data[1];

    HEIGHT_START_NOM    =msg->data[2];
    HEIGHT_START_DENOM  =msg->data[3];

    WIDTH_NOM           =msg->data[4];
    WIDTH_DENOM         =msg->data[5];

    HEIGHT_NOM          =msg->data[6];
    HEIGHT_DENOM        =msg->data[7];
}


int main(int argc, char **argv)
{
    ROS_DEBUG("in main");

    ros::init(argc, argv, "image_processor");
    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);

    cv::namedWindow(WINDOWRAW, CV_WINDOW_AUTOSIZE);

    ros::NodeHandle nparams;
    ros::Subscriber paramsSub = nparams.subscribe("ROI_params", 1000, paramsCallback);


    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1,   imageCallback);


    cv::destroyWindow(WINDOWRAW);

    pub = it.advertise("camera/image_compressed", 100);


    //pub = it.advertise("camera/image_processed", 1);

    /**
    * In this application all user callbacks will be called from within the ros::spin() call.
    * ros::spin() will not return until the node has been shutdown, either through a call
    * to ros::shutdown() or a Ctrl-C.
    */
    ros::spin();

}
