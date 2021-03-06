#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

//Parameters
//============================================
//============================================

//============================================
//============================================

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
static const char WINDOW[] = "Image 1";
 
image_transport::Publisher pub;
cv_bridge::CvImagePtr oldImage(new cv_bridge::CvImage);
bool firsttime=true;
 

void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    	cv_bridge::CvImagePtr cv_ptr;
	ROS_ERROR("In");
	
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

	if(!firsttime)
	{
		//Simulate dvs
		//Mat old=oldImage->image;
		//Mat newi=cv_ptr->image;
		//Mat delta;
		cv_bridge::CvImagePtr difference(new cv_bridge::CvImage);
		absdiff(oldImage->image, cv_ptr->image, difference->image);	
		//difference->image=delta;
		imshow(WINDOW, difference->image);
		cv::waitKey(3);
		Mat grayDelta;
		cvtColor(difference->image, grayDelta, CV_RGB2GRAY);
		if (countNonZero(grayDelta) >0) 
			pub.publish(difference->toImageMsg());
	}
	else
	{
		ROS_ERROR("else");
		firsttime=false;
	}
	oldImage->image=cv_ptr->image;
}


int main(int argc, char **argv)
{
   
    ros::init(argc, argv, "DVS");

    ros::NodeHandle nh;

    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);

    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);


    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1,   imageCallback);


    cv::destroyWindow(WINDOW);

    pub = it.advertise("DVS/image", 100);


    ros::spin();
 
}

