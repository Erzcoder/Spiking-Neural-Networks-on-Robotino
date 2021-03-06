//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Parameters
//============================================
//============================================

#define SAMPLE_COEF_EXP		2
#define DEVELOP_MODE		1
#define ALPHA			2.2
#define BETA			0



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
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
 

//////////////////// Do the image processing ///////////////////////////
	
	int threshold_value=50;
	int max_binary_value=255;
	int threshold_type=0;

	cv::Size s = cv_ptr->image.size();
	int height = s.height;
	int width  = s.width;
	
	cv::Rect myROI(width/6, height/2, (width*2)/3, height/2);

	// Crop the full image to that image contained by the rectangle myROI
	cv::Mat processedImage=cv_ptr->image(myROI);
	// subsampling the image by a coefiecient of pow(2,SAMPLE_COEF_EXP)
	for(int sample=0; sample<SAMPLE_COEF_EXP; sample++)
	{
	    pyrDown( processedImage, processedImage, cv::Size( processedImage.cols/2, processedImage.rows/2 ) );
	}
	// smoothe the image to reduce noise
	cv::Mat testImage=processedImage;
	cvtColor(testImage, testImage, CV_RGB2GRAY);

	//GaussianBlur(processedImage, processedImage,cv::Size(7,7),0,0);
	// transform the image to gray scale
	cvtColor(processedImage, processedImage, CV_RGB2GRAY);

	//image histogram stretching manual setup of ALPHA AND BETA parameters
	//processedImage.convertTo(processedImage, -1, ALPHA, BETA);
	//image histogram equalisation
	equalizeHist(processedImage,processedImage);
	// threshold the image (could alternatively use adaptivThreshold() ) 
	//threshold(processedImage,processedImage,threshold_value,max_binary_value,threshold_type);
	/*int erosion_size=3;
	int erosion_type=cv::MORPH_RECT;
	cv::Mat element=getStructuringElement(erosion_type,cv::Size(2*erosion_size+1,2*erosion_size+1),cv::Point(erosion_size,erosion_size));
	erode(cv_ptr->image,cv_ptr->image,element);*/
//---------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------
 
    //Display the image using OpenCV
    cv::imshow(WINDOW, processedImage);
    cv::imshow(WINDOWRAW, cv_ptr->image);

    if(DEVELOP_MODE)
   	cv::imshow(WINDOWTEST, testImage);
     
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
 
/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
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
    //OpenCV HighGUI call to create a display window on start-up.
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOWRAW, CV_WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOWTEST, CV_WINDOW_AUTOSIZE);

    //subscribing to the camera image topic
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1,   imageCallback);
    //OpenCV HighGUI call to destroy a display window on shut-down.
    cv::destroyWindow(WINDOW);
    cv::destroyWindow(WINDOWRAW);


    pub = it.advertise("camera/image_processed", 1);
    /**
    * In this application all user callbacks will be called from within the ros::spin() call.
    * ros::spin() will not return until the node has been shutdown, either through a call
    * to ros::shutdown() or a Ctrl-C.
    */
    ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
 
}

