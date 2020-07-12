
#include <stdio.h>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
// includes for ros opencv
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
//end
#include <sstream>

#include <sensor_msgs/Image.h>
#include "buoy_identifier/Buoy.h"

//begin functions from opencv template ROS 2016
#define inDataSize 256
#include <iostream>
#define MAXIMUM_TEXT_SIZE 64u
#include <math.h>       /* atan2 */
#define input "video"
#define show false

#define PI 3.1415

#define minPix 20 //minimum pixel size
#define minDist 15 //distance in inches to stop/go
int counter = 0;
int FOV = 42; //This is half the field of view of the camera, so 45 degrees on either side of the camera is visible.
int xres = 1280; //Resolution of the camera
float reddist, greendist;
//struct to hold HSV values; makes coding cleaner and easier
struct colorStruct
{
    std::string name;
    const char *tempName; //name holders
    int hLow;
    int hHigh;
    int sLow;
    int sHigh;
    int vLow;
    int vHigh;//color values
    cv::Scalar color;//color to display for this entity
    std::vector<cv::Point> contour; // Vector for storing contour if found
    int xEstimate = 0;
    int yEstimate = 0;

    float angle;
    float distance; //why not store it here; note: if error initialize under constructor
    //constructor with given formal parameters; trust user to use this
    colorStruct(std::string Name, int Hlow, int Hhigh, int Slow, int Shigh, int Vlow, int Vhigh, cv::Scalar Color)
    {
        name = Name;
        hLow = Hlow;
        hHigh = Hhigh;
        sLow = Slow;
        sHigh = Shigh;
        vLow = Vlow;
        vHigh = Vhigh;
        color = Color;
        distance = -1.0;
    }
    //function to create a trackbar to adjust HSV
    void createTrackBar()
    {
        tempName = (name + " Trackbar").c_str();
        cvNamedWindow(tempName);
        //cvResizeWindow  (tempName, 350, 350);
        cvCreateTrackbar("H-Low", tempName, &hLow, 255, NULL);//these are the trackbars...
        cvCreateTrackbar("H-High", tempName, &hHigh, 255, NULL);
        cvCreateTrackbar("S-Low", tempName, &sLow, 255, NULL);
        cvCreateTrackbar("S-High", tempName, &sHigh, 255, NULL);
        cvCreateTrackbar("V-Low", tempName, &vLow, 255, NULL);
        cvCreateTrackbar("V-High", tempName, &vHigh, 255, NULL);
    }
};

//function to thresh image using HSV information from struct
//returns threshed image
cv::Mat threshHSV(cv::Mat img, colorStruct colorHSV)
{
    cv::Mat imgThreshedHSV;

    //  std::cerr << img << std::endl;

    cvtColor(img, imgThreshedHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    //inputImage,scalar(hlow,slow,vlow),scalar(hhigh,shigh,vhigh),outputImage
    inRange(imgThreshedHSV, cv::Scalar(colorHSV.hLow, colorHSV.sLow, colorHSV.vLow),
            cv::Scalar(colorHSV.hHigh, colorHSV.sHigh, colorHSV.vHigh), imgThreshedHSV); //Threshold the image in range
    //morphological opening (remove small objects from the foreground)
    erode(imgThreshedHSV, imgThreshedHSV, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));//<-kernel size
    dilate(imgThreshedHSV, imgThreshedHSV, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    //morphological closing (fill small holes in the foreground)
    dilate(imgThreshedHSV, imgThreshedHSV, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    erode(imgThreshedHSV, imgThreshedHSV, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    return imgThreshedHSV;
}

//find the max contour of the given color
std::vector<cv::Point> findMaxContour(cv::Mat img, cv::Mat imgThreshed, colorStruct &light)
{
    std::vector< std::vector<cv::Point> > contours; // Vector for storing contour
    std::vector<cv::Point> blank;
    std::vector<cv::Vec4i> hierarchy;
    int largest_area = 0;
    int largest_contour_index = -1;
    cv::Rect bounding_rect;
    // Find the contours in the image
    cv::findContours(imgThreshed, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)  // iterate through each contour.
    {
        double a = contourArea(contours[i], false);  //  Find the area of contour
        if (a > largest_area)
        {
            largest_area = a;
            largest_contour_index = i;                //Store the index of largest contour
        }
    }
    if (largest_contour_index > -1)
        return contours[largest_contour_index];//return largest contour
    else
        return blank;
}

//find block height/width or distance  //////////////////////////////Likely unnecessary
float findDist(cv::Mat &img, colorStruct &light, int centerx, int centery)
{
    //will be finding distance using zed depthmap
    cv::Mat lookUpX = cv::imread("/home/catkin_ws_prog/src/buoy_identifier/xEst.bmp");
    cv::Mat lookUpY = cv::imread("/home/catkin_ws_prog/src/buoy_identifier/yEst.bmp");

    float distance = -1.0; //use as control
    //get rectangle box in order to find width and height regardless of angle
    cv::RotatedRect box = minAreaRect(light.contour);
    cv::Point2f vtx[4];
    box.points(vtx);
    //width
    float x = sqrt(pow((vtx[1].x - vtx[0].x), 2) + pow((vtx[1].y - vtx[0].y), 2));
    //height
    float y = sqrt(pow((vtx[2].x - vtx[1].x), 2) + pow((vtx[2].y - vtx[1].y), 2));

    if(1)
    {
        cv::Point2f center;
        float radius = 0;
        cv::minEnclosingCircle(light.contour, center, radius);
        //find distance
        // distKnown/radiusKnown=distance/radiusFound
        //->distance = radiusFound*distKnown/radiusKnown = radiusFound*ratioKnown

        //***************************************
        distance = 2800 * pow(radius, -1.027); //OLD ONE

        centerx = center.x;
        centery = center.y;
        cv::circle( img, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0 );

        //draw circle around contour
        cv::circle(img, center, cvRound(radius), light.color, 1, CV_AA);

        int d = int(radius * 2);
        int w = int(abs(center.x - 960));
        if(d >= 350)
        {
            d = 349;
        }
        if(w >= 920)
        {
            w = 919;
        }

        std::cerr << "lookUplookUpX=" << lookUpX << w << std::endl;
        std::cerr << "d=" << d << "  w=" << w << std::endl;

        int estimateX = int(lookUpX.at<uchar>(d, w, 0));
        int estimateY = int(lookUpY.at<uchar>(d, w, 0));

        if ((center.x - 960) < 0)
        {
            light.xEstimate = -estimateX;
        }
        else
        {
            light.xEstimate = estimateX;
        }
        light.yEstimate = estimateY;
        distance = sqrt(estimateX * estimateX + estimateY * estimateY);


        //create string for output to image
        char text_array[MAXIMUM_TEXT_SIZE];
        snprintf(text_array, MAXIMUM_TEXT_SIZE, "%4.2f", estimateY);
        std::string temp(text_array);

        snprintf(text_array, MAXIMUM_TEXT_SIZE, "%4.2f", estimateX);
        std::string temp2(text_array);


    }

    if ((x + y) / 4 > minPix)
        return distance;
    else
        return -1;
}

//find a sized contour; used for config
bool findSizedContour(cv::Mat &img, cv::Mat imgThreshed, colorStruct light)
{
    std::vector< std::vector<cv::Point> > contours; // Vector for storing contour
    std::vector<cv::Vec4i> hierarchy;
    int largest_area = 0;
    int largest_contour_index = 0;
    cv::Rect rect;
    bool found = false;
    cv::findContours(imgThreshed, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image
    for (int i = 0; i < contours.size(); i++)  // iterate through each contour.
    {
        rect = boundingRect(contours[i]);
        if (((rect.height + rect.width) / 4) > minPix)
        {
            cv::rectangle(img, rect, light.color, 2, 8, 0);
            int sizeofcontour = (rect.width + rect.height) / 2;
            if (show)
                std::cout << light.name << " size=" << sizeofcontour << std::endl;
            found = true;
        }
    }
    return found;
}
int redx;
int redy;
double redangle;
int greenx;
int greeny;
double greenangle;
cv::Mat threshedImg;
//function to find and classify lights and return 'w' or 's'
std::string classifyStopLight(cv::Mat img, std::vector<colorStruct> &lights, int reedx, int reedy, double reedangle, int greeenx, int greeeny, double greeenangle, int width, int height)
{



    std::string botCmd;
    //std::cout<<"instantiated runtime variables threshedimg and botcmd"<<std::endl;
    int xloc, yloc;
    for (int i = 0; i < lights.size(); i++)
    {
        double angleEstimate = 0;
        threshedImg = threshHSV(img, lights[i]);
        //std::cout<<"threshed the image"<<std::endl;
        //std::cerr << "iteration number=" << i << std::endl;
        if (show)
            imshow("Thresholded Image " + lights[i].name, threshedImg); //show the thresholded image
        //method contours
        lights[i].contour = findMaxContour(img, threshedImg, lights[i]);
        if (lights[i].contour.size() > 0)
        {
            cv::Point2f center;
            float radius = 0;
            cv::minEnclosingCircle(lights[i].contour, center, radius);
            cv::circle(img, center, cvRound(radius), lights[i].color, 1, CV_AA);
            xloc = center.x;
            yloc = center.y;
            //std::cerr << "xloc=" << xloc << "  yloc=" << yloc << std::endl;
            angleEstimate = (double)(xloc - (xres / 2)) / (xres / 2) * FOV;
            //            lights[i].distance = findDist(img, lights[i],xloc,yloc);
            //  cv::imshow("Buoy Detection", img); //show the boxed image

            //////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////
            ///////////////////output
            //      double distanceEstimate =0;

            //  if (lights[i].xEstimate!=0 && lights[i].yEstimate!=0){
            //  distanceEstimate = sqrt(lights[i].xEstimate*lights[i].xEstimate+lights[i].yEstimate*lights[i].yEstimate);
            //   angleEstimate = atan((double)(lights[i].xEstimate)/(double)(lights[i].yEstimate))*180.0/3.14159265;
            if((i == 0) || (i == 2))
            {
                //                    std::cout<<"RED"<<std::endl;
                //std::cout<<"distance  => "<<distanceEstimate<<std::endl;
                //                std::cout<<"angle  => "<<angleEstimate<<std::endl;
                redangle = angleEstimate;
                redx = xloc;
                redy = yloc;
                //std::cerr<<"redx: "<<redx<<" redy: "<<redy <<std::endl;
            }
            else if (i == 1)
            {
                //                    std::cout<<"GREEN"<<std::endl;
                //std::cout<<"distance  => "<<distanceEstimate<<std::endl;
                //                std::cout<<"angle  => "<<angleEstimate<<std::endl;
                greenangle = angleEstimate;
                greenx = xloc;
                greeny = yloc;
                //std::cerr<<"greenx: "<<greenx<<" greeny: "<<greeny <<std::endl;
            }

            //   }
            /////////////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////////////////


        }
    }
    /*
        if ((lights[0].distance>0) && (lights[0].distance<minDist)) {
            counter = counter + 1;

        }
        if (counter > 7)
        {
            counter = 0;
            return "s";
        }

        else if((lights[1].distance>0)&&(lights[1].distance<minDist)){

        }
    */


    return botCmd;
}



//end functions from opencv template ROS 2016

//Stuff for CVbridge ros_msg to mat conversion
static const std::string OPENCV_WINDOW = "Image window";
cv::Mat global_img;//Setup image container
class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    // image_transport::Publisher image_pub_;

public:
    ImageConverter()
        : it_(nh_)
    {
        image_sub_ = it_.subscribe("/zed/zed_node/left/image_rect_color", 1, &ImageConverter::imageCb, this);
        // image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        global_img = cv_ptr->image;



    }
};
//End of ros_msg to mat conversion


void configMode(std::vector< colorStruct> &lights, cv::Mat img)
{
    int key = 0;
    int i = 0;
    bool looped = false;
    cv::Mat tempImg;
    while (key != 27)
    {
        lights[i].createTrackBar();
        while (true)
        {

            cv::imshow("Original Image", img);
            tempImg = threshHSV(img, lights[i]);
            cv::imshow("Threshed " + lights[i].name, tempImg);
            findSizedContour(img, tempImg, lights[i]);
            cv::imshow("Buoy Detection", img);
            key = cv::waitKey(1);
            if (key == 27)   //wait for 'esc' key press . If 'esc' key is pressed, break loop
            {
                std::cout << "Exit config mode." << std::endl;
                cv::destroyAllWindows();
                break;
            }
            else if (key == 110)
            {
                // close the windows
                cv::destroyAllWindows();
                i++;
                if (i >= lights.size())
                {
                    i = 0;
                }
                break;
            }
        }
    }
}


int depthinit = 0;
float *depths;//linear array of points representing the depth cloud
int x, y, depthwidth;
void depthCallback(const sensor_msgs::Image::ConstPtr &msg)
{

    // Get a pointer to the depth values casting the data
    // pointer to floating point
    depths = (float *)(&msg->data[0]);

    depthwidth = msg->width;
    // Image coordinates of the center pixel
    //int u = msg->width / 2;
    //int v = msg->height / 2;

    // Linear index of the pixel
    //   int centerIdx = (msg->width / 2) + msg->width * (msg->height / 2);

    // Output the measure
    //ROS_INFO("Center distance : %g m", depths[(msg->width / 2) + msg->width * (msg->height / 2)]);

    int red_depth_i = redx + (msg->width * redy);
    int green_depth_i = greenx + (msg->width * greeny);
    float red_depth = depths[red_depth_i];
    float green_depth = depths[green_depth_i];

    ROS_INFO("Red distance : %g m", red_depth);
    ROS_INFO("Green distance : %g m", green_depth);
    
    greendist = green_depth;
    reddist = red_depth;

    depthinit = 1;
}

int imgwidth, imgheight;
void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    //ImageConverter ic;
    //std::cout<<"created imageconverter"<<std::endl;
    ROS_INFO("Left Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
    imgwidth = msg->width;
    imgheight = msg->height;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //img=cv_ptr->image;
    cv_ptr->image.copyTo(global_img);
    //std::cout<<"image returned"<<std::endl;
}


int centerIdx;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "buoy_publisher");
    //int redx; int redy; double redangle; int greenx; int greeny; double greenangle;
    buoy_identifier::Buoy redbuoy;
    buoy_identifier::Buoy greenbuoy;
    //Stuff from opencv template

    //create controls for keyboard use in while loop
    int key = 0;

    //  cv::namedWindow("Original Image", CV_WINDOW_AUTOSIZE); //create a window

    //create colorStruct vector to hold HSV colors
    std::vector< colorStruct > lights;

    //Red high level
    lights.push_back(colorStruct ("redhigh", 165, 255, 146, 255, 98, 255, cv::Scalar(0, 0, 255)));


    //Green
    lights.push_back(colorStruct("green", 37, 100, 100, 255, 77, 190, cv::Scalar(0, 255, 0)));
    //Red low level
    //lights.push_back(colorStruct ("redlow",0,10,70,255,50,255,cv::Scalar(0,0,255)));
    //define size of data
    int outDataSize = 1;
    //string to send command to bot
    std::string botCmd;


    //end of stuff from opencv template
    //used to convert ros_msg to mat
    //ImageConverter ic;
    //std::cout<<"created imageconverter"<<std::endl;


    ros::NodeHandle n;
    std::cout << "created nodehandle" << std::endl;

    ros::Subscriber subDepth    = n.subscribe("/zed/zed_node/depth/depth_registered", 10, depthCallback);
    std::cout << "subscribed to depthstream" << std::endl;
    ros::Subscriber subLeftRectified  = n.subscribe("/zed/zed_node/left/image_rect_color", 10,
                                        imageLeftRectifiedCallback);
    std::cout << "subscribed to leftstream" << std::endl;



    ros::Publisher redBuoyPub = n.advertise<buoy_identifier::Buoy>("/redbuoy_publisher", 1000);
    std::cout << "created publisher for redbuoy" << std::endl;
    ros::Publisher greenBuoyPub = n.advertise<buoy_identifier::Buoy>("/greenbuoy_publisher", 1000);
    std::cout << "created publisher for greenbuoy" << std::endl;
    ros::Rate loop_rate(3); //10 messages per second

    //ros::spinOnce();
    //loop_rate.sleep();
    //std::cout<<"printing img: "<<global_img<<std::endl;
    //waiting the img to populate
    int count = 0;
    while (ros::ok())//keeps running loop until user uses ctrl+c
    {



        std::cout << "Loopcount: " << count << std::endl;
        buoy_identifier::Buoy greenmsg;
        //std::cout<<"created greenmsg"<<std::endl;
        buoy_identifier::Buoy redmsg;
        //  std::cout<<"created redmsg"<<std::endl;
        //opencv bridge goes here, need a mat
        if (!global_img.empty() && (!depthinit == 0))
        {

            //imshow("Original Image", img); //show the original image
            botCmd = classifyStopLight(global_img, lights, redx, redy, redangle, greenx, greeny, greenangle, imgwidth, imgheight); //detect and return boxed lights with
            //  std::cout<<"classifiedstoplight :)"<<std::endl;
            //std::cout<<"depthwidth: " <<depthwidth<<std::endl;
            //   centerIdx = redx + depthwidth * redy;
            //std::cout<<"redx: "<<redx<<" redy: "<<redy <<" red centerIdx: "<<centerIdx<<std::endl;

            //  reddist=depths[centerIdx];
            redmsg.color = "red";
            redmsg.distance = reddist;
            //std::cout<<"redmsg distance out of callback: "<<reddist<<std::endl;
            redmsg.angle = redangle;
            
            float redangle_radians = redangle * (PI / 180.0);
            redmsg.rel_vect_len_x = sin(redangle_radians) * reddist;
            redmsg.rel_vect_len_y = cos(redangle_radians) * reddist;

            redBuoyPub.publish(redmsg);
            //std::cout<<"redangle  out of callback: "<<redangle<<std::endl;
            //  centerIdx = greenx + depthwidth * greeny;
            //  greendist=depths[centerIdx];
            greenmsg.color = "green";
            //std::cout<<"greenx: "<<greenx<<" greeny: "<<greeny <<" green centerIdx: "<<centerIdx<<std::endl;
            greenmsg.distance = greendist;
            //std::cout<<"greenmsg distance out of callback: "<<greendist<<std::endl;
            greenmsg.angle = greenangle;

            float greenangle_radians = greenangle * (PI / 180.0);
            greenmsg.rel_vect_len_x = sin(greenangle_radians) * greendist;
            greenmsg.rel_vect_len_y = cos(greenangle_radians) * greendist;

            //std::cout<<"greenangle  out of callback: "<<greenangle<<std::endl;
            greenBuoyPub.publish(greenmsg);

            ROS_INFO("Green x,y : %.3f,%.3f m", greenmsg.rel_vect_len_x, greenmsg.rel_vect_len_y);
            ROS_INFO("Red x,y : %.3f,%.3f m", redmsg.rel_vect_len_x, redmsg.rel_vect_len_y);

            /*

                    key = cv::waitKey(1);
                        if (key == 27) { //wait for 'esc' key press . If 'esc' key is pressed, break loop
                            std::cout << "esc key is pressed by user" << std::endl;
                            break;
                        }
                        else if (key == 99) {
                            std::cout << "Config Mode" << std::endl;
                            configMode(lights, global_img);
                        }*/

        }
        ros::spinOnce();


        loop_rate.sleep();

        ++count;
    }


    return 0;
}
