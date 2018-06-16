#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ardrone_autonomy/Navdata.h>
#include <iostream>
#include <stdio.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "std_msgs/UInt16.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>
#include <sstream>
//#include <ardrone_autonomy/navdata_gps.h>

using namespace cv;
using namespace std;
using namespace ros;  

//ardrone_autonomy::navdata_gps gps_in; 
ardrone_autonomy::Navdata nav_in;  

    int Kpx=100;
    int Kdx=200;
    int Kpy=100;
    int Kdy=200;
    float kpx=0;
    float kdx=0;
    float errorx=0;
    float prev_errorx=0;
    float kpy=0;
    float kdy=0;
    float errory=0;
    float prev_errory=0;
    float derivativex=0;
    float derivativey=0;
    float outputx=0;
    float outputy=0;
    float cmdx=0;
    float cmdy=0;
    double finish=0;
    double dt=0;
    double begin=0;
    float m_straight=0;
    float m_updown=0;
    float m_sides=0;
    float gps_long=0;
    float gps_lat=0;
    float targetlat=51.437266;
    float targetlong=-0.249734;
    float batt_state=0;
    int speed_lim=5;
    float speed_limit=0;
	geometry_msgs::Twist twist_msg;
	geometry_msgs::Twist twist_msg_f;
  geometry_msgs::Twist twist_msg_key;
	geometry_msgs::Twist twist_msg_hover;
	geometry_msgs::Twist twist_msg_up;
	std_msgs::Empty emp_msg;
	

  Publisher pub_empty_land;
	Publisher pub_twist;
	Publisher pub_empty_takeoff;
	Publisher pub_empty_reset;


//------------------------KEYBOARD CONTROL CALLBACK FUNCTION---------------------------------//
/*void cmdRCCallback(const geometry_msgs::Twist::ConstPtr& cmd_RC)
    {

    float m_turn=0;
    m_straight=cmd_RC->linear.x;
    m_sides=cmd_RC->linear.y;
    m_updown=cmd_RC->linear.z;
    m_turn=cmd_RC->angular.z;
   //printf("TeleX: %f  TeleY: %f TeleZ %f \n", m_straight, m_sides, m_updown);      
   //ROS_INFO("%f","%f",cmd_RC->linear.x,cmd_RC->angular.z);
    }*/

void settobottomcamera(void)
{
    system("bash -i -c 'rosservice call /ardrone/setcamchannel 1'");
}


//-------------------BATTERY STATE SUBSCRIBER---------------------------//

void NavCallback(const ardrone_autonomy::Navdata& nav_in)
{
 batt_state = nav_in.batteryPercent;
 //printf("Battery: %f \n", batt_state);
}

//------------GPS SUBSCRIBER CALLBACK FUNCTION-------------------//
/*void gpsCallback(const ardrone_autonomy::navdata_gps& gps_in)
{
 gps_long = gps_in.longitude;
 gps_lat = gps_in.latitude;
 //printf("Longitude coordinate: %f  Latitude coordinate: %f \n", gps_long, gps_lat);
}*/

//-------------GPS WAYPOINT NAVIGATION FUNCTIONS-----------------------//
void GpsSetTargetStation()
{
    system("bash -i -c 'ardrone/setgpstarget {target: {id: {uuid: [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]} ,position: {latitude: 51.437266, longitude: -0.249734, altitude: 1}, props: [""]}}'");
}
void GpsSetTargetInitial()
{
    system("bash -i -c 'ardrone/setgpstarget {target: {id: {uuid: [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]} ,position: {latitude: 51.437266, longitude: -0.249734, altitude: 1}, props: [""]}}'");
}

void AutoFlightEnable()
{
    system("bash -i -c 'rosservice call ardrone/setautoflight True'");
}

void AutoFlightDisable()
{
    system("bash -i -c 'rosservice call ardrone/setautoflight False'");
}

//------------------------KEYBOARD CONTROL CALLBACK FUNCTION---------------------------------//

//void keyboardCallback(const geometry_msgs::TwistConstPtr& twist)
//{
//  ROS_INFO("I heard: [%s]", twist->linear.y.c_str());
//}

//------------------------CONTROL FUNCTIONS---------------------------------//

void stop()
{
  	  twist_msg_hover.linear.x=0.0; 
			twist_msg_hover.linear.y=0.0;
			twist_msg_hover.linear.z=0.0;
			twist_msg_hover.angular.x=0.0; 
			twist_msg_hover.angular.y=0.0;
			twist_msg_hover.angular.z=0.0;

}

void adjusty()
{
  	  twist_msg_f.linear.x=outputy; 
			twist_msg_f.linear.y=-outputx;
			twist_msg_f.linear.z=0.0;
			twist_msg_f.angular.x=0.0; 
			twist_msg_f.angular.y=0.0;
			twist_msg_f.angular.z=0.0; 
}

/*void manual()
{
  	  twist_msg_f.linear.x=m_straight; 
			twist_msg_f.linear.y=m_sides;
			twist_msg_f.linear.z=m_updown;
			twist_msg_f.angular.x=0.0; 
			twist_msg_f.angular.y=0.0;
			twist_msg_f.angular.z=0.0; 
}*/

//------------------------------------------------------      OPENCV MODULE      ---------------------------------------------------------------//

//----------------------HSV FILTER SETTINGS BOTTOM CAMERA---------------//
 int H_MIN = 132;
 int H_MAX = 179;
 int S_MIN = 73; 
 int S_MAX = 255;
 int V_MIN = 115;
 int V_MAX = 255;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
 
  NodeHandle nh_;
  image_transport::ImageTransport it_;      //ImageTransport object "it_" under the namespace 'image_transport'
  image_transport::Subscriber image_sub_;       //Subscriber object "image_sub_"
  image_transport::Publisher image_pub_;        // Publisher object "image_pub_"

   public:
    ImageConverter(): it_(nh_)
    {
      
        // Subscribe to input video feed and publish output video feed
      image_sub_ = it_.subscribe("/ardrone/image_raw", 1,&ImageConverter::imageCb, this);
      image_pub_ = it_.advertise("/image_converter/output_video", 1);

           cv::namedWindow(OPENCV_WINDOW);
    }
 
    ~ImageConverter()
    {
            cv::destroyWindow(OPENCV_WINDOW);
    }

   void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
      }
 
  
//-----------------TRACK BARS INTERFACE----------------------//

 namedWindow("Control", 1); //create a window called "Control"


 //Create trackbars in "Control" window
 createTrackbar("H_MIN", "Control", &H_MIN, 179); //Hue (0 - 179)
 createTrackbar("H_MAX", "Control", &H_MAX, 179);
 createTrackbar("S_MIN", "Control", &S_MIN, 255); //Saturation (0 - 255)
 createTrackbar("S_MAX", "Control", &S_MAX, 255);
 createTrackbar("V_MIN", "Control", &V_MIN, 255); //Value (0 - 255)
 createTrackbar("V_MAX", "Control", &V_MAX, 255);

 createTrackbar("Kpx", "Control", &Kpx, 500);
 createTrackbar("Kdx", "Control", &Kdx, 200);
 createTrackbar("Kpy", "Control", &Kpy, 500);
 createTrackbar("Kdy", "Control", &Kdy, 200);
 createTrackbar("Speed_lim", "Control", &speed_lim, 50);

//begin = ros::Time::now().toSec();
 
//-----------------IMAGE PROCESSING-----------------------------//
   Mat HSVImage;
   cvtColor(cv_ptr->image, HSVImage, CV_BGR2HSV); //HSV
   Size size = HSVImage.size();
   Mat mask = cvCreateMat(size.height, size.width, CV_8UC1); //MASK
   inRange(HSVImage, cv::Scalar(H_MIN, S_MIN, V_MIN), cv::Scalar(H_MAX,S_MAX,V_MAX),mask); //detecting colours
   dilate( mask, mask, getStructuringElement(MORPH_RECT, Size(21, 21)) );
   erode(mask, mask, getStructuringElement(MORPH_RECT, Size(10, 10)) );

   //opening
    erode(mask, mask, getStructuringElement(MORPH_RECT, Size(11, 11)) );
    dilate( mask, mask, getStructuringElement(MORPH_RECT, Size(5, 5)) );

    GaussianBlur(mask, mask, Size(15, 15), 2, 2 );
    vector<Vec3f> circles;
    HoughCircles( mask, circles, CV_HOUGH_GRADIENT, 1.5,size.height/10,100,40,0,0); // hough transform

    float x,y,r;
    for( size_t i = 0; i < circles.size(); i++)
     {
       //coordinates of detected circle
       Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
       int radius = cvRound(circles[i][2]);
       //circle center
       circle(mask, center, 3, Scalar(0,255,0), -1, 8, 0);
       //circle outline
       circle(mask, center, 3, Scalar(0,0,255), 3, 8, 0);
       //circle center
       circle(cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0);
       //circle outline
       circle(cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0);
       x=circles[i][0];
       y=circles[i][1];
       r=circles[i][2];
     }
 
//-----------------------------------------------          PD CONTROLLER MODULE         -------------------------------------------------//
      
     /* if(circles.size()>0)
       {
        //land()
       }*/

      float tempx;
      float tempy; //help variables
      kpx=(float)Kpx/(float)100;
      kdx=(float)Kdx/(float)100;
      kpy=(float)Kpy/(float)100;
      kdy=(float)Kdy/(float)100;
      speed_limit=(float)speed_lim/(float)100;
        //...........................PID....................... 
      
      tempx=320-x;
      tempy=y-180;   
      dt=200;  
     //cmdx=  -0.05+ ((0.05+0.05)/ (320+320))*(tempx+320);
     // cmdy=  -0.05+ ((0.05+0.05)/ (180+180))*(tempy+180);

  
       if(circles.size()==0)
        { 
         // manual();
        //  pub_twist.publish(twist_msg_f);
            stop();
            pub_twist.publish(twist_msg_hover);
          ROS_INFO("No CIRCLE");
        }

       else if(tempx>-10 && tempx<10 && tempy>-5 && tempy<5)
        {
             stop();
             pub_twist.publish(twist_msg_hover);
             pub_empty_land.publish(emp_msg);  //land
             ROS_INFO("LANDING TO CHARGE");
        }

   //     else if(tempx<-2 && tempx>2 && tempy<-1 && tempy>1)
          else if(circles.size()>0)
          {
             cmdx=  -speed_limit+ ((speed_limit+speed_limit)/ (320+320))*(tempx+320);
             cmdy=  -speed_limit+ ((speed_limit+speed_limit)/ (180+180))*(tempy+180);
      
            
            derivativex=(errorx-prev_errorx)/dt;
            derivativey=(errory-prev_errory)/dt;
        
            errorx=0-cmdx;
            errory=0-cmdy;

            outputx=kpx*errorx + kdx*derivativex;
            outputy=kpy*errory + kdy*derivativey;
            
            if (outputx>speed_limit)
            {
                outputx=speed_limit;
            }
            else if(outputx<(-speed_limit))
            {
                outputx=-speed_limit;
            }
            else
            {
                outputx=outputx;
            }

            if (outputy>speed_limit)
            {
                outputy=speed_limit;
            }
            else if(outputy<(-speed_limit))
            {
                outputy=-speed_limit;
            }
            else 
            {
                outputy=outputy;
            }
   
        	  twist_msg_f.linear.x=outputy; 
		      	twist_msg_f.linear.y=-outputx;
		      	twist_msg_f.linear.z=0.0;
		      	twist_msg_f.angular.x=0.0; 
			      twist_msg_f.angular.y=0.0;
			      twist_msg_f.angular.z=0.0;
            
         //   ROS_INFO("ADJUSTING");
            
     //       printf("X: %f Y: %f \n", outputx,outputy);
          } 

        pub_twist.publish(twist_msg_f);
  float prev_errorx=errorx;
  float prev_errory=errory; 
//finish = ros::Time::now().toSec();
printf("X: %f Y: %f \n", outputx,outputy);
//dt = finish - begin;
//printf(" Kpx: %f  Kdx: %f   \n   Kpy: %f  Kdy %f   \n", kpx, kdx, kpy, kdy);
          
          

//--------------------------------OPENCV GUI WINDOWS-------------------------//   

      imshow("Filtered", mask);
      imshow("Original", cv_ptr->image);

  }

  
};

int main(int argc, char** argv)
{
//-------------------------     DRONE CONTROL PART      ------------------------//
	ROS_INFO("ARdrone Test Back and Forth Starting");
	init(argc, argv,"ARDrone_test");
  init(argc, argv, "tele");

  NodeHandle node;
  Rate loop_rate(50);
  settobottomcamera();
	double start_time;


  pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
  pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
  pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
  //Subscriber sub = node.subscribe("/cmd_vel", 100, cmdRCCallback); 
  pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */
  //Subscriber subGPS = node.subscribe("/ardrone/navdata_gps", 100, gpsCallback);
  Subscriber subNAV = node.subscribe("/ardrone/navdata", 100, NavCallback);
	
  float takeoff_time=3.0;

	start_time =(double)ros::Time::now().toSec();	
	ROS_INFO("Starting ARdrone_opencv loop");

 while ((double)ros::Time::now().toSec()< start_time+takeoff_time)
  {
    
   // pub_empty_takeoff.publish(emp_msg); //launches the drone
    pub_twist.publish(twist_msg_hover);
    ROS_INFO("Taking off");

  }
  
//---------------       OPENCV PART      ---------------------//
  init(argc, argv, "image_converter");
  ImageConverter ic;
          while(ok())
          {
            spinOnce();

          if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed break loop
              {
                pub_empty_land.publish(emp_msg); //lands the drone
	              ROS_INFO("Landing");
                cout << "esc key is pressed by user" << endl;
                break;
              }
            loop_rate.sleep();
          }
  return 0;

}//main