#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include <fdetection/output2.h>

#include "face_detection.h"

namespace enc = sensor_msgs::image_encodings;
using namespace std;

class image_node_c
{
public:
  fdetection::output2 output2_msg;
  image_transport::Publisher pub;
  image_transport::Subscriber sub;
  //ros::Subscriber figure_sub;
  ros::Publisher output_pub;
  sensor_msgs::ImagePtr msg;
  cv::Mat result;
  cv::Mat tem;
  int fullwidth;
        int fullheight;
        bool BB_inited;
        cv::Rect cutBB;

  seeta::FaceDetection detector;
  image_node_c(ros::NodeHandle& nh, const char* configPath);
  ~image_node_c();
  void imageCallback(const sensor_msgs::ImageConstPtr& tem_msg);
  bool detectAndDraw(cv::Mat& img);
  //void figureCallback(const struck::roi_input msg);
};

image_node_c::image_node_c(ros::NodeHandle& nh, const char* configPath)
:detector(configPath)
,fullheight(0)
,fullwidth(0)
,BB_inited(false)
{
  image_transport::ImageTransport it(nh);
  pub = it.advertise("/node_c_img", 1);
  sub = it.subscribe("/CamImage",1,&image_node_c::imageCallback,this);//modif,CamImage,/ardrone/front/image_raw
  output_pub = nh.advertise<fdetection::output2>("/ardrone_facefollow/output2_msg",5);
  //figure_sub = nh.subscribe("figure_roi",5,&image_node_c::figureCallback,this);
}
image_node_c::~image_node_c()
{}


/*void image_node_c::figureCallback(const struck::roi_input msg)
{
        static int cnt=0;
        cnt++;

        if(msg.pose2.x - msg.pose1.x > 0 && msg.pose2.y - msg.pose1.y > 0){
                if(msg.pose1.x >= 0&&msg.pose1.y >= 0&&fullwidth!=0&&fullheight!=0){
                        int pose1x_val = msg.pose1.x * fullwidth;
                        int pose1y_val = msg.pose1.y * fullheight;
                        int pose2x_val = msg.pose2.x * fullwidth;
                        int pose2y_val = msg.pose2.y * fullheight;
                        int rahm_w = pose2x_val - pose1x_val;
                        int rahm_h = pose2y_val - pose1y_val;
                        cutBB = cv::Rect(pose1x_val, pose1y_val, rahm_w, rahm_h);
      BB_inited = true;
                        //inputBB = IntRect(msg.pose1.x, msg.pose1.y, (msg.pose2.x - msg.pose1.x), (msg.pose2.y - msg.pose1.y));
                }
        }
}
*/
void image_node_c::imageCallback(const sensor_msgs::ImageConstPtr& tem_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
      /*转化成CVImage*/
       cv_ptr = cv_bridge::toCvCopy(tem_msg, enc::BGR8);
       cv::waitKey(1);
  }

  catch (cv_bridge::Exception& e)
  {
    //ROS_ERROR("Could not convert from '%s' to 'mono8'.", tem_msg->encoding.c_str());
  }
  fullwidth = cv_ptr->image.cols;
  fullheight = cv_ptr->image.rows;
  // cout<<"BB"<<endl;
  // cout<<cutBB.x<<"  "<<cutBB.y<<endl;
  // cout<<cutBB.width<<"  "<<cutBB.height<<endl;
  // cout<<"img"<<endl;
  // cout<<fullwidth<<"  "<<fullheight<<endl;
//  cv_ptr->image = cv::image(cutBB);
//  cvSetImageROI(cv_ptr->image,cutBB);
  //cut tem
  //if(BB_inited){
    tem = cv_ptr->image;//(cutBB);
    if(detectAndDraw(tem)){
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg();
      pub.publish(msg);
    }
  //}
  //imshow(tem);
}



bool image_node_c::detectAndDraw(cv::Mat& img){
  cv::imshow("C_OUT_WINDOW", img);
  cv::waitKey(1);
  //seeta::FaceDetection detector("/home/rover/catkin_ws/src/fdetection/model/seeta_fd_frontal_v1.0.bin");
  detector.SetMinFaceSize(20);
  detector.SetScoreThresh(2.f);
  detector.SetImagePyramidScaleFactor(0.8f);
  detector.SetWindowStep(4, 4);

  cv::Mat img_gray;

  if (img.channels() != 1)
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
  else
    img_gray = img;

  seeta::ImageData img_data;
  img_data.data = img_gray.data;
  img_data.width = img_gray.cols;
  img_data.height = img_gray.rows;
  img_data.num_channels = 1;
  std::vector<seeta::FaceInfo> faces = detector.Detect(img_data);

  cv::Rect face_rect;
  int32_t num_face = static_cast<int32_t>(faces.size());

  if(num_face > 0)
  {
    face_rect.x = faces[0].bbox.x;
    face_rect.y = faces[0].bbox.y;
    face_rect.width = faces[0].bbox.width;
    face_rect.height = faces[0].bbox.height;

    cv::rectangle(img, face_rect, CV_RGB(0, 0, 255), 4, 8, 0);

    output2_msg.pose1.x = fullwidth;
    output2_msg.pose1.y = fullheight;
    output2_msg.pose2.x = face_rect.x + face_rect.width/2;
    output2_msg.pose2.y = face_rect.y + face_rect.height/2;
    output2_msg.pose3.x = face_rect.width;
    output2_msg.pose3.y = face_rect.height;

    if (face_rect.x >= 0 && (face_rect.x + face_rect.width) <= img.cols )
    {
        if(face_rect.y <= 0)
          {
            result = tem(cv::Rect(face_rect.x,0,face_rect.width,output2_msg.pose2.y));
          }
        else result = tem(cv::Rect(face_rect.x,face_rect.y,face_rect.width,face_rect.height));
    }
    output_pub.publish(output2_msg);

    cv::imshow("C_OUT_WINDOW", img);
    cv::waitKey(1);
    return true;
  } else {
    return false;
  }
}


int main(int argc, char** argv)
{
        //string configPath = argv[1];
  string configPath = "/home/wade/catkin_ws/src/fdetection/model/seeta_fd_frontal_v1.0.bin";
  const char *configPathC = configPath.c_str();
  ros::init(argc, argv, "image_node_c");
        ros::NodeHandle nc;
  image_node_c inc(nc,configPathC);


  ros::Rate loop_rate(10);

  while (nc.ok()) {

    ros::spinOnce();
    loop_rate.sleep();
  }

        return 0;
}

