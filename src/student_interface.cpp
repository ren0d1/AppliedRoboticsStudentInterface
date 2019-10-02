#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// classes which are included
#include "ReturnPath.hpp"


#include <ctime>



#include <stdexcept>
#include <sstream>

//global variables needed to store pictures
time_t now = 0;
double elapsed_time =  10.0;
namespace student {
 
 void loadImage(cv::Mat& img_out, const std::string& config_folder){  

   
   throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );
 }

 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
    
    if(now == 0 || double(time(0) - now) > elapsed_time){
      
      // create a path string with a time stamp
      time_t rawtime;
      struct tm * timeinfo;
      char buffer[80];
      time (&rawtime);
      timeinfo = localtime(&rawtime);
      strftime(buffer,sizeof(buffer),"%d-%m-%Y %H:%M:%S",timeinfo);      
      std::string str = "/home/hendrik/Desktop/test_images/img"; //Attention: Insert here the directory where the files should be stored.
      std::string str1(buffer);
      std::string str2 = ".png";
      std::string str3 = str + str1 + str2; 

      //cv::circle(img_in, cv::Point (400,400), 50,  cv::Scalar (128,128,128), -1, 8);

      //show a window
      //cv::imshow("Display window", img_in);

      cv::flip(img_in, img_in, 1);
      //cv::resize(img_in, img_in, cv::Size(), 0.75, 0.75);
      
      int x = cv::waitKey(30);
      //safe image to path
      bool check = cv::imwrite(str3, img_in);

      // if(x == 115){
  
      //   bool check = cv::imwrite(str3, img_in);
        
      //   if(check){
      //     throw std::logic_error( "File safed." );
      //   }else{
      //     throw std::logic_error( "Something went wrong." );
      //   }
      // } 
      now = time(0);

    }
 
  }

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );   
  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){

    throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );  

  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, cv::Mat& plane_transf, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );  
  }


  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, const double scale, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );   
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );   
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );    
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path){
    throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );     
  }


}

