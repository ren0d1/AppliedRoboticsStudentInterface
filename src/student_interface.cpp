#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <opencv2/core.hpp>
#include <iostream>
#include <chrono>

#include <stdexcept>
#include <sstream>
namespace student {

 int imageCount = 0;
 int minutes_interval_between_saving_images = 1;
 auto time_last_image_was_saved = std::chrono::system_clock::now();

 void loadImage(cv::Mat& img_out, const std::string& config_folder){
     throw std::logic_error( "STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED" );
 }

 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
     try
     {
         if (std::chrono::system_clock::now() > time_last_image_was_saved + std::chrono::minutes(minutes_interval_between_saving_images)){
             time_last_image_was_saved = std::chrono::system_clock::now();
         }else if (imageCount != 0){
             return;
         }

         std::string path = "/home/lar2019/workspace/project/images/";
         std::string pictureName = std::to_string(imageCount) + ".jpg";
         std::cout << pictureName << " to be saved\n";
         cv::imwrite(path + pictureName, img_in);
         std::cout << "Image saved\n";
         imageCount++;
         
         cv::Mat test_img = img_in;
         cv::Rect rect(0, 0, test_img.size().width / 2, test_img.size().height / 2);
         cv::rectangle(test_img, rect, cv::Scalar(255, 0, 0));

         cv::imshow( "ImageWithDrawOver", test_img);  // display the image in the specified window
         char c = cv::waitKey(30);               // Waits for a pressed key
     }
     catch (int e)
     {
         throw std::logic_error( "STUDENT FUNCTION - IMAGE LISTENER - NOT CORRECTLY IMPLEMENTED" );
     }
  }

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - EXTRINSIC CALIB - NOT IMPLEMENTED" );
  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){

    throw std::logic_error( "STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED" );

  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, cv::Mat& plane_transf, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED" );
  }


  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, const double scale, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - UNWRAP - NOT IMPLEMENTED" );
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - PROCESS MAP - NOT IMPLEMENTED" );
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    throw std::logic_error( "STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED" );
  }

  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path){
    throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );
  }


}

