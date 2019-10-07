#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <ctime>
#include <stdexcept>
#include <sstream>

// classes which are included
#include "ReturnPath.hpp"



//global variables needed to store pictures 
time_t now = 0;
double elapsed_time =  20.0; //time in seconds when pictures can be shown/stored.

//set to true if cv::initUndistortRectifyMap in imageundistorted() has been already executed. 



namespace student {
 
 void loadImage(cv::Mat& img_out, const std::string& config_folder){  

   
   throw std::logic_error( "STUDENT FUNCTION NOT IMPLEMENTED" );
 }

 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
    
    if(now == 0 || double(time(0) - now) > elapsed_time){
      //update time
      now = time(0);  

      //draws a circle on the image
      //cv::circle(img_in, cv::Point (400,400), 50,  cv::Scalar (128,128,128), -1, 8);

      

      //flips image vertically
      //cv:: Mat img_flip;
      //cv::flip(img_in, img_flip, 1);

      //scales image
      //cv:: Mat img_out;
      //cv::resize(img_in,img_out , cv::Size(), 0.75, 0.75);

      //show the image in a window
      cv::imshow("Display window", img_in);
      //waits for input, otherwise the window is open infinitely.
      char x = cv::waitKey(0);

      //enters if statement if s is pressed.
      if(x == 's'){
        // create a path string with a time stamp to store the image
        ReturnPath path;
        std::string str3 = path.givePath();
        //store image
        bool check = cv::imwrite(str3, img_in);
      }
      //force windows to close
      cv::destroyAllWindows();
    }
  }
 
  

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, 
  cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    
    cv::solvePnP(object_points, InputArray imagePoints, camera_matrix, NULL, rvec, tvec, bool useExtrinsicGuess=true, int flags=ITERATIVE )
    
       
  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
      const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
        
      static bool alreadyexecuted = false;
      static cv::Mat full_map1, full_map2;
      
      // cv::undistort(img_in,img_out,cam_matrix,dist_coeffs);
      if(alreadyexecuted== false){
        cv::Mat R;
        cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, img_in.size(), CV_16SC2, full_map1, full_map2);
        alreadyexecuted == true;
      }

      cv::remap(img_in, img_out, full_map1, full_map2, cv::INTER_LINEAR);

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

