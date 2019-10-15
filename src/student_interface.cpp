#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <opencv2/core.hpp>
#include <iostream>
#include <chrono>

#include <vector>
#include <atomic>
#include <unistd.h>

#include <experimental/filesystem>
#include <stdexcept>
#include <sstream>
namespace student {

 int imageCount = 0;
 int minutes_interval_between_saving_images = 1;
 auto time_last_image_was_saved = std::chrono::system_clock::now();
 bool maps_initialized = false;
 cv::Mat full_map1, full_map2;

 void loadImage(cv::Mat& img_out, const std::string& config_folder){
     try
     {
         std::string path = "/home/lar2019/workspace/project/images/";
         std::string pictureName = std::to_string(imageCount) + ".jpg";
         cv::Mat loaded_image = cv::imread(path + pictureName, cv::IMREAD_COLOR ); // Read the file
         cv::imshow( "ImageLoadedFromFile", loaded_image);  // display the image in the specified window
         char c = cv::waitKey(0);               // Waits for a pressed key
     }
     catch (int e)
     {
         throw std::logic_error( "STUDENT FUNCTION - LOAD IMAGE - NOT CORRECTLY IMPLEMENTED" );
     }
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
         imageCount++;

         cv::Mat test_img;
         //cv::resize(img_in, test_img, cv::Size(), 0.75, 0.75);
         //cv::Rect rect(0, 0, img_in.size().width / 2, img_in.size().height / 2);
         //cv::rectangle(test_img, rect, cv::Scalar(255, 0, 0));

         cv::imshow( "ImageCaptured", test_img);  // display the image in the specified window
         char c = cv::waitKey(0);               // Waits for a pressed key
         if (c == 's')
         {
             cv::imwrite(path + pictureName, img_in);
             std::cout << "Image saved\n";
         }

     }
     catch (int e)
     {
         throw std::logic_error( "STUDENT FUNCTION - IMAGE LISTENER - NOT CORRECTLY IMPLEMENTED" );
     }
  }

    //-------------------------------------------------------------------------
    //          EXTRINSIC CALIB IMPLEMENTATION
    //-------------------------------------------------------------------------

    // Defintion of the function pickNPoints and the callback mouseCallback.
    // The function pickNPoints is used to display a window with a background
    // image, and to prompt the user to select n points on this image.
    static cv::Mat bg_img;
    static std::vector<cv::Point2f> result;
    static std::string name;
    static std::atomic<bool> done;
    static int n;
    static double show_scale = 1.0;

    void mouseCallback(int event, int x, int y, int, void* p)
    {
        if (event != cv::EVENT_LBUTTONDOWN || done.load()) return;

        result.emplace_back(x*show_scale, y*show_scale);
        cv::circle(bg_img, cv::Point(x,y), 20/show_scale, cv::Scalar(0,0,255), -1);
        cv::imshow(name.c_str(), bg_img);

        if (result.size() >= n) {
            usleep(500*1000);
            done.store(true);
        }
    }

    std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat& img)
    {
        result.clear();
        cv::Size small_size(img.cols/show_scale, img.rows/show_scale);
        cv::resize(img, bg_img, small_size);
        //bg_img = img.clone();
        name = "Pick " + std::to_string(n0) + " points";
        cv::imshow(name.c_str(), bg_img);
        cv::namedWindow(name.c_str());
        n = n0;

        done.store(false);

        cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
        while (!done.load()) {
            cv::waitKey(500);
        }

        cv::destroyWindow(name.c_str());
        return result;
    }

    bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){

        std::string file_path = config_folder + "extrinsicCalib.csv";

        std::vector<cv::Point2f> image_points;

        if (!std::experimental::filesystem::exists(file_path)){

            std::experimental::filesystem::create_directories(config_folder);

            image_points = pickNPoints(4, img_in);
            // SAVE POINT TO FILE
            // std::cout << "IMAGE POINTS: " << std::endl;
            // for (const auto pt: image_points) {
            //   std::cout << pt << std::endl;
            // }
            std::ofstream output(file_path);
            if (!output.is_open()){
                throw std::runtime_error("Cannot write file: " + file_path);
            }
            for (const auto pt: image_points) {
                output << pt.x << " " << pt.y << std::endl;
            }
            output.close();
        }else{
            // LOAD POINT FROM FILE
            std::ifstream input(file_path);
            if (!input.is_open()){
                throw std::runtime_error("Cannot read file: " + file_path);
            }
            while (!input.eof()){
                double x, y;
                if (!(input >> x >> y)) {
                    if (input.eof()) break;
                    else {
                        throw std::runtime_error("Malformed file: " + file_path);
                    }
                }
                image_points.emplace_back(x, y);
            }
            input.close();
        }

        cv::Mat dist_coeffs;
        dist_coeffs   = (cv::Mat1d(1,4) << 0, 0, 0, 0, 0);
        bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

        // cv::Mat Rt;
        // cv::Rodrigues(rvec_, Rt);
        // auto R = Rt.t();
        // auto pos = -R * tvec_;

        if (!ok) {
            std::cerr << "FAILED SOLVE_PNP" << std::endl;
        }

        return ok;
    }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
      try
      {
          /*
          // OPTIMIZED VERSION
          if(!maps_initialized){
              // Note: m1type=CV_16SC2 to use fast fixed-point maps (see cv::remap)
              cv::Mat R;
              cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, img_in.size(), CV_16SC2, full_map1, full_map2);

              maps_initialized = true;
          }

          // Initialize output image
          cv::remap(img_in, img_out, full_map1, full_map2, cv::INTER_LINEAR);
           */
          undistort(img_in, img_out, cam_matrix, dist_coeffs);
      }
      catch (int e)
      {
          throw std::logic_error( "STUDENT FUNCTION - IMAGE UNDISTORT - NOT CORRECTLY IMPLEMENTED" );
      }
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

