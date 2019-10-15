#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <ctime>
#include <stdexcept>
#include <sstream>

#include <vector>
#include <atomic>
#include <unistd.h>
#include <experimental/filesystem>

// classes which are included
#include "ReturnPath.hpp"

//global variables needed to store pictures
time_t now = 0;
double elapsed_time = 20.0; //time in seconds when pictures can be shown/stored.

//set to true if cv::initUndistortRectifyMap in imageundistorted() has been already executed.

namespace student
{

void loadImage(cv::Mat &img_out, const std::string &config_folder)
{

  throw std::logic_error("STUDENT FUNCTION NOT IMPLEMENTED");
}

void genericImageListener(const cv::Mat &img_in, std::string topic, const std::string &config_folder)
{

  if (now == 0 || double(time(0) - now) > elapsed_time)
  {
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
    if (x == 's')
    {
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

///Extrinsic calibration

// Defintion of the function pickNPoints and the callback mouseCallback.
// The function pickNPoints is used to display a window with a background
// image, and to prompt the user to select n points on this image.
static cv::Mat bg_img;
static std::vector<cv::Point2f> result;
static std::string name;
static std::atomic<bool> done;
static int n;
static double show_scale = 1.0;

void mouseCallback(int event, int x, int y, int, void *p)
{
  if (event != cv::EVENT_LBUTTONDOWN || done.load())
    return;

  result.emplace_back(x * show_scale, y * show_scale);
  cv::circle(bg_img, cv::Point(x, y), 20 / show_scale, cv::Scalar(0, 0, 255), -1);
  cv::imshow(name.c_str(), bg_img);

  if (result.size() >= n)
  {
    usleep(500 * 1000);
    done.store(true);
  }
}

std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat &img)
{
  result.clear();
  cv::Size small_size(img.cols / show_scale, img.rows / show_scale);
  cv::resize(img, bg_img, small_size);
  //bg_img = img.clone();
  name = "Pick " + std::to_string(n0) + " points";
  cv::imshow(name.c_str(), bg_img);
  cv::namedWindow(name.c_str());
  n = n0;

  done.store(false);

  cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
  while (!done.load())
  {
    cv::waitKey(500);
  }

  cv::destroyWindow(name.c_str());
  return result;
}

bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, const cv::Mat &camera_matrix,
                    cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder)
{

  std::string file_path = config_folder + "/extrinsicCalib.csv";

  std::vector<cv::Point2f> image_points;

  if (!std::experimental::filesystem::exists(file_path)){

    std::experimental::filesystem::create_directories(config_folder);

    image_points = pickNPoints(4, img_in);
    // SAVE POINT TO FILE
    std::cout << "IMAGE POINTS: " << std::endl;
    for (const auto pt: image_points) {
      std::cout << pt << std::endl;
    }
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
  if (!input.is_open())
  {
    throw std::runtime_error("Cannot read file: " + file_path);
  }
  while (!input.eof())
  {
    double x, y;
    if (!(input >> x >> y))
    {
      if (input.eof())
        break;
      else
      {
        throw std::runtime_error("Malformed file: " + file_path);
      }
    }
    image_points.emplace_back(x, y);
  }



  input.close();
}
cv::Mat dist_coeffs;
dist_coeffs = (cv::Mat1d(1, 4) << 0, 0, 0, 0, 0);
bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

if (!ok)
{
  std::cerr << "FAILED SOLVE_PNP" << std::endl;
}

return ok;

} // namespace student

void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out,
                    const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, const std::string &config_folder)
{

  static bool alreadyexecuted = false;
  static cv::Mat full_map1, full_map2;

  // cv::undistort(img_in,img_out,cam_matrix,dist_coeffs);
  if (alreadyexecuted == false)
  {
    cv::Mat R;
    cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, img_in.size(), CV_16SC2, full_map1, full_map2);
    alreadyexecuted == true;
  }

  cv::remap(img_in, img_out, full_map1, full_map2, cv::INTER_LINEAR);
}

  //-------------------------------------------------------------------------
  //          FIND PLANE TRANSFORM
  //-------------------------------------------------------------------------
  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, 
                          const cv::Mat& tvec, 
                          const std::vector<cv::Point3f>& object_points_plane, 
                          const std::vector<cv::Point2f>& dest_image_points_plane, 
                          cv::Mat& plane_transf, const std::string& config_folder){
    
    cv::Mat image_points;

    // project points
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
  }

  //-------------------------------------------------------------------------
  //          UNWARP TRANSFORM
  //-------------------------------------------------------------------------
  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
              const std::string& config_folder){
    cv::warpPerspective(img_in, img_out, transf, img_in.size());
  }
bool processMap(const cv::Mat &img_in, const double scale, std::vector<Polygon> &obstacle_list, std::vector<std::pair<int, Polygon>> &victim_list, Polygon &gate, const std::string &config_folder)
{
  throw std::logic_error("STUDENT FUNCTION NOT IMPLEMENTED");
}

bool findRobot(const cv::Mat &img_in, const double scale, Polygon &triangle, double &x, double &y, double &theta, const std::string &config_folder)
{
  throw std::logic_error("STUDENT FUNCTION NOT IMPLEMENTED");
}

bool planPath(const Polygon &borders, const std::vector<Polygon> &obstacle_list, const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x, const float y, const float theta, Path &path)
{
  throw std::logic_error("STUDENT FUNCTION NOT IMPLEMENTED");
}
}
