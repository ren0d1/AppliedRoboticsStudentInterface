#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
namespace student {

    void loadImage(cv::Mat& img_out, const std::string& config_folder){
        throw std::logic_error( "STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED" );
    }

    void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
        throw std::logic_error( "STUDENT FUNCTION - IMAGE LISTENER - NOT CORRECTLY IMPLEMENTED" );
    }

    bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
        throw std::logic_error( "STUDENT FUNCTION - EXTRINSIC CALIB - NOT IMPLEMENTED" );
    }

    void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out,
                        const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){

        throw std::logic_error( "STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED" );

    }

    void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec,
                            const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane,
                            const std::vector<cv::Point2f>& dest_image_points_plane,
                            cv::Mat& plane_transf, const std::string& config_folder){
        throw std::logic_error( "STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED" );
    }


    void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf,
                const std::string& config_folder){
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