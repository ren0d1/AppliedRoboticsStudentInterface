#ifndef VISUAL_PROCESSOR
#define VISUAL_PROCESSOR

#include "utils.hpp"

#include <opencv2/opencv.hpp>
#include <stdexcept>

namespace visual_processing {
    using namespace std; // string, vector, experimental and to_string

    class recognizer {
        private:
            // Class variables
            string config_folder;

            // Helpers variables


        public:
            recognizer(const string& config_folder);
            bool findRobotInfo(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta);
            //bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate);
    };
}

#endif
