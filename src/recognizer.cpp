#include "recognizer.hpp"

namespace visual_processing {
    /* Constructor */
    recognizer::recognizer(const string& config_folder){
        this->config_folder = config_folder;
    }

    /* Processing functions */
    bool recognizer::findRobotInfo(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta)
    {
        // Convert to HSV for easier color separation
        cv::Mat hsv_img;
        cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

        // Blue mask to retrieve robot triangle (in this fixed context, this is our only blue object)
        cv::Mat blue_mask;
        cv::inRange(hsv_img, cv::Scalar(110, 120, 150), cv::Scalar(130, 255, 255), blue_mask);

        // Find contours of the triangle
        vector<vector<cv::Point>> contours;
        cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Approximate the contours (0 because we only get the uttermost contour's points list
        vector<cv::Point> approx_curve;
        cv::approxPolyDP(contours[0], approx_curve, 30, true);

        if(approx_curve.size() != 3)
        {
            return false;
        }

        for (const cv::Point pt: approx_curve) {
            triangle.emplace_back(pt.x/scale, pt.y/scale);
        }

        // Find triangle baricenter
        x = 0;
        y = 0;
        for(auto vertex: triangle)
        {
            x += vertex.x;
            y += vertex.y;
        }

        x /= static_cast<double>(triangle.size());
        y /= static_cast<double>(triangle.size());

        // Retrieve the vertex of the triangle corresponding to the "head" of the robot
        Point top_vertex;
        double highest_distance = 0;
        for(auto vertex: triangle)
        {
            double dx = vertex.x - x;
            double dy = vertex.y - y;
            double current_vertex_distance = dx * dx + dy * dy;
            if (current_vertex_distance > highest_distance)
            {
                highest_distance = current_vertex_distance;
                top_vertex = vertex;
            }
        }

        const double dx = x - top_vertex.x;
        const double dy = y - top_vertex.y;
        theta = atan2(dy, dx);

        return true;
    }
}