#ifndef PATHPLANNER
#define PATHPLANNER

#include "utils.hpp"

#include <opencv2/opencv.hpp>

//region Structures

struct SmartPoint {
    cv::Point2d coordinates;
    int obstacle_id;
};
//endregion Structures

class PathPlanner {
    private:
        //region Helpers

        bool intersection(Point o1, Point p1, Point o2, Point p2, cv::Point2d &r);
        int findEdgeId(std::pair<SmartPoint, SmartPoint> ray, std::vector<std::pair<SmartPoint, SmartPoint>> opened_cells_edges, double borders_min_Y, double borders_max_Y);
        //endregion Helpers

        //region algorithms

        std::vector<std::pair<cv::Point2d, cv::Point2d>> exactCellDecomposition(const Polygon& borders, const std::vector<Polygon>& obstacle_list);
        //endregion algorithms

    public:
        PathPlanner() = default;

        enum algorithms {
            EXACT_CELL, MAX_CLEARANCE, AMOUNT_OF_ALGORITHMS
        };

        int planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list,
                      const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate,
                      const float x, const float y, const float theta, int algorithm);
};

#endif
