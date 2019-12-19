#ifndef EXTRINSIC_CALIBRATOR
#define EXTRINSIC_CALIBRATOR

#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <atomic>

namespace calibration {
    using namespace std; // string, vector, experimental and to_string
    using namespace cv; // Mat, Point3f, Size, resize, imshow, namedWindow, setMouseCallback, waitKey, destroyWindow, EVENT_LBUTTONDOWN, circle, Point and Scalar

    class extrinsic_calibrator {
        private:
            // Class variables
            string config_folder;

            // Helpers variables
            const double show_scale = 1.0;
            const int amount_of_points_to_pick = 4;
            const string window_name = "Pick " + to_string(amount_of_points_to_pick) + " points";
            Mat resized_img;
            vector<Point2f> result;
            atomic<bool> done;

            void mouseCallbackLogic( int event, int x, int y, int flags );
            vector<Point2f> pickPoints(const Mat& img_to_pick_from);

        public:
            extrinsic_calibrator(const string& config_folder);
            static void mouseCallback(int event, int x, int y, int flags, void* that);
            bool calibrate(const Mat& img_in, vector<Point3f> object_points, const Mat& camera_matrix, Mat& rvec, Mat& tvec);
    };
}

#endif
