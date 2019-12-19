#include "extrinsic_calibrator.hpp"

#include <experimental/filesystem>
#include <unistd.h> // usleep

namespace calibration {
    using namespace experimental::filesystem; // exists and create_directories

    /* Constructor */
    extrinsic_calibrator::extrinsic_calibrator(const string& config_folder){
        this->config_folder = config_folder;
    }

    /* Helpers */
    void extrinsic_calibrator::mouseCallback(int event, int x, int y, int flags, void* that){
        extrinsic_calibrator* myclass = reinterpret_cast<extrinsic_calibrator*>(that);
        myclass->mouseCallbackLogic(event,  x, y, flags );
    }

    void extrinsic_calibrator::mouseCallbackLogic(int event, int x, int y, int flags){
        static int n = 4;
        if (event != EVENT_LBUTTONDOWN || done.load()) return;

        result.emplace_back(x*show_scale, y*show_scale);
        circle(resized_img, cv::Point(x,y), 20/show_scale, Scalar(0,0,255), -1);
        imshow(window_name.c_str(), resized_img);

        if (result.size() >= n) {
            usleep(500*1000);
            done.store(true);
        }
    }

    vector<Point2f> extrinsic_calibrator::pickPoints(const Mat& img_to_pick_from){
        result.clear();

        Size small_size(img_to_pick_from.cols/show_scale, img_to_pick_from.rows/show_scale);
        resize(img_to_pick_from, resized_img, small_size);

        imshow(window_name.c_str(), resized_img);
        namedWindow(window_name.c_str());

        done.store(false);

        setMouseCallback(window_name.c_str(), extrinsic_calibrator::mouseCallback, this);
        while (!done.load()) {
            waitKey(500);
        }

        destroyWindow(window_name.c_str());
        return result;
    }

    /* Calibrator functions */
    bool extrinsic_calibrator::calibrate(const Mat& img_in, vector<Point3f> object_points, const Mat& camera_matrix, Mat& rvec, Mat& tvec) {
        string file_path = config_folder + "/extrinsicCalib.csv";

        vector<Point2f> image_points;

        if (!exists(file_path)){
            create_directories(config_folder);

            image_points = pickPoints(img_in);
            // SAVE POINT TO FILE
            // cout << "IMAGE POINTS: " << endl;
            // for (const auto pt: image_points) {
            //   cout << pt << endl;
            // }
            ofstream output(file_path);
            if (!output.is_open()){
                throw runtime_error("Cannot write file: " + file_path);
            }
            for (const auto pt: image_points) {
                output << pt.x << " " << pt.y << endl;
            }
            output.close();
        }else{
            // LOAD POINT FROM FILE
            ifstream input(file_path);
            if (!input.is_open()){
                throw runtime_error("Cannot read file: " + file_path);
            }
            while (!input.eof()){
                double x, y;
                if (!(input >> x >> y)) {
                    if (input.eof()) break;
                    else {
                        throw runtime_error("Malformed file: " + file_path);
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
            cerr << "FAILED SOLVE_PNP" << endl;
        }

        return ok;
    }
}