//
// Created by hendrik on 2019-11-12.
//

#ifndef WORKSPACE_EXCTRINSIC_CALIBRATION_H
#define WORKSPACE_EXCTRINSIC_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include <atomic>
#include <unistd.h>
#include <string>
#include <experimental/filesystem>


using namespace std;
using namespace cv;
using namespace experimental::filesystem;

class extrinsic_calibration {
    private:




    public:

        static string config_folder;
        static Mat bg_img;
        static vector<Point2f> result;
        static string name;
        static atomic<bool> done;
        static int n;
        static double show_scale;

        extrinsic_calibration(const string config_folder){
            show_scale = 2.0;

        }


        static void mouseCallback(int event, int x, int y, int, void* p){

            if (event != EVENT_LBUTTONDOWN || done.load()) return;

            result.emplace_back(x*show_scale, y*show_scale);
            cv::circle(bg_img, cv::Point(x,y), 20/show_scale, Scalar(0,0,255), -1);
            imshow(name.c_str(), bg_img);

            if (result.size() >= n) {
                usleep(500*1000);
                done.store(true);
            }
        }

        vector<Point2f> pickNPoints(int nPoints, const Mat& img){

            name = "Pick " + to_string(nPoints) + " points";

            //result.clear();
            //resize and show image
            Size small_size(img.cols/show_scale, img.rows/show_scale);
            resize(img, bg_img, small_size);
            //bg_img = img.clone();

            imshow(name.c_str(), bg_img);
            namedWindow(name.c_str());
            n = nPoints;

            done.store(false);

            //helper function to retrieve location of the mouse
            setMouseCallback(name.c_str(), mouseCallback);
            while (!done.load()) {
                waitKey(500);
            }

            destroyWindow(name.c_str());
            return result;
        }

        bool extrinsicCalib(const Mat& img_in, vector<Point3f> object_points, const Mat& camera_matrix, Mat& rvec, cv::Mat& tvec){

            string file_path = config_folder + "extrinsicCalib.csv";

            vector<Point2f> image_points;



            if (!experimental::filesystem::exists(file_path)){

                experimental::filesystem::create_directories(config_folder);

                //helper points to extract 4 points from img_in
                image_points = pickNPoints(4, img_in);
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

            Mat dist_coeffs;
            dist_coeffs   = (Mat1d(1,4) << 0, 0, 0, 0, 0);
            bool ok = solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

            // Mat Rt;
            // Rodrigues(rvec_, Rt);
            // auto R = Rt.t();
            // auto pos = -R * tvec_;

            if (!ok) {
                cerr << "FAILED SOLVE_PNP" << endl;
            }

            return ok;
        }

};


#endif //WORKSPACE_EXCTRINSIC_CALIBRATION_H
