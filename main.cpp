#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <direct.h> // _mkdir
#include <regex>
#include <numeric>
#include <cmath>
#include <iomanip>
#ifdef _WIN32
#include <windows.h>
#include <locale.h>
#endif

using namespace std;
using namespace cv;

#define BOARD_TYPE "charuco"
#define SQUARES_X 11 // 内部角点数量（棋盘格）
#define SQUARES_Y 8 // 内部角点数量（棋盘格）
#define SQUARE_LENGTH 0.04f // 单位：米
#define MARKER_LENGTH 0.02f // 单位：米
#define DICTIONARY_NAME "DICT_4X4_50" // ArUco 字典名称
#define IMAGES_DIR "collect_data" // 图像文件夹路径
#define POSES_PATH "collect_data/poses.txt" //  机器人位姿文件路径
// Camera matrix data (row-major, 3x3) [fx, 0, cx; 0, fy, cy; 0, 0, 1]
#define CAM_DATA_0 0.0
#define CAM_DATA_1 0.0
#define CAM_DATA_2 0.0
#define CAM_DATA_3 0.0
#define CAM_DATA_4 0.0
#define CAM_DATA_5 0.0
#define CAM_DATA_6 0.0
#define CAM_DATA_7 0.0
#define CAM_DATA_8 1.0
// Distortion coefficients (1x5) [k1, k2, p1, p2, k3]
#define DIST_DATA_0 0.0
#define DIST_DATA_1 0.0
#define DIST_DATA_2 0.0
#define DIST_DATA_3 0.0
#define DIST_DATA_4 0.0


// 获取文件名（不含路径）
static string basename(const string& p) {
    size_t pos = p.find_last_of("/\\\\");
    if (pos == string::npos) return p;
    return p.substr(pos + 1);
}

// 将欧拉角 (rx, ry, rz)（弧度）转换为旋转矩阵 R
static void eulerToRotation(double rx, double ry, double rz, Mat& R) {
    // 与 Python 中相同的顺序：R = Rz * Ry * Rx
    Mat Rx = (Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(rx), -sin(rx),
        0, sin(rx), cos(rx));
    Mat Ry = (Mat_<double>(3, 3) <<
        cos(ry), 0, sin(ry),
        0, 1, 0,
        -sin(ry), 0, cos(ry));
    Mat Rz = (Mat_<double>(3, 3) <<
        cos(rz), -sin(rz), 0,
        sin(rz), cos(rz), 0,
        0, 0, 1);
    R = Rz * Ry * Rx;
}

// 构造齐次变换矩阵 4x4，输入 R(3x3), t(3x1)，输出 H(4x4)
static Mat toHomogeneous(const Mat& R, const Mat& t) {
    Mat H = Mat::eye(4, 4, CV_64F);
    R.convertTo(H(Range(0, 3), Range(0, 3)), CV_64F);
    H(Range(0, 3), Range(3, 4)) = t.clone();
    return H;
}

// 查找字符串中的第一个整数，作为自然排序键
static int naturalKey(const string& s) {
    smatch m;
    regex r("\\d+");
    if (regex_search(s, m, r)) return stoi(m.str());
    return -1;
}

// 读取机器人位姿，格式为 collect_data/poses.txt：每行 x,y,z,rx,ry,rz
static bool readRobotPoses(const string& path, vector<Mat>& R_out, vector<Mat>& t_out) {
    ifstream ifs(path);
    if (!ifs) return false;
    string line;
    while (getline(ifs, line)) {
        if (line.find_first_not_of(" \t\r\n") == string::npos) continue;
        // 按逗号或空白分隔
        replace(line.begin(), line.end(), ',', ' ');
        stringstream ss(line);
        vector<double> v; double x;
        while (ss >> x) v.push_back(x);
        if (v.size() < 6) return false;
        double px = v[0], py = v[1], pz = v[2], rx = v[3], ry = v[4], rz = v[5];
        Mat R(3, 3, CV_64F);
        eulerToRotation(rx, ry, rz, R);
        Mat t = (Mat_<double>(3, 1) << px, py, pz);
        R_out.push_back(R);
        t_out.push_back(t);
    }
    return true;
}

// 评估一对变换的残差，返回旋转误差（度）和位移误差（范数）
static pair<double, double> evalPairResidual(const Mat& X, const Mat& A_i, const Mat& A_j, const Mat& B_i, const Mat& B_j) {
    Mat A_j_inv = A_j.inv();
    Mat B_i_inv = B_i.inv();
    Mat Left = A_j_inv * A_i * X;
    Mat Right = X * B_j * B_i_inv;
    Mat E = Right.inv() * Left;
    Mat R_err = E(Range(0, 3), Range(0, 3));
    Mat t_err = E(Range(0, 3), Range(3, 4));
    double tracev = R_err.at<double>(0, 0) + R_err.at<double>(1, 1) + R_err.at<double>(2, 2);
    double cos_angle = (tracev - 1.0) / 2.0;
    cos_angle = max(-1.0, min(1.0, cos_angle));
    double angle = acos(cos_angle);
    double angle_deg = angle * 180.0 / CV_PI;
    double trans_norm = norm(t_err);
    return{ angle_deg, trans_norm };
}


int main(int argc, char** argv) {

    // Configuration comes from compile-time macros (no external config file)
#ifdef _WIN32
    // Ensure Windows console uses UTF-8 so Chinese output isn't garbled
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
    // Make C runtime use UTF-8 locale for narrow I/O
    setlocale(LC_ALL, ".UTF8");
#endif
    string board_type = string(BOARD_TYPE);
    int squares_x = SQUARES_X;
    int squares_y = SQUARES_Y;
    float square_length = SQUARE_LENGTH;
    float marker_length = MARKER_LENGTH;
    double L = square_length;
    string dictionary_name = string(DICTIONARY_NAME);
    string images_dir = string(IMAGES_DIR);
    string poses_path = string(POSES_PATH);
    Mat cameraMatrix = Mat::zeros(3, 3, CV_64F);
    cameraMatrix.at<double>(0,0) = CAM_DATA_0; cameraMatrix.at<double>(0,1) = CAM_DATA_1; cameraMatrix.at<double>(0,2) = CAM_DATA_2;
    cameraMatrix.at<double>(1,0) = CAM_DATA_3; cameraMatrix.at<double>(1,1) = CAM_DATA_4; cameraMatrix.at<double>(1,2) = CAM_DATA_5;
    cameraMatrix.at<double>(2,0) = CAM_DATA_6; cameraMatrix.at<double>(2,1) = CAM_DATA_7; cameraMatrix.at<double>(2,2) = CAM_DATA_8;
    Mat distCoeffs = Mat::zeros(1, 5, CV_64F);
    distCoeffs.at<double>(0,0) = DIST_DATA_0; distCoeffs.at<double>(0,1) = DIST_DATA_1; distCoeffs.at<double>(0,2) = DIST_DATA_2; distCoeffs.at<double>(0,3) = DIST_DATA_3; distCoeffs.at<double>(0,4) = DIST_DATA_4;
    string debug_dir = images_dir + "/debug";
    _mkdir(debug_dir.c_str());

    // 列出图像文件
    vector<string> files_full, tmp;
    cv::glob(images_dir + "/*.jpg", files_full, false);
    cv::glob(images_dir + "/*.png", tmp, false);
    files_full.insert(files_full.end(), tmp.begin(), tmp.end());
    sort(files_full.begin(), files_full.end(), [](const string& pa, const string& pb) {
        return naturalKey(basename(pa)) < naturalKey(basename(pb));
        });
    std::cout << "Found images: " << files_full.size() << "\n";
    Size imageSize;
    TermCriteria criteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 300, 0.0001); //创建了一个OpenCV的TermCriteria对象，用于定义迭代算法的终止条件。

    if (board_type == "charuco") {
        vector<vector<Point2f>> all_charuco_corners;
        vector<vector<int>> all_charuco_ids;
        vector<string> charuco_image_files;
        //获取预定义的字典

        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
        if (dictionary_name == "DICT_4X4_50") dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
        else if (dictionary_name == "DICT_5X5_100") dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_100);


        cv::aruco::CharucoBoard charucoBoard = cv::aruco::CharucoBoard(cv::Size(squares_x, squares_y), square_length, marker_length, dictionary);
        // 2. （可选）配置参数
        cv::aruco::DetectorParameters detectorParams; // 标记检测参数
        cv::aruco::CharucoParameters charucoParams;   // Charuco角点插值参数

        // 如果已有相机标定参数，可以在这里设置，提高角点插值精度
        if (!cameraMatrix.empty() && !distCoeffs.empty()) {
            charucoParams.cameraMatrix = cameraMatrix;
            charucoParams.distCoeffs = distCoeffs;
        }

        // 3. 创建 CharucoDetector 对象
        cv::aruco::CharucoDetector charucoDetector(charucoBoard, charucoParams, detectorParams);

        for (size_t idx = 0; idx < files_full.size(); ++idx) {
            string fname = files_full[idx];
            Mat img = imread(fname);
            if (img.empty()) { std::cout << "无法读取: " << fname << "\n"; continue; }
            if (imageSize.width == 0) imageSize = img.size();
            Mat gray; cvtColor(img, gray, COLOR_BGR2GRAY);

            // 新版：一站式检测
            vector<cv::Point2f> charucoCorners;
            vector<int> charucoIds;
            vector<vector<Point2f>> markerCorners;
            vector<int> markerIds;

            charucoDetector.detectBoard(gray, charucoCorners, charucoIds, markerCorners, markerIds);



            if (!markerIds.empty()) {
                all_charuco_corners.push_back(charucoCorners);
                all_charuco_ids.push_back(charucoIds);
                charuco_image_files.push_back(basename(fname));
                // 可视化(绘制标记和角点)
                Mat vis = img.clone();
                if (!markerIds.empty()) { // 如果检测到了标记
                    cv::aruco::drawDetectedMarkers(vis, markerCorners, markerIds);
                }
                cv::aruco::drawDetectedCornersCharuco(vis, charucoCorners, charucoIds, Scalar(0, 255, 0));
                string outdet = debug_dir + "/charuco_" + basename(fname);
                cv::imwrite(outdet, vis);
                std::cout << "保存可视化结果至: " << outdet << "\n";
            }
            else {
                std::cout << "图像中未找到有效的Charuco角点: " << basename(fname) << "\n";
            }
        }
        std::cout << "Charuco valid images: " << all_charuco_corners.size() << endl;
    }





    // ...existing code...

    // 定义检测模式：棋盘格 (11x8)
    struct PatternSet {
        string name;
        Size pattern;
        bool useCircles;
        vector<vector<Point3f>> obj_points;
        vector<vector<Point2f>> img_points;
        vector<string> image_files;
        vector<int> used_indices; // 在 files 列表中的索引
    };

    vector<PatternSet*> patterns;
    PatternSet chess;
    if (board_type == "chessboard") {
        chess.name = "chessboard_" + to_string(squares_x) + "x" + to_string(squares_y);
        chess.pattern = Size(squares_x, squares_y);
        chess.useCircles = false;
        patterns.push_back(&chess);
    }

    // 预先计算每个模式的对象点
    auto makeObj = [&](const Size& s) {
        vector<Point3f> obj;
        for (int y = 0; y < s.height; y++)
            for (int x = 0; x < s.width; x++)
                obj.push_back(Point3f(x * L, y * L, 0));
        return obj;
        };

    // 为每个 pattern 生成对象点
    for (auto p : patterns) {
        p->obj_points.clear();
        p->img_points.clear();
        p->image_files.clear();
        p->used_indices.clear();
    }

    for (size_t idx = 0; idx < files_full.size(); ++idx) {
        string fname = files_full[idx];
        Mat img = imread(fname);
        if (img.empty()) { cout << "Cannot read " << fname << "\n"; continue; }
        if (imageSize.width == 0) imageSize = img.size();
        Mat gray; cvtColor(img, gray, COLOR_BGR2GRAY);
        bool anyFound = false;
        for (size_t pi = 0; pi < patterns.size(); ++pi) {
            auto p = patterns[pi];
            vector<Point2f> corners;
            bool found = false;
            if (!p->useCircles) {
                int flags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
                found = findChessboardCorners(gray, p->pattern, corners, flags);
                if (found) cornerSubPix(gray, corners, Size(5, 5), Size(-1, -1), criteria);
            }
            else {
                int flags = CALIB_CB_SYMMETRIC_GRID;
                found = findCirclesGrid(gray, p->pattern, corners, flags);
            }
            if (found) {
                anyFound = true;
                // 这里应为每张图片都生成对象点
                p->obj_points.push_back(makeObj(p->pattern));
                p->img_points.push_back(corners);
                // 保存用于输出 CSV 的基本文件名，以及用于位姿匹配的 files_full 索引
                string b = basename(files_full[idx]);
                p->image_files.push_back(b);
                p->used_indices.push_back((int)idx);
                cout << "Used image: " << b << " for pattern " << p->name << "\n";
                // 将检测可视化结果保存到调试文件夹
                Mat vis = img.clone();
                if (!p->useCircles) drawChessboardCorners(vis, p->pattern, corners, found);
                else for (auto& pt : corners) circle(vis, pt, 4, Scalar(0, 0, 255), -1);
                string outdet = debug_dir + "/" + p->name + "_" + b;
                imwrite(outdet, vis);
                cout << "Saved detection image to " << outdet << "\n";
                break; // 发现第一个匹配模式后停止
            }
        }
        if (!anyFound) cout << "No pattern detected in " << basename(files_full[idx]) << "\n";
    }

    // 确保至少有一种模式检测到结果
    bool anyDetected = false; for (auto p : patterns) if (!p->obj_points.empty()) anyDetected = true;
    if (!anyDetected) { cerr << "No valid detections for any pattern. Exiting." << endl; return -1; }

    // 读取机器人位姿
    vector<Mat> R_arm_all, t_arm_all;
    string pose_file = (argc > 2) ? string(argv[2]) : (images_dir + "/poses.txt");
    if (!readRobotPoses(pose_file, R_arm_all, t_arm_all)) {
        cerr << "Failed to read robot poses from " << pose_file << "\n";
        return -1;
    }

    // 准备比较结果 CSV
    string compare_out = images_dir + "/handeye_comparison.csv";
    ofstream compofs(compare_out.c_str());
    compofs << "pattern,method,num_images,calib_rms,mean_rot_deg,mean_trans_m\n";

    // 准备每张图像的 rvec/tvec CSV（pattern,image,rvecx,rvecy,rvecz,tx,ty,tz）
    string per_image_out = images_dir + "/handeye_per_image.csv";
    ofstream perofs(per_image_out.c_str());
    perofs << "pattern,image_name,rvec_x,rvec_y,rvec_z,t_x,t_y,t_z\n";

    // 对每种检测到的模式运行摄像机标定并进行手眼标定比较
    for (auto p : patterns) {
        if (p->obj_points.empty()) continue;
        Mat cameraMatrix, distCoeffs; vector<Mat> rvecs, tvecs; //保存标定结果 rvecs：每张图像的旋转向量，tvecs：每张图像的平移向量
        double rms = calibrateCamera(p->obj_points, p->img_points, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
        cout << p->name << " calibrateCamera RMS: " << rms << " images=" << p->img_points.size() << "\n";

        // 保存当前模式的内参和畸变参数
        {
            string intrinsics_out = images_dir + "/intrinsics_" + p->name + ".yml";
            FileStorage fs(intrinsics_out, FileStorage::WRITE);
            fs << "cameraMatrix" << cameraMatrix;
            fs << "distCoeffs" << distCoeffs;
            fs << "rms" << rms;
            fs << "imageWidth" << imageSize.width;
            fs << "imageHeight" << imageSize.height;
            fs << "pattern" << p->name;
            fs.release();
            cout << "Saved intrinsics to " << intrinsics_out << "\n";
        }

        // 构建用于手眼标定的机器人和相机位姿
        // 使用旋转矩阵传入 calibrateHandEye，以避免 OutputArray 大小/类型问题
        vector<Mat> robot_R_mats, robot_tvecs, cam_R_mats, cam_tvecs;
        for (size_t i = 0; i < p->used_indices.size(); ++i) {
            int orig_idx = p->used_indices[i];
            if (orig_idx < 0 || orig_idx >= (int)R_arm_all.size()) { cerr << "Pose index out of range for pattern " << p->name << "\n"; continue; }
            Mat R = R_arm_all[orig_idx];
            if (R.type() != CV_64F) R.convertTo(R, CV_64F);
            robot_R_mats.push_back(R);
            Mat t = t_arm_all[orig_idx]; if (t.type() != CV_64F) t.convertTo(t, CV_64F);
            robot_tvecs.push_back(t);
        }
        for (size_t i = 0; i < rvecs.size(); ++i) {
            Mat Rcam; Rodrigues(rvecs[i], Rcam); if (Rcam.type() != CV_64F) Rcam.convertTo(Rcam, CV_64F);
            cam_R_mats.push_back(Rcam);
            Mat tc = tvecs[i]; if (tc.type() != CV_64F) tc.convertTo(tc, CV_64F);
            cam_tvecs.push_back(tc);
        }

        // 将每张图像的 rvec/tvec 写入 CSV（使用 calibrateCamera 输出的 rvecs）
        for (size_t i = 0; i < rvecs.size() && i < p->image_files.size(); ++i) {
            Vec3d rv, tv;
            if (rvecs[i].total() == 3) {
                Mat rv64; rvecs[i].convertTo(rv64, CV_64F);
                rv = Vec3d(rv64.at<double>(0), rv64.at<double>(1), rv64.at<double>(2));
            }
            else rv = Vec3d(0, 0, 0);
            if (tvecs[i].total() == 3) {
                Mat tv64; tvecs[i].convertTo(tv64, CV_64F);
                tv = Vec3d(tv64.at<double>(0), tv64.at<double>(1), tv64.at<double>(2));
            }
            else tv = Vec3d(0, 0, 0);
            perofs << p->name << "," << p->image_files[i] << "," << rv[0] << "," << rv[1] << "," << rv[2] << "," << tv[0] << "," << tv[1] << "," << tv[2] << "\n";
        }

        // 可用的手眼标定方法
        vector<pair<string, int>> methods;

        methods.push_back(make_pair(string("CALIB_HAND_EYE_TSAI"), CALIB_HAND_EYE_TSAI));
        methods.push_back(make_pair(string("CALIB_HAND_EYE_PARK"), CALIB_HAND_EYE_PARK));
        methods.push_back(make_pair(string("CALIB_HAND_EYE_HORAUD"), CALIB_HAND_EYE_HORAUD));
        methods.push_back(make_pair(string("CALIB_HAND_EYE_DANIILIDIS "), CALIB_HAND_EYE_DANIILIDIS));


        for (auto& m : methods) {
            try {
                Mat Rcam2ee, tcam2ee;

                calibrateHandEye(
                    robot_R_mats, robot_tvecs,
                    cam_R_mats, cam_tvecs,
                    Rcam2ee, tcam2ee,
                    (cv::HandEyeCalibrationMethod)(m.second)
                );

                // 直接构造 X（4x4 变换）
                Mat X = Mat::eye(4, 4, CV_64F);
                Rcam2ee.copyTo(X(Range(0, 3), Range(0, 3)));
                tcam2ee.copyTo(X(Range(0, 3), Range(3, 4)));

                // 在所有位姿对上评估残差
                vector<double> rot_errors, trans_errors; vector<Mat> A_list, B_list;
                for (size_t i = 0; i < robot_R_mats.size(); ++i) A_list.push_back(toHomogeneous(robot_R_mats[i], robot_tvecs[i]));
                for (size_t i = 0; i < cam_R_mats.size(); ++i) B_list.push_back(toHomogeneous(cam_R_mats[i], cam_tvecs[i]));
                for (size_t i = 0; i < B_list.size(); ++i) for (size_t j = i + 1; j < B_list.size(); ++j) {
                    try { auto pr = evalPairResidual(X, A_list[i], A_list[j], B_list[i], B_list[j]); rot_errors.push_back(pr.first); trans_errors.push_back(pr.second); }
                    catch (...) {}
                }
                double mean_rot = 0, mean_tr = 0; if (!rot_errors.empty()) mean_rot = accumulate(rot_errors.begin(), rot_errors.end(), 0.0) / rot_errors.size(); if (!trans_errors.empty()) mean_tr = accumulate(trans_errors.begin(), trans_errors.end(), 0.0) / trans_errors.size();
                compofs << p->name << "," << m.first << "," << p->img_points.size() << "," << rms << "," << mean_rot << "," << mean_tr << "\n";
                cout << p->name << " " << m.first << " mean_rot(deg)=" << mean_rot << " mean_trans(m)=" << mean_tr << "\n";

                // 保存 RT 矩阵到文件
                ostringstream oss;
                oss << images_dir << "/handeye_RT_" << p->name << "_" << m.first << ".txt";
                ofstream rtfs(oss.str().c_str());

                rtfs << fixed << setprecision(10);
                rtfs << "R_cam2ee:\n" << Rcam2ee << "\n\n";
                rtfs << "t_cam2ee:\n" << tcam2ee << "\n\n";
                rtfs << "T_cam2ee (4x4):\n" << X << "\n";

                rtfs.close();

            }
            catch (const Exception& e) { cout << p->name << " " << m.first << " failed: " << e.what() << "\n"; }
        }
    }
    cout << "Wrote comparison CSV to " << compare_out << "\n";
    cout << "Wrote per-image CSV to " << per_image_out << "\n";
    // system("pause");
    return 0;
}  