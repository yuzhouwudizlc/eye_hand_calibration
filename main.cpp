#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <regex>
#include <numeric>
#include <cmath>
#include <errno.h>
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#endif
#include <sstream>
#include <iomanip>
#include <vector>

using namespace std;
using namespace cv;

// =================== ChArUco 参数 ===================
#define SQUARES_X 12
#define SQUARES_Y 9
#define SQUARE_LENGTH 0.04   // meter
#define MARKER_LENGTH 0.02   // meter

// =================== 相机内参（示例） ===================
Mat K = (Mat_<double>(3,3) <<
    1000, 0, 640,
    0, 1000, 360,
    0, 0, 1);
Mat dist = Mat::zeros(1,5,CV_64F);

// =================== 欧拉角 → 旋转矩阵 ===================
static Mat eulerToR(double rx, double ry, double rz)
{
    Mat Rx = (Mat_<double>(3,3) <<
        1,0,0,
        0,cos(rx),-sin(rx),
        0,sin(rx),cos(rx));

    Mat Ry = (Mat_<double>(3,3) <<
        cos(ry),0,sin(ry),
        0,1,0,
        -sin(ry),0,cos(ry));

    Mat Rz = (Mat_<double>(3,3) <<
        cos(rz),-sin(rz),0,
        sin(rz),cos(rz),0,
        0,0,1);

    return Rz * Ry * Rx;
}

// 将 3x3 R 和 3x1 t 转为 4x4 齐次矩阵
static Mat toHomogeneous(const Mat &R, const Mat &t)
{
    Mat T = Mat::eye(4,4,CV_64F);
    Mat R64;
    R.convertTo(R64, CV_64F);
    R64.copyTo(T(Range(0,3), Range(0,3)));
    Mat t64;
    t.convertTo(t64, CV_64F);
    // Ensure t is 3x1
    if (t64.rows == 1 && t64.cols == 3) t64 = t64.t();
    t64.copyTo(T(Range(0,3), Range(3,4)));
    return T;
}

// 评估一对相对变换对的残差：返回 <rotation_deg, translation_m>
static pair<double,double> evalPairResidual(const Mat &X, const Mat &A1, const Mat &A2, const Mat &B1, const Mat &B2)
{
    Mat Aij = A1.inv() * A2;
    Mat Bij = B1.inv() * B2;
    Mat left = Aij * X;
    Mat right = X * Bij;

    Mat Rl = left(Range(0,3), Range(0,3));
    Mat tl = left(Range(0,3), Range(3,4));
    Mat Rr = right(Range(0,3), Range(0,3));
    Mat tr = right(Range(0,3), Range(3,4));

    // rotation difference
    Mat Rdiff = Rl * Rr.t();
    double trace = Rdiff.at<double>(0,0) + Rdiff.at<double>(1,1) + Rdiff.at<double>(2,2);
    double cosang = (trace - 1.0) / 2.0;
    if (cosang > 1.0) cosang = 1.0;
    if (cosang < -1.0) cosang = -1.0;
    double ang = acos(cosang);
    double ang_deg = ang * 180.0 / CV_PI;

    // translation difference
    double trans_err = norm(tl - tr);

    return {ang_deg, trans_err};
}

int main()
{
    // ========== 1. ChArUco 板 ==========
    auto dict = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
    aruco::CharucoBoard board(
        Size(SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH,
        MARKER_LENGTH,
        dict
    );
    aruco::CharucoDetector detector(board);

    // ========== 2. Hand–Eye 数据容器 ==========
    vector<Mat> R_cam_board, t_cam_board;
    vector<Mat> R_base_ee, t_base_ee;

    // ========== 3. 读取机器人位姿 ==========
    // 支持以逗号或空白分隔的每行: x,y,z,rx,ry,rz 或 x y z rx ry rz
    ifstream ifs("collect_data/poses.txt");
    if (!ifs.is_open()) {
        cerr << "Failed to open collect_data/poses.txt" << endl;
        return -1;
    }
    string line;
    while (std::getline(ifs, line)) {
        if (line.find_first_not_of(" \t\r\n") == string::npos) continue;
        // 把逗号换成空格，再用 stringstream 解析
        for (auto &c : line) if (c == ',') c = ' ';
        stringstream ss(line);
        vector<double> vals;
        double v;
        while (ss >> v) vals.push_back(v);
        if (vals.size() < 6) {
            cerr << "Skipping invalid pose line: '" << line << "'" << endl;
            continue;
        }
        double px = vals[0], py = vals[1], pz = vals[2];
        double erx = vals[3], ery = vals[4], erz = vals[5];
        R_base_ee.push_back(eulerToR(erx, ery, erz));
        t_base_ee.push_back((Mat_<double>(3,1) << px, py, pz));
    }

    // ========== 4. 图像处理：按文件自动列出并按编号排序 ==========
    vector<string> files;
    cv::glob("collect_data/images*.jpg", files, false);
    vector<string> tmp;
    cv::glob("collect_data/images*.png", tmp, false);
    files.insert(files.end(), tmp.begin(), tmp.end());

    // 提取文件名中的数字用于自然排序
    auto basename = [](const string &p) {
        size_t pos = p.find_last_of("/\\");
        return (pos == string::npos) ? p : p.substr(pos + 1);
    };
    auto extract_number = [](const string &name) {
        std::smatch m;
        std::regex r("(\\d+)");
        if (std::regex_search(name, m, r)) return stoi(m.str());
        return 0;
    };
    sort(files.begin(), files.end(), [&](const string &a, const string &b) {
        int na = extract_number(basename(a));
        int nb = extract_number(basename(b));
        if (na == nb) return a < b;
        return na < nb;
    });

    cout << "Processing " << files.size() << " images from collect_data" << endl;
    for (size_t idx = 0; idx < files.size(); ++idx) {
        string fname = files[idx];
        Mat img = imread(fname);
        if (img.empty()) { cout << "Cannot read: " << fname << endl; continue; }

        Mat gray; cvtColor(img, gray, COLOR_BGR2GRAY);

        vector<Point2f> ch_corners;
        vector<int> ch_ids;
        vector<vector<Point2f>> mk_corners;
        vector<int> mk_ids;

        detector.detectBoard(gray, ch_corners, ch_ids, mk_corners, mk_ids); // 检测 ChArUco 板
        cout << "Image " << basename(fname) << ": Charuco corners=" << ch_ids.size() << " markers=" << mk_ids.size() << endl;

        // 保存检测可视化到 collect_data/debug/charuco_<name>.png
    #ifdef _WIN32
        _mkdir("collect_data\\debug");
    #else
        mkdir("collect_data/debug", 0755);
    #endif
        Mat vis = img.clone();
        if (!mk_ids.empty()) aruco::drawDetectedMarkers(vis, mk_corners, mk_ids);
        if (!ch_ids.empty()) aruco::drawDetectedCornersCharuco(vis, ch_corners, ch_ids, Scalar(0,255,0));
        string dbgname = string("collect_data/debug/charuco_") + basename(fname);
        size_t dotpos = dbgname.find_last_of('.');
        if (dotpos != string::npos) dbgname = dbgname.substr(0, dotpos);
        dbgname += ".png";
        imwrite(dbgname, vis);
        cout << "  wrote debug image: " << dbgname << endl;

        if (ch_ids.size() < 4) continue; // accept at least 4 corners

        vector<Point3f> objPoints;
        vector<Point2f> imgPoints;
        board.matchImagePoints(ch_corners, ch_ids, objPoints, imgPoints);
        if (objPoints.size() < 4 || objPoints.size() != imgPoints.size()) { cout << "  insufficient points\n"; continue; }

        Vec3d rvec, tvec;
        bool ok = solvePnP(objPoints, imgPoints, K, dist, rvec, tvec, false, SOLVEPNP_ITERATIVE);
        if (!ok) { cout << "  solvePnP failed\n"; continue; }

        Mat rvec_mat = (Mat_<double>(3,1) << rvec[0], rvec[1], rvec[2]);
        Mat tvec_mat = (Mat_<double>(3,1) << tvec[0], tvec[1], tvec[2]);
        Mat R; Rodrigues(rvec_mat, R);
        R_cam_board.push_back(R.clone());
        t_cam_board.push_back(tvec_mat.clone());
    }

    // ========== 5. Hand–Eye 标定 ==========
    Mat R_cam_ee, t_cam_ee;
    // 检查位姿对数量
    if (R_base_ee.size() < 3 || R_cam_board.size() < 3) {
        cerr << "Not enough pose pairs for hand-eye calibration. Need >=3. robot=" << R_base_ee.size() << " cam=" << R_cam_board.size() << endl;
        return -1;
    }

    // 如果 robot/cam 数量不同，取最小长度并截断
    size_t n = min(R_base_ee.size(), R_cam_board.size());
    if (R_base_ee.size() != R_cam_board.size()) {
        cout << "Warning: robot poses (" << R_base_ee.size() << ") and camera poses (" << R_cam_board.size() << ") differ; truncating to " << n << " pairs." << endl;
    }
    vector<Mat> robot_R_mats(R_base_ee.begin(), R_base_ee.begin() + n);
    vector<Mat> robot_tvecs(t_base_ee.begin(), t_base_ee.begin() + n);
    vector<Mat> cam_R_mats(R_cam_board.begin(), R_cam_board.begin() + n);
    vector<Mat> cam_tvecs(t_cam_board.begin(), t_cam_board.begin() + n);

    // 写入比较 CSV
    string compare_out = string("collect_data/handeye_comparison.csv");
    ofstream compofs(compare_out.c_str());
    compofs << "method,num_pairs,mean_rot_deg,mean_trans_m\n";

    // 可用方法集合
    vector<pair<string,int>> methods;
    methods.push_back({"CALIB_HAND_EYE_TSAI", CALIB_HAND_EYE_TSAI});
    methods.push_back({"CALIB_HAND_EYE_PARK", CALIB_HAND_EYE_PARK});
    methods.push_back({"CALIB_HAND_EYE_HORAUD", CALIB_HAND_EYE_HORAUD});
    methods.push_back({"CALIB_HAND_EYE_DANIILIDIS", CALIB_HAND_EYE_DANIILIDIS});

    // 遍历各方法
    for (auto &m : methods) {
        try {
            Mat Rcam2ee, tcam2ee;
            calibrateHandEye(robot_R_mats, robot_tvecs, cam_R_mats, cam_tvecs, Rcam2ee, tcam2ee, (cv::HandEyeCalibrationMethod)(m.second));

            Mat X = Mat::eye(4,4,CV_64F);
            Rcam2ee.copyTo(X(Range(0,3), Range(0,3)));
            tcam2ee.copyTo(X(Range(0,3), Range(3,4)));

            // 计算残差
            vector<double> rot_errors, trans_errors; vector<Mat> A_list, B_list;
            for (size_t i = 0; i < n; ++i) A_list.push_back(toHomogeneous(robot_R_mats[i], robot_tvecs[i]));
            for (size_t i = 0; i < n; ++i) B_list.push_back(toHomogeneous(cam_R_mats[i], cam_tvecs[i]));
            for (size_t i = 0; i < B_list.size(); ++i) for (size_t j = i+1; j < B_list.size(); ++j) {
                try { auto pr = evalPairResidual(X, A_list[i], A_list[j], B_list[i], B_list[j]); rot_errors.push_back(pr.first); trans_errors.push_back(pr.second); } catch(...) {}
            }
            double mean_rot = rot_errors.empty() ? 0.0 : std::accumulate(rot_errors.begin(), rot_errors.end(), 0.0)/rot_errors.size();
            double mean_tr = trans_errors.empty() ? 0.0 : std::accumulate(trans_errors.begin(), trans_errors.end(), 0.0)/trans_errors.size();

            compofs << m.first << "," << n << "," << mean_rot << "," << mean_tr << "\n";
            cout << m.first << " mean_rot(deg)=" << mean_rot << " mean_trans(m)=" << mean_tr << "\n";

            // 保存 RT 文件
            ostringstream oss; oss << "collect_data/handeye_RT_" << m.first << ".txt";
            ofstream rtfs(oss.str().c_str());
            rtfs << fixed << setprecision(10);
            rtfs << "R_cam2ee:\n" << Rcam2ee << "\n\n";
            rtfs << "t_cam2ee:\n" << tcam2ee << "\n\n";
            rtfs << "T_cam2ee (4x4):\n" << X << "\n";
            rtfs.close();
        }
        catch (const Exception &e) {
            cout << "Method " << m.first << " failed: " << e.what() << "\n";
        }
    }

    cout << "Wrote comparison CSV to " << compare_out << "\n";

    return 0;
}
