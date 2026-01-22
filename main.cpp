#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <direct.h>
#include <regex>
#include <numeric>
#include <cmath>
#include <iomanip>

using namespace std;
using namespace cv;
// no std::filesystem: use OpenCV glob and simple path helpers for VS2015

static string basename(const string &p) {
	size_t pos = p.find_last_of("/\\\\");
	if (pos == string::npos) return p;
	return p.substr(pos + 1);
}

static void eulerToRotation(double rx, double ry, double rz, Mat &R) {
	// Same order as Python: R = Rz * Ry * Rx
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

static Mat toHomogeneous(const Mat &R, const Mat &t) {
	Mat H = Mat::eye(4, 4, CV_64F);
	R.convertTo(H(Range(0, 3), Range(0, 3)), CV_64F);
	H(Range(0, 3), Range(3, 4)) = t.clone();
	return H;
}

// Natural sort key: find first number in filename
static int naturalKey(const string &s) {
	smatch m;
	regex r("\\d+");
	if (regex_search(s, m, r)) return stoi(m.str());
	return -1;
}

// Read robot poses in collect_data/poses.txt format: x,y,z,rx,ry,rz per line
static bool readRobotPoses(const string &path, vector<Mat> &R_out, vector<Mat> &t_out) {
	ifstream ifs(path);
	if (!ifs) return false;
	string line;
	while (getline(ifs, line)) {
		if (line.find_first_not_of(" \t\r\n") == string::npos) continue;
		// split by comma or whitespace
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

// Evaluate a single pair residual following the Python code
static pair<double, double> evalPairResidual(const Mat &X, const Mat &A_i, const Mat &A_j, const Mat &B_i, const Mat &B_j) {
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
	// Run full pipeline using workspace-local collect_data by default
	string root = string("../collect_data");
	// debug output folder: collect_data/debug
	string debug_dir = root + "/debug";
	// create debug directory if not exists
	_mkdir(debug_dir.c_str());

	// Define detection patterns: chessboard (11x8) and symmetric circle grids (3x3,3x4)
	struct PatternSet {
		string name;
		Size pattern;
		bool useCircles;
		vector<vector<Point3f>> obj_points;
		vector<vector<Point2f>> img_points;
		vector<string> image_files;
		vector<int> used_indices; // index in files list
	};

	const double L = 0.035; // meters
	PatternSet chess;
	chess.name = "chessboard_11x8"; chess.pattern = Size(11, 8); chess.useCircles = false;
	PatternSet circ33; circ33.name = "circles_3x3"; circ33.pattern = Size(3, 3); circ33.useCircles = true;
	PatternSet circ34; circ34.name = "circles_3x4"; circ34.pattern = Size(3, 4); circ34.useCircles = true;
	vector<PatternSet*> patterns = { &chess, &circ33, &circ34 };

	// precompute object points for each pattern
	auto makeObj = [&](const Size &s) { vector<Point3f> obj; for (int y = 0; y<s.height; y++) for (int x = 0; x<s.width; x++) obj.push_back(Point3f(x*L, y*L, 0)); return obj; };
	vector<vector<Point3f>> patternObjs;
	for (auto p : patterns) patternObjs.push_back(makeObj(p->pattern));

	// list images using OpenCV glob (returns full paths)
	vector<string> files_full, tmp;
	cv::glob(root + "/*.jpg", files_full, false);
	cv::glob(root + "/*.png", tmp, false);
	files_full.insert(files_full.end(), tmp.begin(), tmp.end());
	// sort by natural key on basename
	sort(files_full.begin(), files_full.end(), [](const string &pa, const string &pb) {
		return naturalKey(basename(pa)) < naturalKey(basename(pb));
	});

	cout << "Found images: " << files_full.size() << "\n";
	Size imageSize;
	TermCriteria criteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.001);

	for (size_t idx = 0; idx<files_full.size(); ++idx) {
		string fname = files_full[idx];
		Mat img = imread(fname);
		if (img.empty()) { cout << "Cannot read " << fname << "\n"; continue; }
		if (imageSize.width == 0) imageSize = img.size();
		Mat gray; cvtColor(img, gray, COLOR_BGR2GRAY);
		bool anyFound = false;
		for (size_t pi = 0; pi<patterns.size(); ++pi) {
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
				p->obj_points.push_back(patternObjs[pi]);
				p->img_points.push_back(corners);
				// store basename for output CSVs, and index into files_full for pose matching
				string b = basename(files_full[idx]);
				p->image_files.push_back(b);
				p->used_indices.push_back((int)idx);
				cout << "Used image: " << b << " for pattern " << p->name << "\n";
				// save detection visualization to debug folder
				Mat vis = img.clone();
				if (!p->useCircles) drawChessboardCorners(vis, p->pattern, corners, found);
				else for (auto &pt : corners) circle(vis, pt, 4, Scalar(0, 0, 255), -1);
				string outdet = debug_dir + "/" + p->name + "_" + b;
				imwrite(outdet, vis);
				cout << "Saved detection image to " << outdet << "\n";
				break; // stop after first successful pattern
			}
		}
		if (!anyFound) cout << "No pattern detected in " << basename(files_full[idx]) << "\n";
	}

	// Ensure at least one detection across patterns
	bool anyDetected = false; for (auto p : patterns) if (!p->obj_points.empty()) anyDetected = true;
	if (!anyDetected) { cerr << "No valid detections for any pattern. Exiting." << endl; return -1; }

	// Read robot poses
	vector<Mat> R_arm_all, t_arm_all;
	string pose_file = root + "/poses.txt";
	if (!readRobotPoses(pose_file, R_arm_all, t_arm_all)) {
		cerr << "Failed to read robot poses from " << pose_file << "\n";
		return -1;
	}

	// Prepare comparison CSV
	string compare_out = root + "/handeye_comparison.csv";
	ofstream compofs(compare_out.c_str());
	compofs << "pattern,method,num_images,calib_rms,mean_rot_deg,mean_trans_m\n";

	// Prepare per-image CSV (pattern,image,rvecx,rvecy,rvecz,tx,ty,tz)
	string per_image_out = root + "/handeye_per_image.csv";
	ofstream perofs(per_image_out.c_str());
	perofs << "pattern,image_name,rvec_x,rvec_y,rvec_z,t_x,t_y,t_z\n";

	// For each detected pattern, run calibrateCamera and hand-eye comparisons
	for (auto p : patterns) {
		if (p->obj_points.empty()) continue;
		Mat cameraMatrix, distCoeffs; vector<Mat> rvecs, tvecs;
		double rms = calibrateCamera(p->obj_points, p->img_points, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
		cout << p->name << " calibrateCamera RMS: " << rms << " images=" << p->img_points.size() << "\n";

		// build robot and camera poses for handeye
		// Use rotation matrices for calibrateHandEye to avoid OutputArray size/type issues
		vector<Mat> robot_R_mats, robot_tvecs, cam_R_mats, cam_tvecs;
		for (size_t i = 0; i<p->used_indices.size(); ++i) {
			int orig_idx = p->used_indices[i];
			if (orig_idx < 0 || orig_idx >= (int)R_arm_all.size()) { cerr << "Pose index out of range for pattern " << p->name << "\n"; continue; }
			Mat R = R_arm_all[orig_idx];
			if (R.type() != CV_64F) R.convertTo(R, CV_64F);
			robot_R_mats.push_back(R);
			Mat t = t_arm_all[orig_idx]; if (t.type() != CV_64F) t.convertTo(t, CV_64F);
			robot_tvecs.push_back(t);
		}
		for (size_t i = 0; i<rvecs.size(); ++i) {
			Mat Rcam; Rodrigues(rvecs[i], Rcam); if (Rcam.type() != CV_64F) Rcam.convertTo(Rcam, CV_64F);
			cam_R_mats.push_back(Rcam);
			Mat tc = tvecs[i]; if (tc.type() != CV_64F) tc.convertTo(tc, CV_64F);
			cam_tvecs.push_back(tc);
		}

		// write per-image rvec/tvec to CSV (use rvecs from calibrateCamera output)
		for (size_t i = 0; i<rvecs.size() && i<p->image_files.size(); ++i) {
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

		// available methods (only add those defined in this OpenCV version)
		vector<pair<string, int>> methods;

		methods.push_back(make_pair(string("CALIB_HAND_EYE_TSAI"), CALIB_HAND_EYE_TSAI));
		methods.push_back(make_pair(string("CALIB_HAND_EYE_PARK"), CALIB_HAND_EYE_PARK));
		methods.push_back(make_pair(string("CALIB_HAND_EYE_HORAUD"), CALIB_HAND_EYE_HORAUD));
		methods.push_back(make_pair(string("CALIB_HAND_EYE_DANIILIDIS "), CALIB_HAND_EYE_DANIILIDIS));


		for (auto &m : methods) {
			try {
				Mat Rcam2ee, tcam2ee;

				calibrateHandEye(
					robot_R_mats, robot_tvecs,
					cam_R_mats, cam_tvecs,
					Rcam2ee, tcam2ee,
					(cv::HandEyeCalibrationMethod)(m.second)
					);

				// ? 直接构造 X
				Mat X = Mat::eye(4, 4, CV_64F);
				Rcam2ee.copyTo(X(Range(0, 3), Range(0, 3)));
				tcam2ee.copyTo(X(Range(0, 3), Range(3, 4)));

				// evaluate residuals across pairs
				vector<double> rot_errors, trans_errors; vector<Mat> A_list, B_list;
				for (size_t i = 0; i<robot_R_mats.size(); ++i) A_list.push_back(toHomogeneous(robot_R_mats[i], robot_tvecs[i]));
				for (size_t i = 0; i<cam_R_mats.size(); ++i) B_list.push_back(toHomogeneous(cam_R_mats[i], cam_tvecs[i]));
				for (size_t i = 0; i<B_list.size(); ++i) for (size_t j = i + 1; j<B_list.size(); ++j) {
					try { auto pr = evalPairResidual(X, A_list[i], A_list[j], B_list[i], B_list[j]); rot_errors.push_back(pr.first); trans_errors.push_back(pr.second); }
					catch (...) {}
				}
				double mean_rot = 0, mean_tr = 0; if (!rot_errors.empty()) mean_rot = accumulate(rot_errors.begin(), rot_errors.end(), 0.0) / rot_errors.size(); if (!trans_errors.empty()) mean_tr = accumulate(trans_errors.begin(), trans_errors.end(), 0.0) / trans_errors.size();
				compofs << p->name << "," << m.first << "," << p->img_points.size() << "," << rms << "," << mean_rot << "," << mean_tr << "\n";
				cout << p->name << " " << m.first << " mean_rot(deg)=" << mean_rot << " mean_trans(m)=" << mean_tr << "\n";


				
				// 保存 RT 矩阵
				ostringstream oss;
				oss << root << "/handeye_RT_" << p->name << "_" << m.first << ".txt";
				ofstream rtfs(oss.str().c_str());

				rtfs << fixed << setprecision(10);
				rtfs << "R_cam2ee:\n" << Rcam2ee << "\n\n";
				rtfs << "t_cam2ee:\n" << tcam2ee << "\n\n";
				rtfs << "T_cam2ee (4x4):\n" << X << "\n";

				rtfs.close();

			}
			catch (const Exception &e) { cout << p->name << " " << m.first << " failed: " << e.what() << "\n"; }
		}
	}
	cout << "Wrote comparison CSV to " << compare_out << "\n";
	cout << "Wrote per-image CSV to " << per_image_out << "\n";
	// system("pause");
	return 0;
}
