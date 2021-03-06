#include "opencv2/opencv.hpp"
#include <iostream>

using namespace std;
using namespace cv;

void find_HSign();

int main(void) {
	find_HSign();
	return 0;
}

void find_HSign()
{
	VideoCapture cap(0);
	Mat templ = imread("book.jpg", IMREAD_GRAYSCALE);

	if (!cap.isOpened() || templ.empty()) {
		cerr << "Camera or Template image open failed" << endl;
		return;
	}

	cout << "Frame width: " << cvRound(cap.get(CAP_PROP_FRAME_WIDTH)) << endl;
	cout << "Frame height: " << cvRound(cap.get(CAP_PROP_FRAME_HEIGHT)) << endl;
	cout << "Template Size: " << templ.size << endl;
	

	int w = cvRound(cap.get(CAP_PROP_FRAME_WIDTH));
	int h = cvRound(cap.get(CAP_PROP_FRAME_HEIGHT));
	double fps = cap.get(CAP_PROP_FPS);

	int fourcc = VideoWriter::fourcc('D', 'X', '5', '0');
	int delay = cvRound(1000 / fps);

	// 동영상 파일 저장

	VideoWriter writer;
	writer.open("test.mpeg", fourcc, fps, Size(w + templ.cols, h));

	Ptr<Feature2D> orb = ORB::create();
	vector<KeyPoint> keypoints_templ; // template의 특징점과 기술자 먼저 계산
	Mat desc_templ;
	orb->detectAndCompute(templ, Mat(), keypoints_templ, desc_templ);

	Mat img, img_color;
	while (true) {
		cap >> img_color;
		if (img_color.empty()) break;
		cvtColor(img_color, img, COLOR_BGR2GRAY);

		vector<KeyPoint> keypoints_img; // 각 프레임(img) 별로 특징점과 기술자 계산
		Mat desc_img;
		orb->detectAndCompute(img, Mat(), keypoints_img, desc_img);

		Ptr<DescriptorMatcher> matcher = BFMatcher::create(NORM_HAMMING);

		vector<DMatch> matches;
		matcher->match(desc_templ, desc_img, matches);

		std::sort(matches.begin(), matches.end());
		vector<DMatch> good_matches(matches.begin(), matches.begin() + 50);

		Mat dst;
		drawMatches(templ, keypoints_templ, img, keypoints_img, good_matches, dst, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		vector<Point2f> pts1, pts2;
		for (size_t i = 0; i < good_matches.size(); i++) {
			pts1.push_back(keypoints_templ[good_matches[i].queryIdx].pt); //keyoints_templ에는 query 특징점이 저장되어 있다. 이 특징점을 pts1에 저장한다.
			pts2.push_back(keypoints_img[good_matches[i].trainIdx].pt);
		}

		Mat H = findHomography(pts1, pts2, RANSAC);

		vector<Point2f> corners1, corners2;
		corners1.push_back(Point2f(0, 0));
		corners1.push_back(Point2f(templ.cols - 1.f, 0));
		corners1.push_back(Point2f(templ.cols - 1.f, templ.rows - 1.f));
		corners1.push_back(Point2f(0, templ.rows - 1.f));
		perspectiveTransform(corners1, corners2, H);

		vector<Point> corners_dst;
		for (Point2f pt : corners2) {
			corners_dst.push_back(Point(cvRound(pt.x + templ.cols), cvRound(pt.y)));
		}

		polylines(dst, corners_dst, true, Scalar(0, 255, 0), 2, LINE_AA);

		writer << dst;

		imshow("dst", dst);

		if (waitKey(10) == 27) break;
	}
}
