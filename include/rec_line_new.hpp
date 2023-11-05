#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "../include/EDLib.h"
using namespace std;
using namespace cv;
// ���ڽ���
Mat transformCorner(Mat src, RotatedRect rect);
Mat transformQRcode(Mat src, RotatedRect rect, double angle);
// �����жϽǵ�
bool IsQrPoint(vector<Point>& contour, Mat& img);
bool isCorner(Mat& image);
double Rate(Mat& count);
int leftTopPoint(vector<Point> centerPoint);
vector<int> otherTwoPoint(vector<Point> centerPoint, int leftTopPointIndex);
double rotateAngle(Point leftTopPoint, Point rightTopPoint, Point leftBottomPoint);
void recbarcode(Mat& in_img, bool& isapp, int& j);
Point recqrcode(Mat& in_img, bool& isapp, int& j);
//otherTwoPointIndex���ض�ά���Ӧ������

Mat transformCorner(Mat src, RotatedRect rect)
{
	// �����ת����
	Point center = rect.center;
	// ������ϽǺ����½ǵĽǵ㣬����Ҫ��֤������ͼƬ��Χ�����ڿ�ͼ
	Point TopLeft = Point(cvRound(center.x), cvRound(center.y)) - Point(rect.size.height / 2, rect.size.width / 2);  //��ת���Ŀ��λ��
	TopLeft.x = TopLeft.x > src.cols ? src.cols : TopLeft.x;
	TopLeft.x = TopLeft.x < 0 ? 0 : TopLeft.x;
	TopLeft.y = TopLeft.y > src.rows ? src.rows : TopLeft.y;
	TopLeft.y = TopLeft.y < 0 ? 0 : TopLeft.y;

	int after_width, after_height;
	if (TopLeft.x + rect.size.width > src.cols) {
		after_width = src.cols - TopLeft.x - 1;
	}
	else {
		after_width = rect.size.width - 1;
	}
	if (TopLeft.y + rect.size.height > src.rows) {
		after_height = src.rows - TopLeft.y - 1;
	}
	else {
		after_height = rect.size.height - 1;
	}
	// ��ö�ά���λ��
	Rect RoiRect = Rect(TopLeft.x, TopLeft.y, after_width, after_height);

	//	dst�Ǳ���ת��ͼƬ roiΪ���ͼƬ maskΪ��ģ
	double angle = rect.angle;
	Mat mask, roi, dst;
	Mat image;
	// �����н�ͼ��������ͼ��

	vector<Point> contour;
	// ��þ��ε��ĸ���
	Point2f points[4];
	rect.points(points);
	for (int i = 0; i < 4; i++)
		contour.push_back(points[i]);

	vector<vector<Point>> contours;
	contours.push_back(contour);
	// ���н�ͼ���л�������
	drawContours(mask, contours, 0, Scalar(255, 255, 255), -1);
	// ͨ��mask��Ĥ��src���ض�λ�õ����ؿ�����dst�С�
	src.copyTo(dst, mask);
	// ��ת
	Mat M = getRotationMatrix2D(center, angle, 1);
	warpAffine(dst, image, M, src.size());
	// ��ͼ
	roi = image(RoiRect);

	return roi;
}

// �ò������ڼ���Ƿ��ǽǵ㣬�����������������
bool IsQrPoint(vector<Point>& contour, Mat& img) {
	double area = contourArea(contour);
	// �ǵ㲻����̫С
	if (area < 30)
		return 0;
	RotatedRect rect = minAreaRect(Mat(contour));
	double w = rect.size.width;
	double h = rect.size.height;
	double rate = min(w, h) / max(w, h);
	if (rate > 0.7)
	{
		// ������ת���ͼƬ�����ڰѡ��ء����������ڴ���
		Mat image = transformCorner(img, rect);
		if (isCorner(image))
		{
			return 1;
		}
	}
	return 0;
}

// �����ڲ����а�ɫ����ռȫ���ı���
double Rate(Mat& count)
{
	int number = 0;
	int allpixel = 0;
	for (int row = 0; row < count.rows; row++)
	{
		for (int col = 0; col < count.cols; col++)
		{
			if (count.at<uchar>(row, col) == 255)
			{
				number++;
			}
			allpixel++;
		}
	}
	//cout << (double)number / allpixel << endl;
	return (double)number / allpixel;
}

// �����ж��Ƿ����ڽ��ϵ�������
bool isCorner(Mat& image)
{
	// ����mask
	Mat imgCopy, dstCopy;
	Mat dstGray;
	imgCopy = image.clone();
	// ת��Ϊ�Ҷ�ͼ��
	cvtColor(image, dstGray, COLOR_BGR2GRAY);
	// ���ж�ֵ��

	threshold(dstGray, dstGray, 0, 255, THRESH_BINARY | THRESH_OTSU);
	dstCopy = dstGray.clone();  //����

	// �ҵ������봫�ݹ�ϵ
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(dstCopy, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);


	for (int i = 0; i < contours.size(); i++)
	{
		//cout << i << endl;
		if (hierarchy[i][2] == -1 && hierarchy[i][3])
		{
			Rect rect = boundingRect(Mat(contours[i]));
			rectangle(image, rect, Scalar(0, 0, 255), 2);
			// ������ľ�����������ľ��εĶԱ�
			if (rect.width < imgCopy.cols * 2 / 7)      //2/7��Ϊ�˷�ֹһЩ΢С�ķ���
				continue;
			if (rect.height < imgCopy.rows * 2 / 7)      //2/7��Ϊ�˷�ֹһЩ΢С�ķ���
				continue;
			// �ж����к�ɫ���ɫ�Ĳ��ֵı���
			if (Rate(dstGray) > 0.20)
			{
				return true;
			}
		}
	}
	return  false;
}

int leftTopPoint(vector<Point> centerPoint) {
	int minIndex = 0;
	int multiple = 0;
	int minMultiple = 10000;
	multiple = (centerPoint[1].x - centerPoint[0].x) * (centerPoint[2].x - centerPoint[0].x) + (centerPoint[1].y - centerPoint[0].y) * (centerPoint[2].y - centerPoint[0].y);
	if (minMultiple > multiple) {
		minIndex = 0;
		minMultiple = multiple;
	}
	multiple = (centerPoint[0].x - centerPoint[1].x) * (centerPoint[2].x - centerPoint[1].x) + (centerPoint[0].y - centerPoint[1].y) * (centerPoint[2].y - centerPoint[1].y);
	if (minMultiple > multiple) {
		minIndex = 1;
		minMultiple = multiple;
	}
	multiple = (centerPoint[0].x - centerPoint[2].x) * (centerPoint[1].x - centerPoint[2].x) + (centerPoint[0].y - centerPoint[2].y) * (centerPoint[1].y - centerPoint[2].y);
	if (minMultiple > multiple) {
		minIndex = 2;
		minMultiple = multiple;
	}
	return minIndex;
}

vector<int> otherTwoPoint(vector<Point> centerPoint, int leftTopPointIndex) {
	vector<int> otherIndex;
	double waiji = (centerPoint[(leftTopPointIndex + 1) % 3].x - centerPoint[(leftTopPointIndex) % 3].x) *
		(centerPoint[(leftTopPointIndex + 2) % 3].y - centerPoint[(leftTopPointIndex) % 3].y) -
		(centerPoint[(leftTopPointIndex + 2) % 3].x - centerPoint[(leftTopPointIndex) % 3].x) *
		(centerPoint[(leftTopPointIndex + 1) % 3].y - centerPoint[(leftTopPointIndex) % 3].y);
	if (waiji > 0) {
		otherIndex.push_back((leftTopPointIndex + 1) % 3);
		otherIndex.push_back((leftTopPointIndex + 2) % 3);
	}
	else {
		otherIndex.push_back((leftTopPointIndex + 2) % 3);
		otherIndex.push_back((leftTopPointIndex + 1) % 3);
	}
	return otherIndex;
}

double rotateAngle(Point leftTopPoint, Point rightTopPoint, Point leftBottomPoint) {
	double dy = rightTopPoint.y - leftTopPoint.y;
	double dx = rightTopPoint.x - leftTopPoint.x;
	double k = dy / dx;
	double angle = atan(k) * 180 / CV_PI;//ת���Ƕ�
	if (leftBottomPoint.y < leftTopPoint.y)
		angle -= 180;
	return angle;
}

Mat transformQRcode(Mat src, RotatedRect rect, double angle)
{
	// �����ת����
	Point center = rect.center;
	// ������ϽǺ����½ǵĽǵ㣬����Ҫ��֤������ͼƬ��Χ�����ڿ�ͼ
	Point TopLeft = Point(cvRound(center.x), cvRound(center.y)) - Point(rect.size.height / 2, rect.size.width / 2);  //��ת���Ŀ��λ��
	TopLeft.x = TopLeft.x > src.cols ? src.cols : TopLeft.x;
	TopLeft.x = TopLeft.x < 0 ? 0 : TopLeft.x;
	TopLeft.y = TopLeft.y > src.rows ? src.rows : TopLeft.y;
	TopLeft.y = TopLeft.y < 0 ? 0 : TopLeft.y;

	int after_width, after_height;
	if (TopLeft.x + rect.size.width > src.cols) {
		after_width = src.cols - TopLeft.x - 1;
	}
	else {
		after_width = rect.size.width - 1;
	}
	if (TopLeft.y + rect.size.height > src.rows) {
		after_height = src.rows - TopLeft.y - 1;
	}
	else {
		after_height = rect.size.height - 1;
	}
	// ��ö�ά���λ��
	Rect RoiRect = Rect(TopLeft.x, TopLeft.y, after_width, after_height);

	// dst�Ǳ���ת��ͼƬ��roiΪ���ͼƬ��maskΪ��ģ
	Mat mask, roi, dst;
	Mat image;
	// �����н�ͼ��������ͼ��

	vector<Point> contour;
	// ��þ��ε��ĸ���
	Point2f points[4];
	rect.points(points);
	for (int i = 0; i < 4; i++)
		contour.push_back(points[i]);

	vector<vector<Point>> contours;
	contours.push_back(contour);
	// ���н�ͼ���л�������
	drawContours(mask, contours, 0, Scalar(255, 255, 255), -1);
	// ͨ��mask��Ĥ��src���ض�λ�õ����ؿ�����dst�С�
	src.copyTo(dst, mask);
	// ��ת
	Mat M = getRotationMatrix2D(center, angle, 1);
	warpAffine(dst, image, M, src.size());
	// ��ͼ
	roi = image(RoiRect);

	return roi;
}
std::vector<cv::Point> get_line_erect(cv::Mat& in_img,bool& isapp,bool& isappbarcode,bool& isappqrcode,int& bar_num,int& qr_num)
{
	//isapp = false;
	//isappbarcode = false;
	//isappqrcode = false;
	std::vector<cv::Point> vec1;
	//// ��ȡͼ��
	//Mat image = imread("D:/Desktop/zuanquan/img/line6.jpg");
	Mat testImg;
	cv::Mat imghsv;
	cv::cvtColor(in_img, imghsv, cv::COLOR_BGR2HSV);
	cv::inRange(imghsv, cv::Scalar(125, 43, 46), cv::Scalar(155, 255, 255), testImg); //紫色
	//cv::inRange(imghsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 30), testImg); //黑色
	//imshow("123", testImg);
	Mat gray = testImg;
	//cvtColor(image, gray, COLOR_BGR2GRAY);
	//gray = 255 - gray;
	// ʹ��Theil-Sen�㷨���ֱ��
	vector<Point> points;
	cv::Point point1, point2;
	int opt = 0;
	if(isappbarcode && bar_num > 10 && qr_num < 11)
	{
		recqrcode(in_img,isappqrcode,qr_num);
	}
	for (int i = 0; i < gray.cols; i++) {
	//cout<< sum(gray.col(i))[0] <<endl;

		if (sum(gray.col(i))[0] > 50000)
		{
			opt = 2;
		//notfind = false;
			//cout << sum(gray.col(i))[0] << endl;
			for (int j = 0; j < gray.rows; j++) {

				if (gray.at<uchar>(j, i) > 128) {
					points.push_back(Point(i, j));
				}
			}
				//int c = 0;
		}
			//if(sum(gray.col(i))[0] <= 10000 && !notfind )
			//{
			//	//find_over = true;
			//	break;
			//}
	}
	//cv::Point point1, point2;

	if (!points.size())
	{
		isapp = true;
		if(bar_num<11)
		{
			recbarcode(in_img,isappbarcode,bar_num);
		}		
		opt = 1;
		for (int i = 0; i < gray.cols; i++) {
				//cout<< sum(gray.col(i))[0] <<endl;
			for (int j = 0; j < gray.rows; j++) {

				if (gray.at<uchar>(j, i) > 128) {
					points.push_back(Point(i, j));
				}
			}
		}
	}
	cv::Vec4f line_para;
	if(points.size())
	{
	//isapp = true;
	cv::fitLine(points, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);
	//std::cout << "line_para = " << line_para << std::endl;

	//��ȡ��бʽ�ĵ��б��
	cv::Point point0;
	point0.x = line_para[2];
	point0.y = line_para[3];

	double k = line_para[1] / line_para[0];
	cv::Point pointk;
	pointk.x = line_para[0]*100;
	pointk.y = line_para[1]*100;
	//����ֱ�ߵĶ˵�(y = k(x - x0) + y0)
	cv::Point point1, point2, point3;
	if(opt == 1)
	{
		point1.x = 0;
		point1.y = k * (0 - point0.x) + point0.y;
		point2.x = 640;
		point2.y = k * (640 - point0.x) + point0.y;
		point3.x = 1;
		point3.y = 0;
	}	
	if(opt == 2)
	{
		point1.x = (0-point0.y)/k+point0.x;
		point1.y = 0;
		point2.x = (480-point0.y)/k+point0.x;;
		point2.y = 480;	
		point3.x = 2;
		point3.y = 0;
	}
	if(opt == 0)
	{
		point1.x = 0;
		point1.y = 0;
		point2.x = 0;
		point2.y = 0;
		point3.x = 0;
		point3.y = 0;	
	}
	//cv::Point point1, point2;
	//point1.x = 0;
	//point1.y = k * (0 - point0.x) + point0.y;
	//point2.x = 640;
	//point2.y = k * (640 - point0.x) + point0.y;

	cv::line(in_img, point1, point2, cv::Scalar(0, 255, 0), 2, 8, 0);
	
	cv::Point point_mid;
	point_mid.x = (point1.x + point2.x) / 2;
	point_mid.y = (point1.y + point2.y) / 2;
	std::vector<cv::Point> vec;
	vec.push_back(point_mid);
	vec.push_back(pointk);
	vec.push_back(point3);
	cv::imshow("image", in_img);
	cv::waitKey(30);
	isapp = true;
	return vec;
}
return vec1;
}

void recbarcode(Mat& in_img, bool& isapp, int& j)
{
	std::cout << "xunzao " << std::endl;
	//isapp = false;
	Mat testImg;
	cv::Mat imghsv;
	cv::cvtColor(in_img, imghsv, cv::COLOR_BGR2HSV);
	cv::inRange(imghsv, cv::Scalar(26, 43, 46), cv::Scalar(34, 255, 255), testImg); //��ɫ
	ED testED = ED(testImg, SOBEL_OPERATOR, 36, 8, 1, 10, 1.0, true); // apply ED algorithm
	EDLines testEDLines = EDLines(testED);
	std::vector<LS> lines = testEDLines.getLines();
	int noline = lines.capacity();
	//if (noline > 10 && lines[0].start.x > 100 && lines[0].start.x < 300)
	if (noline > 10)
	{
		isapp = true;
		if (j > 0 && j < 10)
		{
			std::cout << "paizao " << std::endl;
			cv::imwrite("/home/liuqing/acfly_ws11/src/line-tracing/img/" + std::to_string(j) + ".jpg", in_img);
			j++;
		}
		
	}
}

Point recqrcode(Mat& in_img, bool& isapp, int& j)
{
	//isapp = false;
	Mat srcCopy = in_img.clone();
	//canvasΪ���� ���ҵ��Ķ�λ����������
	Mat canvas;
	canvas = Mat::zeros(in_img.size(), CV_8UC3);
	Mat srcGray;
	//center_all��ȡ��������
	vector<Point> center_all;
	// ת��Ϊ�Ҷ�ͼ
	cvtColor(in_img, srcGray, COLOR_BGR2GRAY);
	// 3X3ģ��
	//blur(srcGray, srcGray, Size(3, 3));
	// ����ֱ��ͼ
	convertScaleAbs(in_img, in_img);
	equalizeHist(srcGray, srcGray);
	int s = srcGray.at<Vec3b>(0, 0)[0];
	// ������ֵ����ʵ����� ����ͼ�����Ҳ������� ����������
	threshold(srcGray, srcGray, 25, 255, THRESH_BINARY);
	imshow("threshold", srcGray);
	/*contours�ǵ�һ��Ѱ������*/
	/*contours2��ɸѡ��������*/
	vector<vector<Point>> contours;
	//	�����������
	vector<Vec4i> hierarchy;
	findContours(srcGray, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
	// С���������
	int numOfRec = 0;
	// ��ⷽ��
	int ic = 0;
	int parentIdx = -1;
	for (int i = 0; i < contours.size(); i++)
	{
		if (hierarchy[i][2] != -1 && ic == 0)
		{
				parentIdx = i;
				ic++;
		}
		else if (hierarchy[i][2] != -1)
		{
				ic++;
		}
		else if (hierarchy[i][2] == -1)
		{
				parentIdx = -1;
				ic = 0;
		}
		if (ic >= 2 && ic <= 2)
		{
			if (IsQrPoint(contours[parentIdx], in_img)) 
			{
				RotatedRect rect = minAreaRect(Mat(contours[parentIdx]));

				// ��ͼ����
				Point2f points[4];
				rect.points(points);
				for (int j = 0; j < 4; j++) {
					line(in_img, points[j], points[(j + 1) % 4], Scalar(0, 255, 0), 2);
				}
				drawContours(canvas, contours, parentIdx, Scalar(0, 0, 255), -1);

				// ����������������
				center_all.push_back(rect.center);
				numOfRec++;
			}
				ic = 0;
				parentIdx = -1;
		}
	}
	if(center_all.size()>=3)
	{
		int leftTopPointIndex = leftTopPoint(center_all);
		Point leftTop = center_all[leftTopPointIndex];
		isapp = true;
		if (j > 0 && j < 10)
		{
			//std::cout << "paizao " << std::endl;
			cv::imwrite("/home/liuqing/acfly_ws11/src/line-tracing/img/" + std::to_string(j)+"qrcode.jpg", in_img);
			j++;
		}
		return leftTop;
	}

}

