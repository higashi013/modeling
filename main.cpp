#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <stdio.h>
#include <direct.h>
#include <string.h>
#include <fstream>
#include <vector>
#include <math.h>
#include <numeric>
#include <time.h>
#include <unordered_set>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
//#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/opencv.hpp>
#include <opencv2/superres/optical_flow.hpp>
//#include <opencv2/ximgproc.hpp>
#include <opencv2/optflow.hpp>
//#include <opencv2/ximgproc/seeds.hpp>
//ここまで
#include <opencv2/ximgproc.hpp>
using namespace std;
typedef cv::Vec<uchar,3> Vec3b;

// ビューワー起動時の一回だけ呼ばれる
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0.2, 0.2, 0.2); //black0-white1 default=0.2
	cout << "viewerOneOff" << std::endl;
}

// ビューワー起動中の毎フレーム実行される
//void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
//{
//	cout << "viewerPsycho" << std::endl;
//}
void print4x4Matrix(const Eigen::Matrix4d & matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void color_filter(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2) {

	for (int h = 0; h < cloud1->height; h++) {
		for (int w = 0; w < cloud1->width; w++) {

			//depth ga toretenai mataha object no ryouiki janai tennha mushi
			if (((cloud1->points[w + h * cloud1->width].r == 255) && (cloud1->points[w + h * cloud1->width].g == 255) && (cloud1->points[w + h * cloud1->width].b == 255))|| (cloud1->points[w + h * cloud1->width].z < 50) || (h < 30) || (460 < h) || (w < 30) || (580 < w)) {
				continue;
			}
			else {
				//			pcl::PointXYZRGBA &point1 = cloud1->points[w + h * cloud1->width];
				pcl::PointXYZRGBA &point = cloud2->points[w + h * cloud1->width];
				point.x = cloud1->points[w + h * cloud1->width].x;
				point.y = cloud1->points[w + h * cloud1->width].y;
				point.z = cloud1->points[w + h * cloud1->width].z;
				point.r = cloud1->points[w + h * cloud1->width].r;
				point.g = cloud1->points[w + h * cloud1->width].g;
				point.b = cloud1->points[w + h * cloud1->width].b;
				point.a = 0;
			}
		}
	}

//	return;
}

int xrand(std::unordered_set<int> &history, int max) {
	int num = max - history.size();
	if (num <= 0) return -1;

	// 乱数生成はマシなやつ使ったほうがいいかも
	int ret = rand() % num;

	int skip = 0;
	for (auto i : history) {
		if (i <= ret) skip++;
	}

	while (0 < skip) {
		ret++;
		if (history.find(ret) == history.end()) skip--;
	}
	history.insert(ret);
	return ret;
}


int decrease_points(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud, int count1, int count2) {
	
	int pix_num;
	int count = 0;

	if (count1 >= count2) {
		std::unordered_set<int> history;
		for (int i = 0; i < count2; i++) {
//			pix_num = xrand(history, count1);
			pcl::PointXYZRGBA &point = new_cloud->points[i];
			point.x = cloud1->points[i].x;
			point.y = cloud1->points[i].y;
			point.z = cloud1->points[i].z;
			point.r = cloud1->points[i].r;
			point.g = cloud1->points[i].g;
			point.b = cloud1->points[i].b;
			point.a = 0;
			count++;

		}


	}
	else {
		//cloud2 herasu
		std::unordered_set<int> history;
		for (int i = 0; i < count1; i++) {
//			pix_num = xrand(history, count2);
			pcl::PointXYZRGBA &point = new_cloud->points[i];
			point.x = cloud2->points[i].x;
			point.y = cloud2->points[i].y;
			point.z = cloud2->points[i].z;
			point.r = cloud2->points[i].r;
			point.g = cloud2->points[i].g;
			point.b = cloud2->points[i].b;
			point.a = 0;
			count++;
		}
	}


	return count;

}

int main()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	
	
	// PointCloudの大きさを決定
	p_cloud1->width = 640;
	p_cloud1->height = 480;
	p_cloud1->points.resize(p_cloud1->width * p_cloud1->height);
	p_cloud1->is_dense = false;

	cout << "Size : " << p_cloud1->width * p_cloud1->height << std::endl;

	char img_dir[100] = "images/20190204";
	char c_img[100] = { 0 };
	char d_img[100] = { 0 };
	sprintf(c_img, "%s/frame_372_l.png", img_dir);
	sprintf(d_img, "%s/Depth_img3/Depth_372.png", img_dir);

	cv::Mat point_c = cv::imread(c_img, 1);
	if (point_c.empty()) {
		std::cout << "failed to load c_image." << std::endl;
		return 1;
	}
	cout << "c_img1 ->" << c_img << endl;
	std::cout << "c_image.size(): " << point_c.size() << std::endl;
//	imshow("color",point_c);
//	cv::waitKey(0);
	cv::Mat point_d = cv::imread(d_img, 0);
	if (point_c.empty()) {
		std::cout << "failed to load d_image." << std::endl;
		return 1;
	}
//	imshow("depth",point_d);
//	cv::waitKey(0);
	printf("owaran1");
	
	int count1 = 0;
	// 直接、値を入力してPointCloudを作成
	for (int h = 0; h < p_cloud1->height; h++) {
		for (int w = 0; w < p_cloud1->width; w++) {
			
			//depth ga toretenai mataha object no ryouiki janai tennha mushi
			if ((point_d.at<uchar>(h, w) < 50) || ((point_c.at<Vec3b>(h,w)[2] == 255) && (point_c.at<Vec3b>(h, w)[1] == 255) && (point_c.at<Vec3b>(h, w)[0] == 255)) || (h < 30) || (460 < h) || (w < 30 ) || (580 < w) ) {
				continue;
			}
			else {
//				pcl::PointXYZRGBA &point = p_cloud->points[w + h * p_cloud->width];
				pcl::PointXYZRGBA &point = p_cloud1->points[count1++];
				point.x = w * 0.1;
				point.y = -h * 0.1;
				point.z = point_d.at<uchar>(h, w)*0.5;
				point.r = point_c.at<Vec3b>(h, w)[2];
				point.g = point_c.at<Vec3b>(h, w)[1];
				point.b = point_c.at<Vec3b>(h, w)[0];
				point.a = 0;
			}
		}
	}
	printf("owaran2");
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr p_cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
//	pcl::PointCloud<pcl::PointXYZRGBA> p_cloud2;
//	pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud2(new pcl::PointCloud<pcl::PointXYZ>);

	// PointCloudの大きさを決定
	p_cloud2->width = 640;
	p_cloud2->height = 480;
	p_cloud2->points.resize(p_cloud2->width * p_cloud2->height);
	p_cloud2->is_dense = false;
	memset(c_img, 0, strlen(c_img));
	memset(c_img, 0, strlen(d_img));
	sprintf(c_img, "%s/frame_372_r.png", img_dir);
	sprintf(d_img, "%s/Depth_img3/Depth_372.png", img_dir);

	cv::Mat point_c2 = cv::imread(c_img, 1);
	if (point_c.empty()) {
		std::cout << "failed to load c_image2." << std::endl;
		return 1;
	}
	cout << "c_img2 ->" << c_img << endl;

	std::cout << "c_image2.size(): " << point_c2.size() << std::endl;
//		imshow("color",point_c2);
//		cv::waitKey(0);
	cv::Mat point_d2 = cv::imread(d_img, 0);
	if (point_c.empty()) {
		std::cout << "failed to load d_image2." << std::endl;
		return 1;
	}
//	imshow("depth",point_d2);
//	cv::waitKey(0);
	int count2 = 0;
		// 直接、値を入力してPointCloudを作成
	for (int h = 0; h < p_cloud2->height; h++) {
		for (int w = 0; w < p_cloud2->width; w++) {

			//depth ga toretenai mataha object no ryouiki janai tennha mushi
			if ((point_d2.at<uchar>(h, w) < 50) || ((point_c2.at<Vec3b>(h, w)[2] == 255) && (point_c2.at<Vec3b>(h, w)[1] == 255) && (point_c2.at<Vec3b>(h, w)[0] == 255)) || (h < 30) || (460 < h) || (w < 30) || (580 < w)) {
				continue;
			}
			else {
//				pcl::PointXYZRGBA &point = p_cloud2->points[w + h * p_cloud2->width];
				pcl::PointXYZRGBA &point = p_cloud2->points[count2++];
				point.x = w * 0.1;
				point.y = -h * 0.1;
				point.z = point_d2.at<uchar>(h, w)*0.5;
				point.r = point_c2.at<Vec3b>(h, w)[2];
				point.g = point_c2.at<Vec3b>(h, w)[1];
				point.b = point_c2.at<Vec3b>(h, w)[0];
				point.a = 0;
			}
		}
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	new_cloud->width = 640;
	new_cloud->height = 480;
	new_cloud->points.resize(new_cloud->width * new_cloud->height);
	new_cloud->is_dense = false;

	std::cout << "kokoha doko1" << std::endl;
	std::cout << count1 << " and " << count2 << std::endl;
	std::cout << decrease_points(p_cloud1, p_cloud2, new_cloud, count1, count2) << std::endl;
	

	std::cout << "kokoha doko2" << std::endl;
	
	pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
//	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//	icp.setInputCloud(p_cloud);
//	icp.setInputTarget(p_cloud2);

	

	icp.setInputSource(new_cloud);
	if (count1 >= count2) {
		 icp.setInputTarget(p_cloud2);
	}
	else {
		icp.setInputTarget(p_cloud1);
	}
//
//

	 std::cout << "kokomade3" << std::endl;

	 

	pcl::PointCloud<pcl::PointXYZRGBA> Final;
//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Final(new pcl::PointCloud<pcl::PointXYZRGBA>);
//	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);



	std::cout << "kokomade4" << std::endl;

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	transformation_matrix = icp.getFinalTransformation().cast<double>();
	print4x4Matrix(transformation_matrix);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr F_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>(Final));
//	pcl::PointCloud<pcl::PointXYZ>::Ptr Final_cloud(new pcl::PointCloud<pcl::PointXYZ>(Final));

//	*F_cloud += *p_cloud2;
	if (count1 >= count2) {
		*F_cloud += *p_cloud2;
	}
	else {
		*F_cloud += *p_cloud1;
	}

//	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr F_cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
//	F_cloud2->width = F_cloud->width;
//	F_cloud2->height = F_cloud->height;
//	F_cloud2->points.resize(F_cloud2->width * F_cloud2->height);
	
//	color_filter(F_cloud, F_cloud2);


	// ビューワーの作成
	pcl::visualization::CloudViewer viewer("PointCloudViewer");
	viewer.showCloud(F_cloud);


	// ビューワー起動時の一回だけ呼ばれる関数をセット
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	// ビューワー起動中の毎フレーム実行される関数をセット
//	viewer.runOnVisualizationThread(viewerPsycho);

	// ビューワー視聴用ループ
	while (!viewer.wasStopped())
	{

	}
	return 0;
}