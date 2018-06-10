#pragma once
#include <stdio.h>
#include<iostream>
#include<fstream>
#include<Windows.h>

#include<omp.h>

#include<opencv2/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/xfeatures2d/nonfree.hpp>
#include<opencv2/calib3d.hpp>
#include<opencv2/highgui.hpp>
using namespace cv;
using namespace std;
namespace vr
{
	//相机标定
	void vrStartCalibration();
	void vrGetImagesFromVideo();
	void vrUndistortVideo();
	

	//柱面投影
	void adjustWidth(Mat& src);
	void set4By1(const Mat& src, Mat& dst,
		Point srcPt, Point dstPt,
		uint nHalfWidthSrc, uint nHalfHeightSrc,
		uint nHalfWidthDst, uint nHalfHeightDst);
	int plane2Cylinder(const Mat& src, Mat& dst, uint z);
	int cylinder2Plane(const Mat& src, Mat& dst, uint z);
	int plane2Sphere(const Mat& src, Mat& dst, uint z);
	int sphere2Plane(const Mat& src, Mat& dst, uint z);

	Mat vrStitch(); 

}
