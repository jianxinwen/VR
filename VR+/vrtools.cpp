#include"vrtools.h"

namespace vr
{
	string path = "new";
	void vrStartCalibration() 
	{

		ofstream fout("caliberation_result.txt");

		cout<< "=====开始提取角点=====" << endl;
		int image_count = 12;
		Size board_size = Size(6, 9);
		vector<Point2f> corners;
		vector<vector<Point2f>>  corners_Seq;
		vector<Mat>  image_Seq;
		int successImageNum = 0;
		for (int i = 0; i != image_count; i++)
		{
			cout<< "第" << i + 1 << "张..." << endl;
			string;
			std::stringstream filename;
			filename << "img/" + path + "/img" << i + 1 << ".jpg";
			cv::Mat image = imread(filename.str());
			/* 提取角点 */
			Mat imageGray;
			cvtColor(image, imageGray, CV_RGB2GRAY);
			bool patternfound = findChessboardCorners(image, board_size, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE +
				CALIB_CB_FAST_CHECK);
			if (!patternfound) 
			{
				cout << "=====找不到角点=====\n";
				continue;
				exit(1);
			}
			else 
			{
				cornerSubPix(imageGray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
				Mat imageTemp = image.clone();
				for (int j = 0; j < corners.size(); j++) 
				{
					circle(imageTemp, corners[j], 10, Scalar(0, 0, 255), 2, 8, 0);
				}
				std::stringstream filename;
				filename << "img/" + path + "/" << i + 1 << "_corner.jpg";
				imwrite(filename.str(), imageTemp);
				cout << "保存第" << i + 1 << "张角点图片...完成" << endl;

				successImageNum = successImageNum + 1;
				corners_Seq.push_back(corners);
			}
			image_Seq.push_back(image);
		}
		cout << "=====角点提取完成=====\n";
		//摄像机定标
		cout << "=====开始定标=====" << endl;
		Size square_size = Size(20, 20);
		vector<vector<Point3f>>  object_Points;

		vector<int>  point_counts;
		/* 初始化定标板上角点的三维坐标 */
		for (int t = 0; t<successImageNum; t++)
		{
			vector<Point3f> tempPointSet;
			for (int i = 0; i<board_size.height; i++)
			{
				for (int j = 0; j<board_size.width; j++) 
				{
					/* 假设定标板放在世界坐标系中z=0的平面上 */
					Point3f tempPoint;
					tempPoint.x = i * square_size.width;
					tempPoint.y = j * square_size.height;
					tempPoint.z = 0;
					tempPointSet.push_back(tempPoint);
				}
			}
			object_Points.push_back(tempPointSet);
		}
		for (int i = 0; i< successImageNum; i++)
		{
			point_counts.push_back(board_size.width*board_size.height);
		}
		/* 开始定标 */
		Size image_size = image_Seq[0].size();
		cv::Mat intrinsic_matrix;
		cv::Vec4d distortion_coeffs;
		std::vector<cv::Vec3d> rotation_vectors;
		std::vector<cv::Vec3d> translation_vectors;
		int flags = 0;
		flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
		flags |= cv::fisheye::CALIB_CHECK_COND;
		flags |= cv::fisheye::CALIB_FIX_SKEW;
		fisheye::calibrate(object_Points, corners_Seq, image_size, intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, flags, cv::TermCriteria(3, 20, 1e-6));
		cout << "=====定标完成=====\n";

		/************************************************************************
		对定标结果进行评价
		*************************************************************************/
		cout << "=====开始评价定标结果=====" << endl;
		double total_err = 0.0;
		double err = 0.0;
		vector<Point2f>  image_points2;

		cout << "每幅图像的定标误差：" << endl << endl;
		for (int i = 0; i<image_count; i++)
		{
			vector<Point3f> tempPointSet = object_Points[i];
			/****    通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点     ****/
			fisheye::projectPoints(tempPointSet, image_points2, rotation_vectors[i], translation_vectors[i], intrinsic_matrix, distortion_coeffs);
			/* 计算新的投影点和旧的投影点之间的误差*/
			vector<Point2f> tempImagePoint = corners_Seq[i];
			Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
			Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
			for (size_t i = 0; i != tempImagePoint.size(); i++) {
				image_points2Mat.at<Vec2f>(0, i) = Vec2f(image_points2[i].x, image_points2[i].y);
				tempImagePointMat.at<Vec2f>(0, i) = Vec2f(tempImagePoint[i].x, tempImagePoint[i].y);
			}
			err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
			total_err += err /= point_counts[i];
			cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
			fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
		}
		cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
		fout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
		cout << "=====评价完成=====" << endl;

		//保存定标结果
		FileStorage fs("calibration_result.xml", FileStorage::WRITE);

		fs << "intrinsic_matrix" << intrinsic_matrix
			<< "distortion_coeffs" << distortion_coeffs;
		fs.release();
		cout << "=====完成保存=====" << endl;


		//显示定标结果
		Mat mapx = Mat(image_size, CV_32FC1);
		Mat mapy = Mat(image_size, CV_32FC1);
		Mat R = Mat::eye(3, 3, CV_32F);

		cout << "=====保存矫正图像=====" << endl;
		for (int i = 0; i != image_count; i++) 
		{
			cout << "第" << i + 1 << "张..." << endl;
			Mat distort_img = image_Seq[i].clone();
			Mat undistort_img;
			Mat intrinsic_mat(intrinsic_matrix), new_intrinsic_mat;

			intrinsic_mat.copyTo(new_intrinsic_mat);
			//调节视场大小,乘的系数越小视场越大
			new_intrinsic_mat.at<double>(0, 0) *= 0.8;
			new_intrinsic_mat.at<double>(1, 1) *= 0.95;
			//调节校正图中心，一般不做改变
			new_intrinsic_mat.at<double>(0, 2) += 0;
			new_intrinsic_mat.at<double>(1, 2) += 0;
			Mat newCameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
			fisheye::undistortImage(distort_img, undistort_img, intrinsic_matrix, distortion_coeffs, new_intrinsic_mat);

			std::stringstream filename;
			filename << "img/" + path + "/" << i + 1 << "_d.jpg";
			imwrite(filename.str(), undistort_img);
		}
		cout << "=====保存结束=====" << endl;



	}

	void vrGetImagesFromVideo() 
	{
		VideoCapture video("img/GOPR0058.MP4");
		for (int i = 0; i < 12; i++) 
		{
			video.set(CAP_PROP_POS_FRAMES, i * 3);
			Mat tmp;
			video >> tmp;
			//pyrDown(tmp, tmp);
			stringstream str;
			str << i + 1;
			imwrite("img/" + path + "/img" + str.str() + ".jpg", tmp);
			cout << i + 1 << " done" << endl;
		}
	}


	void vrUndistortVideo() 
	{
		//读取参数
		cv::Mat intrinsic_matrix;
		cv::Vec4d distortion_coeffs;
		FileStorage fs("calibration_result.xml", FileStorage::READ);
		fs["intrinsic_matrix"] >> intrinsic_matrix;
		fs["distortion_coeffs"] >> distortion_coeffs;


		Mat intrinsic_mat(intrinsic_matrix), new_intrinsic_mat;
		intrinsic_mat.copyTo(new_intrinsic_mat);
		//调节视场大小,乘的系数越小视场越大
		new_intrinsic_mat.at<double>(0, 0) *= 0.8;
		new_intrinsic_mat.at<double>(1, 1) *= 0.95;
		//调节校正图中心，一般不做改变
		new_intrinsic_mat.at<double>(0, 2) += 0;
		new_intrinsic_mat.at<double>(1, 2) += 0;
		Mat newCameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));


		VideoCapture video("img/GOPR0058.MP4");
		int frame_count = (int)video.get(CAP_PROP_FRAME_COUNT);
		int sum = 0;
		vector<Mat> bufImg{ Mat(),Mat(),Mat() };
		vector<Mat> bufUndistortImg{ Mat(),Mat(),Mat() };
		vector<Mat> bufDistortImg{ Mat(),Mat(),Mat() };


		cout << "开始计时：";
		namedWindow("undistort", WINDOW_AUTOSIZE);
		double start = GetTickCount();
		double end;

		//多线程执行
		for (int i = 0; i < frame_count; i += 3 * 3) 
		{
			video >> bufImg[0];
			video >> bufImg[1];
			video >> bufImg[2];
#pragma omp parallel sections  
			{
#pragma omp section  
				{
					if (!bufImg[0].empty()) 
					{
						bufDistortImg[0] = bufImg[0];
						fisheye::undistortImage(bufDistortImg[0], bufUndistortImg[0], intrinsic_matrix, distortion_coeffs, new_intrinsic_mat);
					}
				}
#pragma omp section  
				{
					if (!bufImg[1].empty()) 
					{
						bufDistortImg[1] = bufImg[1];
						fisheye::undistortImage(bufDistortImg[1], bufUndistortImg[1], intrinsic_matrix, distortion_coeffs, new_intrinsic_mat);
					}
				}
#pragma omp section  
				{
					if (!bufImg[2].empty()) 
					{
						bufDistortImg[2] = bufImg[2];
						fisheye::undistortImage(bufDistortImg[2], bufUndistortImg[2], intrinsic_matrix, distortion_coeffs, new_intrinsic_mat);
					}
				}
			}
			end = GetTickCount();
			cout << endl << "当前耗时(ms):" << end - start << endl;

			for (int j = 0; j < 3; j++) 
			{
				pyrDown(bufUndistortImg[j], bufUndistortImg[j]);
				pyrDown(bufUndistortImg[j], bufUndistortImg[j]);
				imshow("undistort", bufUndistortImg[j]);
				waitKey(10);
			}

		}
		cout << endl << "结束";
	}





	void adjustWidth(cv::Mat& src)
	{
		if (src.cols*src.channels() % 4 != 0)
		{
			uint right = (src.cols + 3) / 4 * 4 - src.cols;
			copyMakeBorder(src, src, 0, 0, 0, right, BORDER_REPLICATE);
		}
	}


	void set4By1(const cv::Mat& src, cv::Mat& dst,
		cv::Point srcPt, cv::Point dstPt,
		uint nHalfWidthSrc, uint nHalfHeightSrc,
		uint nHalfWidthDst, uint nHalfHeightDst)
	{
		uint u = srcPt.x;
		uint v = srcPt.y;
		uint x = dstPt.x;
		uint y = dstPt.y;
		Point srcPoint[4] =
		{ Point(u + nHalfWidthSrc,v + nHalfHeightSrc),	//右上)
			Point(u + nHalfWidthSrc,nHalfHeightSrc - v),
			Point(nHalfWidthSrc - u,nHalfHeightSrc - v),
			Point(nHalfWidthSrc - u,nHalfHeightSrc + v) };

		Point dstPoint[4] =
		{ Point(x + nHalfWidthDst,y + nHalfHeightDst),	//右上)
			Point(x + nHalfWidthDst,nHalfHeightDst - y),
			Point(nHalfWidthDst - x,nHalfHeightDst - y),
			Point(nHalfWidthDst - x,nHalfHeightDst + y) };

		bool bGray = src.channels() == 3 ? false : true;

		for (uint i = 0; i < 4; ++i)
		{
			if (srcPoint[i].x < src.cols && srcPoint[i].y < src.rows &&
				srcPoint[i].x >= 0 && srcPoint[i].y >= 0 /*&&
														 dstPoint[i].x<dst.cols && dstPoint[i].y<dst.rows &&
														 dstPoint[i].x>=0 && dstPoint[i].y>=0*/)
			{
				if (bGray)
				{
					dst.at<uchar>(dstPoint[i]) = src.at<uchar>(srcPoint[i]);
				}
				else
				{
					dst.at<Vec3b>(dstPoint[i]) = src.at<Vec3b>(srcPoint[i]);
				}
			}
		}
	}



	/************************************************************
	*实现平面图像到柱面图像的转换
	************************************************************/
	int plane2Cylinder(const cv::Mat& src, cv::Mat& dst, uint z)
	{
		if (src.empty() ||
			(src.data == dst.data) ||
			z > 10000)
		{
			return -1;
		}

		const uint nWidth = 2 * z * atan(double(src.cols / 2.0 / z));
		const uint nHeight = src.rows;

		const uint nHalfWidthDst = nWidth / 2;
		const uint nHalfHeightDst = nHeight / 2;
		const uint nHalfWidthSrc = src.cols / 2;
		const uint nHalfHeightSrc = src.rows / 2;

		dst = Mat::zeros(nHeight, nWidth, src.type());

		for (uint v = 0; v < nHalfHeightDst; ++v)
		{
			for (uint u = 0; u < nHalfWidthDst; ++u)
			{
				//	double temp = ;
				uint x = z * tan(double(u*1.0 / z));
				uint y = v * sqrt(double(x*x + z * z)) / z;

				set4By1(src, dst, Point(x, y), Point(u, v),
					nHalfWidthSrc, nHalfHeightSrc,
					nHalfWidthDst, nHalfHeightDst);
			}
		}
		adjustWidth(dst);
	}

	int cylinder2Plane(const cv::Mat& src, cv::Mat& dst, uint z)
	{
		if (src.empty() ||
			(src.data == dst.data) ||
			z > 10000)
		{
			return -1;
		}

		const uint nWidth = 2 * z * tan(double(src.cols / 2.0 / z));
		const uint nHeight = src.rows;

		const uint nHalfWidthDst = nWidth / 2;
		const uint nHalfHeightDst = nHeight / 2;
		const uint nHalfWidthSrc = src.cols / 2;
		const uint nHalfHeightSrc = src.rows / 2;

		dst = Mat::zeros(nHeight, nWidth, src.type());

		for (uint y = 0; y < nHalfHeightDst; ++y)
		{
			for (uint x = 0; x < nHalfWidthDst; ++x)
			{
				uint u = z * atan(double(x*1.0 / z));
				uint v = y * z / sqrt(double(x*x + z * z));

				set4By1(src, dst, Point(u, v), Point(x, y),
					nHalfWidthSrc, nHalfHeightSrc,
					nHalfWidthDst, nHalfHeightDst);
			}
		}
		adjustWidth(dst);

	}


	/**********************************************************
	*平面到球面变换
	**********************************************************/
	int plane2Sphere(const cv::Mat& src, cv::Mat& dst, uint z)
	{
		if (src.empty() ||
			(src.data == dst.data) ||
			z > 10000)
		{
			return -1;
		}

		const uint nWidth = 2 * z * atan(double(src.cols / 2.0 / z));
		const uint nHeight = 2 * z * atan(double(src.rows / 2.0 / z));


		const uint nHalfWidthDst = nWidth / 2;
		const uint nHalfHeightDst = nHeight / 2;
		const uint nHalfWidthSrc = src.cols / 2;
		const uint nHalfHeightSrc = src.rows / 2;

		dst = Mat::zeros(nHeight, nWidth, src.type());

		for (uint v = 0; v < nHalfHeightDst; ++v)
		{
			for (uint u = 0; u < nHalfWidthDst; ++u)
			{
				uint x = z * tan(double(u*1.0 / z));
				uint y = z * tan(v*1.0 / z) / cos(u*1.0 / z);

				set4By1(src, dst, Point(x, y), Point(u, v),
					nHalfWidthSrc, nHalfHeightSrc,
					nHalfWidthDst, nHalfHeightDst);

			}
		}
		adjustWidth(dst);

		return 0;
	}

	/**********************************************************
	*球面到平面变换
	**********************************************************/
	int sphere2Plane(const cv::Mat& src, cv::Mat& dst, uint z)
	{
		if (src.empty() ||
			(src.data == dst.data) ||
			z > 10000)
		{
			return -1;
		}

		uint nWidth = 2 * z * tan(double(src.cols / 2.0 / z));
		uint nHeight = 2 * z * tan(double(src.rows / 2.0 / z)) /
			cos(double(src.cols / 2.0 / z));

		dst = Mat::zeros(nHeight, nWidth, src.type());

		const uint nHalfWidthDst = nWidth / 2;
		const uint nHalfHeightDst = nHeight / 2;
		const uint nHalfWidthSrc = src.cols / 2;
		const uint nHalfHeightSrc = src.rows / 2;

		dst = Mat::zeros(nHeight, nWidth, src.type());

		for (uint y = 0; y < nHalfHeightDst; ++y)
		{
			for (uint x = 0; x < nHalfWidthDst; ++x)
			{
				uint u = z * atan(double(x*1.0 / z));
				uint v = z * atan(y*1.0 / sqrt(double(x*x + z * z)));

				set4By1(src, dst, Point(u, v), Point(x, y),
					nHalfWidthSrc, nHalfHeightSrc,
					nHalfWidthDst, nHalfHeightDst);

			}
		}
		adjustWidth(dst);
		return 0;
	}




	/**********************************************************
	*图像拼接
	**********************************************************/
	Mat vrStitch() {
		Mat imgLeft = imread("img/left.jpg", 1);
		Mat imgRight = imread("img/right.jpg", 1);

		pyrDown(imgLeft, imgLeft);
		pyrDown(imgLeft, imgLeft);
		pyrDown(imgLeft, imgLeft);
		pyrDown(imgRight, imgRight);
		pyrDown(imgRight, imgRight);
		pyrDown(imgRight, imgRight);



		auto surf = cv::xfeatures2d::SURF::create(1000);//Hessian矩阵阈值:1000,越大越精确
		vector<KeyPoint> leftKeyPoints, rightKeyPoints;//特征点
		Mat descriptors4Left, descriptors4Right;//描述子
		cv::FlannBasedMatcher descriptorMatcher;//匹配器
		vector<DMatch> matches;//匹配结果  

							   //检测特征点并计算描述子
		surf->detectAndCompute(imgLeft, noArray(), leftKeyPoints, descriptors4Left);
		surf->detectAndCompute(imgRight, noArray(), rightKeyPoints, descriptors4Right);

		//进行特征点匹配
		descriptorMatcher.match(descriptors4Left, descriptors4Right, matches);

		cout<< "左图特征点个数:" << leftKeyPoints.size() << endl;
		cout<< "右图特征点个数:" << rightKeyPoints.size() << endl;
		cout<< "Match个数：" << matches.size() << endl;

		//计算匹配结果中距离的最大和最小值    
		//距离是指两个特征向量间的欧式距离，表明两个特征的差异，值越小表明两个特征点越接近    
		double maxDist = 0;
		double minDist = 100;
		for (int i = 0; i<matches.size(); i++)
		{
			double dist = matches[i].distance;
			if (dist < minDist) minDist = dist;
			if (dist > maxDist) maxDist = dist;
		}

		//筛选出较好的匹配点   
		vector<DMatch> goodMatches;
		for (int i = 0; i<matches.size(); i++) if (matches[i].distance < 0.2 * maxDist) goodMatches.push_back(matches[i]);
		cout<< "goodMatch个数：" << goodMatches.size() << endl;
		
		if (goodMatches.size() <= 10) return Mat();

		// 分配空间,并提取KeyPoint的pt 
		int ptCount = (int)goodMatches.size();
		Mat leftMatchPt(ptCount, 2, CV_32F);
		Mat rightMatchPt(ptCount, 2, CV_32F);
		Point2f pt;
		for (int i = 0; i<ptCount; i++)
		{
			pt = leftKeyPoints[goodMatches[i].queryIdx].pt;//左
			leftMatchPt.at<float>(i, 0) = pt.x;
			leftMatchPt.at<float>(i, 1) = pt.y;

			pt = rightKeyPoints[goodMatches[i].trainIdx].pt;//右
			rightMatchPt.at<float>(i, 0) = pt.x;
			rightMatchPt.at<float>(i, 1) = pt.y;
		}

		//此步骤可省略
		// 用RANSAC方法计算F进一步筛选特征点 
		vector<uchar> RANSACStatus; //存储RANSAC后每个点的状态  
		Mat F = findFundamentalMat(leftMatchPt, rightMatchPt, RANSACStatus, FM_RANSAC);

		int inlinerCount = 0;// 内点个数 
		for (int i = 0; i<ptCount; i++) if (RANSACStatus[i] != 0) inlinerCount++; // 状态为0表示野点  

		cout<< "内点数为：" << inlinerCount << endl;

		vector<Point2f> leftInlier(inlinerCount);//于保存内点和匹配关系 
		vector<Point2f> rightInlier(inlinerCount);
		vector<DMatch> inlierMatches(inlinerCount);
		float inlierRightMinX = imgLeft.cols;//存储内点中右图最小横坐标，以便后续融合 
		inlinerCount = 0;
		for (int i = 0; i<ptCount; i++)
		{
			if (RANSACStatus[i] != 0)
			{
				leftInlier[inlinerCount].x = leftMatchPt.at<float>(i, 0);
				leftInlier[inlinerCount].y = leftMatchPt.at<float>(i, 1);
				rightInlier[inlinerCount].x = rightMatchPt.at<float>(i, 0);
				rightInlier[inlinerCount].y = rightMatchPt.at<float>(i, 1);
				inlierMatches[inlinerCount].queryIdx = inlinerCount;
				inlierMatches[inlinerCount].trainIdx = inlinerCount;

				if (rightInlier[inlinerCount].x<inlierRightMinX) inlierRightMinX = rightInlier[inlinerCount].x;//存储内点中右图最小横坐标 

				inlinerCount++;
			}
		}

		//RANSAC得到的单应矩阵  
		Mat H = findHomography(leftInlier, rightInlier, RANSAC);

		//存储左图四角，及其变换到右图后的四角位置  
		std::vector<Point2f> left4Corners(4);
		left4Corners[0] = Point(0, 0);
		left4Corners[1] = Point(imgLeft.cols, 0);
		left4Corners[2] = Point(imgLeft.cols, imgLeft.rows);
		left4Corners[3] = Point(0, imgLeft.rows);

		std::vector<Point2f> cornersAfterTransformed(4);
		perspectiveTransform(left4Corners, cornersAfterTransformed, H);

		int offsetX = cornersAfterTransformed[1].x; //旧到新的偏移量  
		int width = int(max(abs(cornersAfterTransformed[1].x), abs(cornersAfterTransformed[2].x)));
		int height = imgLeft.rows;//拼接前融合图像时所需高度
		float translateX = 0;//变换后,四角最左边的x,一般为负的

		if (cornersAfterTransformed[0].x<0)
			if (cornersAfterTransformed[3].x<0) translateX += min(cornersAfterTransformed[0].x, cornersAfterTransformed[3].x);
			else translateX += cornersAfterTransformed[0].x;

			//更新的四角位置，使图像完整显示 ,前一次的四角位置并不能显示完整图像,只能显示右边的部分,将其向右平移(-translateX）
			for (int i = 0; i<4; i++) cornersAfterTransformed[i].x -= translateX;
			width -= int(translateX);//拼接前融合图像时所需宽度
			Mat tmpImg4Merge = Mat::zeros(height, width, imgLeft.type());
			//最终所需单应矩阵
			Mat H1 = getPerspectiveTransform(left4Corners, cornersAfterTransformed);

			//进行图像变换 
			warpPerspective(imgLeft, tmpImg4Merge, H1, Size(width, height));

			/*
			* 开始图像融合
			* 线性融合
			* 求得融合区域
			* 越靠近融合区域左边，像素的权重越大
			*/
			int startX4LeftImg = int(inlierRightMinX - translateX);//左图开始融合部分的x
			int widthOfMerge = width - startX4LeftImg;//需要融合的宽度

			uchar* ptrOfTmpImg4Merge = tmpImg4Merge.data;
			double alpha = 0, beta = 1;//权重系数
			for (int row = 0; row<height; row++) {
				//先取一个像素的尺寸 elemSize ，再取里面的各个通道值 elemSize1
				//&(mtx i,j)=mtx.data+mtx.step[0]*i+mtx.step[1]*j
				ptrOfTmpImg4Merge = tmpImg4Merge.data + row * tmpImg4Merge.step[0] + (startX4LeftImg)*tmpImg4Merge.elemSize();//左图的B
				for (int col = 0; col<widthOfMerge; col++)
				{
					uchar* ptrOfLeftCG = ptrOfTmpImg4Merge + tmpImg4Merge.elemSize1(); //左图的G
					uchar* ptrOfLeftCR = ptrOfLeftCG + tmpImg4Merge.elemSize1();//左图的R

					uchar* ptrOfRightImg = imgRight.data + row * imgRight.step[0] + (col + int(inlierRightMinX))*imgRight.elemSize();//右图的B
					uchar* ptrOfRightCG = ptrOfRightImg + imgRight.elemSize1();//右图的G
					uchar* ptrOfRightCR = ptrOfRightCG + imgRight.elemSize1();//右图的R

					alpha = double(col) / double(widthOfMerge);
					beta = 1 - alpha;

					//左图变换后黑色区域由右图对应像素直接代替代替
					if (*ptrOfTmpImg4Merge == 0 && *ptrOfLeftCG == 0 && *ptrOfLeftCR == 0) {
						*ptrOfTmpImg4Merge = (*ptrOfRightImg);
						*ptrOfLeftCG = (*ptrOfRightCG);
						*ptrOfLeftCR = (*ptrOfRightCR);
					}
					//其他重合地方进行线性融合
					*ptrOfTmpImg4Merge = (*ptrOfTmpImg4Merge)*beta + (*ptrOfRightImg)*alpha;
					*ptrOfLeftCG = (*ptrOfLeftCG)*beta + (*ptrOfRightCG)*alpha;
					*ptrOfLeftCR = (*ptrOfLeftCR)*beta + (*ptrOfRightCR)*alpha;

					ptrOfTmpImg4Merge += tmpImg4Merge.elemSize();
				}
			}

			//目标图像
			Mat imgResult = Mat::zeros(height, width + imgRight.cols - offsetX, imgLeft.type());
			uchar* ptrOfResultImg = imgResult.data;

			for (int row = 0; row<height; row++) {
				ptrOfResultImg = imgResult.data + row * imgResult.step[0];//目标图像的B
																		  //复制左图
				for (int col = 0; col<tmpImg4Merge.cols; col++)
				{
					uchar* ptrOfResultImgCG = ptrOfResultImg + imgResult.elemSize1(); //G
					uchar* ptrOfResultImgCR = ptrOfResultImgCG + tmpImg4Merge.elemSize1();//R

					uchar* ptrOfTmpImg4Merge = tmpImg4Merge.data + row * tmpImg4Merge.step[0] + col * tmpImg4Merge.elemSize();//左图融合后的B
					uchar* ptrOfLeftCG = ptrOfTmpImg4Merge + tmpImg4Merge.elemSize1();//G
					uchar* ptrOfLeftCR = ptrOfLeftCG + tmpImg4Merge.elemSize1();//R

					*ptrOfResultImg = *ptrOfTmpImg4Merge;
					*ptrOfResultImgCG = *ptrOfLeftCG;
					*ptrOfResultImgCR = *ptrOfLeftCR;

					ptrOfResultImg += imgResult.elemSize();
				}
				//复制右图
				ptrOfResultImg = imgResult.data + row * imgResult.step[0] + tmpImg4Merge.cols*imgResult.elemSize();//目标图像的B
				for (int col = tmpImg4Merge.cols; col<imgResult.cols; col++)
				{
					uchar* ptrOfResultImgCG = ptrOfResultImg + tmpImg4Merge.elemSize1();//G
					uchar* ptrOfResultImgCR = ptrOfResultImgCG + tmpImg4Merge.elemSize1();//R

					uchar* ptrOfRightImg = imgRight.data + row * imgRight.step + (col - tmpImg4Merge.cols + offsetX)*imgRight.elemSize();//右图减掉融合区域后的图像的B
					uchar* ptrOfRightCG = ptrOfRightImg + imgRight.elemSize1();//G
					uchar* ptrOfRightCR = ptrOfRightCG + imgRight.elemSize1();//R

					*ptrOfResultImg = *ptrOfRightImg;
					*ptrOfResultImgCG = *ptrOfRightCG;
					*ptrOfResultImgCR = *ptrOfRightCR;

					ptrOfResultImg += imgResult.elemSize();
				}
			}
			imwrite("img/stitchResult.jpg", imgResult);
			namedWindow("stitchResult.jpg", WINDOW_NORMAL);
			imshow("stitchResult.jpg", imgResult);
			waitKey(0);
			return imgResult;

	}
}