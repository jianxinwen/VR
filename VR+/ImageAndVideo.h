#pragma once

#include <QWidget>
#include<QImage>
#include<QLabel>

#include<opencv2/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/video.hpp>

using namespace cv;
class ImageAndVideo : public QWidget
{
	Q_OBJECT

public:
	ImageAndVideo(QWidget *parent=Q_NULLPTR);
	~ImageAndVideo();
	QImage getImageFromVideo();
	QImage mat2QImage(const Mat& mat);
	Mat QImage2Mat(QImage image);

signals:


public slots:

private:
	QLabel * imgLabel;
	VideoCapture video;
};
