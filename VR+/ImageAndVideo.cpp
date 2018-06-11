#include "ImageAndVideo.h"

ImageAndVideo::ImageAndVideo(QWidget *parent)
	: QWidget(parent)
{
	setWindowTitle("MediaPlayer");
	resize(1000, 800);
	video.open("Resources/img/GOPR0058.MP4");
	imgLabel = new QLabel(this);
	imgLabel->setScaledContents(true);
	QImage src = getImageFromVideo();
	
	
	
	imgLabel->setPixmap(QPixmap::fromImage(src));

}

ImageAndVideo::~ImageAndVideo()
{
}


QImage ImageAndVideo::getImageFromVideo()
{
	Mat frame;
	video >> frame;
	//
	//
	//
	cv::resize(frame, frame, cv::Size(geometry().width(), geometry().height()));
	return mat2QImage(frame);
}

QImage ImageAndVideo::mat2QImage(const Mat& mat)
{
	if (mat.type() == CV_8UC1)
	{
		QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
		image.setColorCount(256);
		for (int i = 0; i < 256; i++)
		{
			image.setColor(i, qRgb(i, i, i));
		}
		uchar *pSrc = mat.data;
		for (int row = 0; row < mat.rows; row++)
		{
			uchar *pDest = image.scanLine(row);
			memcpy(pDest, pSrc, mat.cols);
			pSrc += mat.step[0];
		}
		return image;
	}
	else if (mat.type() == CV_8UC3)
	{
		const uchar *pSrc = (const uchar*)mat.data;
		QImage image(pSrc, mat.cols, mat.rows, mat.step[0], QImage::Format_RGB888);
		return image.rgbSwapped();
	}
	else if (mat.type() == CV_8UC4)
	{
		const uchar *pSrc = (const uchar*)mat.data;
		QImage image(pSrc, mat.cols, mat.rows, mat.step[0], QImage::Format_ARGB32);
		return image.copy();
	}
	else
	{
		return QImage();
	}
}


Mat ImageAndVideo::QImage2Mat(QImage image)
{
	Mat mat;
	switch (image.format())
	{
	case QImage::Format_ARGB32:
	case QImage::Format_RGB32:
	case QImage::Format_ARGB32_Premultiplied:
		mat = Mat(image.height(), image.width(), CV_8UC4, (void*)image.constBits(), image.bytesPerLine());
		break;
	case QImage::Format_RGB888:
		mat = Mat(image.height(), image.width(), CV_8UC3, (void*)image.constBits(), image.bytesPerLine());
		cvtColor(mat, mat, CV_BGR2RGB);
		break;
	case QImage::Format_Indexed8:
		mat = Mat(image.height(), image.width(), CV_8UC1, (void*)image.constBits(), image.bytesPerLine());
		break;
	}
	return mat;
}
