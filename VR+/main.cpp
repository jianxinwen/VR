#include "VR.h"
#include"ImageAndVideo.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	ImageAndVideo w;
	w.show();
	return a.exec();
}
