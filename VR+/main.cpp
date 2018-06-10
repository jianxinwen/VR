#include "VR.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	VR w;
	w.show();
	return a.exec();
}
