#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_VR.h"

class VR : public QMainWindow
{
	Q_OBJECT

public:
	VR(QWidget *parent = Q_NULLPTR);

private:
	Ui::VRClass ui;
};
