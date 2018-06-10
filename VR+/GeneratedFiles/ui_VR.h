/********************************************************************************
** Form generated from reading UI file 'VR.ui'
**
** Created by: Qt User Interface Compiler version 5.9.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VR_H
#define UI_VR_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_VRClass
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *VRClass)
    {
        if (VRClass->objectName().isEmpty())
            VRClass->setObjectName(QStringLiteral("VRClass"));
        VRClass->resize(600, 400);
        menuBar = new QMenuBar(VRClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        VRClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(VRClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        VRClass->addToolBar(mainToolBar);
        centralWidget = new QWidget(VRClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        VRClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(VRClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        VRClass->setStatusBar(statusBar);

        retranslateUi(VRClass);

        QMetaObject::connectSlotsByName(VRClass);
    } // setupUi

    void retranslateUi(QMainWindow *VRClass)
    {
        VRClass->setWindowTitle(QApplication::translate("VRClass", "VR", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class VRClass: public Ui_VRClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VR_H
