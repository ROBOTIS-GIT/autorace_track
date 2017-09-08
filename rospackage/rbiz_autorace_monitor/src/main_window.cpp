/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/rbiz_autorace_monitor/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace rbiz_autorace_monitor {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	/*********************
  ** Auto Start
  **********************/
	if (!qnode.init("http://localhost:11311/",
						"localhost"))
	{
		qnode.log(QNode::Info, "Couldn't find the ros master. ");

		close();
		// log(Info,std::string("Couldn't find the ros master. "));
		// ui.pButtonMission1->setEnabled(false);
	}

	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    // QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();


	// setWindowIcon(QIcon(":/images/icon.png"));
	// ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

		QObject::connect(&qnode, SIGNAL(fnUpdateLapTime()), this, SLOT(fnCheckLapTime()));


		// QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(fnUpdateLapTime()));


	/*********************
	** Logging
	**********************/
	// ui.view_logging->setModel(qnode.loggingModel());
    // QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

		// while(1)
		// {
		// 	;
		// }

  // if ( ui.checkbox_remember_settings->isChecked() ) {
  	// on_button_connect_clicked(true);
  // }
	// on_pButtonMission1_clicked(true);
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

// void MainWindow::showNoMasterMessage() {
// 	QMessageBox msgBox;
// 	msgBox.setText("Couldn't find the ros master.");
// 	msgBox.exec();
//     close();
// }

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::fnCheckLapTime() {
	// ui.lcdNumberMin1->display(((int)((int)(qnode.lap_time / 600000.0))));
	if ((int)((int)(qnode.lap_time / 60000.0) % 10) < 5)
	{
		ui.lcdNumberMin2->display(((int)((int)(qnode.lap_time / 60000.0) % 10)));
		ui.lcdNumberSec1->display(((int)((int)(qnode.lap_time / 10000.0) % 6)));
		ui.lcdNumberSec2->display(((int)((int)(qnode.lap_time / 1000.0) % 10)));
		ui.lcdNumberTri1->display(((int)((double)((int)qnode.lap_time % 1000) / 100.0)));
	}
	else
	{
		ui.lcdNumberMin2->display(((int)((int)(qnode.lap_time / 60000.0) % 10)));
		ui.lcdNumberSec1->display(0);
		ui.lcdNumberSec2->display(0);
		ui.lcdNumberTri1->display(0);

		ui.lcdNumberMin1->setPalette(Qt::red);
		ui.lcdNumberMin2->setPalette(Qt::red);
		ui.lcdNumberSec1->setPalette(Qt::red);
		ui.lcdNumberSec2->setPalette(Qt::red);
		ui.lcdNumberTri1->setPalette(Qt::red);
		ui.lcdNumberTri2->setPalette(Qt::red);
		ui.lcdNumberMin1->setAutoFillBackground(true);
		ui.lcdNumberMin2->setAutoFillBackground(true);
		ui.lcdNumberSec1->setAutoFillBackground(true);
		ui.lcdNumberSec2->setAutoFillBackground(true);
		ui.lcdNumberTri1->setAutoFillBackground(true);
		ui.lcdNumberTri2->setAutoFillBackground(true);
	}

	if (qnode.stopwatchStatus == 2)
	{
		ui.lcdNumberMin1->setPalette(Qt::green);
		ui.lcdNumberMin2->setPalette(Qt::green);
		ui.lcdNumberSec1->setPalette(Qt::green);
		ui.lcdNumberSec2->setPalette(Qt::green);
		ui.lcdNumberTri1->setPalette(Qt::green);
		ui.lcdNumberTri2->setPalette(Qt::green);
		ui.lcdNumberMin1->setAutoFillBackground(true);
		ui.lcdNumberMin2->setAutoFillBackground(true);
		ui.lcdNumberSec1->setAutoFillBackground(true);
		ui.lcdNumberSec2->setAutoFillBackground(true);
		ui.lcdNumberTri1->setAutoFillBackground(true);
		ui.lcdNumberTri2->setAutoFillBackground(true);
	}

}

void MainWindow::on_pButtonMissionTrafficLight_clicked(bool check)
{
	if (ui.rButtonMissionPass->isChecked())
	{
		ui.pButtonMissionTrafficLight->setStyleSheet("background-color: rgb(0,255,125)");
	}
	else if (ui.rButtonMissionFail->isChecked())
	{
		ui.pButtonMissionTrafficLight->setStyleSheet("background-color: rgb(255,0,0)");
	}
	else if (ui.rButtonMissionInit->isChecked())
	{
		qnode.pbInitStateTrafficLight();
	}
	else if (ui.rButtonMissionTest->isChecked())
	{
		qnode.pbTestStateTrafficLight();
	}
}

void MainWindow::on_pButtonMissionLevelCrossing_clicked(bool check)
{
	if (ui.rButtonMissionPass->isChecked())
	{
		ui.pButtonMissionLevelCrossing->setStyleSheet("background-color: rgb(0,255,125)");
	}
	else if (ui.rButtonMissionFail->isChecked())
	{
		ui.pButtonMissionLevelCrossing->setStyleSheet("background-color: rgb(255,0,0)");
	}
	else if (ui.rButtonMissionInit->isChecked())
	{
		qnode.pbInitStateLevelCrossing();
	}
	else if (ui.rButtonMissionTest->isChecked())
	{
		qnode.pbTestStateLevelCrossing();
	}
}

void MainWindow::on_pButtonMissionParkingLot_clicked(bool check)
{
	if (ui.rButtonMissionPass->isChecked())
	{
		ui.pButtonMissionParkingLot->setStyleSheet("background-color: rgb(0,255,125)");
	}
	else if (ui.rButtonMissionFail->isChecked())
	{
		ui.pButtonMissionParkingLot->setStyleSheet("background-color: rgb(255,0,0)");
	}
}

void MainWindow::on_pButtonMissionTunnel_clicked(bool check)
{
	if (ui.rButtonMissionPass->isChecked())
	{
		ui.pButtonMissionTunnel->setStyleSheet("background-color: rgb(0,255,125)");
	}
	else if (ui.rButtonMissionFail->isChecked())
	{
		ui.pButtonMissionTunnel->setStyleSheet("background-color: rgb(255,0,0)");
	}
}

void MainWindow::on_pButtonAllTestMode_clicked(bool check)
{
	qnode.pbTestStateTrafficLight();

	qnode.pbTestStateLevelCrossing();
}

void MainWindow::on_pButtonAllActiveMode_clicked(bool check)
{
	qnode.pbInitStateTrafficLight();

	qnode.pbInitStateLevelCrossing();
}

void MainWindow::on_pButtonInitStopwatch_clicked(bool check)
{
	qnode.pbInitStateStopwatch();

	ui.lcdNumberMin1->setAutoFillBackground(false);
	ui.lcdNumberMin2->setAutoFillBackground(false);
	ui.lcdNumberSec1->setAutoFillBackground(false);
	ui.lcdNumberSec2->setAutoFillBackground(false);
	ui.lcdNumberTri1->setAutoFillBackground(false);
	ui.lcdNumberTri2->setAutoFillBackground(false);
}

void MainWindow::on_pButtonInitAllTerminals_clicked(bool check)
{
	qnode.pbInitStateTrafficLight();

	qnode.pbInitStateLevelCrossing();
}

void MainWindow::on_pButtonInitAllStatus_clicked(bool check)
{
	qnode.pbInitStateTrafficLight();

	qnode.pbInitStateLevelCrossing();

	qnode.pbInitStateStopwatch();

	ui.lcdNumberMin1->setAutoFillBackground(false);
	ui.lcdNumberMin2->setAutoFillBackground(false);
	ui.lcdNumberSec1->setAutoFillBackground(false);
	ui.lcdNumberSec2->setAutoFillBackground(false);
	ui.lcdNumberTri1->setAutoFillBackground(false);
	ui.lcdNumberTri2->setAutoFillBackground(false);
}

// void MainWindow::on_button_connect_clicked(bool check ) {
// 	if ( ui.checkbox_use_environment->isChecked() ) {
// 		if ( !qnode.init() ) {
// 			showNoMasterMessage();
// 		} else {
// 			ui.button_connect->setEnabled(false);
// 		}
// 	} else {
// 		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
// 				   ui.line_edit_host->text().toStdString()) ) {
// 			showNoMasterMessage();
// 		} else {
// 			ui.button_connect->setEnabled(false);
// 			ui.line_edit_master->setReadOnly(true);
// 			ui.line_edit_host->setReadOnly(true);
// 			ui.line_edit_topic->setReadOnly(true);
// 		}
// 	}
// }


// void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
// 	bool enabled;
// 	if ( state == 0 ) {
// 		enabled = true;
// 	} else {
// 		enabled = false;
// 	}
// 	ui.line_edit_master->setEnabled(enabled);
// 	ui.line_edit_host->setEnabled(enabled);
// 	//ui.line_edit_topic->setEnabled(enabled);
// }

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
// void MainWindow::updateLoggingView() {
//         ui.view_logging->scrollToBottom();
// }
//
// /*****************************************************************************
// ** Implementation [Menu]
// *****************************************************************************/
//
// void MainWindow::on_actionAbout_triggered() {
//     QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
// }

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "rbiz_autorace_monitor");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    // ui.line_edit_master->setText(master_url);
    // ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    // ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    // ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	// ui.line_edit_master->setEnabled(false);
    	// ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "rbiz_autorace_monitor");
    // settings.setValue("master_url",ui.line_edit_master->text());
    // settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    // settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    // settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace rbiz_autorace_monitor
