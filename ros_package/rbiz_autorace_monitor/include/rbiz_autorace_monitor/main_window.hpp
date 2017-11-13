/**
 * @file /include/rbiz_autorace_monitor/main_window.hpp
 *
 * @brief Qt based gui for rbiz_autorace_monitor.
 *
 * @date November 2010
 **/
#ifndef rbiz_autorace_monitor_MAIN_WINDOW_H
#define rbiz_autorace_monitor_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace rbiz_autorace_monitor {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	// void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_pButtonMissionTrafficLight_clicked(bool check);
	void on_pButtonMissionLevelCrossing_clicked(bool check);
	void on_pButtonMissionParkingLot_clicked(bool check);
	void on_pButtonMissionTunnel_clicked(bool check);

	void on_pButtonAllTestMode_clicked(bool check);

	void on_pButtonAllActiveMode_clicked(bool check);


	void on_pButtonInitStopwatch_clicked(bool check);

	void on_pButtonInitAllTerminals_clicked(bool check);

	void on_pButtonInitAllStatus_clicked(bool check);

	void fnCheckLapTime();

	// void on_button_connect_clicked(bool check );
	// void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
	// void fnUpdateLapTime();
    // void updateLoggingView(); // no idea why this can't connect automatically

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace rbiz_autorace_monitor

#endif // rbiz_autorace_monitor_MAIN_WINDOW_H
