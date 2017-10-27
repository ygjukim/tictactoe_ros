/**
 * @file /include/tictactoe_ros/main_window.hpp
 *
 * @brief Qt based gui for tictactoe_ros.
 *
 * @date November 2010
 **/
#ifndef tictactoe_ros_MAIN_WINDOW_H
#define tictactoe_ros_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <QImage>

#include "gamewidget.h"
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace tictactoe_ros {

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

	void showNoMasterMessage();

	//
	// Overloaded functions
	// 
	void closeEvent(QCloseEvent *event);

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
//	void on_actionAbout_triggered();
//	void on_button_connect_clicked(bool check );
//	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
//    void updateLoggingView(); // no idea why this can't connect automatically
	void onImageUpdated(QImage* image);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace tictactoe_ros

#endif // tictactoe_ros_MAIN_WINDOW_H
