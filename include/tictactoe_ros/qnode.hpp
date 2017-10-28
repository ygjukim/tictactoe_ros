/**
 * @file /include/tictactoe_ros/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef tictactoe_ros_QNODE_HPP_
#define tictactoe_ros_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/macros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <find_object_2d/ObjectsStamped.h>
#include <tf/transform_listener.h>

#include <opencv2/core/core.hpp>

#include <QImage>
#include <QThread>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tictactoe_ros {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT

public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void objectDetected(int pos, int type);
	void imageUpdated(QImage* image);
	void loggingUpdated();
    void rosShutdown();

protected:
	void callbackImage(const sensor_msgs::Image::ConstPtr& msg);
	void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg);

private:
	int init_argc;
	char** init_argv;

//	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

	image_transport::Subscriber subscriber_;
	cv::Mat conversion_mat_;

	std::string camLinkFrameId_;
	std::string objFramePrefix_;
    ros::Subscriber objectsSubs_;
    tf::TransformListener *tfListener_;

    int convertToCellIndex(float y, float z);
};

}  // namespace tictactoe_ros

#endif /* tictactoe_ros_QNODE_HPP_ */
