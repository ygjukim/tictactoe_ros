/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "tictactoe_ros/globals.h"
#include "tictactoe_ros/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tictactoe_ros {

#define	X_OBJECT_ID			57
#define	O1_OBJECT_ID		66
#define	O2_OBJECT_ID		71

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv),
	camLinkFrameId_("camera_link"),
	objFramePrefix_("object")
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc, init_argv, "tictactoe_ros");

	if ( ! ros::master::check() ) {
		return false;
	}

	ros::start(); // explicitly needed since our node handle is going out of scope.

	ros::NodeHandle pnh("~");
	pnh.param("camera_link_frame_id", camLinkFrameId_, camLinkFrameId_);
	pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);

	ros::NodeHandle nh;
	// Add your ros communications here.
	//chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

	QString topic("/tictactoe_image");
//	QString topic("/camera/rgb/image_rect_color");
	QString transport("raw");

	image_transport::ImageTransport it(nh);
	image_transport::TransportHints hints(transport.toStdString());
	try {
		subscriber_ = it.subscribe(topic.toStdString(), 1, &QNode::callbackImage, this, hints);
		//qDebug("QNode::initImageTopic() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
	} catch (image_transport::TransportLoadException& e) {
		//QMessageBox::warning(NULL, tr("Loading image transport plugin failed"), e.what());
	}

	objectsSubs_ = nh.subscribe("objectsStamped", 1, &QNode::objectsDetectedCallback, this);

	tfListener_ =  new tf::TransformListener();

	start();
	
	return true;
}

#if 0
bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	
	ros::init(remappings,"tictactoe_ros");
	
	if ( ! ros::master::check() ) {
		return false;
	}
	
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	
	ros::NodeHandle nh;
	// Add your ros communications here.
	//chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

	QString topic("/tictactoe_image");
	QString transport("raw");

	image_transport::ImageTransport it(nh);
	image_transport::TransportHints hints(transport.toStdString());
	try {
		subscriber_ = it.subscribe(topic.toStdString(), 1, &QNode::callbackImage, this, hints);
		//qDebug("QNode::initImageTopic() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
	} catch (image_transport::TransportLoadException& e) {
		//QMessageBox::warning(NULL, tr("Loading image transport plugin failed"), e.what());
	}
	
	start();
	
	return true;
}
#endif

void QNode::run() {
	ros::Rate loop_rate(20);
//	int count = 0;
	while ( ros::ok() ) {
#if 0
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
#else
		ros::spinOnce();
		loop_rate.sleep();
#endif		
	}

  	subscriber_.shutdown();

	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
	try
	{
		// First let cv_bridge do its magic
		cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
		conversion_mat_ = cv_ptr->image;

//		if (num_gridlines_ > 0)
//		  	overlayGrid();
	}
	catch (cv_bridge::Exception& e)
	{
		try
		{
			// If we're here, there is no conversion that makes sense, but let's try to imagine a few first
			cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
			if (msg->encoding == "CV_8UC3")
			{
				// assuming it is rgb
				conversion_mat_ = cv_ptr->image;
			} else if (msg->encoding == "8UC1") {
				// convert gray to rgb
				cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
			} else if (msg->encoding == "16UC1" || msg->encoding == "32FC1") {
				// scale / quantify
				double min = 0;
				double max = 10.0;
//				double max = ui_.max_range_double_spin_box->value();
				if (msg->encoding == "16UC1") max *= 1000;
#if 0
				if (ui_.dynamic_range_check_box->isChecked())
				{
				  // dynamically adjust range based on min/max in image
				  cv::minMaxLoc(cv_ptr->image, &min, &max);
				  if (min == max) {
				    // completely homogeneous images are displayed in gray
				    min = 0;
				    max = 2;
				  }
				}
#endif
				cv::Mat img_scaled_8u;
				cv::Mat(cv_ptr->image-min).convertTo(img_scaled_8u, CV_8UC1, 255. / (max - min));
				cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
			} else {
				qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
				// ui_.image_frame->setImage(QImage());
				Q_EMIT imageUpdated(new QImage());
				return;
			}
		}
		catch (cv_bridge::Exception& e)
		{
			qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
			//ui_.image_frame->setImage(QImage());
			Q_EMIT imageUpdated(new QImage());
			return;
		}
	}

	// image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
//	QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
//	Q_EMIT imageUpdated((QImage *)&image);
	Q_EMIT imageUpdated(new QImage(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888));

#if 0
	if (!ui_.zoom_1_push_button->isEnabled())
	{
		ui_.zoom_1_push_button->setEnabled(true);
	}

	// Need to update the zoom 1 every new image in case the image aspect ratio changed,
	// though could check and see if the aspect ratio changed or not.
	onZoom1(ui_.zoom_1_push_button->isChecked());
#endif	
}

#define	CELL_WIDTH		0.06
#define	ALLOWABLE_GAP	0.01
#define	Z_OFFSET		0.15

int QNode::convertToCellIndex(float y, float z) {
	int pos = -1;

	y -= CELL_WIDTH;
	y = fabs(y);
	if (y < ALLOWABLE_GAP)  pos = 0;
	else if (y < CELL_WIDTH + ALLOWABLE_GAP) pos = 3;
	else if (y < 2 * CELL_WIDTH + ALLOWABLE_GAP) pos = 6;

	if (pos >= 0) {
		z -= (Z_OFFSET + CELL_WIDTH);
		z = fabs(z);
		if (z > ALLOWABLE_GAP) {
			if (z < CELL_WIDTH + ALLOWABLE_GAP) pos += 1;
			else if (z < 2 * CELL_WIDTH + ALLOWABLE_GAP) pos += 2;
		}
	}

//	ROS_INFO("[%f, %f] ==> %d", y, z, pos);

	return pos;
}

void QNode::objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg)
{
	if(msg->objects.data.size())
	{
		for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
		{
			// get data
			int id = (int)msg->objects.data[i];
			std::string objectFrameId = QString("%1_%2").arg(objFramePrefix_.c_str()).arg(id).toStdString(); // "object_1", "object_2"

			tf::StampedTransform pose;
			try
			{
				// Get transformation from "object_#" frame to target frame "camera_link"
				// The timestamp matches the one sent over TF
				tfListener_->lookupTransform(camLinkFrameId_, objectFrameId, msg->header.stamp, pose);
			}
			catch(tf::TransformException & ex)
			{
				ROS_WARN("%s",ex.what());
				continue;
			}

			int pos = convertToCellIndex(pose.getOrigin().y(), pose.getOrigin().z());
			if (pos < 0)  continue;
			int type = (id == X_OBJECT_ID) ? X : O;

			Q_EMIT objectDetected(pos, type);
		}
	}
}

}  // namespace tictactoe_ros
