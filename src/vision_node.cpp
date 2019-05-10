/**
 * Example node for reading from the Raspberry PiCam and processing it for
 * AprilTag data.
 */

#include <iostream>

#include <ros/ros.h>

#include <camera_info_manager/camera_info_manager.h>

// raspberry pi cam and image handling
#include <raspicam/raspicam_cv.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// apriltag library
extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag16h5.h>
}

// topics
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>


/**
 * The node contents in wrapped into a class to allow for easier handling of
 * shared information between the callbacks and main loop
 */
class VisionNode {

public:

	/**
	 * constructor for the vision node
	 * @param frame_width   the integer width of the image frame in pixels
	 * @param frame_height  the integer height of the image frame in pixels
	 * @param publish_image boolean flag for whether or not to publish the images
	 */
	VisionNode(sensor_msgs::CameraInfo cam_info, int tag_id, float tag_size);


	/**
	 * the main loop to be run for this node (called by the `main` function)
	 * @return exit code
	 */
	int run();

	inline void setDebug(bool debug) { _debug = debug; };

private:

	// node handler and image transport handler
	ros::NodeHandle _nh;
	image_transport::ImageTransport _it;

	int _tag_id;			// the id of the tag to compute a range estimate to
	float _tag_size;

	bool _debug = false;	// true to add debug data to the image when publishing

	// settings, etc
	sensor_msgs::CameraInfo _camera_info;

	// camera stuff
	raspicam::RaspiCam_Cv _camera;	// the camera object

	// publishers
	ros::Publisher _tag_relative_position_pub;	// the relative position vector to the truck (NOT IMPLEMENTED)
	ros::Publisher _tag_details_pub;			// the raw tag details (for debugging) (NOT IMPLEMENTED)
	image_transport::Publisher _image_pub;		// the raw annotated image (for debugging)


	void annotateImage(cv::Mat image, apriltag_detection_t* det);
};


VisionNode::VisionNode(sensor_msgs::CameraInfo cam_info, int tag_id, float tag_size) :
_camera_info(cam_info),
_tag_id(tag_id),
_tag_size(tag_size),
_it(_nh)
{
	// publishers
	_image_pub = _it.advertise("image", 1);	// NOTE: should not be used in flight
	_tag_relative_position_pub = _nh.advertise<geometry_msgs::PoseStamped>("landing_pose", 1);	// EXAMPLE publishing


    // configure the camera
    _camera.set(cv::CAP_PROP_FORMAT, CV_8UC1);					// 8 bit image data -> means grayscale image
    _camera.set(cv::CAP_PROP_FRAME_WIDTH, _camera_info.width);		// set the width of the image
	_camera.set(cv::CAP_PROP_FRAME_HEIGHT, _camera_info.height);	// set the height of the image
}


int VisionNode::run() {

    // open the camera
    ROS_INFO("opening camera");
	if (!_camera.open()) {
        ROS_ERROR("Error opening the camera");
        std::cerr << "Error opening the camera" << std::endl;
        return -1;
    }

    // apriltag handling setup
    // see readme for details: https://github.com/AprilRobotics/apriltag
	apriltag_family_t *tf = tag16h5_create();	// specify the tag family

	// initialize the detector and some attributes for detection
	apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 3.0;
    td->quad_sigma = 0.0;
    td->refine_edges = 0;
    //td->decode_sharpening = 0.25;


    // initialize the detection information with the general info
    apriltag_detection_info_t info;
	info.tagsize = _tag_size;
	info.fx = _camera_info.K[0];
	info.fy = _camera_info.K[4];
	info.cx = _camera_info.K[2];
	info.cy = _camera_info.K[5];


	ros::Time image_time; 	// timestamp of when the image was grabbed
	cv::Mat frame_gray;		// the image in grayscale

	// loop forever while the node should be running
	while (ros::ok()) {

		// grab the frame from the camera
		// see readme for details: https://github.com/cedricve/raspicam
        _camera.grab();
		_camera.retrieve(frame_gray);
		image_time = ros::Time::now();

        // Make an image_u8_t header for the Mat data to pass over to the april
        // tag detector
        image_u8_t im = { .width = frame_gray.cols,
            .height = frame_gray.rows,
            .stride = frame_gray.cols,
            .buf = frame_gray.data
        };

        // run the detector
		zarray_t *detections = apriltag_detector_detect(td, &im);
        ROS_INFO("%d tags detected", zarray_size(detections));

        // NOTE: this is where the relative vector should be computed

        // if flagged to do so, annotate and publish the image

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);


            if (det->id != _tag_id) {
            	continue;
            }

            // set the detection to the detection info
            info.det = det;

			// Then call estimate_tag_pose
			apriltag_pose_t pose;
			double err = estimate_tag_pose(&info, &pose);

			// get the range information
			float rx = pose.t->data[0];
			float ry = pose.t->data[1];
			float rz = pose.t->data[2];

			// TODO: publish the range information
			geometry_msgs::PoseStamped range_msg;
			range_msg.header.stamp = ros::Time::now();
			range_msg.pose.position.x = rx;
			range_msg.pose.position.y = ry;
			range_msg.pose.position.z = rz;

			// TODO: use the rotation matrix to compute the heading of the tag
			// TODO: this would allow you to align yourself with the tag, but we will ignore this for now

			// publish the range message
			_tag_relative_position_pub.publish(range_msg);

            // Draw detection outlines
			if (_debug) {
				annotateImage(frame_gray, det);
			}

        }

		// publish the image
		std_msgs::Header header;
		header.stamp = image_time;
		sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "mono8", frame_gray).toImageMsg();
		_image_pub.publish(img_msg);

        // clean up the detections
        zarray_destroy(detections);

        // call spin once to trigger any callbacks
        // while there are none specified so far, it's good practice to just
        // throw this in so if/when you add callbacks you don't spend hours
        // trying to figure out why they aren't being called!
		ros::spinOnce();
	}

    // need to stop the camera
    ROS_INFO("stopping camera");
	_camera.release();

    // remove apriltag stuff
	apriltag_detector_destroy(td);
	tag16h5_destroy(tf);

}

void VisionNode::annotateImage(cv::Mat image, apriltag_detection_t *det) {
	line(image, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[1][0], det->p[1][1]), cv::Scalar(0, 0xff, 0), 2);
	line(image, cv::Point(det->p[0][0], det->p[0][1]), cv::Point(det->p[3][0], det->p[3][1]), cv::Scalar(0, 0, 0xff), 2);
	line(image, cv::Point(det->p[1][0], det->p[1][1]), cv::Point(det->p[2][0], det->p[2][1]), cv::Scalar(0xff, 0, 0), 2);
	line(image, cv::Point(det->p[2][0], det->p[2][1]), cv::Point(det->p[3][0], det->p[3][1]), cv::Scalar(0xff, 0, 0), 2);

	std::stringstream ss;
	ss << det->id;
	cv::String text = ss.str();

	int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
	double fontscale = 1.0;
	int baseline;
	cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2, &baseline);
	cv::putText(image, text, cv::Point(det->c[0]-textsize.width/2, det->c[1]+textsize.height/2), fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
}


// the main function


int main(int argc, char **argv) {

	// initialize th enode
	ros::init(argc, argv, "vision_node");

	// get parameters from the launch file which define some camera settings
	ros::NodeHandle private_nh("~");
	int tag_id;
	float tag_size;
	bool debug;
	std::string camera_url;
	std::string camera_name;
	private_nh.param("camera_name", camera_name, std::string("pi1280x720"));
	private_nh.param("camera_url", camera_url, std::string("package://aa241x_vision/camera_info/pi1280x720.yaml"));
	private_nh.param("debug", debug, false);
	private_nh.param("tag_id", tag_id, 0);
	private_nh.param("tag_size", tag_size, 0.09f);

	// use the camera manager to load up the configuration
	camera_info_manager::CameraInfoManager manager(private_nh, camera_name, camera_url);

	if (!manager.loadCameraInfo(camera_url)) {
		ROS_ERROR("no calibration file found");
		return EXIT_FAILURE;
	}

	sensor_msgs::CameraInfo cam_info = manager.getCameraInfo();

	// create the node
	VisionNode node(cam_info, tag_id, tag_size);
	node.setDebug(debug);

	// run the node
	return node.run();
}
