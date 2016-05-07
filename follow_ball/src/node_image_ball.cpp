#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "follow_ball/Point.h"


class ImageCar {

public:
	ImageCar() :
			it_(nh_) {
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1000, &ImageCar::imageCb, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1000);
		point_pub_ = nh_.advertise<follow_ball::Point>("/Point", 1000);
		point_.x  = 0.0;
		point_.y = 0.0;
	}

	void drawCentralRows(cv_bridge::CvImagePtr cv_ptr, int row) {
		// Draw an example circle on the video stream
		if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
			cv::rectangle(cv_ptr->image, cv::Point(0, row - 1),
					cv::Point(639, row - 1), CV_RGB(0, 0, 255)); // The line is printer in the image really.
	}

	void drawTraceShift(cv_bridge::CvImagePtr cv_ptr, int posLine) {
		// Draw variable line
		if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
			cv::rectangle(cv_ptr->image, cv::Point(posLine, 0),
					cv::Point(posLine, 639), CV_RGB(0, 255, 0));
	}

	void drawTraceSize(cv_bridge::CvImagePtr cv_ptr, int i) {
		// Draw variable line
		if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
			cv::rectangle(cv_ptr->image, cv::Point(i, 0), cv::Point(i, 639),
					CV_RGB(255, 255, 0));
	}

	bool isWhite(cv_bridge::CvImagePtr cv_ptr, int posdata) {
		return cv_ptr->image.data[posdata] == MAX_SAT &&
				cv_ptr->image.data[posdata + 1] == MAX_SAT &&
				cv_ptr->image.data[posdata + 2] == MAX_SAT;
	}

	int moveCorrection() {
		return 0;
	}

	void traceBall(cv_bridge::CvImagePtr cv_ptr) {
		drawTraceShift(cv_ptr, point_.x);
		drawCentralRows(cv_ptr , point_.y);
	}

	bool isSurface(cv_bridge::CvImagePtr cv_ptr , int posdata , int i , int j , int step , int channels)
	{
		int countSurface = 0;
		for (int k = 0 ; k < BALLSURFACE ; k++)
			if (isWhite(cv_ptr , i * step + (j + k) * channels) , isWhite(cv_ptr , (i + k) * step + j * channels))
				countSurface++;
			else
				countSurface = 0;

		for (int k = BALLSURFACE ; k >= 0 ; k--)
			if (isWhite(cv_ptr , i * step + (j + k) * channels) , isWhite(cv_ptr , (i + k) * step + j * channels))
				countSurface++;
			else
				countSurface = 0;


		if (countSurface >= BALLSURFACE)
			return true;
		return false;
	}

	void getBall(cv_bridge::CvImagePtr cv_ptr, int width, int heigth, int step, int channels) {

		int posdata = 0;
		float countWhites = 0;

		point_.x = 0;
		point_.y = 0;
		for (int i = 0; i < heigth; i++){
			for (int j = 0; j < width; j++){
				posdata = i * step + j * channels;
				if (isWhite(cv_ptr , posdata)  && isSurface(cv_ptr , posdata , i , j , step , channels))
				{
					point_.x += j;
					point_.y += i;
					countWhites = countWhites + 1;
				}
			}
		}
		if (countWhites > BALLSURFACE)
		{
			point_.x = int (point_.x / countWhites);
			point_.y = int (point_.y / countWhites);
		}
		else
		{
			point_.x = 0;
			point_.y = 0;
		}
	}



	void imageBall(cv_bridge::CvImagePtr cv_ptr) {
		int width = cv_ptr->image.cols;
		int step = cv_ptr->image.step;
		int heigth = cv_ptr->image.rows;
		int channels = 3;  //RGB

		getBall(cv_ptr, width, heigth, step, channels);
		traceBall(cv_ptr);

	}

	void binarySegment(cv_bridge::CvImagePtr cv_ptr, cv::Mat hsv) {
		int height = hsv.rows;
		int width = hsv.cols;
		int step = hsv.step;
		int channels = 3;  //RGB
		int posdata = 0;

		// Filter the image.
		for (int i = 0; i < height; i++)
			for (int j = 0; j < width; j++) {
				posdata = i * step + j * channels;
				if  (
						(hsv.data[posdata] >= MIN_HUE && hsv.data[posdata] <= MAX_HUE) &&
						(hsv.data[posdata + 1] >= MIN_SAT && hsv.data[posdata + 1] <= MAX_SAT) &&
						(hsv.data[posdata + 2] >= MIN_INTENSITY && hsv.data[posdata + 2] <= MAX_INTENSITY)
					)
				{
					cv_ptr->image.data[posdata] = MAX_SAT;
					cv_ptr->image.data[posdata + 1] = MAX_SAT;
					cv_ptr->image.data[posdata + 2] = MAX_SAT;
				} else {
					cv_ptr->image.data[posdata] = 0;
					cv_ptr->image.data[posdata + 1] = 0;
					cv_ptr->image.data[posdata + 2] = 0;
				}

			}
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg) {
		// Copy image to convert with openvc SOURCE IMAGE MSG
		cv_bridge::CvImagePtr cv_ptr;
		try {

			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

		} catch (cv_bridge::Exception& e) {

			ROS_ERROR("cv_bridge " "exception: %s", e.what());
			return;

		}

		cv::Mat hsv;
		cv::cvtColor(cv_ptr->image, hsv, CV_RGB2HSV);

		// Correction of Image...
		binarySegment(cv_ptr, hsv); // Filter by HSV values... saturation in backgraound green.
		imageBall(cv_ptr);

		cv::imshow("Image filtered", cv_ptr->image);
		// Update GUI Window
		cv::waitKey(3);

		// Output modified video stream
		point_pub_.publish(point_);
		image_pub_.publish(cv_ptr->toImageMsg());
	}

private:

	const static int MAX_HUE = 155;
	const static int MIN_HUE = 111;

	const static int MAX_SAT = 255;
	const static int MIN_SAT = 178;

	const static int MAX_INTENSITY = 255;
	const static int MIN_INTENSITY = 0;

	const static int BALLSURFACE = 1; /* Contrast noise of image in real robot */

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Publisher point_pub_;
	follow_ball::Point point_;
	ros::Time inLine_ts_;

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "node_image");
	ImageCar ic;
	ros::spin();
	return 0;
}
