#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Float32.h"



#define INLINE_TIME 13.0


class ImageCar
{

public:
  ImageCar()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1000 , &ImageCar::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1000);
    shift_pub_ =nh_.advertise<std_msgs::Float32>("/Shift" , 1000 );
    shift_.data = 320;
    right_ = false;
    left_ = false;
    outLine_ = false;
    usedBeg_ = 0;
    usedEnd_ = 0;
    lastShift_ = 0;
  }

  ~ImageCar()
  {
	  ;
  }

	void drawCentralRows(cv_bridge::CvImagePtr cv_ptr)
	{
		// Draw an example circle on the video stream
		if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
		   cv::rectangle(cv_ptr->image , cv::Point(0 , POS_LINE_HOR - 1) , cv::Point(639 , POS_LINE_HOR - 1)  , CV_RGB(0, 0, 255)); // The line is printer in the image really.
	}

	void drawVariableShift(cv_bridge::CvImagePtr cv_ptr)
	{
	    // Draw variable line
	    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	    	cv::rectangle(cv_ptr->image , cv::Point(shift_.data  , 0) , cv::Point(shift_.data , 639) , CV_RGB(255 , 0 , 0));
	}

	void drawTraceShift(cv_bridge::CvImagePtr cv_ptr , int posLine)
	{
	    // Draw variable line
	    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	    	cv::rectangle(cv_ptr->image , cv::Point(posLine  , 0) , cv::Point(posLine , 639) , CV_RGB(0 , 255 , 0));
	}

	void drawVariableResults(cv_bridge::CvImagePtr cv_ptr , int i)
	{
	    // Draw variable line
	    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	    	cv::rectangle(cv_ptr->image , cv::Point(results_[i] , 0) , cv::Point(results_[i] , 639) , CV_RGB(0 , 255 , 255));
	}

	void drawTraceSize(cv_bridge::CvImagePtr cv_ptr , int i)
	{
	    // Draw variable line
		if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
		    cv::rectangle(cv_ptr->image , cv::Point(i , 0) , cv::Point(i , 639) , CV_RGB(255 , 255 , 0));
	}


	bool isWhite (cv_bridge::CvImagePtr cv_ptr , int posdata)
	{
		return cv_ptr->image.data[posdata] == MAX_SAT && cv_ptr->image.data[posdata+1] == MAX_SAT && cv_ptr->image.data[posdata+2] == MAX_SAT;
	}

	bool isEndLine (cv_bridge::CvImagePtr cv_ptr , int posdata , int* countBlacks , int i , int width)
	{
		bool begLine = true;

		/* Count Black */
		if (!isWhite(cv_ptr , posdata))
			*countBlacks = *countBlacks + 1;
		else{
			*countBlacks = 0;
			posEnd_[usedEnd_] = 0;
		}

		if (*countBlacks == 1)
			posEnd_[usedEnd_] = i;

		if (*countBlacks == LINE_LENGTH)
		{
			usedEnd_ = usedEnd_ + 1;
			*countBlacks = 0;
			begLine = false;
		}

		return begLine;
	}

	bool isBegLine (cv_bridge::CvImagePtr cv_ptr , int posdata , int* countWhites , int i)
	{
		bool begLine = false;

		/* Count Begin */
		if (isWhite(cv_ptr, posdata))
			*countWhites = *countWhites + 1;
		else
		{
			*countWhites = 0;
			posBeg_[usedBeg_] = 0;
		}

		if (*countWhites == 1)
			posBeg_[usedBeg_] = i;

		if (*countWhites == LINE_LENGTH)
		{
			usedBeg_ = usedBeg_ + 1;
			*countWhites = 0;
			begLine = true;
		}
		return begLine;
	}

	void getLines(cv_bridge::CvImagePtr cv_ptr , int width , int step , int channels)
	{
		usedBeg_ = 0;
		usedEnd_ = 0;

		int posdata = 0;
		int countWhites = 0;
		int countBlacks = 0;
		bool begLine = false;

		for (int i = 0 ; i < width && usedBeg_ < MAX_LINES ; i++)
		{
			posdata = POS_LINE_HOR * step + i * channels;

			if (!begLine)
				begLine = isBegLine(cv_ptr , posdata , &countWhites , i);
			else
				begLine = isEndLine(cv_ptr , posdata , &countBlacks , i , width);
		}
		/* Limit of image in line ... whites*/
		if (begLine)
		{
			posEnd_[usedEnd_] = width;
			usedEnd_ = usedEnd_ + 1;
		}
	}

	bool inLine ()
	{
		bool isline = false;
		for (int i = 0 ; i < MAX_LINES && !isline ; i++)
			isline = isline || (posBeg_[i] > 0 && posEnd_[i] > 0);
		return isline;
	}

	int distance (int i)
	{
		return abs(results_[i] - lastShift_);
	}

	bool isLeft()
	{
		return lastShift_ > results_[0] && results_[0] != 0;
	}

	bool isRight() {
		return results_[1] > lastShift_ && results_[0] != 0;
	}

	void detectOthLines(cv_bridge::CvImagePtr cv_ptr) {

		/* There is a line on the right! */
		if (isRight() && !outLine_ && !right_)
		{
			inLine_ts_ = ros::Time::now();
			left_ = false;
			right_ = true;
		}
		/* There is a line on the left */
		if (isLeft() && !outLine_ && !left_)
		{
			inLine_ts_ = ros::Time::now();
			right_ = false;
			left_ = true;
		}

		/* We have found the line again */
		if (outLine_ && left_ && results_[0] != 0)
			left_ = false;

		if (outLine_ && right_ && results_[0] != 0)
			right_ = false;


		if((ros::Time::now()-inLine_ts_).toSec() > INLINE_TIME && !outLine_)
		{
			right_ = false;
			left_ = false;
			inLine_ts_ = ros::Time::now();
		}

		if (left_)
			drawTraceSize(cv_ptr, 100);

		if (right_)
			drawTraceSize(cv_ptr, 500);
	}

	void initResults() {
		for (int i = 0; i < MAX_LINES; i++)
			results_[i] = 0;
	}

	int moveCorrection() {
		int result = 0;

		/* Correcter normal*/
		if (usedBeg_ > 1 && !outLine_)
			/* There  is more than one line!!! */
			if (distance(0) < distance(1))
				result = results_[0];
			else
				result = results_[1];
		else if (!outLine_)
			/* Only one line */
			result = results_[0];

		if (outLine_)
		{
			if (right_)
				result = 640;
			else if (left_)
				result = 0;
			else
				result = 320;
		}


		return result;
	}

	void traceLines(cv_bridge::CvImagePtr cv_ptr) {
		for (int i = 0; i < usedBeg_; i++)
			drawTraceShift(cv_ptr, posBeg_[i]);

		for (int i = 0; i < usedEnd_; i++)
			drawTraceShift(cv_ptr, posEnd_[i]);
	}

	void calcResults(cv_bridge::CvImagePtr cv_ptr) {
		initResults();
		for (int i = 0; i < MAX_LINES && (i < usedBeg_ || i < usedEnd_); i++) {
			results_[i] = (posBeg_[i] + posEnd_[i]) / 2;
			drawVariableResults(cv_ptr, i);
		}
	}

	float correct(cv_bridge::CvImagePtr cv_ptr) {
		int width = cv_ptr->image.cols;
		int step = cv_ptr->image.step;
		int channels = 3;  //RGB
		int result = 0;

		getLines(cv_ptr, width, step, channels);
		traceLines(cv_ptr);

		detectOthLines(cv_ptr);


		/* We have found the line */
		if (results_[0] != 0 && outLine_)
		{
			/* We are in line!!!! */
			ROS_ERROR("INLINE!!!");
			outLine_ = false;
			inLine_ts_ = ros::Time::now();
		}
		calcResults(cv_ptr);

		/* We have Lost the line */
		if (results_[0] == 0 && !outLine_) {
			/* We have lost the line!! */
			ROS_ERROR("OUTLINE!!!!!");
			outLine_ = true;
		}

		return moveCorrection();
	}


	void binarySegment (cv_bridge::CvImagePtr cv_ptr , cv::Mat hsv)
	{
		int height = hsv.rows;
		int width = hsv.cols;
		int step = hsv.step;
		int channels = 3;  //RGB
		int posdata = 0;

		// Filter the image.
		for(int i = 0 ; i < height ; i	++)
			for(int j = 0; j < width; j++)
			{
				posdata = i*step+j*channels;
				if  (
						(hsv.data[posdata] >= MIN_HUE && hsv.data[posdata] <= MAX_HUE) &&
						(hsv.data[posdata+1] >= MIN_SAT && hsv.data[posdata+1] <= MAX_SAT) &&
						(hsv.data[posdata+2] >= MIN_INTENSITY && hsv.data[posdata+2] <= MAX_INTENSITY)
					)
				{
					cv_ptr->image.data[posdata] = 0;
					cv_ptr->image.data[posdata+1] = 0;
					cv_ptr->image.data[posdata+2] = 0;
				}
				else
				{
					cv_ptr->image.data[posdata] = MAX_SAT;
					cv_ptr->image.data[posdata + 1] = MAX_SAT;
					cv_ptr->image.data[posdata + 2] = MAX_SAT;
				}
			}
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg) {
		// Copy image to convert with openvc SOURCE IMAGE MSG
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg,
					sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge " "exception: %s", e.what());
			return;
		}

		cv::Mat hsv;
		cv::cvtColor(cv_ptr->image, hsv, CV_RGB2HSV);

		// Correction of Image...
		binarySegment(cv_ptr, hsv); // Filter by HSV values... saturation in backgraound green.
		drawCentralRows(cv_ptr);
		shift_.data = correct(cv_ptr);
		lastShift_ = int(shift_.data); /* Save last  Shift_ */
		ROS_ERROR("LAST SHIFT : %d" , lastShift_);
		ROS_ERROR("RESULT 0 : %d" , results_[0]);
		ROS_ERROR("RESULT 1 : %d" , results_[1]);
		drawVariableShift(cv_ptr);

		//drawTraceShift(cv_ptr , shift_.data - ABS_SHIFT);
		//drawTraceShift(cv_ptr , shift_.data + ABS_SHIFT);

		cv::imshow("Image filtered", cv_ptr->image);
		// Update GUI Window
		cv::waitKey(3);

		// Output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());
		shift_pub_.publish(shift_);
	}

private:

	const static int MAX_HUE = 91;
	const static int MIN_HUE = 21;

	const static int MAX_SAT = 255;
	const static int MIN_SAT = 40;

	const static int MAX_INTENSITY = 255;
	const static int MIN_INTENSITY = 0;

	const static int POS_LINE_HOR = 400;
	const static int LINE_LENGTH = 25;
	const static int ABS_SHIFT = 10;
	const static int MAX_LINES = 3;
	const static int NEXT_LINE = 30;

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Publisher shift_pub_;
	std_msgs::Float32 shift_;
	ros::Time inLine_ts_;

	bool right_;
	bool left_;
	bool outLine_;
	int results_[MAX_LINES];
	int posBeg_[MAX_LINES];
	int posEnd_[MAX_LINES];
	int usedBeg_;
	int usedEnd_;
	int lastShift_;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "node_image");
	ImageCar ic;
	ros::spin();
	return 0;
}
