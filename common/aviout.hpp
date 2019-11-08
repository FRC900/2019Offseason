#pragma once
#include <opencv2/highgui/highgui.hpp>
#include "mediaout.hpp"

// Class to write MJPEG AVI files using
// OpenCV's VideoWriter class
class AVIOut : public MediaOut
{
	public:
		AVIOut(const char *outFile, const cv::Size &size, int framesPerFile = 250, int frameSkip = 1);
		~AVIOut();

		// Make non-copyable
		AVIOut(const AVIOut &aviout) = delete;
		AVIOut& operator=(const AVIOut &aviout) = delete;

	private :
		bool openNext(int fileCounter) override;
		bool write(const cv::Mat &frame, const cv::Mat &depth) override;

		cv::Size         size_;
		cv::VideoWriter *writer_;
		std::string      fileName_;
};
