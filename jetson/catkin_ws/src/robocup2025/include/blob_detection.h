#ifndef OPENCV_BLOB_DETECTION_H
#define OPENCV_BLOB_DETECTION_H

#include <algorithm>
#include <optional>
#include <opencv4/opencv2/opencv.hpp>

class [[maybe_unused]] BlobDetection {
private:
	cv::Scalar lower_bound = cv::Scalar(0, 0, 0);
	cv::Scalar upper_bound = cv::Scalar(179, 255, 255);
	int min_area = 1000;
	int max_area = 100000;
public:
	struct Blob {
		int x;
		int y;
		int width;
		int height;

		long area() const {
			if (width < 0 || height < 0) return 0;
			return static_cast<long>(width) * height;
		}
	};

	BlobDetection();

	[[maybe_unused]] std::vector<Blob> detect(cv::Mat &frame);

	[[maybe_unused]] static void plot_blobs(cv::Mat &frame, std::vector<Blob> &blobs, cv::Scalar color);

	[[maybe_unused]] static std::optional<BlobDetection::Blob>  biggest_blob(std::vector<Blob> blobs);

	[[maybe_unused]] void set_area(int minArea, int maxArea);

	[[maybe_unused]] void set_color_range(cv::Scalar lowerBound, cv::Scalar upperBound);
};

#endif //OPENCV_BLOB_DETECTION_H
