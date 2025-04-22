#include "blob_detection.h"

#include <utility>

BlobDetection::BlobDetection() = default;

[[maybe_unused]] void BlobDetection::set_color_range(cv::Scalar lowerBound, cv::Scalar upperBound) {
    this->lower_bound = std::move(lowerBound);
    this->upper_bound = std::move(upperBound);
}

[[maybe_unused]] void BlobDetection::set_area(int minArea, int maxArea) {
    this->min_area = minArea;
    this->max_area = maxArea;
}

[[maybe_unused]] std::vector<BlobDetection::Blob> BlobDetection::detect(cv::cuda::GpuMat &frame) {
	cv::cuda::GpuMat gpu_processed_frame, gpu_mask;
	cv::Mat mask_cpu;

	cv::cuda::cvtColor(frame, gpu_processed_frame, cv::COLOR_BGR2HSV);
	cv::cuda::inRange(gpu_processed_frame, lower_bound, upper_bound, gpu_mask);
	gpu_mask.download(mask_cpu);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	// Detect blobs using GPU is extremely difficult since no native cv::cura blob-detection-functions are available
	cv::findContours(mask_cpu, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
		return cv::contourArea(a) > cv::contourArea(b);
	});

	std::vector<Blob> blobs;

	for (const auto &contour : contours) {
		double area = cv::contourArea(contour);

		if (area >= min_area && area <= max_area) {
			cv::Rect rect = cv::boundingRect(contour);
			blobs.push_back({rect.x, rect.y, rect.width, rect.height});
		}
	}

	return blobs;
}

[[maybe_unused]] void BlobDetection::plot_blobs(cv::Mat &frame, std::vector<BlobDetection::Blob> &blobs, cv::Scalar color) {
    for (const auto &blob : blobs) {
        cv::rectangle(
            frame,
            cv::Point(blob.x, blob.y),
            cv::Point(blob.x + blob.width, blob.y + blob.height),
            color,
            2
        );
    }
}

