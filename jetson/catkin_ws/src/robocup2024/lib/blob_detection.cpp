#include "blob_detection.h"

#include <utility>

// https://github.com/opencv/opencv/issues/6295
__global__ void inRange_kernel(const cv::cuda::PtrStepSz<uchar3> src, cv::cuda::PtrStepSzb dst,
							   int lbc0, int ubc0, int lbc1, int ubc1, int lbc2, int ubc2) {
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;

	if (x >= src.cols || y >= src.rows) return;

	uchar3 v = src(y, x);
	if (v.x >= lbc0 && v.x <= ubc0 && v.y >= lbc1 && v.y <= ubc1 && v.z >= lbc2 && v.z <= ubc2)
		dst(y, x) = 255;
	else
		dst(y, x) = 0;
}

void inRange_gpu(cv::cuda::GpuMat &src, cv::Scalar &lowerb, cv::Scalar &upperb,
				 cv::cuda::GpuMat &dst) {
	const int m = 32;
	int numRows = src.rows, numCols = src.cols;
	if (numRows == 0 || numCols == 0) return;
	// Attention! Cols Vs. Rows are reversed
	const dim3 gridSize(ceil((float)numCols / m), ceil((float)numRows / m), 1);
	const dim3 blockSize(m, m, 1);

	inRange_kernel<<<gridSize, blockSize>>>(src, dst, lowerb[0], upperb[0], lowerb[1], upperb[1],
											lowerb[2], upperb[2]);
}

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
	inRange_gpu(gpu_processed_frame, lower_bound, upper_bound, gpu_mask);
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

