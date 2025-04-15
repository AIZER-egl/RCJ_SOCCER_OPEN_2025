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

[[maybe_unused]] std::vector<BlobDetection::Blob> BlobDetection::detect(cv::Mat &frame) {
    cv::Mat mask;
    cv::inRange(frame, lower_bound, upper_bound, mask);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b) {
        return cv::contourArea(a) > cv::contourArea(b);
    });

    std::vector<Blob> blobs;

    for (const auto &contour : contours) {
        double area = cv::contourArea(contour);

        if (area > min_area && area < max_area) {
            cv::Rect rect = cv::boundingRect(contour);
            Blob blob = {
                    rect.x,
                    rect.y,
                    rect.width,
                    rect.height
            };

            blobs.push_back(blob);
        }
    }

    return blobs;
}

[[maybe_unused]] void BlobDetection::plot_blobs(cv::Mat &frame, std::vector<BlobDetection::Blob> &blobs, cv::Scalar color) {
    for (const auto &blob : blobs) {
        cv::rectangle(
            frame,
            cv::Point(blob.x, blob.y),
            cv::Point(blob.x + blob.w, blob.y + blob.h),
            color,
            2
        );
    }
}

