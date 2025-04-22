#ifndef OPENCV_BLOB_DETECTION_H
#define OPENCV_BLOB_DETECTION_H

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>

class [[maybe_unused]] BlobDetection {
private:
    cv::Scalar lower_bound = cv::Scalar(0, 0, 0);
    cv::Scalar upper_bound = cv::Scalar(255, 255, 255);
    int min_area = 1000;
    int max_area = 100000;
public:
    struct Blob {
        int x;
        int y;
        int width;
        int height;
    };

    BlobDetection();

    [[maybe_unused]] std::vector<Blob> detect(cv::cuda::GpuMat &frame);

    [[maybe_unused]] static void plot_blobs(cv::Mat &frame, std::vector<Blob> &blobs, cv::Scalar color);

    [[maybe_unused]] void set_area(int minArea, int maxArea);

    [[maybe_unused]] void set_color_range(cv::Scalar lowerBound, cv::Scalar upperBound);
};

#endif //OPENCV_BLOB_DETECTION_H