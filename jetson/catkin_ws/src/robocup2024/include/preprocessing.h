#ifndef OPENCV_IMAGE_PREPROCESSING_H
#define OPENCV_IMAGE_PREPROCESSING_H

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>

namespace preprocessing {
    [[maybe_unused]] void saturation(cv::cuda::GpuMat &image, float alpha);

    [[maybe_unused]] void brightness(cv::cuda::GpuMat &image, int beta);

    [[maybe_unused]] void contrast(cv::cuda::GpuMat &image, float alpha, int beta);

    [[maybe_unused]] void gamma_correction(cv::cuda::GpuMat &image, float gamma);

    [[maybe_unused]] void white_balance(cv::cuda::GpuMat &image, float temperature, float tint);

    [[maybe_unused]] void sharpen(cv::cuda::GpuMat &image);

    [[maybe_unused]] void edge_detection(cv::cuda::GpuMat &image);

    [[maybe_unused]] void color_space_conversion(cv::cuda::GpuMat &image, int code);

    [[maybe_unused]] void threshold(cv::cuda::GpuMat &image, int threshold_value, int max_value, int threshold_type);

    [[maybe_unused]] void blur(cv::cuda::GpuMat &image, int kernel_size);

    [[maybe_unused]] void histogram_equalization(cv::cuda::GpuMat &image);

    [[maybe_unused]] void mirror(cv::cuda::GpuMat &image);

    [[maybe_unused]] void rotate(cv::cuda::GpuMat &image, int angle);

    [[maybe_unused]] void resize(cv::cuda::GpuMat &image, int width, int height);
}

#endif //OPENCV_IMAGE_PREPROCESSING_H
