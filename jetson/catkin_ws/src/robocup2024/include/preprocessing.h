#ifndef OPENCV_IMAGE_PREPROCESSING_H
#define OPENCV_IMAGE_PREPROCESSING_H

#include <opencv4/opencv2/opencv.hpp>

namespace preprocessing {
    [[maybe_unused]] void saturation(cv::Mat &image, float alpha);

    [[maybe_unused]] void brightness(cv::Mat &image, int beta);

    [[maybe_unused]] void contrast(cv::Mat &image, float alpha, int beta);

    [[maybe_unused]] void gamma_correction(cv::Mat &image, float gamma);

    [[maybe_unused]] void white_balance(cv::Mat &image, float temperature, float tint);

    [[maybe_unused]] void sharpen(cv::Mat &image);

    [[maybe_unused]] void edge_detection(cv::Mat &image);

    [[maybe_unused]] void color_space_conversion(cv::Mat &image, int code);

    [[maybe_unused]] void noise_reduction(cv::Mat &image);

    [[maybe_unused]] void threshold(cv::Mat &image, int threshold_value, int max_value, int threshold_type);

    [[maybe_unused]] void blur(cv::Mat &image, int kernel_size);

    [[maybe_unused]] void histogram_equalization(cv::Mat &image);

    [[maybe_unused]] void mirror(cv::Mat &image);

    [[maybe_unused]] void rotate(cv::Mat &image, int angle);

    [[maybe_unused]] void resize(cv::Mat &image, int width, int height);
}

#endif //OPENCV_IMAGE_PREPROCESSING_H