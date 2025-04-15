#include "preprocessing.h"

[[maybe_unused]] void preprocessing::saturation(cv::Mat &image, float alpha) {
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);

    channels[1] = channels[1] * alpha;

    cv::merge(channels, hsv);

    cv::cvtColor(hsv, image, cv::COLOR_HSV2BGR);
}

[[maybe_unused]] void preprocessing::brightness(cv::Mat &image, int beta) {
    image.convertTo(image, -1, 1, beta);
}

[[maybe_unused]] void preprocessing::contrast(cv::Mat &image, float alpha, int beta) {
    image.convertTo(image, -1, alpha, beta);
}

[[maybe_unused]] void preprocessing::gamma_correction(cv::Mat &image, float gamma) {
    cv::Mat lut(1, 256, CV_8U);
    for (int i = 0; i < 256; i++) {
        lut.at<uchar>(i) = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }

    cv::LUT(image, lut, image);
}

[[maybe_unused]] void preprocessing::white_balance(cv::Mat &image, float temperature, float tint) {
    image.convertTo(image, CV_32F);

    float blue_scale = tint < 0 ? (1 + tint / 100) : (1 - tint / 100);
    float red_scale = tint > 0 ? (1 + tint / 100) : (1 - tint / 100);

    float kelvin_scale = temperature / 6500.0f;
    float red_channel = 1.0f, green_channel = 1.0f, blue_channel = 1.0f;
    if (kelvin_scale > 1.0) {
        red_channel = kelvin_scale;
    } else {
        blue_channel = static_cast<float>(1.0 / kelvin_scale);
    }

    image.forEach<cv::Vec3f>([&](cv::Vec3f &pixel, const int *pos) {
        pixel[0] *= blue_channel * blue_scale; // Blue channel
        pixel[1] *= green_channel;             // Green channel (unchanged)
        pixel[2] *= red_channel * red_scale;   // Red channel
    });

    image.convertTo(image, CV_8U);
}

[[maybe_unused]] void preprocessing::sharpen(cv::Mat &image) {
    cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    cv::filter2D(image, image, -1, kernel);
}

[[maybe_unused]] void preprocessing::edge_detection(cv::Mat &image) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Canny(gray, image, 100, 200);
}

[[maybe_unused]] void preprocessing::color_space_conversion(cv::Mat &image, int code) {
    cv::cvtColor(image, image, code);
}

[[maybe_unused]] void preprocessing::noise_reduction(cv::Mat &image) {
    cv::fastNlMeansDenoisingColored(image, image, 10, 10, 7, 21);
}

[[maybe_unused]] void preprocessing::threshold(cv::Mat &image, int threshold_value, int max_value, int threshold_type) {
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    cv::threshold(image, image, threshold_value, max_value, threshold_type);
}

[[maybe_unused]] void preprocessing::blur(cv::Mat &image, int kernel_size) {
    cv::blur(image, image, cv::Size(kernel_size, kernel_size));
}

[[maybe_unused]] void preprocessing::histogram_equalization(cv::Mat &image) {
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(image, image);
}

[[maybe_unused]] void preprocessing::mirror(cv::Mat &image) {
    cv::flip(image, image, 1);
}

[[maybe_unused]] void preprocessing::rotate(cv::Mat &image, int angle) {
    cv::Point2f center(static_cast<float>(image.cols / 2.0), static_cast<float>(image.rows / 2.0));
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::warpAffine(image, image, rotation_matrix, image.size());
}

[[maybe_unused]] void preprocessing::resize(cv::Mat &image, int width, int height) {
    cv::resize(image, image, cv::Size(width, height));
}