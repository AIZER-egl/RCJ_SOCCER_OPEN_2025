#include "preprocessing.h"

[[maybe_unused]] void preprocessing::saturation(cv::cuda::GpuMat &image, float alpha) {
    cv::cuda::GpuMat hsv_gpu;
    cv::cuda::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    std::vector<cv::cuda::GpuMat> channels_gpu;
	cv::cuda::split(hsv_gpu, channels_gpu);

	channels_gpu[1].convertTo(channels_gpu[1], channels_gpu[1].type(), alpha);

	cv::cuda::merge(channels_gpu, hsv_gpu);

	cv::cuda::cvtColor(hsv_gpu, image, cv::COLOR_HSV2BGR);
}

[[maybe_unused]] void preprocessing::brightness(cv::cuda::GpuMat &image, int beta) {
	cv::cuda::add(image, cv::Scalar::all(beta), image);
}

[[maybe_unused]] void preprocessing::contrast(cv::cuda::GpuMat &image, float alpha, int beta) {
	image.convertTo(image, -1, alpha, beta);
}

[[maybe_unused]] void preprocessing::gamma_correction(cv::cuda::GpuMat &image, float gamma) {
	cv::Mat lut_cpu(1, 256, CV_8U);
	for (int i = 0; i < 256; i++) {
		lut_cpu.at<uchar>(i) = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
	}

	std::vector<cv::cuda::GpuMat> channels_gpu;
	cv::cuda::split(image, channels_gpu);
	cv::Ptr<cv::cuda::LookUpTable> lut_transformer = cv::cuda::createLookUpTable(lut_cpu);
	for(int i=0; i<image.channels(); ++i) {
		lut_transformer->transform(channels_gpu[i], channels_gpu[i]);
	}
	cv::cuda::merge(channels_gpu, image);
}

[[maybe_unused]] void preprocessing::white_balance(cv::cuda::GpuMat &image, float temperature, float tint) {
	cv::cuda::GpuMat image_float;
	image.convertTo(image_float, CV_32F);

	float blue_scale_factor = tint < 0 ? (1.0f + tint / 100.0f) : (1.0f - tint / 100.0f);
	float red_scale_factor = tint > 0 ? (1.0f + tint / 100.0f) : (1.0f - tint / 100.0f);
	float kelvin_scale = temperature / 6500.0f;
	float red_channel_scale = 1.0f, green_channel_scale = 1.0f, blue_channel_scale = 1.0f;
	if (kelvin_scale > 1.0) {
		red_channel_scale = kelvin_scale;
	} else {
		blue_channel_scale = 1.0f / kelvin_scale;
	}

	std::vector<cv::cuda::GpuMat> channels_gpu;
	cv::cuda::split(image_float, channels_gpu); // B, G, R

	cv::cuda::multiply(channels_gpu[0], cv::Scalar::all(blue_channel_scale * blue_scale_factor), channels_gpu[0]); // Blue
	// cv::cuda::multiply(channels_gpu[1], cv::Scalar::all(green_channel_scale), channels_gpu[1]); // Green (si escala es 1.0, no hace falta)
	cv::cuda::multiply(channels_gpu[2], cv::Scalar::all(red_channel_scale * red_scale_factor), channels_gpu[2]); // Red

	cv::cuda::merge(channels_gpu, image_float);

	image_float.convertTo(image, CV_8U);
}

[[maybe_unused]] void preprocessing::sharpen(cv::cuda::GpuMat &image) {
	cv::Mat kernel_cpu = (cv::Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
	cv::Ptr<cv::cuda::Filter> filter = cv::cuda::createLinearFilter(image.type(), image.type(), kernel_cpu);
	filter->apply(image, image);
}

[[maybe_unused]] void preprocessing::edge_detection(cv::cuda::GpuMat &image) {
	cv::cuda::GpuMat gray_gpu;
	cv::cuda::cvtColor(image, gray_gpu, cv::COLOR_BGR2GRAY);

	cv::Ptr<cv::cuda::CannyEdgeDetector> canny_detector = cv::cuda::createCannyEdgeDetector(100, 200); // Umbrales bajo y alto

	cv::cuda::GpuMat edges_gpu;
	canny_detector->detect(gray_gpu, edges_gpu);

	cv::cuda::cvtColor(edges_gpu, image, cv::COLOR_GRAY2BGR);
}

[[maybe_unused]] void preprocessing::color_space_conversion(cv::cuda::GpuMat &image, int code) {
    cv::cuda::cvtColor(image, image, code);
}

[[maybe_unused]] void preprocessing::noise_reduction(cv::cuda::GpuMat &image) {
	cv::cuda::GpuMat dst;
	cv::cuda::fastNlMeansDenoisingColored(image, dst, 10, 10, 7, 21);
	image = dst;
}

[[maybe_unused]] void preprocessing::threshold(cv::cuda::GpuMat &image, int threshold_value, int max_value, int threshold_type) {
	cv::cuda::GpuMat gray_gpu;
	cv::cuda::cvtColor(image, gray_gpu, cv::COLOR_BGR2GRAY);

	cv::cuda::GpuMat thresh_gpu;
	cv::cuda::threshold(gray_gpu, thresh_gpu, threshold_value, max_value, threshold_type);

	cv::cuda::cvtColor(thresh_gpu, image, cv::COLOR_GRAY2BGR);
}

[[maybe_unused]] void preprocessing::blur(cv::cuda::GpuMat &image, int kernel_size) {
	cv::Ptr<cv::cuda::Filter> filter = cv::cuda::createBoxFilter(image.type(), image.type(), cv::Size(kernel_size, kernel_size));

	filter->apply(image, image);
}

[[maybe_unused]] void preprocessing::histogram_equalization(cv::cuda::GpuMat &image) {
	cv::cuda::GpuMat gray_gpu;
	cv::cuda::cvtColor(image, gray_gpu, cv::COLOR_BGR2GRAY);

	cv::cuda::GpuMat equalized_gpu;
	cv::cuda::equalizeHist(gray_gpu, equalized_gpu);

	cv::cuda::cvtColor(equalized_gpu, image, cv::COLOR_GRAY2BGR);
}

[[maybe_unused]] void preprocessing::mirror(cv::cuda::GpuMat &image) {
    cv::cuda::flip(image, image, 1);
}

[[maybe_unused]] void preprocessing::rotate(cv::cuda::GpuMat &image, int angle) {
	cv::Point2f center(static_cast<float>(image.cols / 2.0), static_cast<float>(image.rows / 2.0));
	cv::Mat rotation_matrix_cpu = cv::getRotationMatrix2D(center, angle, 1.0);

	cv::cuda::GpuMat rotated_gpu;
	cv::cuda::warpAffine(image, rotated_gpu, rotation_matrix_cpu, image.size());
	image = rotated_gpu;
}

[[maybe_unused]] void preprocessing::resize(cv::cuda::GpuMat &image, int width, int height) {
    cv::cuda::resize(image, image, cv::Size(width, height));
}