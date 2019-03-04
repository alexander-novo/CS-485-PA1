/**
 * @file main.cpp
 * @author Alexander Novotny (anovotny@nevada.unr.edu)
 * @brief A program which takes in several test images and normalizes them
 * @version 0.1
 * @date 2019-03-02
 *
 * @copyright Copyright (c) 2019
 *
 */
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <string>

/**
 * @brief A simple struct used to hold information about test images
 *
 */
struct TestImage {
	/**
	 * @brief Filename of the test image (relative to input/output directories)
	 *
	 */
	std::string filename;
	/**
	 * @brief Points where these locations are (found through harsh manual labor)
	 *
	 */
	cv::Point2d leftEye, rightEye, noseTip, mouth;

	/**
	 * @brief Construct a new Test Image object
	 *
	 * @param _filename Filename
	 * @param _leftEye Point where the left eye is located
	 * @param _rightEye Point where the right eye is located
	 * @param _noseTip Point where the tip of the nose is locaed
	 * @param _mouth Point where the center of the mouth is located
	 */
	TestImage(std::string _filename, cv::Point2d _leftEye, cv::Point2d _rightEye,
	          cv::Point2d _noseTip, cv::Point2d _mouth)
	    : filename(_filename),
	      leftEye(_leftEye),
	      rightEye(_rightEye),
	      noseTip(_noseTip),
	      mouth(_mouth) {}
};

/**
 * @brief Array of test images
 *
 */
const TestImage IMAGES[20] = {{"S1/1.pgm", {27, 51}, {64, 50}, {45, 70}, {45, 89}},
                              {"S1/2.pgm", {31, 44}, {75, 44}, {66, 65}, {60, 87}},
                              {"S1/3.pgm", {25, 48}, {62, 44}, {46, 74}, {46, 89}},
                              {"S1/4.pgm", {16, 45}, {57, 42}, {28, 63}, {32, 84}},
                              {"S1/5.pgm", {34, 44}, {75, 43}, {69, 63}, {64, 85}},
                              {"S1/6.pgm", {15, 47}, {54, 44}, {27, 66}, {30, 85}},
                              {"S1/7.pgm", {24, 44}, {62, 43}, {44, 59}, {43, 80}},
                              {"S1/8.pgm", {29, 46}, {65, 43}, {49, 65}, {49, 85}},
                              {"S1/9.pgm", {30, 36}, {68, 35}, {54, 46}, {52, 74}},
                              {"S1/10.pgm", {34, 52}, {75, 49}, {63, 76}, {58, 92}},
                              {"S2/1.pgm", {25, 52}, {60, 54}, {40, 67}, {40, 89}},
                              {"S2/2.pgm", {35, 51}, {68, 52}, {54, 65}, {53, 89}},
                              {"S2/3.pgm", {21, 51}, {56, 54}, {33, 66}, {33, 88}},
                              {"S2/4.pgm", {32, 53}, {65, 53}, {51, 67}, {50, 88}},
                              {"S2/5.pgm", {40, 53}, {74, 52}, {60, 67}, {59, 89}},
                              {"S2/6.pgm", {15, 51}, {50, 53}, {27, 67}, {29, 89}},
                              {"S2/7.pgm", {29, 52}, {61, 53}, {47, 69}, {45, 89}},
                              {"S2/8.pgm", {22, 51}, {58, 53}, {35, 66}, {36, 88}},
                              {"S2/9.pgm", {34, 53}, {68, 55}, {52, 69}, {50, 91}},
                              {"S2/10.pgm", {46, 54}, {77, 53}, {72, 68}, {67, 88}}};

/**
 * @brief Normalises an image, then outputs it to the screen and a file
 *
 * @param image Image to normalise
 * @param inputDirectory Prepended to the file name when reading in
 * @param outputDirectory Prepended to the file name when outputting
 * @param displayImageToScreen Whether or not to display to screen
 */
void normaliseImage(const TestImage& image, const std::string& inputDirectory,
                    const std::string& outputDirectory, bool displayImageToScreen);

/**
 * @brief Main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char** argv) {
	if (argc < 3) {
		std::cout
		    << "Usage: normalise <input directory> <output directory> [-v]" << std::endl
		    << std::endl
		    << "    <input directory> - Where the images can be found. Must end in '/' (i.e. "
		       "'Images/'). Must be formatted as follows:"
		    << std::endl
		    << "        <input directory>-" << std::endl
		    << "                         |- S1-" << std::endl
		    << "                         |    |- 1.pgm" << std::endl
		    << "                         |    |- 2.pgm" << std::endl
		    << "                         |       ..." << std::endl
		    << "                         |- S2-" << std::endl
		    << "                              |- 1.pgm" << std::endl
		    << "                              |- 2.pgm" << std::endl
		    << "                                 ..." << std::endl
		    << "    <output directory> - Where the output images should be stored. Must end "
		       "in '/' (i.e. 'Out/'). Must exist and include sub-directories S1/S2 before running."
		    << std::endl
		    << "    -v - Flag to enable visual output. Cycle through by pressing any key"
		    << std::endl;

		return 1;
	}

	// Pull command line arguments
	std::string inputDirectory = argv[1], outputDirectory = argv[2];
	bool displayImageToScreen = (argc >= 4 && !strcmp("-v", argv[3]));

	// Acknowledge choice to display images and open windows
	if (displayImageToScreen) {
		std::cout << "Displaying comparison images to screen." << std::endl
		          << "Focus on window and press any key to cycle." << std::endl;
		cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
		cv::namedWindow("Display Image Original", cv::WINDOW_AUTOSIZE);
	}

	// Normalise all test images
	for (int i = 0; i < 20; i++) {
		normaliseImage(IMAGES[i], inputDirectory, outputDirectory, displayImageToScreen);

		cv::waitKey(0);
	}

	// Then close all windows. Important - will segmentation fault if not included.
	cv::destroyAllWindows();
	return 0;
}

void normaliseImage(const TestImage& image, const std::string& inputDirectory,
                    const std::string& outputDirectory, bool displayToScreen) {
	// Where the left eye, right eye, nose tip, and mouth should be mapped to in the normalised
	// image. Arbitrarily chosen.
	static const cv::Mat pHat_x({5.0f, 35.0f, 20.0f, 20.0f}), pHat_y({10.0f, 10.0f, 25.0f, 37.0f});

	// System of equations matrices
	cv::Mat p(4, 3, CV_32F), c_1(3, 1, CV_32F), c_2(3, 1, CV_32F), A(2, 2, CV_32F), AInv,
	    b(2, 1, CV_32F), PHat(2, 1, CV_32F), result, resultPixel;
	// Images
	cv::Mat oldImage, normalImage(48, 40, CV_8U), normalImageGrey;
	cv::SVD svd;

	// Load the starting image in grayscale
	// If the grayscale otion is not included, will not work (learned the hard way)
	oldImage = cv::imread(inputDirectory + image.filename, cv::IMREAD_GRAYSCALE);

	// Construct the P matrix
	p.at<cv::Vec3f>(0) = cv::Vec3b(image.leftEye.x, image.leftEye.y, 1.0f);
	p.at<cv::Vec3f>(1) = cv::Vec3b(image.rightEye.x, image.rightEye.y, 1.0f);
	p.at<cv::Vec3f>(2) = cv::Vec3b(image.noseTip.x, image.noseTip.y, 1.0f);
	p.at<cv::Vec3f>(3) = cv::Vec3b(image.mouth.x, image.mouth.y, 1.0f);

	// Use SVD to find least squares soltuions to c_1 and c_2
	svd(p);

	svd.backSubst(pHat_x, c_1);
	svd.backSubst(pHat_y, c_2);

	// Construct A matrix from original system of equations
	A.at<float>(0, 0) = c_1.at<float>(0, 0);
	A.at<float>(0, 1) = c_1.at<float>(1, 0);
	A.at<float>(1, 0) = c_2.at<float>(0, 0);
	A.at<float>(1, 1) = c_2.at<float>(1, 0);

	b = cv::Mat({c_1.at<float>(2, 0), c_2.at<float>(2, 0)});

	// Original system is P^ = AP + b
	// But we want to solve for P, so we need A inverse
	// Then P = A^{-1}(P^ - b)
	AInv = A.inv();

	// Uncomment to view transformation parameters
	// std::cout << "Image: " << image.filename << std::endl
	//           << "A:\n"
	//           << A << std::endl
	//           << "b:\n"
	//           << b << std::endl
	//           << std::endl;

	// Uncomment to view error information
	// cv::Mat leftEyeFixed({pHat_x.at<float>(0, 0), pHat_y.at<float>(0, 0)}),
	//     rightEyeFixed({pHat_x.at<float>(1, 0), pHat_y.at<float>(1, 0)}),
	//     noseTipFixed({pHat_x.at<float>(2, 0), pHat_y.at<float>(2, 0)}),
	//     mouthFixed({pHat_x.at<float>(3, 0), pHat_y.at<float>(3, 0)}), leftEye, rightEye, noseTip,
	//     mouth, error;

	// cv::transpose(p(cv::Rect(0, 0, 2, 1)), leftEye);
	// cv::transpose(p(cv::Rect(0, 1, 2, 1)), rightEye);
	// cv::transpose(p(cv::Rect(0, 2, 2, 1)), noseTip);
	// cv::transpose(p(cv::Rect(0, 3, 2, 1)), mouth);

	// leftEye  = A * leftEye + b;
	// rightEye = A * rightEye + b;
	// noseTip  = A * noseTip + b;
	// mouth    = A * mouth + b;

	// error = ((leftEye - leftEyeFixed) + (rightEye - rightEyeFixed) + (noseTip - noseTipFixed) +
	//          (mouth - mouthFixed)) /
	//         4;

	// std::cout << "Image: " << image.filename << std::endl
	//           << "Avg. Error:\n"
	//           << error << std::endl
	//           << std::endl;

	for (int x = 0; x < normalImage.cols; x++) {
		for (int y = 0; y < normalImage.rows; y++) {
			// Solve for P in the above equation
			// Solution is unlikely to be inteeger pixel, so we round
			PHat   = cv::Mat({(float) x, (float) y});
			result = AInv * (PHat - b);
			resultPixel =
			    cv::Mat({(int) floor(result.at<float>(0, 0)), (int) floor(result.at<float>(1, 0))});

			// TODO: averaging

			normalImage.at<uint8_t>(y, x) =
			    oldImage.at<uint8_t>(resultPixel.at<int>(1, 0), resultPixel.at<int>(0, 0));
		}
	}

	cv::imwrite(outputDirectory + image.filename, normalImage);

	if (displayToScreen) {
		cv::imshow("Display Image", normalImage);
		cv::imshow("Display Image Original", oldImage);
	}
}
