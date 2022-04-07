#include "BallFinderPipeline.h"

namespace grip {

BallFinderPipeline::BallFinderPipeline() {
	this->switchSwitch = source1;
}
/**
* Runs an iteration of the pipeline and updates outputs.
*/
void BallFinderPipeline::Process(cv::Mat& source0, bool& source1){
	//Step Resize_Image0:
	//input
	cv::Mat resizeImageInput = source0;
	double resizeImageWidth = 267.0;  // default Double
	double resizeImageHeight = 216.0;  // default Double
	int resizeImageInterpolation = cv::INTER_CUBIC;
	resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, this->resizeImageOutput);
	//Step Blur0:
	//input
	cv::Mat blur0Input = resizeImageOutput;
	BlurType blur0Type = BlurType::GAUSSIAN;
	double blur0Radius = 1.8018013722187767;  // default Double
	blur(blur0Input, blur0Type, blur0Radius, this->blur0Output);
	//Step HSV_Threshold0:
	//input
	cv::Mat hsvThreshold0Input = blur0Output;
	double hsvThreshold0Hue[] = {97.02067801522387, 116.92940656990577};
	double hsvThreshold0Saturation[] = {104.0489634551571, 255.0};
	double hsvThreshold0Value[] = {0.0, 255.0};
	hsvThreshold(hsvThreshold0Input, hsvThreshold0Hue, hsvThreshold0Saturation, hsvThreshold0Value, this->hsvThreshold0Output);
	//Step HSV_Threshold1:
	//input
	cv::Mat hsvThreshold1Input = blur0Output;
	double hsvThreshold1Hue[] = {0.0, 31.42450742023067};
	double hsvThreshold1Saturation[] = {86.86965084781404, 255.0};
	double hsvThreshold1Value[] = {0.0, 255.0};
	hsvThreshold(hsvThreshold1Input, hsvThreshold1Hue, hsvThreshold1Saturation, hsvThreshold1Value, this->hsvThreshold1Output);
	//Step HSV_Threshold2:
	//input
	cv::Mat hsvThreshold2Input = blur0Output;
	double hsvThreshold2Hue[] = {168.36157814931065, 180.0};
	double hsvThreshold2Saturation[] = {25.61203173998386, 255.0};
	double hsvThreshold2Value[] = {0.0, 255.0};
	hsvThreshold(hsvThreshold2Input, hsvThreshold2Hue, hsvThreshold2Saturation, hsvThreshold2Value, this->hsvThreshold2Output);
	//Step CV_add0:
	//input
	cv::Mat cvAddSrc1 = hsvThreshold2Output;
	cv::Mat cvAddSrc2 = hsvThreshold1Output;
	cvAdd(cvAddSrc1, cvAddSrc2, this->cvAddOutput);
	//Step Switch0:
	//input
	bool switchSwitch = this->switchSwitch;
	cv::Mat switchIfTrue = hsvThreshold0Output;
	cv::Mat switchIfFalse = cvAddOutput;
	pipelineswitch(switchSwitch, switchIfTrue, switchIfFalse, this->switchOutput);
	//Step CV_dilate0:
	//input
	cv::Mat cvDilateSrc = switchOutput;
	cv::Mat cvDilateKernel;
	cv::Point cvDilateAnchor(-1, -1);
	double cvDilateIterations = 1.0;  // default Double
    int cvDilateBordertype = cv::BORDER_CONSTANT;
	cv::Scalar cvDilateBordervalue(-1);
	cvDilate(cvDilateSrc, cvDilateKernel, cvDilateAnchor, cvDilateIterations, cvDilateBordertype, cvDilateBordervalue, this->cvDilateOutput);
	//Step CV_erode0:
	//input
	cv::Mat cvErodeSrc = cvDilateOutput;
	cv::Mat cvErodeKernel;
	cv::Point cvErodeAnchor(-1, -1);
	double cvErodeIterations = 5.0;  // default Double
    int cvErodeBordertype = cv::BORDER_CONSTANT;
	cv::Scalar cvErodeBordervalue(-1);
	cvErode(cvErodeSrc, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, this->cvErodeOutput);
	//Step Blur1:
	//input
	cv::Mat blur1Input = cvErodeOutput;
	BlurType blur1Type = BlurType::GAUSSIAN;
	double blur1Radius = 1.8867929026765644;  // default Double
	blur(blur1Input, blur1Type, blur1Radius, this->blur1Output);
	//Step Find_Blobs0:
	//input
	cv::Mat findBlobsInput = blur1Output;
	double findBlobsMinArea = 20.0;  // default Double
	double findBlobsCircularity[] = {0.8160703779389405, 1.0};
	bool findBlobsDarkBlobs = false;  // default Boolean
	findBlobs(findBlobsInput, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs, this->findBlobsOutput);
	//Step Mask0:
	//input
	cv::Mat maskInput = blur0Output;
	cv::Mat maskMask = blur1Output;
	mask(maskInput, maskMask, this->maskOutput);
}

/**
 * This method is a generated setter for the condition of Switch 0
 * @param the condition to set
 */
void BallFinderPipeline::setSwitch0(bool value){
	switchSwitch = value;
}
/**
 * This method is a generated getter for the output of a Resize_Image.
 * @return Mat output from Resize_Image.
 */
cv::Mat* BallFinderPipeline::GetResizeImageOutput(){
	return &(this->resizeImageOutput);
}
/**
 * This method is a generated getter for the output of a Blur.
 * @return Mat output from Blur.
 */
cv::Mat* BallFinderPipeline::GetBlur0Output(){
	return &(this->blur0Output);
}
/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* BallFinderPipeline::GetHsvThreshold0Output(){
	return &(this->hsvThreshold0Output);
}
/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* BallFinderPipeline::GetHsvThreshold1Output(){
	return &(this->hsvThreshold1Output);
}
/**
 * This method is a generated getter for the output of a HSV_Threshold.
 * @return Mat output from HSV_Threshold.
 */
cv::Mat* BallFinderPipeline::GetHsvThreshold2Output(){
	return &(this->hsvThreshold2Output);
}
/**
 * This method is a generated getter for the output of a CV_add.
 * @return Mat output from CV_add.
 */
cv::Mat* BallFinderPipeline::GetCvAddOutput(){
	return &(this->cvAddOutput);
}
/**
 * This method is a generated getter for the output of a Switch.
 * @return Mat output from Switch.
 */
cv::Mat* BallFinderPipeline::GetSwitchOutput(){
	return &(this->switchOutput);
}
/**
 * This method is a generated getter for the output of a CV_dilate.
 * @return Mat output from CV_dilate.
 */
cv::Mat* BallFinderPipeline::GetCvDilateOutput(){
	return &(this->cvDilateOutput);
}
/**
 * This method is a generated getter for the output of a CV_erode.
 * @return Mat output from CV_erode.
 */
cv::Mat* BallFinderPipeline::GetCvErodeOutput(){
	return &(this->cvErodeOutput);
}
/**
 * This method is a generated getter for the output of a Blur.
 * @return Mat output from Blur.
 */
cv::Mat* BallFinderPipeline::GetBlur1Output(){
	return &(this->blur1Output);
}
/**
 * This method is a generated getter for the output of a Find_Blobs.
 * @return BlobsReport output from Find_Blobs.
 */
std::vector<cv::KeyPoint>* BallFinderPipeline::GetFindBlobsOutput(){
	return &(this->findBlobsOutput);
}
/**
 * This method is a generated getter for the output of a Mask.
 * @return Mat output from Mask.
 */
cv::Mat* BallFinderPipeline::GetMaskOutput(){
	return &(this->maskOutput);
}
	/**
	 * Scales and image to an exact size.
	 *
	 * @param input The image on which to perform the Resize.
	 * @param width The width of the output in pixels.
	 * @param height The height of the output in pixels.
	 * @param interpolation The type of interpolation.
	 * @param output The image in which to store the output.
	 */
	void BallFinderPipeline::resizeImage(cv::Mat &input, double width, double height, int interpolation, cv::Mat &output) {
		cv::resize(input, output, cv::Size(width, height), 0.0, 0.0, interpolation);
	}

	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue.
	 * @param sat The min and max saturation.
	 * @param val The min and max value.
	 * @param output The image in which to store the output.
	 */
	void BallFinderPipeline::hsvThreshold(cv::Mat &input, double hue[], double sat[], double val[], cv::Mat &out) {
		cv::cvtColor(input, out, cv::COLOR_BGR2HSV);
		cv::inRange(out,cv::Scalar(hue[0], sat[0], val[0]), cv::Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Calculates the sum of two Mats.
	 * @param src1 the first Mat.
	 * @param src2 the second Mat.
	 * @param out the Mat that is the sum of the two Mats.
	 */
	void BallFinderPipeline::cvAdd(cv::Mat &src1, cv::Mat &src2, cv::Mat &out) {
		cv::add(src1, src2, out);
	}

	/**
	 * Selects an output from two inputs based on a boolean.
	 *
	 * @param sw The boolean that determines the output.
	 * @param onTrue The output if sw is true.
	 * @param onFalse The output if sw is false.
	 * @param output The output which is equal to either onTrue or onFalse.
	 */
	template<typename T>
	void BallFinderPipeline::pipelineswitch(bool sw, T &onTrue, T &onFalse, T &output) {
		if (sw) {
			output = onTrue;
		}
		else {
			output = onFalse;
		}
	}

	/**
	 * Expands area of higher value in an image.
	 * @param src the Image to dilate.
	 * @param kernel the kernel for dilation.
	 * @param anchor the center of the kernel.
	 * @param iterations the number of times to perform the dilation.
	 * @param borderType pixel extrapolation method.
	 * @param borderValue value to be used for a constant border.
	 * @param dst Output Image.
	 */
	void BallFinderPipeline::cvDilate(cv::Mat &src, cv::Mat &kernel, cv::Point &anchor, double iterations, int borderType, cv::Scalar &borderValue, cv::Mat &dst) {
		cv::dilate(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}

	/**
	 * Expands area of lower value in an image.
	 * @param src the Image to erode.
	 * @param kernel the kernel for erosion.
	 * @param anchor the center of the kernel.
	 * @param iterations the number of times to perform the erosion.
	 * @param borderType pixel extrapolation method.
	 * @param borderValue value to be used for a constant border.
	 * @param dst Output Image.
	 */
	void BallFinderPipeline::cvErode(cv::Mat &src, cv::Mat &kernel, cv::Point &anchor, double iterations, int borderType, cv::Scalar &borderValue, cv::Mat &dst) {
		cv::erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
	}

	/**
	 * Softens an image using one of several filters.
	 *
	 * @param input The image on which to perform the blur.
	 * @param type The blurType to perform.
	 * @param doubleRadius The radius for the blur.
	 * @param output The image in which to store the output.
	 */
	void BallFinderPipeline::blur(cv::Mat &input, BlurType &type, double doubleRadius, cv::Mat &output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;
		switch(type) {
			case BOX:
				kernelSize = 2 * radius + 1;
				cv::blur(input,output,cv::Size(kernelSize, kernelSize));
				break;
			case GAUSSIAN:
				kernelSize = 6 * radius + 1;
				cv::GaussianBlur(input, output, cv::Size(kernelSize, kernelSize), radius);
				break;
			case MEDIAN:
				kernelSize = 2 * radius + 1;
				cv::medianBlur(input, output, kernelSize);
				break;
			case BILATERAL:
				cv::bilateralFilter(input, output, -1, radius, radius);
				break;
        }
	}
	/**
	 * Detects groups of pixels in an image.
	 *
	 * @param input The image on which to perform the find blobs.
	 * @param minArea The minimum size of a blob that will be found.
	 * @param circularity The minimum and maximum circularity of blobs that will be found.
	 * @param darkBlobs The boolean that determines if light or dark blobs are found.
	 * @param blobList The output where the MatOfKeyPoint is stored.
	 */
	//void findBlobs(Mat *input, double *minArea, double circularity[2],
		//bool *darkBlobs, vector<KeyPoint> *blobList) {
	void BallFinderPipeline::findBlobs(cv::Mat &input, double minArea, double circularity[], bool darkBlobs, std::vector<cv::KeyPoint> &blobList) {
		blobList.clear();
		cv::SimpleBlobDetector::Params params;
		params.filterByColor = 1;
		params.blobColor = (darkBlobs ? 0 : 255);
		params.minThreshold = 10;
		params.maxThreshold = 220;
		params.filterByArea = true;
		params.minArea = minArea;
		params.filterByCircularity = true;
		params.minCircularity = circularity[0];
		params.maxCircularity = circularity[1];
		params.filterByConvexity = false;
		params.filterByInertia = false;
		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
		detector->detect(input, blobList);
	}

		/**
		 * Filter out an area of an image using a binary mask.
		 *
		 * @param input The image on which the mask filters.
		 * @param mask The binary image that is used to filter.
		 * @param output The image in which to store the output.
		 */
		void BallFinderPipeline::mask(cv::Mat &input, cv::Mat &mask, cv::Mat &output) {
			mask.convertTo(mask, CV_8UC1);
			cv::bitwise_xor(output, output, output);
			input.copyTo(output, mask);
		}



} // end grip namespace

