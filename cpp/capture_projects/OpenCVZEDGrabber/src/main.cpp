///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015, STEREOLABS.
// 
// All rights reserved.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////




/**************************************************************************************************
** This sample demonstrates how to grab images and depth/disparity map with the ZED SDK          **
** Both images and depth/disparity map are displayed with OpenCV                                 **
** Most of the functions of the ZED SDK are linked with a key press event (using opencv)         **
***************************************************************************************************/

//#define GPUstuff
#define NOMINMAX
//standard includes
#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include <iostream>
#include <ctime>

//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifdef GPUstuff
#include <opencv2/gpu/gpu.hpp>
#endif


//ZED Includes
#include <zed/Camera.hpp>

using namespace sl::zed;
using namespace std;

//Define the structure and callback for mouse event
typedef struct mouseOCVStruct {
    float* data;
    uint32_t step;
    cv::Size _image;
    cv::Size _resize;
    std::string name;
} mouseOCV;

mouseOCV mouseStruct;

static void onMouseCallback(int32_t event, int32_t x, int32_t y, int32_t flag, void * param) {
    if (event == CV_EVENT_LBUTTONDOWN) {
        mouseOCVStruct* data = (mouseOCVStruct*) param;

        int y_int = (y * data->_image.height / data->_resize.height);
        int x_int = (x * data->_image.width / data->_resize.width);

        float* ptr_image_num = (float*)((int8_t*)data->data + y_int * data->step);
        float dist = ptr_image_num[x_int] / 1000.f;

        if (dist > 0.)
            printf("\n%s : %2.2f m\n", data->name.c_str(), dist);
        else
            printf("\n : NAN\n");
    }
}

// UTILS fct to convert a sl::zed::Mat to a cv::Mat
cv::Mat slMat2cvMat(sl::zed::Mat slMat) {
    int cvType;
    if (slMat.data_type == DATA_TYPE::UCHAR)
        cvType = CV_8U;
    else if (slMat.data_type == DATA_TYPE::FLOAT)
        cvType = CV_32F;
    else
        std::cerr << "slMat2cvMat: mat type currently not supported " <<
                     slMat.channels << " " << slMat.getDataSize() << std::endl;
    return cv::Mat(slMat.height, slMat.width, CV_MAKETYPE(cvType, slMat.channels), slMat.data);
}

#ifdef GPUstuff
cv::gpu::GpuMat slMat2cvMatGPU(sl::zed::Mat slMat) {
	int cvType;
	if (slMat.data_type == DATA_TYPE::UCHAR)
		cvType = CV_8U;
	else if (slMat.data_type == DATA_TYPE::FLOAT)
		cvType = CV_32F;
	else
		std::cerr << "slMat2cvMat: mat type currently not supported " <<
		slMat.channels << " " << slMat.getDataSize() << std::endl;
	return cv::gpu::GpuMat(slMat.height, slMat.width, CV_MAKETYPE(cvType, slMat.channels), slMat.data);
}
#endif

// UTILS fct to convert a cv::Mat to a sl::zed::Mat
sl::zed::Mat cvMat2slMat(cv::Mat &cvMat) {
    DATA_TYPE type;
    sl::zed::Mat output;
    switch (cvMat.depth()) {
    case CV_8U:
        type = DATA_TYPE::UCHAR;
        break;
    case CV_32F:
        type = DATA_TYPE::FLOAT;
        break;
    default:
        throw std::runtime_error("Cannot cast cv::Mat into sl::zed::Mat, incompatible depth");
        break;
    }
    output.setUp(cvMat.cols, cvMat.rows, cvMat.step, cvMat.data, cvMat.channels(), type);
    return output;
}



// save function using opencv
void saveSbSimage(Camera* zed, std::string filename) {
    resolution imSize = zed->getImageSize();

    cv::Mat SbS(imSize.height, imSize.width * 2, CV_8UC4);
    cv::Mat leftIm(SbS, cv::Rect(0, 0, imSize.width, imSize.height));
    cv::Mat rightIm(SbS, cv::Rect(imSize.width, 0, imSize.width, imSize.height));

    sl::zed::slMat2cvMat(zed->retrieveImage(SIDE::LEFT)).copyTo(leftIm);
	sl::zed::slMat2cvMat(zed->retrieveImage(SIDE::RIGHT)).copyTo(rightIm);

    cv::imshow("Saving Image", SbS);
    cv::cvtColor(SbS, SbS, CV_RGBA2RGB);

    cv::imwrite(filename, SbS);
}


//main  function
int main(int argc, char **argv) {

    if (argc > 2){
        std::cout << "Only the path of a SVO can be passed in arg" << std::endl;
        //Sleep(2000);
        return -1;
    }

	SENSING_MODE dm_type = FULL; //FULL||RAW
    Camera* zed;

    if (argc == 1 ) // Use in Live Mode
		zed = new Camera(HD1080); //HD2K || HD1080 || VGA 
    else			// Use in SVO playback mode
        zed = new Camera(argv[1]);

    int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;

	ERRCODE err = zed->init(MODE::PERFORMANCE, 0, true); //MODE::QUALITY || MODE::PERFORMANCE

    // ERRCODE display
    cout << errcode2str(err) << endl;


    // Quit if an error occurred
    if (err != SUCCESS) {
        delete zed;
        return 1;
    }

    int key = ' ';
    int ViewID = 2;
    int count = 0;
    int ConfidenceIdx = 100;

	bool showOutput = true;
    bool DisplayDisp = true;
    bool displayConfidenceMap = false;

	cv::Mat greyMat(height, width, CV_8UC1);

    cv::Mat disp(height, width, CV_8UC4);
    cv::Mat anaplyph(height, width, CV_8UC4);
    cv::Mat confidencemap(height, width, CV_8UC4);

    cv::Size DisplaySize(1280, 720);
    cv::Mat dispDisplay(DisplaySize, CV_8UC4);
    cv::Mat anaplyphDisplay(DisplaySize, CV_8UC4);
    cv::Mat confidencemapDisplay(DisplaySize, CV_8UC4);

	cv::Mat fpsDisplay(cv::Size(500,100), CV_8UC3);
#ifdef GPUstuff
	cv::gpu::GpuMat DispMat(height, width, CV_8UC4);
#endif
    /* Init mouse callback */
    Mat depth;
    zed->grab(dm_type);
    depth = zed->retrieveMeasure(MEASURE::DEPTH); // Get the pointer
    // Set the structure
    mouseStruct._image = cv::Size(width, height);
    mouseStruct._resize = DisplaySize;
    mouseStruct.data = (float*) depth.data;
    mouseStruct.step = depth.step;
    mouseStruct.name = "DEPTH";
    /***/

    //create Opencv Windows
    cv::namedWindow("Disp", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Disp", onMouseCallback, (void*)&mouseStruct);
    cv::namedWindow("VIEW", cv::WINDOW_AUTOSIZE);

	cv::VideoWriter outputVideo("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(width, height), false);

    //loop until 'q' is pressed
    while (key != 'q') {

		clock_t begin = clock();

        // DisparityMap filtering
        //zed->setDispReliability(reliabilityIdx); !!function name has been change in Release 0.8 --see ChangeLog
        zed->setConfidenceThreshold(ConfidenceIdx);


        // Get frames and launch the computation
        bool res = zed->grab(dm_type);

        depth = zed->retrieveMeasure(MEASURE::DEPTH); // Get the pointer

		//clock_t end1 = clock();
		//double elapsed_secs1 = double(end1 - begin) / CLOCKS_PER_SEC;
		//cout << "elapsed secs/frame: " << 1/elapsed_secs1 << endl;

        // The following is the best way to save a disparity map/ Image / confidence map in Opencv Mat.
        // Be Careful, if you don't save the buffer/data on your own, it will be replace by a next retrieve (retrieveImage, NormalizeMeasure, getView....)
        // !! Disparity, Depth, confidence are in 8U,C4 if normalized format !! //
        // !! Disparity, Depth, confidence are in 32F,C1 if only retrieve !! //

		if (showOutput)
		{
			/***************  DISPLAY:  ***************/
			// Normalize disparity map (resolution defined by MODE in Camera::init, the quality factor)
			if (DisplayDisp){
				sl::zed::slMat2cvMat(zed->normalizeMeasure(MEASURE::DISPARITY)).copyTo(disp);
			}else{

				sl::zed::slMat2cvMat(zed->normalizeMeasure(MEASURE::DEPTH)).copyTo(disp);
			}

			// To get the depth at a given position, click on the disparity map
			//cout << " rows: " << disp.rows << " cols: " << disp.cols << endl;
		
			//cv::cvtColor(disp, greyMat, CV_BGRA2GRAY);
			//outputVideo.write(greyMat);

			cv::resize(disp, dispDisplay,DisplaySize);
			imshow("Disp", dispDisplay);

			if (displayConfidenceMap) {
				sl::zed::slMat2cvMat(zed->normalizeMeasure(MEASURE::CONFIDENCE)).copyTo(confidencemap);
				cv::resize(confidencemap, confidencemapDisplay, DisplaySize);
				imshow("confidence", confidencemapDisplay);
			}



			//Even if Left and Right images are still available through getView() function, it's better since v0.8.1 to use retrieveImage for cpu readback because GPU->CPU is done async during depth estimation. 
			// Therefore :
			// -- if disparity estimation is enabled in grab function, retrieveImage will take no time because GPU->CPU copy has already been done during disp estimation
			// -- if disparity estimation is not enabled, GPU->CPU copy is done in retrieveImage fct, and this function will take the time of copy.
			if (ViewID==sl::zed::STEREO_LEFT || ViewID==sl::zed::STEREO_RIGHT)
			{
				sl::zed::slMat2cvMat(zed->retrieveImage(static_cast<SIDE>(ViewID))).copyTo(anaplyph);
				cv::resize(anaplyph, anaplyphDisplay, DisplaySize);
			}
			else
			{
				sl::zed::slMat2cvMat(zed->getView(static_cast<VIEW_MODE>(ViewID))).copyTo(anaplyph);
				cv::resize(anaplyph, anaplyphDisplay, DisplaySize);
			}

			imshow("VIEW", anaplyphDisplay);
		}
		else
		{
#ifdef GPUstuff
			if (DisplayDisp){
				slMat2cvMatGPU(zed->normalizeMeasure(MEASURE::DISPARITY)).copyTo(DispMat);
				//cv::imshow("Disp", slMat2cvMatGPU(zed->normalizeMeasure(MEASURE::DISPARITY)));
			}
			else{
				slMat2cvMatGPU(zed->normalizeMeasure(MEASURE::DEPTH)).copyTo(DispMat);
			}
			cv::imshow("Disp", DispMat);
#endif
		}
		


#ifdef WIN32
        key = cv::waitKey(5);
#else
        key = cv::waitKey(5)% 256;
#endif

        // Keyboard shortcuts
        switch (key) {
        // ______________  THRESHOLD __________________
        case 'b':
            ConfidenceIdx -= 10;
            break;
        case 'n':
            ConfidenceIdx += 10;
            break;

       //re-compute stereo alignment
       case 'a' :
            zed->reset();
            break;

          //Change camera settings (here --> gain)
       case 'g': //increase gain of 1
        {
           int current_gain = zed->getCameraSettingsValue(sl::zed::ZED_GAIN);
           zed->setCameraSettingsValue(sl::zed::ZED_GAIN,current_gain+1);
           cout<<"set Gain to "<<current_gain+1<<endl;
        }
        break;

        case 'h': //decrease gain of 1
         {
            int current_gain = zed->getCameraSettingsValue(sl::zed::ZED_GAIN);
            zed->setCameraSettingsValue(sl::zed::ZED_GAIN,current_gain-1);
            cout<<"set Gain to "<<current_gain-1<<endl;
         }
         break;
            // ______________  VIEW __________________
        case '0': // left
            ViewID = 0;
            break;
        case '1': // right
            ViewID = 1;
            break;
        case '2': // anaglyph
            ViewID = 2;
            break;
        case '3': // gray scale diff
            ViewID = 3;
            break;
        case '4': // Side by side
            ViewID = 4;
            break;
        case '5': // overlay
            ViewID = 5;
            break;
            // ______________  Display Confidence Map __________________
        case 's':
            displayConfidenceMap = !displayConfidenceMap;
            break;
            //______________ SAVE ______________
        case 'w': // image
            saveSbSimage(zed, string("ZEDImage") + to_string(count) + string(".png"));
            count++;
            break;
        case 'v': // disparity
        {
            string filename = string(("ZEDDisparity") + to_string(count) + string(".png"));
            cv::Mat dispSnapshot;
            disp.copyTo(dispSnapshot);
            cv::imshow("Saving Disparity", dispSnapshot);
            cv::imwrite(filename, dispSnapshot);
            count++;
            break;
        }
        case 'r':
            dm_type = SENSING_MODE::RAW;
            cout << "SENSING_MODE: Raw" << endl;
            break;
        case 'f':
            dm_type = SENSING_MODE::FULL;
            cout << "SENSING_MODE: FULL" << endl;
            break;

        case 'd':
            DisplayDisp = !DisplayDisp;
            break;
		case 'o':
			showOutput = !showOutput;
			break;
        }

        ConfidenceIdx = ConfidenceIdx < 1 ? 1 : ConfidenceIdx;
        ConfidenceIdx = ConfidenceIdx > 100 ? 100 : ConfidenceIdx;

		clock_t end = clock();
		double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

		std::ostringstream oss, ass;
		oss << 1/elapsed_secs << " fps" << endl;
		ass << "w: " << width << " h: " << height;
		
		std::string timerString = oss.str();
		std::string resString = ass.str();

		cout << "elapsed secs/frame: " << 1 / elapsed_secs << endl;

		if (showOutput)
		{
			fpsDisplay.setTo(cv::Scalar(0));
			putText(fpsDisplay, timerString, cvPoint(10, 20), cv::FONT_HERSHEY_PLAIN, 1.5, cvScalar(0, 0, 255), 1, CV_AA);
			putText(fpsDisplay, resString, cvPoint(10, 60), cv::FONT_HERSHEY_PLAIN, 1.5, cvScalar(0, 255, 0), 1, CV_AA);
			cv::imshow("Frame execution time", fpsDisplay);
			
		}
    }

	outputVideo.release();

    delete zed;
    return 0;
}
