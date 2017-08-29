#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <ctime>

using namespace std;

int main( int _argc, char** _argv )
{
	// Images
	cv::Mat source, target, lab_source_f, lab_target_f;
	cv::Mat output;
	source = cv::imread("/Users/markpp/Desktop/eks/red.jpg");
	target = cv::imread("/Users/markpp/Desktop/eks/yellow.jpeg");
	cv::resize(source, source, cv::Size(), 0.5, 0.5);
	cv::resize(target, target, cv::Size(), 0.5, 0.5);

	cv::imshow("source", source);
	cv::imshow("target", target);

	cv::cvtColor(source, source, CV_BGR2Lab);
	cv::cvtColor(target, target, CV_BGR2Lab);

	source.convertTo(lab_source_f, CV_32FC3);
	target.convertTo(lab_target_f, CV_32FC3);

    cv::Scalar source_mean, source_std, target_mean, target_std;
    cv::meanStdDev(lab_source_f, source_mean, source_std);
    cv::meanStdDev(lab_target_f, target_mean, target_std);
    //cout << "mean: " << source_mean[0] << ", " << source_mean[1] << ", " << source_mean[2] << endl;

    cv::Mat target_channels[3];   //destination array
    split(lab_target_f, target_channels);//split source

    //cv::imshow("before", target_channels[0]);
    // subtract the means from the target image
    subtract(target_channels[0], target_mean[0], target_channels[0]);
    subtract(target_channels[1], target_mean[1], target_channels[1]);
    subtract(target_channels[2], target_mean[2], target_channels[2]);

    // scale by the standard deviations
	target_channels[0] = (source_std[0] / target_std[0]) * target_channels[0];
	target_channels[1] = (source_std[1] / target_std[1]) * target_channels[1];
	target_channels[2] = (source_std[2] / target_std[2]) * target_channels[2];

    // add in the source mean
	add(target_channels[0], source_mean[0], target_channels[0]);
	add(target_channels[1], source_mean[1], target_channels[1]);
	add(target_channels[2], source_mean[2], target_channels[2]);

    merge(target_channels, 3, output);
    output.convertTo(output, CV_8UC3);
    cv::cvtColor(output, output, CV_Lab2BGR);
    cv::imshow("output", output);
    cv::waitKey();
	//for( ; ; ){}

	return 0;	
}		
