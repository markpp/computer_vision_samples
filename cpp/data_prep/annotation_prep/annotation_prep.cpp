#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>
#include <sstream>
#include <fstream>

bool update = true;
bool quitBool = false;
bool cont = false;


void erodeAnno(cv::Mat annoMat, cv::Mat depthMat, int counter, std::string root_path, std::string view_id)
{

  cv::Mat full_anno(annoMat.rows, annoMat.cols, CV_8UC1, cv::Scalar(0,0,0));

  cv::Mat misc(annoMat.rows, annoMat.cols, CV_8UC1, cv::Scalar(0,0,0));
  cv::Mat heart(annoMat.rows, annoMat.cols, CV_8UC1, cv::Scalar(0,0,0));
  cv::Mat liver(annoMat.rows, annoMat.cols, CV_8UC1, cv::Scalar(0,0,0));
  cv::Mat lung(annoMat.rows, annoMat.cols, CV_8UC1, cv::Scalar(0,0,0));

  for( int i = 0; i < annoMat.rows; ++i)
  //for( int i = 260; i < annoMat.rows-140; ++i)
  {
    for( int j = 0; j < annoMat.cols; ++j)
    //for( int j = 200; j < annoMat.cols-140; ++j)
    {
      // Disregard pixels with extreeme depth
      if(depthMat.at<short>(i,j) > 280 && depthMat.at<short>(i,j) < 380)
      {
        uint8_t label = annoMat.at<uchar>(i,j);
        if(label==0)
        {
          label = 50;
          misc.at<uchar>(i,j) = label;
          full_anno.at<uchar>(i,j) = label;
        }
        else if(label==100)
        {
          heart.at<uchar>(i,j) = label;
          full_anno.at<uchar>(i,j) = label;
        }
        else if(label==150)
        {
          liver.at<uchar>(i,j) = label;
          full_anno.at<uchar>(i,j) = label;
        }
        else if(label==200)
        {
          lung.at<uchar>(i,j) = label;
          full_anno.at<uchar>(i,j) = label;
        }
        else
        {
          std::cout << "Unexpected label was: " << label << std::endl;
          full_anno.at<uchar>(i,j) = label;
        }
      }
    }
  }

  std::string erodeAnnoPath, fixedAnnoPath;
  erodeAnnoPath = root_path + "/BORDER_ANNO/BORDER_ANNO_" + std::to_string(counter) + view_id + ".png";
  fixedAnnoPath = root_path + "/COMPLETE_ANNO/COMPLETE_ANNO_" + std::to_string(counter) + view_id + ".png";
  cv::imwrite(fixedAnnoPath, full_anno);

  cv::Mat mask(annoMat.rows, annoMat.cols, CV_8UC1, cv::Scalar(255,255,255));
  /// Find contours
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  int lineThickness = 1;
  findContours( misc, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
  for( int i = 0; i< contours.size(); i++ )
  {
    for( int i = 0; i< contours.size(); i++ )
    {
      drawContours( mask, contours, i, cv::Scalar(0,0,0), lineThickness, 8, hierarchy );
    }
  }

  findContours( heart, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
  for( int i = 0; i< contours.size(); i++ )
  {
    for( int i = 0; i< contours.size(); i++ )
    {
      drawContours( mask, contours, i, cv::Scalar(0,0,0), lineThickness, 8, hierarchy );
    }
  }

  findContours( liver, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
  for( int i = 0; i< contours.size(); i++ )
  {
    for( int i = 0; i< contours.size(); i++ )
    {
      drawContours( mask, contours, i, cv::Scalar(0,0,0), lineThickness, 8, hierarchy );
    }
  }

  findContours( lung, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
  for( int i = 0; i< contours.size(); i++ )
  {
    for( int i = 0; i< contours.size(); i++ )
    {
      drawContours( mask, contours, i, cv::Scalar(0,0,0), lineThickness, 8, hierarchy );
    }
  }

  cv::Mat border_anno;
  //cv::imshow("mask", mask);
  bitwise_and(mask, full_anno, border_anno);
  cv::imwrite(erodeAnnoPath, border_anno);
  cv::imshow("border_anno", border_anno);
}


int main(int argc, char ** argv)
{
  int counter = 0;
  std::string root_path, view_id, path_anno, path_depth;
  //std::cout << "arg was: " << argv[1] << std::endl;
  root_path = argv[1];
  view_id = argv[2];

  while (1)
  {
    path_anno = root_path + "/ORG_ANNO/ORG_ANNO_" + std::to_string(counter) + view_id + ".png";
    path_depth = root_path + "/DEPTH/DEPTH_" + std::to_string(counter) + view_id + ".png";
    //std::cout << path_anno << std::endl;
    //std::cout << path_depth << std::endl;

    cv::Mat img_anno = cv::imread(path_anno,0);
    if (img_anno.empty())
    {
      std::cout << "empty frame.." << std::endl;
      std::cout << "path was: " << path_anno << std::endl;
      break;
    }
    cv::Mat img_depth = cv::imread(path_depth,-1);
    if (img_depth.empty())
    {
      std::cout << "empty frame.." << std::endl;
      std::cout << "path was: " << path_depth << std::endl;
      break;
    }

    erodeAnno(img_anno, img_depth, counter, root_path, view_id);

    //cv::waitKey();
    counter++;
    if(counter == 151) // 500
    {
      break; //9
    }
    //cv::waitKey();

    int k = cv::waitKey(20);
    if (k=='q')
    {
    	quitBool = true;
    }
    if (k=='c')
    {
    	cont = true;
    }
    if(quitBool)
    {
      break;
    }
  }

  return (0);
}
