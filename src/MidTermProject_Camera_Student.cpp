/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

void testing(std::string detectorType, std::string descriptorType)
{
      
    
    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = true;            // visualize results
  
    std::vector<float> detector_total_number;
    std::vector<float> match_total_number;
    std::vector<float> detection_total_time;
    std::vector<float> description_total_time;
    

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);
        if (dataBuffer.size() > dataBufferSize)
        {
         
          dataBuffer.erase(dataBuffer.begin());
        }
        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        //string detectorType = "AKAZE";
          
        double det_t = (double)cv::getTickCount();

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, false);
   
        }
        else if (detectorType.compare("FAST") == 0 || detectorType.compare("BRISK") == 0 ||detectorType.compare("ORB") == 0 ||detectorType.compare("AKAZE") == 0 ||detectorType.compare("SIFT") == 0)
        {
            detKeypointsModern(keypoints, imgGray, detectorType, false);
        }
        cout << "Keypoints on the preceeding vehicle before sifting:" << keypoints.size() << endl;
      //// EOF STUDENT ASSIGNMENT //finish the key point detector timing

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        vector<cv::KeyPoint> keypoints_roi;
        if (bFocusOnVehicle)
        {
            for(int i=0; i<keypoints.size(); i++){
               if (vehicleRect.contains(keypoints[i].pt)){
                   keypoints_roi.push_back(keypoints[i]);
               }
            }
            cout << "Keypoints on the preceeding vehicle after sifting:" << keypoints_roi.size() << endl;
            //keypoints = keypoints_roi;

        }

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        //bool bLimitKpts = false;
        //if (bLimitKpts)
        //{
          //  int maxKeypoints = 50;

            //if (detectorType.compare("SHITOMASI") == 0)
            //{ // there is no response info, so keep the first 50 as they are sorted in descending quality order
              //  keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
           // }
            //cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            //cout << " NOTE: Keypoints have been limited!" << endl;
        //}

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints_roi;  //use keypoints after sifting 
        det_t = ((double)cv::getTickCount() - det_t) / cv::getTickFrequency();
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT
        double des_t = (double)cv::getTickCount();

        cv::Mat descriptors;
        //string descriptorType = "BRIEF"; // BRIEF, ORB, FREAK, AKAZE, SIFT
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;
        
        des_t = ((double)cv::getTickCount() - des_t) / cv::getTickFrequency();
        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string descriptorFormat {};
            double mat_t = (double)cv::getTickCount();
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN           
            if (descriptorType.compare("SIFT") == 0) {
                descriptorFormat = "DES_HOG";
            }
            else {
                descriptorFormat = "DES_BINARY";
            }
            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorFormat, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;
            mat_t = ((double)cv::getTickCount() - mat_t) / cv::getTickFrequency();
            
            cout << "detectorType: " << detectorType << ",";
            cout << "descriptorType: " << descriptorType << ",";
            cout << "number of matched key points: " << matches.size() << ",";
            cout << "detection time: " << 1000 * det_t / 1.0 << "ms,";
            cout << "descriptor extraction time: " << 1000 * des_t / 1.0 << "ms,";
            cout << "matching time: " << 1000 * mat_t / 1.0 << "ms,";
            cout << "total time: " << (1000 * det_t / 1.0) + (1000 * des_t / 1.0) + (1000 * mat_t / 1.0) << "ms" << endl;
          
            detector_total_number.push_back(keypoints.size());  //collect the keypoints number of each image and save them in the vector
            match_total_number.push_back(matches.size());
            detection_total_time.push_back(det_t);
            description_total_time.push_back(des_t);
          
          
          
            // visualize matches between current and previous image
            bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }
    } // eof loop over all images
    //float average = accumulate( detector_total_number.begin(), detector_total_number.end(), 0.0/ detector_total_number.size());
    float detector_average_number;
    float match_average_number;
    float detection_average_time;
    float description_average_time;
    float detector_sum = 0;
    float match_sum = 0;
    float detection_time_sum = 0;
    float description_time_sum = 0;
  
    for (int p=0 ; p<10 ;p++)
    {
      detector_sum += detector_total_number[p];   //sum up all
      detection_time_sum += detection_total_time[p];
      description_time_sum += description_total_time[p];
    }
    
    for (int m=0 ; m<9 ;m++)
    {
      match_sum += match_total_number[m];
    }
  
    detector_average_number = detector_sum/10;    //get the average number of keypoints before sifting for each detector/descriptor pair type
    match_average_number = match_sum/9;
    detection_average_time = detection_time_sum/10;
    description_average_time = description_time_sum/10;
    cout << "detectorType: " << detectorType << ",";
    cout << "descriptorType: " << descriptorType << ",";
    cout << "detector_average_number: " << detector_average_number << endl;
    cout << "matching_average_number: " << match_average_number << endl;
    cout << "detection_average_time: " << 1000 * detection_average_time/1.0 << "ms" << endl;
    cout << "description_average_time: " << 1000 * description_average_time/1.0 << "ms" << endl;
}


int main(int argc, const char *argv[])
{
  string detector[6] = { "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "SIFT" };
  string descriptor[4] = { "BRIEF", "ORB", "FREAK", "SIFT" };
  
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      if (i==5 && j==1)   //sift plus orb cause ram fault
      {
        continue;
      }
      testing(detector[i], descriptor[j]);
      
    }
  }
  testing("AKAZE", "AKAZE");  //akaze special case
  
  return 0;
}