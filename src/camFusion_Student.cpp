
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
        double t = 1.0/frameRate;
//        float lanewidth = 4.0;
//        std::vector<LidarPoint> point_holder;
//
//        //filter points that are not in or close to our lane as we just want to comupte TTC
//        for(auto it = lidarPointsPrev.begin();it != lidarPointsPrev.end();++it)
//        {
//            if(abs(it->y) <= lanewidth/2)
//            {
//                point_holder.emplace_back(*it);
//            }
//        }
//        lidarPointsPrev = point_holder;
//        point_holder.clear();
//
//        for(auto it = lidarPointsCurr.begin();it != lidarPointsCurr.end();++it)
//        {
//            if(abs(it->y) <= lanewidth/2)
//            {
//                point_holder.emplace_back(*it);
//            }
//        }
//        lidarPointsCurr = point_holder;

        //sort points and take median value for more stability
        std::sort(lidarPointsPrev.begin(), lidarPointsPrev.end(),[](LidarPoint one,LidarPoint two)
        {
            return one.x < two.x;
        });
        std::sort(lidarPointsCurr.begin(), lidarPointsCurr.end(),[](LidarPoint one,LidarPoint two)
        {
            return one.x < two.x;
        });
        int prev_mid = lidarPointsPrev.size()/2;
        int cur_mid = lidarPointsCurr.size()/2;
        double distance_0 = lidarPointsPrev[prev_mid].x;
        double distance_1 = lidarPointsCurr[cur_mid].x;

        TTC = (distance_1*t)/(distance_0-distance_1);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    int kpt_counter[prevFrame.boundingBoxes.size()][currFrame.boundingBoxes.size()] = { };
    //check if keypoints are in the bounding boxes and store number of matches between consecutive frames to determine id later on
    for(auto it = matches.begin();it!=matches.end()-1; ++it)
    {
        cv::KeyPoint train, query;
        query = prevFrame.keypoints[it->queryIdx];
        train = currFrame.keypoints[it->trainIdx];
        vector<int> query_ids, train_ids;
        for (int i = 0; i < prevFrame.boundingBoxes.size(); i++)
        {
            if (prevFrame.boundingBoxes[i].roi.contains(cv::Point(query.pt.x, query.pt.y)))
            {
                query_ids.emplace_back(i);
            }
        }

        for (int i = 0; i < currFrame.boundingBoxes.size(); i++)
        {
            if (currFrame.boundingBoxes[i].roi.contains(cv::Point(train.pt.x, train.pt.y)))
            {
                train_ids.emplace_back(i);
            }
        }

        if(!train_ids.empty() && !query_ids.empty())
        {
            for(auto i : query_ids)
            {
                for(auto j:train_ids)
                {
                    kpt_counter[i][j] +=1;
                }
            }
        }
    }

    //get max keypoint region for each bounding box in frame 1
    for(int i = 0; i < prevFrame.boundingBoxes.size(); ++i)
    {   int idx = 0;
        int max_kpts = 0;
        for(int j = 0; j < currFrame.boundingBoxes.size();++j)
        {
           if(max_kpts < kpt_counter[i][j])
           {
               max_kpts = kpt_counter[i][j];
               idx = j;
           }
        }
        bbBestMatches[i] = idx;
    }
   for(int i = 0; i < prevFrame.boundingBoxes.size();i++)
   {cout << " Bounding Box Number : " << i << " from first frame best matches with Box Number : " << bbBestMatches[i] << " from frame 2 " << endl;}
}
