/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SPEXTRACTOR_H
#define SPEXTRACTOR_H

#include <vector>
#include <list>
#include "Eigen/Core"
#include <opencv2/opencv.hpp>

#include "super_point.h"


namespace ORB_SLAM3
{

class SPextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };
    /**
     * @brief Construct a new SPextractor object
     * 
     * @param scaleFactor scale pyraimd scale factor
     * @param nlevels scale pyramid max level
     */
    SPextractor(float scaleFactor, int nlevels);

    ~SPextractor(){}

    /**
     * @brief Image feature extract to ORB
     * 
     * @param _image input image
     * @param _keypoints all pyramid SuperPoint vector
     * @param _descriptors all keypoint and descriptor matrix
     * @param vLappingArea limit feature extract area
     * @return int all keypoint length
     */
    int operator()( cv::InputArray _image,
                    std::vector<cv::KeyPoint>& _keypoints, cv::OutputArray _ORBDescriptors, Eigen::Matrix<double, 259, Eigen::Dynamic>& _descriptors, std::vector<int> &vLappingArea);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;

protected:
    /**
     * @brief configure image pyramid
     * mvImagePyramid에 level별 scale에 맞는 image 저장
     * @param image 
     */
    void ComputePyramid(cv::Mat image);
    /**
     * @brief Extract keypoints from all image pyramid levels
     * 이미지 pyramid 별로 keypoint를 추출한다
     * @param allKeypoints all image pyramid keypoints
     */
    void ComputeKeypointAndDescriptor(std::vector<std::vector<cv::KeyPoint> >& allKeypoints, Eigen::Matrix<double, 259, Eigen::Dynamic>& descriptors);    
    /**
     * @brief 
     * 
     * @param vToDistributeKeys 
     * @param minX 
     * @param maxX 
     * @param minY 
     * @param maxY 
     * @param nFeatures 
     * @param level 
     * @return std::vector<cv::KeyPoint> 
     */
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);


    int nfeatures; //< pyraimd by max features
    double scaleFactor; //< scale pyraimd scale factor
    int nlevels; //< scale pyramid max level

    std::vector<int> mnFeaturesPerLevel; //<

    std::vector<int> umax; //<

    std::vector<float> mvScaleFactor; //< pyramid by scale factor
    std::vector<float> mvInvScaleFactor; //< pyramid by inverse scale factor
    std::vector<float> mvLevelSigma2; //< pyramid by scale factor square
    std::vector<float> mvInvLevelSigma2; //<  pyramid by inverse scale factor square

    int mInputImageWidth;
    int mInputImageHeight;

    std::vector<cv::Point> pattern; //< ORB descriptor bit pattern
private:
    static std::shared_ptr<SuperPoint> mSuperpoint; //< SuperPoint model
    static bool mModelInitializationFlag; //< model initialization flag
};

} //namespace ORB_SLAM

#endif

