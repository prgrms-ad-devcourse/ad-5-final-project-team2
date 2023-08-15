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

/**
* ORBextractor 요약
* 
* ORB = Oriented FAST + Rotated BRIEF
* Image Pyramid : 이미지의 크기를 단계적으로 축소시키며 일련의 축소된 이미지를 피라미드를 생성한다.
* - > 여러가지 scale 별로 특징점을 검출함으로써 scale에 invariant하게 특징점 검출을 수행할 수 있다.
* 
* 
*/

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>


namespace ORB_SLAM3
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ExtractorNode>::iterator lit;
    bool bNoMore;
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };
    /**
     * @brief Construct a new ORBextractor object
     * 
     * @param nfeatures sum of the number of feature points extracted from all levels of image pyramid
     * @param scaleFactor scaling factor between adjacent levels of the image pyramid
     * @param nlevels scale pyramid max level (number of pyramid levels), (level 0 = 피라미드 가장 아래층)
     * @param iniThFAST descriptor threshold for extracting feature points (high)
     * @param minThFAST descriptor threshold for extracting feature points (low)
     */
    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    /**
     * @brief Image feature extract to ORB
     * 
     * @param _image input image
     * @param _mask noused argument
     * @param _keypoints all pyramid ORB keypoint vector
     * @param _descriptors all keypoint descriptor matrix
     * @param vLappingArea limit keypoint extract area
     * @return int all keypoint length
     */
    int operator()( cv::InputArray _image, cv::InputArray _mask,
                    std::vector<cv::KeyPoint>& _keypoints,
                    cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

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
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
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

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    std::vector<cv::Point> pattern; //< ORB descriptor bit pattern

    int nfeatures; //< sum of the number of feature points extracted from all levels of image pyramid
    double scaleFactor; //< scaling factor between adjacent levels of the image pyramid
    int nlevels; //< scale pyramid max level (number of pyramid levels), (level 0 = 피라미드 가장 아래층)
    int iniThFAST; //< descriptor threshold for extracting feature points (high)
    int minThFAST; //< descriptor threshold for extracting feature points (low)

    std::vector<int> mnFeaturesPerLevel; //< the number of feature points extracted from each level of the pyramid
    // ex) mnFeaturesPerLevel = [216, 181, 151, 126, 105, 87, 73, 61], sum is nfeatures

    std::vector<int> umax; //< 

    std::vector<float> mvScaleFactor; //< pyramid by scale factor
    std::vector<float> mvInvScaleFactor; //< pyramid by inverse scale factor
    std::vector<float> mvLevelSigma2; //< pyramid by scale factor square
    std::vector<float> mvInvLevelSigma2; //<  pyramid by inverse scale factor square
};

} //namespace ORB_SLAM

#endif

