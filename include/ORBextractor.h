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
*
* Image Pyramid : 이미지의 크기를 단계적으로 축소시키며 일련의 축소된 이미지를 피라미드를 생성한다.
* - > 여러가지 scale 별로 특징점을 검출함으로써 scale에 invariant하게 특징점 검출을 수행할 수 있다.
* 
* ORBextractor Class 주요 기능들
* 1. Image pyramid를 만들고 각 scale에서 feature를 추출함 (feature points & descriptors)
* 2. 하위 Tracking 작업과 Optimization 작업에서 사용되는 image pyramid, scale factor, scale variance 등이 만들어짐
* 
* ORBextractor 함수 기능 정리
* ComputePyramid() : 입력 이미지의 피라미드를 계산함
* ComputeKeyPointsOctTree(), DistributeOctTree() : octree 분포 방법을 사용하여 이미지 피라미드의 각 layer에서 feature 추출
* ComputeKeyPointsOld() : octree를 사용하지 않고 feature를 추출하는 대체 방법
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

    std::vector<cv::Mat> mvImagePyramid; //< image for each layer of the image pyramid

protected:
    /**
     * @brief Calculate the image pyramid layer by layer, and performs the follwing two steps for each layer of image
     *        1) Scale the image first and scale it to mvScaleFactor the corresponding size.
     *        2) Add a circle of thickness outside the image for padding
     *           extracting FAST feature points require 3 (= a circle with a radius of around the feature point)
     *           calculating ORB descriptor requires 16 (= a circle with a radius of around the feature point)
     *
     *        ORBextractor::ComputePytramid()에서
     *           Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
     *        EDGE_THRESHOLD = 19 이므로 padding 수행 후 결과 이미지의 size를 만드는 과정임을 알 수 있다.
     *
     *        copyMakeBorder() implements copying and padding 
     * 
     * @param image 
     */
    void ComputePyramid(cv::Mat image);

    /**
     * @brief Extract feature points and filter
     *        To strive for feature points to be evenly distributed in all parts of the image.
     *        1) Search for feature points separately CELL.
     *           If the response value of a certain feature point is generally small, lower the score line and search again
     *        2) Perform octree screening on all obtained feature points (ORBextractor::DistributeOctTree)
     *           If the number of feature points in a certain area is too dense, only take the one with the largest response value
     * 
     * @param allKeypoints all image pyramid keypoints
     */
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

    /**
     * @brief This function acts in ORBextractor::ComputeKeyPointsOctTree()
     *        Perform octree screening(non-maximum value suppression), continuously divides the image area with feature points into four equal parts,
     *        until enough partitions are separated, and only the feature point with the largest response value is retained in each partition.
     *        
     * @param vToDistributeKeys all feature points of the current pyramid image layer
     * @param minX 
     * @param maxX 
     * @param minY 
     * @param maxY 
     * @param nFeatures number of feature points to be extracted
     * @param level 
     * @return std::vector<cv::KeyPoint> the feature point vector container that has been evenly dispersed
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

    std::vector<int> umax; //< umax[v] = u
    // ex) umax[0] = 8, umax[1] = 7, umax[2] = 7, umax[3] = 7, umax[4] = 6, umax[5] = 6, umax[6] = 4, umax[7] = 1

    std::vector<float> mvScaleFactor; //< zoom factor for each level
    // ex) mvScaleFactor = [1, 1.2, 1.44, 1.728, 2.074, 2.488, 2.986, 3.583]

    std::vector<float> mvInvScaleFactor; //< the reciprocal of the zoom factor at each level
    // ex) mvInvScaleFactor = [1, 0.833, 0.694, 0.579, 0.482, 0.402, 0.335, 0.2791]

    std::vector<float> mvLevelSigma2; //< the square of the zoom factor at each level
    // ex) mvLevelSigma2 = [1, 1.44, 2.074, 2.986, 4.300, 6.190, 8.916, 12.838]

    std::vector<float> mvInvLevelSigma2; //< the reciprocal of the square of the zoom factor at each level
    // ex) mvInvLevelSigma2 = [1, 0.694, 0.482, 0.335, 0.233, 0.162, 0.112, 0.078]
};

} //namespace ORB_SLAM

#endif