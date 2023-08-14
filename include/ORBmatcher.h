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
 * ORBmatcher 요약
 * 
 * ORBmatcher Class 주요 기능들
 * 1. Initialization, Tracking, Relocation, Loop detection, Triangulation of new map points etc.
 * 2. 두 ORB Descriptors 간의 Hamming Distance를 계산
 * 3. 특징점 사이에서 Epipolar constraints를 충족하는지 여부를 판단
 * 
 * ORBmatcher 함수 기능 정리
 * DescriptorDistance() : 두 ORB descriptors 사이의 Hamming distance를 계산한다.
 * SearchByProjection() : Map point를 Frame 또는 KeyFrame에 투영하여 feature를 matching한다.
 * SearchByBow() : Relocation 및 Loop closure detection 상황에서 feature를 matching하기 위해 Bag of Words 모델을 사용한다.
 * SearchForInitialiation() : Map 초기화를 위해 feature matching을 한다. (오직 Monocular 상황에서만 사용)
 * SearchForTriangulation() : 새로운 map point들을 삼각측량하기 위해 feature matching을 한다. epipolar 제약조건을 체크한다.
 * SearchBySim3() : Sim3변환(scale, rotation, and translation matrix)이 된 keyframe들 사이에서 feature matching을 한다.
 * Fuse() : map point 들을 keyframe에 투영하고 중복된 map point 들을 찾는다.
 */

#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include"sophus/sim3.hpp"

#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"


namespace ORB_SLAM3
{

    class ORBmatcher
    {
    public:
        /**
         * @brief Construct a new ORBmatcher object
         * 
         * @param nnratio ratio of the best and the second distance
         * @param checkOri check feature orientation flag (true == check)
         */
        ORBmatcher(float nnratio=0.6, bool checkOri=true);

        /**
         * @brief Calculate the Hamming Distance for a pair of descriptors
         * 
         * @param a one descriptor 
         * @param b the other descriptor
         * @return (int) 
         */
        static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

        /**
         * @brief Tracking of Local MapPoint through projection (Tracking)
         *        Project the Local MapPoint into the current frame, thereby increasing the MapPoints of current frame
         *        Step1. Traverse valid Local MapPoints.
         *        Step2. Set the size of the search window.
         *               Depends on the anlge of view, if the angle between the current angle of view and the average angle is small,
         *               r takes a smaller value.
         *        Step3. Search through the projection point,
         *               search window and predicted scale to find out the index of candidate matching points within the search radius.
         *        Step4. Find the best and second best matching points among the candidate matching points.
         *        Step5. Filter the best matching points.
         * 
         *        
         * @param F current Frame
         * @param vpMapPoints Local MapPoints
         * @param th search radius
         * @param bFarPoints FarPoints flag
         * @param thFarPoints FarPoints threshold
         * if (bFarPoints == True) and (mTrackDepth > thFarPoints), continue
         * @return (int) Number of successful matches
         */
        int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3, const bool bFarPoints = false, const float thFarPoints = 50.0f);

        /**
        * @brief Tracking of last frame through projection (Tracking)
        *        The previous frame contained MapPoints, and these MapPoints were tracked. thereby increasing the MapPoints of the current frame.
        *        Step1. Project the MapPoints of the previous frame to the current frame. (the Tcw of the current frame can be estimated according to the velocity model)
        *        Step2. Select a match (based on the descriptor distance near the projection point, and the final direction voting mechanism to eliminate)
        * 
        * @param CurrentFrame Current frame
        * @param LastFrame previous frame
        * @param th Max search threshold
        * @param bMono Monocular slam flag
        * @return (int) number of successful matches
        */
        int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

        // Project MapPoints seen in KeyFrame into the Frame and search matches.
        // Used in relocalization (Tracking)
        /**
         * @brief 
         * 
         * @param CurrentFrame
         * @param pKF
         * @param sAlreadyFound 
         * @param th 
         * @param ORBdist 
         * @return (int)
         */
        int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

        // Project MapPoints using a Similarity Transformation and search matches.
        // Used in loop detection (Loop Closing)
        /**
         * @brief According to the Sim3 transformation,
         *        Project each vpPoints onto pKF, Determine the search area according to the scale, Match the feature points in the area according to the MapPoint Descriptor.
         *        If the matching error is less than TH_LOW * ratioHamming, the matching is successful, and vpMatched is updated.
         * 
         * @param pKF KeyFrame
         * @param Scw Sim3 matrix, the transformation matrix
         * @param vpPoints MapPoint
         * @param vpMatched MapPoint mathcing points
         * @param th factor of the search radius
         * @param ratioHamming 
         * @return (int) Number of successful matches
         */
        int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th, float ratioHamming=1.0);

        // Project MapPoints using a Similarity Transformation and search matches.
        // Used in Place Recognition (Loop Closing and Merging)
        /**
         * @brief 
         * 
         * @param pKF 
         * @param Scw 
         * @param vpPoints 
         * @param vpPointsKFs 
         * @param vpMatched 
         * @param vpMatchedKF 
         * @param th 
         * @param ratioHamming 
         * @return (int) 
         */
        int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, const std::vector<KeyFrame*> &vpPointsKFs, std::vector<MapPoint*> &vpMatched, std::vector<KeyFrame*> &vpMatchedKF, int th, float ratioHamming=1.0);

        // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
        // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
        // Used in Relocalization and Loop Detection
        /**
         * @brief Quiclkly match the feature points in the key frame (pKF) and the current frame (F) through the Bag of Words (bow).
         *        The feature points that do not belong to the same node (node) skip the matching directly.
         *        The feature points that belong to the same node (node0) perform the matching by describing the sub-distance.
         *        According to the matching, the MapPoints corresponding to the feature points in the key frame (pKF) are used to update the MapPoints corresponding to the feature points in current frame (F).
         *        The false matches are eliminated through distance threshold, ratio threshold and anlge voting.
         *
         * @param pKF KeyFrame
         * @param F Current Frame
         * @param vpMapPointMatches The match corresponding to MapPoints in Current Frame, NULL means no match
         * @return (int) Number of successful matches
         */
        int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
        /**
         * @brief Track the feature points of the key frame through the Bag of Words.
         *        This function is used to match the feature points between the two key frames in the closed loop detection.
         *        Quickly match the feature points in pKF1 and pKF2 through BoW.
         *        (Features that do not belong to the same node points directly skip matching.)
         *        Match the feature points belonging to the same node through the descriptor distance, and update vpMatches12 according to the matching.
         * 
         * @param pKF1 one KeyFrame
         * @param pKF2 the other KeyFrame
         * @param vpMatches12 MapPoint in pKF2 that matches pKF1, NULL means no match
         * @return (int) Number of successful matches
         */
        int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);

        // Matching for the Map Initialization (only used in the monocular case)
        /**
         * @brief Step1. Build a rotation histogram.
         *        Step2. Search all candidate matching feature points in the current frame 2 within the radius window.
         *        Step3. Traverse all potential matching candidate points in the search window to find the optimal and sub-optimal.
         *        Step4. Check the optimal and sub-optimal results, meet the threshold, optimal / sub-optimal ratio, and delete duplicate matches.
         *        Step5. Calculate the histogram where the rotation angle difference of the matching point is located
         *        Step6. Screen out the "non-mainstream" part in the rotation histogram.
         *        Step7. Save the matched feature points that passed the srceening.
         * 
         * @param F1 reference frame
         * @param F2 current frame
         * @param vbPrevMatched 이전 매칭 결과를 담은 벡터. 매칭 결과를 구한 후 업데이트 됨.
         * @param vnMatches12 vnMatches[0] = 4는 F1의 첫번째 특징점과 F2의 다섯번째 특징점이 성공적으로 일차함을 의미함
         * @param windowSize the size of the search window
         * @return (int) Number of successful matches
         */
        int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

        // Matching to triangulate new MapPoints. Check Epipolar Constraint.
        /**
         * @brief Use the fundamental matrix to generate new 3d points in the unmatched feature points between two keyframes.
         * 
         * @param pKF1 key frame1
         * @param pKF2 key frame2
         * @param vMatchedPairs save matching feature point pairs, feature points are represented by their indexes in keyframes
         * @param bOnlyStereo In the case of binocular and rgbd, the feature points are required to match on the right image
         * @param bCoarse 
         * @return (int) Number of successful matches
         */
        int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2,
                                   std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse = false);

        // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
        // In the stereo and RGB-D case, s12=1
        // int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);
        /**
         * @brief Through the Sim3 transformation, determine the approximate area of the feature points of pKF1 and pKF2.
         *        Similarly, determine the approximate area of the feature points of pKF2 in pKF1.
         *        In this area, match the descriptors to capture the missing matching feature points of pKF1 and pKF2.
         *        Update vpMatches12
         * 
         * @param pKF1 key frame1
         * @param pKF2 key frame2
         * @param vpMatches12 save matching feature point pairs, feature points are represented by their indexes in keyframes
         * @param S12 scale
         * @param th factor of the search radius
         * @return (int) Number of repeated MapPoints
         */
        int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const Sophus::Sim3f &S12, const float th);

        // Project MapPoints into KeyFrame and search for duplicated MapPoints.
        /**
         * @brief If the MapPoint can match the feature point of the keyframe, and the point has a corresponding MapPoint, then merge the two MapPoints (select the one with the most observations).
         *        If the MapPoint can match the feature point of the keyframe, and the point does not have a corresponding MapPoint, then add a MapPoint for the point.
         * 
         * @param pKF adjacent key frame
         * @param vpMapPoints MapPoints of the current key frame
         * @param th factor of the search radius
         * @param bRight 
         * @return (int) Number of repeated MapPoints
         */
        int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0, const bool bRight = false);

        // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
        /**
         * @brief 
         * 
         * @param pKF adjacent key frame
         * @param Scw Sim3 transformation from the world coordinate system to the pKF body coordinate system, which is used to transform vpPoints in the world coordinate system to the body coordinate system
         * @param vpPoints MapPoints of the current key frame
         * @param th factor of the search radius
         * @param vpReplacePoint save the pMPinKF that needs to be replaced
         * @return (int) Number of repeated MapPoints
         */
        int Fuse(KeyFrame* pKF, Sophus::Sim3f &Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);

    public:

        static const int TH_LOW; //<
        static const int TH_HIGH; //<
        static const int HISTO_LENGTH; //<
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        /**
         * @brief 
         * 
         * @param viewCos Between to angles(current, average)
         * @return float Search radius
         */
        float RadiusByViewingCos(const float &viewCos);

        /**
         * @brief 
         * 
         * @param histo 
         * @param L 
         * @param ind1 
         * @param ind2 
         * @param ind3 
         */
        void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

        float mfNNratio; //< ratio of the best and the second distance
        bool mbCheckOrientation; //< check feature orientation flag (true == check)
    };

}// namespace ORB_SLAM

#endif // ORBMATCHER_H