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
 * ORBmatcher Class 주요 기능들
 * 1. 
 * 
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
         * @return int 
         */
        static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

        /**
         * @brief Tracking of Local MapPoint through projection (Tracking)
         *        Project the Local MapPoint into the current frame, thereby increasing the MapPoints of current frame
         *        
         * @param F current Frame
         * @param vpMapPoints Local MapPoints
         * @param th threshold
         * @param bFarPoints FarPoints flag
         * @param thFarPoints FarPoints threshold
         * if (bFarPoints == True) and (mTrackDepth > thFarPoints), continue
         * @return int number of successful matches
         */
        int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3, const bool bFarPoints = false, const float thFarPoints = 50.0f);

        /**
        * @brief Tracking of last frame through projection (Tracking)
        * 
        * @param CurrentFrame Current frame
        * @param LastFrame Lastest frame
        * @param th Max search threshold
        * @param bMono Monocular slam flag
        * @return int number of successful matches
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
         * @return int 
         */
        int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

        // Project MapPoints using a Similarity Transformation and search matches.
        // Used in loop detection (Loop Closing)
        /**
         * @brief 
         * 
         * @param pKF 
         * @param Scw 
         * @param vpPoints 
         * @param vpMatched 
         * @param th 
         * @param ratioHamming 
         * @return int 
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
         * @return int 
         */
        int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, const std::vector<KeyFrame*> &vpPointsKFs, std::vector<MapPoint*> &vpMatched, std::vector<KeyFrame*> &vpMatchedKF, int th, float ratioHamming=1.0);

        // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
        // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
        // Used in Relocalisation and Loop Detection
        /**
         * @brief Track the feature points of the key frame through the bag of words
         *
         * @param pKF KeyFrame
         * @param F Current Frame
         * @param vpMapPointMatches The match corresponding to MapPoints in Current Frame, NULL means no match
         * @return int the number of successful matches
         */
        int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
        /**
         * @brief 
         * 
         * @param pKF1 one KeyFrame
         * @param pKF2 the other KeyFrame
         * @param vpMatches12 
         * @return int 
         */
        int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);

        // Matching for the Map Initialization (only used in the monocular case)
        /**
         * @brief 
         * 
         * @param F1 
         * @param F2 
         * @param vbPrevMatched 
         * @param vnMatches12 
         * @param windowSize 
         * @return int 
         */
        int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

        // Matching to triangulate new MapPoints. Check Epipolar Constraint.
        /**
         * @brief 
         * 
         * @param pKF1 
         * @param pKF2 
         * @param vMatchedPairs 
         * @param bOnlyStereo 
         * @param bCoarse 
         * @return int 
         */
        int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2,
                                   std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse = false);

        // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
        // In the stereo and RGB-D case, s12=1
        // int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);
        /**
         * @brief 
         * 
         * @param pKF1 
         * @param pKF2 
         * @param vpMatches12 
         * @param S12 
         * @param th 
         * @return int 
         */
        int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const Sophus::Sim3f &S12, const float th);

        // Project MapPoints into KeyFrame and search for duplicated MapPoints.
        /**
         * @brief 
         * 
         * @param pKF 
         * @param vpMapPoints 
         * @param th 
         * @param bRight 
         * @return int 
         */
        int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0, const bool bRight = false);

        // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
        /**
         * @brief 
         * 
         * @param pKF 
         * @param Scw 
         * @param vpPoints 
         * @param th 
         * @param vpReplacePoint 
         * @return int 
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