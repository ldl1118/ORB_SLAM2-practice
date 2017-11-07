#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"

namespace ORB_SLAM2
{

class ORBmatcher
{    
public:

    ORBmatcher(float nnratio=0.6, bool checkOri=true);

    // 计算两个ORB描绘子之间的汉明距离
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    // 寻找frame中的特征点与投影的MapPoint之间的匹配，返回match的数量。
    // 通过投影，对Loccal MapPoint进行跟踪。        用于tracking线程的track local map
    //
    // 将Local MapPoint投影到当前帧，由此增加当前帧的MapPoints
    // 在SearchLocalPoints()中已经将Local MapPoints重投影(isInFrustum)到当前帧，并标记这些点是否在当前帧的视野中，即mbTrackInView
    //
    // 对于这些MapPoints，在其投影点附近根据描述子的距离选取匹配，并通过最终的方向投票机制剔除
    // 参数：
    // F                当前帧
    // vpMapPoints      Local MapPoints
    // th               阈值
    // return           成功匹配的数量
    // 参见 SearchLocalPoints()  isInFrustum()
    int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3);

    // 通过投影，对上一帧的特征点进行跟踪。    用于Tracking中的 track from previous frame
    // 1. 将上一帧的MapPoints投影到当前帧（根据速度模型可以估计当前帧的Tcw）
    // 2. 在投影点附近，根据描述子距离选取匹配，并通过最终的方向投票机制剔除
    // 参数：
    // CurrentFrame     当前帧
    // LastFrame        上一帧
    // th               阈值
    // bMono            是否为单目
    // return           成功匹配的数量
    // 参见 SearchByBoW()
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

    // 将关键帧的MapPoints投影到当前帧，并寻求匹配。  用于Tracking中的relocalisation
    int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

    // 用相似变换投影MapPoints，并寻求匹配。     用于Loop Closing中的回环检测
     int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th);

    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    // Used in Relocalisation and Loop Detection
    int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
    int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);

    // 地图初始化时的mathcing，仅用于单目
    int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

    // Matching，用于三角化新的MapPoints，并检验Epipolar约束。
    int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12,
                               std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);

    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    // In the stereo and RGB-D case, s12=1
    int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);

    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0);

    // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
    int Fuse(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);

public:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;

protected:

    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF);

    float RadiusByViewingCos(const float &viewCos);

    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    float mfNNratio;
    bool mbCheckOrientation;
};

}// namespace ORB_SLAM2

#endif // ORBMATCHER_H
