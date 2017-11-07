#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;

// MapPoint是一个地图点
class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    std::map<KeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    int GetIndexInKeyFrame(KeyFrame* pKF);
    bool IsInKeyFrame(KeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(MapPoint* pMP);    
    MapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound()
    {
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

public:
    // MapPoint的全局ID
    long unsigned int mnId;

    static long unsigned int nNextId;
    long int mnFirstKFid; // 创建该MapPoint的关键帧ID
    long int mnFirstFrame; // 创建该MapPoint的帧ID（每一个关键帧有一个帧ID）
    int nObs;

    // Tracking中用到的变量
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // bool mTrackInView:
    // TrackLocalMap-SearchByProjection中决定是否对该点进行投影的变量
    // mbTrackInView == false 的点有以下几种：
    // a 已经和当前帧经过匹配(TrackReferenceKeyFrame, TrackWithMotionModel)，但在优化过程中认为是outlier
    // b 已经和当前帧经过匹配且为inlier，这类点也不需要再投影
    // c 不再当前相机视野中的点（即未通过isFrustum判断）

    // long unsigned int mnTrackReferenceForFrame:
    // TrackLocalMap-UpdateLocalPoints中防止将MapPpoints重复添加至mvpLocalMapPoints的标记

    // long unsigned int mnLastFrameSeen:
    // TrackLocalMap-SearchLocalPoints中决定是否进行isInFrustum判断的变量
    // mnLastFrameSeen == mCurrentFrame.mnId的点有以下几种：
    // a 已经和当前帧经过匹配(TrackReferenceKeyFrame, TrackWithMotionModel)，但在优化过程中认为是outlier
    // b 已经和当前帧经过匹配且为inlier，这类点也不需要再投影

    // Local mapping中用到的变量
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Loop closing中用到的变量
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;

    static std::mutex mGlobalMutex;

protected:    

     // MapPoint在世界坐标系下的坐标
     cv::Mat mWorldPos;

     // 观察到该MapPoint的关键帧和该MapPpoint在关键帧中的索引
     std::map<KeyFrame*,size_t> mObservations;

     // 该MapPoint的平均观测方向
     cv::Mat mNormalVector;

     // 通过ComputeDistinctiveDescriptors()得到的最优描述子
     // 每个3D点有一个descriptor
     // 如果MapPoint与很多帧图像特征点对应（这种情况发生在由key frame构造时），那么距离其他描述子的平均距离最小的描述子就是最佳描述子
     // 如果MapPoint只与一帧的图像特征点（这种情况发生在由frame构造时），那么这个特征点的描述子就是该3D点的描述子
     cv::Mat mDescriptor;

     // 参考关键帧
     KeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad;
     MapPoint* mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM2

#endif // MAPPOINT_H
