#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex> // 加互斥锁

namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

// 与关键帧有关的很多数据会被三个线程同时访问，所以用锁的地方很多。
class KeyFrame
{
public:
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // 位姿的相关函数，都用到锁
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();


    void ComputeBoW();

    // Covisibility graph 的相关函数
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame*> GetConnectedKeyFrames();
    std::vector<KeyFrame*> GetVectorCovisibleKeyFrames();
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree 的相关函数
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // 观察MapPoint的相关函数 
    void AddMapPoint(MapPoint* pMP, const size_t &idx); // size_t是与机器相关的unsigned类型，其大小足以保证存储内存中musician的大小
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(MapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();
    std::vector<MapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint 的相关函数
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    bool IsInImage(const float &x, const float &y) const;
    // const用在成员函数之后，主要是针对类的const对象。这里，成员函数属于KeyFrame类。
    // [定义]:
    // 使用const说明的成员函数，成为[常成员函数]。常成员函数的说明格式如下：
    // <类型说明符> <函数名>(参数表) const;
    // const时加载函数说明后面的类型修饰符，它是函数类型的一个组成部分。因此，在函数实现部分也要带const关键字。
    // -------------------------------------------------------------------------------------------
    // 只有常成员函数才有资格操作常量或对象。
    // const对象只能调用const成员函数，在const函数中调用非const成员函数是错误的。
    // const对象的值不能被修改，在const成员函数中修改const对象数据成员的值是错误的。

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // 计算场景的深度，用于单目情况。   q=2 median 
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b)
    {
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2)
    {
        return pKF1->mnId<pKF2->mnId;
    }

    // 以下的变量要么仅会有一个线程访问，要么保持不变。不需要互斥锁(mutex)
public:
    // 上一个KeyFrame的ID              建议改成nLastID
    static long unsigned int nNextId;
    // mnId = nNextId+1，为当前KeyFrame的ID
    long unsigned int mnId;
    // KeyFrame初始化的时候需要Frame，mnFrameId记录了该KeyFrame是由哪个Frame初始化的
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)  与Frame类中的定义相同
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Tracking中用到的变量
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Local mapping中用到的变量
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Keyframe database中用到的变量
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Loop closing中用到的变量
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // 标定的参数
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // 特征点的数量
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    // 和Frame类中的定义相同
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // 单目情况下为负值
    const std::vector<float> mvDepth; // 单目情况下为负值
    const cv::Mat mDescriptors;

    // BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors; // 尺度因子    scale^n n为层数
    const std::vector<float> mvLevelSigma2; // 尺度因子的平方 
    const std::vector<float> mvInvLevelSigma2;

    // 图像边缘和标定参数
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;

    // 以下变量需要通过加互斥锁才能访问，以保证其安全性
protected:

    // SE3 位姿和相机中心
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;
    cv::Mat Cw; // 立体相机的中点，只用于可视化

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // 图像上的网格，用于加速特征匹配。
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    // Covisibility Graph
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights; // 与该关键帧链接的关键帧和权重
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames; // 排序后的关键帧
    std::vector<int> mvOrderedWeights; // 排序后的权重  降序

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;
    // std::set是集合，相比vector，进行插入数据这样的操作时，会自动排序。

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};
} //namespace ORB_SLAM2

#endif // KEYFRAME_H