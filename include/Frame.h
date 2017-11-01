#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
    #define FRAME_GRID_ROWS 48
    #define FRAME_GRID_COLS 64

    class MapPoint;
    class KeyFrame;

    class Frame
    { // 可以允许很多重名的函数存在吗？如何区分？
    public:
        Frame();
        // 拷贝构造器(constructor)
        Frame(const Frame &frame);

        // 立体相机的构造器
        Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
             ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc,
             cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
        
        // RGB-D相机的构造器
        Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp,
             ORBextractor* extractor, ORBVocabulary* voc,
             cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
        
        // 单目相机的构造器
        Frame(const cv::Mat &imGray, const double &timeStamp,
             ORBextractor* extractor, ORBVocabulary* voc,
             cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
        
        // 提取ORB特征。0：左图   1：右图
        // 提取的关键点存放在mvKeys和mDescriptors中
        // ORB是直接调用orbExtractor提取特征点的
        void ExtractORB(int flag, const cv::Mat &im);

        // 将特征点转换为词袋(BoW)向量，并存放在mBowVec中
        void ComputeBoW();

        // 设置位姿。用Tcw更新mTcw
        void SetPose(cv::Mat Tcw);

        // 计算旋转，平移和相机中心位置的矩阵
        void UpdatePoseMatrices();

        // 返回相机中心的位置
        inline cv::Mat GetCameraCenter()
        {
            return mOw.clone();
        }

        // 返回旋转矩阵的逆矩阵
        inline cv::Mat GetRotationInverse()
        {
            return mRwc.clone();
        }

        // 判断MapPoint(路标点)是否在视野中，并填充MapPoints中的变量，用于tracking。
        bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

        // 计算特征点的单元。若特征点在grid之外，则返回false
        bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

        vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel = -1, const int maxLevel = -1) const;

        // 寻找左图中每一个关键点在右图中对应的匹配关键点。如果匹配，则计算深度信息，并保存以左图关键点为基础的右手系(right coordinate)。
        void ComputeStereoMatches();

        // 如果深度图的深度信息合法(valid)，则生成以关键点为基础的右手系。
        void ComputeStereoFromRGBD(const cv::Mat &imDepth);

        // 若可得到立体/深度信息，则将关键点反投影到三维的世界坐标系中。
        cv::Mat UnprojectStereo(const int &i);

    public:
        // 字典，用于重定位
        ORBVocabulary* mpORBvocabulary;
        
        // 特征提取器。只在立体相机情况下才使用right。 
        ORBextractor *mpORBextractorLeft, *mpORBextractorRight;

        // 每帧的时间戳
        double mTimeStamp;

        // 标定矩阵和畸变校正参数
        cv::Mat mK;
        static float fx;
        static float fy;
        static float cx;
        static float cy;
        static float invfx;
        static float invfy;
        cv::Mat mDistCoef;

        // 立体相机中使用的基线，乘以fx
        float mbf;

        // 立体相机中使用的基线，单位：米
        float mb;

        // 远近点的阈值
        // 近点只需1帧即可插入地图，（对单目情况）远点需要2帧来判断后才能插入地图。
        float mThDepth;

        // 特征点的数量
        int N;

        // mvKeys: 原始左图提取出的特征点（未校正）
        // mvKeysRight: 原始右图提取出的特征点（未校正）
        // mvKeysUn: 校正mvKeys后的特征点。对于双目摄像头，一般得到的图像都是已经校正过的，再校正一次有些多余。
        std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
        std::vector<cv::KeyPoint> mvKeysUn;

        // 对于双目立体相机，mvuRight储存了左目像素点在右目中的对应点的横坐标
        // mvDepth：特征点对应的深度
        // 对于单目相机，这两个容器中的值都是-1
        std::vector<float> mvuRight;
        std::vector<float> mvDepth;

        // 词袋向量的结构
        DBoW2::BowVector mBowVec;
        DBoW2::FeatureVector mFeatVec;

        // 左目和右目中特征点对应的描述子。每一行对应一个特征点。
        cv::Mat mDescriptors, mDescriptorsRight;

        // 每个特征点对应的MapPoint。若没有找到特征点和MapPoint有关联，则指针为NULL
        std::vector<MapPoint*> mvpMapPoints;

        // 地图中观测不到的3D点
        std::vector<bool> mvbOutulier;

        // 将特征点分配到网格单元中，来降低MapPoints投影时匹配的复杂性
        // 坐标乘以mfGridELementWidthInv和mfGridElementHeightInv就可以确定在哪个格子
        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;

        // 每个格子分配的特征点数。将图像分成网格，保证提取的特征点比较均匀。
        // FRAME_GRID_ROWS 48
        // FRAME_GRID_COLS 64
        std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

        // 相机位姿 世界坐标系到相机坐标系的变换矩阵
        cv::Mat mTcw;

        static long unsigned int nNextId; // 下一帧的ID
        long unsigned int mnId; // 当前帧的ID

        // 指向参考关键帧的指针
        KeyFrame* mpReferenceKF;

        // 金字塔相关参数
        int mnScaleLevels; // 图像提取金字塔的层数
        float mfScaleFctor; // 图像提取金字塔的尺度因子
        float mfLogScaleFactor;
        vector<float> mvScaleFactors;
        vector<float> mvInvScaleFactors;
        vector<float> mvLevelSigma2;
        vector<float> mvInvLevelSigma2;

        // 未畸变时的图像边界（只计算一次），用于确定画格子时的边界
        static float mnMinX;
        static float mnMaxX;
        static float mnMinY;
        static float mnMaxY;

        static bool mbInitialComputations;

    private:
        // 未畸变情况下的特征点。仅用于RGB-D的情况，对于立体相机，则畸变已经得到了校正。在constructor中调用。
        void UndistortKeyPoints();

        // 计算未畸变图像的边缘。在constructor中调用。
        void ComputeImageBounds(const cv::Mat &imLeft);

        // 将特征点分配到网格中，加速特征点匹配。在constructor中调用。
        void AssignFeaturesToGrid();

        cv::Mat mRcw; // 从世界坐标系到相机坐标系的旋转矩阵
        cv::Mat mtcw; // 从世界坐标系到相机坐标系的平移矩阵
        cv::Mat mRwc; // 从相机坐标系到世界坐标系的旋转矩阵
        cv::Mat mOw;  // 从相机坐标系到世界坐标系的平移矩阵
    };
} // namespace ORB_SLAM2

#endif // FRAME_H