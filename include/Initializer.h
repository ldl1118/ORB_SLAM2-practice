#ifndef INITIALIZER_H
#define INITIALIZER_H

#include<opencv2/opencv.hpp>
#include "Frame.h"

namespace ORB_SLAM2
{

// 单目SLAM会用到这个初始化，双目和RGB-D不会使用这个类
class Initializer
{
    typedef pair<int,int> Match;
    // pair是一种模板类型，每个pair可以储存两个值，也可以将自己写的struct对象放进去。
    // 应用：如果需要一个函数有两个返回值，若返回的是相同类型，则可以用数组;如果是不同类型，可以自己写个struct。但是为了方便，就可以使用
    // c++自带的pair。另外，在一个对象有多个属性的时候，也可以用pair操作。

public:
    // 用reference frame初始化。这个reference frame就是SLAM正式开始的第一帧
    Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

    // 并行计算平面模型和立体模型，并根据得分选择合适的模型，尝试从运动中恢复轨迹和环境结构
    // 用current frame（即SLAM逻辑上的第二帧）来初始化整个SLAM，得到最开始两帧之间的旋转矩阵R和平移矩阵t，以及点云
    bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);

private:
    // 假设场景为平面，通过前两帧(current frame 2 到 reference frame 1)求Homography矩阵，并得到该模型的评分
    void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
    // 假设场景为立体的，通过前两帧(current frame 2 到 reference frame 1)求Fundamental矩阵，并得到该模型的评分
    void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

    // 被FindHomography函数调用，来计算Homography矩阵
    cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    // 被FindFundamental函数调用，来计算Fundamental矩阵
    cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

    // 被FindHomography函数调用，来计算假设使用Homography模型的得分
    float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);
    // 被FindFundamental函数调用，来计算假设使用Fundamental模型的得分
    float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

    // 分解Fundamental矩阵F，并从分解后的多个解中找出合适的R，t
    bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);
    // 分解Homography矩阵H，并从分解后的多个解中找出合适的R，t
    bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    // 三角化，利用反投影矩阵将特征点恢复为3D点
    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

    // 归一化三维空间点和帧间位移t
    void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

    // 被ReconstructF函数调用，进行cheirality check，从而找出F分解后最适合的解
    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

    // F矩阵通过结合内参可以得到Essential矩阵，该函数用于分解E矩阵，得到4组解
    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

    // 储存Reference Frame中的特征点   Frame 1
    vector<cv::KeyPoint> mvKeys1;
    // 储存Current Frame中的特征点     Frame 2
    vector<cv::KeyPoint> mvKeys2;

    // Current Matches from Reference-1 to Current-2
    vector<Match> mvMatches12; // Match的数据结构是pair，mvMatches12只记录Reference到Current的过程中匹配成功的特征点对
    vector<bool> mvbMatched1; // 记录Reference Frame中的每个特征点在Current Frame中是否有匹配的特征点 

    // 相机内参，用于标定
    cv::Mat mK;

    // 测量误差：标准差和方差
    float mSigma, mSigma2;

    // 计算F和H矩阵时，RANSAC的迭代次数
    int mMaxIterations;

    // 二维容器。外层容器大小为迭代次数，内层容器大小为每次迭代计算H或F需要的点
    vector<vector<size_t> > mvSets;
};
} //namespace ORB_SLAM2

#endif // INITIALIZER_H