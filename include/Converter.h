#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{
    // Converter.h提供了一些常见的转换
    // ORB_SLAM中都是以cv::Mat为基本的存储结构，而在参与运算时需要用到g2o和Eigen，因此需要一个转换
    // 该文件可以整个单独从ORB_SLAM中抽出来，不影响其他功能

    class Converter
    {
    public:
        // 描述子矩阵到一串单行的描述子向量
        static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

        // cv::Mat 转换为 g2o::SEQuat
        static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
        
        // g2o::Sim3 转换为 g2o::SE3Quat
        static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

        // 转换为cv::Mat
        static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
        static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
        static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
        static cv::Mat toCvMat(const Eigen::Matrix3d &m);
        static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
        static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

        // 转换为Eigen
        static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
        static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
        static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
        static std::vector<float> toQuaternion(const cv::Mat &M); // 转换为四元数
    };
} // namespace ORB_SLAM2

#endif // CONVERTER_H