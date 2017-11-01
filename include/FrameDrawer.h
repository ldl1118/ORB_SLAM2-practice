#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>

namespace ORB_SLAM2
{
    class Tracking;
    class Viewer;
    class FrameDrawer
    {
    public:
        FrameDrawer(Map* pMap);

        // Update info from the last processed frame.
        void Update(Tracking *pTracker);

        // 绘制上个处理过的帧
        cv::Mat DrawFrame();

    protected:
        void DrawTextInfo(cv::Mat &im, int nState, cvc::Mat &imText);

        // 即将要绘制的帧的信息
        cv::Mat mIm;
        int N;
        vector<cv::KeyPoint> mvCurrentKeys;
        vector<bool> mbvMap, mvbVO;
        bool mbOnlyTracking;
        int mnTracked, mnTrackedVO;
        vector<cv::KeyPoint> mnIniKeys;
        vector<int> mvIniMatches;
        int mState;

        Map* mpMap;

        std::mutex mMutex;
    };
} // namespace ORB_SLAM2

#endif // FRAMEDRAWER_H