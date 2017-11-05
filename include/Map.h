#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>

namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned KeyFramesInMap();
    
    long unsigned int GetMaxKFid();

    void clear();

    std::vector<KeyFrame*> mvpKeyFrameOrigins;

    // 在Map更新的时候加互斥锁
    std::mutex mMutexMapUpdate;

    // 加互斥锁，防止两个线程同时产生一个点(ID conflict)
    std::mutex mMutexPointCreation;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;
    std::vector<MapPoint*> mvpReferenceMapPoints;
    
    long unsigned int mnMaxKFid;
    
    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;
    
    std::mutex mMutexMap;
};

} // namespace ORB_SLAM2

# endif // MAP_H