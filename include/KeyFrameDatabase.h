#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include<mutex> // 加互斥锁

namespace ORB_SLAM2
{

class KeyFrame;
class Frame;

class KeyFrameDatabase
{
public:

   KeyFrameDatabase(const ORBVocabulary &voc);

   void add(KeyFrame* pKF);
   void erase(KeyFrame* pKF);
   void clear();

   // 回环检测时，检测可能表示回环的候选帧
   std::vector<KeyFrame*> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Relocalization
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

protected:
  // 预先训练好的词典
  const ORBVocabulary* mpVoc;

  // 倒排索引，mvInvertedFile[i]包含了第i个wordID的所有关键帧
  std::vector<list<KeyFrame*> > mvInvertedFile;

  // 互斥锁
  std::mutex mMutex;
};
} //namespace ORB_SLAM2

#endif
