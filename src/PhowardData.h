#ifndef __PHOWARDDATA_H__
#define __PHOWARDDATA_H__

#include <vector>
#include "vl/VLd.h"

class Model;

class PhowardData
{
 public:
  PhowardData(){
    mFPS = 60.0;
    mHowFast = 1.0 / 60.0;
    mSelectedModel = NULL;
  };
  ~PhowardData(){};

  std::vector<Model*> mModels;
  Model *mSelectedModel;

  double mFPS;
  double mHowFast;
  bool mTermination;
};
#endif

