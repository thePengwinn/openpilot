#pragma once

#include "selfdrive/modeld/runners/runmodel.h"
#include "selfdrive/modeld/thneed/thneed.h"

class BodyThneedModel : public RunModel {
public:
  BodyThneedModel(const char *path, float *loutput, size_t loutput_size, int runtime, bool luse_extra = false, bool use_tf8 = false, cl_context context = NULL);
  void addRecurrent(float *state, int state_size);
  void addTrafficConvention(float *state, int state_size);
  void addDesire(float *state, int state_size);
  void addNavFeatures(float *state, int state_size);
  void addDrivingStyle(float *state, int state_size);
  void addImage(float *image_buf, int buf_size);
  void addExtra(float *image_buf, int buf_size);
  void execute();
  void* getInputBuf();
  void* getExtraBuf();
private:
  Thneed *thneed = NULL;
  bool recorded;

  float *input;
  float *output;
};
