#include "selfdrive/modeld/runners/bodythneedmodel.h"

#include <cassert>

BodyThneedModel::BodyThneedModel(const char *path, float *loutput, size_t loutput_size, int runtime, bool luse_extra, bool luse_tf8, cl_context context) {
  thneed = new Thneed(true, context);
  thneed->load(path);
  thneed->clexec();

  recorded = false;
  output = loutput;
}

void BodyThneedModel::addRecurrent(float *state, int state_size) {}
void BodyThneedModel::addTrafficConvention(float *state, int state_size) {}
void BodyThneedModel::addDesire(float *state, int state_size) {}
void BodyThneedModel::addDrivingStyle(float *state, int state_size) {}
void BodyThneedModel::addNavFeatures(float *state, int state_size) {}
void BodyThneedModel::addExtra(float *extra_input_buf, int buf_size) {}

void BodyThneedModel::addImage(float *image_input_buf, int buf_size) {
  input = image_input_buf;
}

void* BodyThneedModel::getInputBuf() {
  return &(thneed->input_clmem[0]);
}

void* BodyThneedModel::getExtraBuf() {
  return nullptr;
}

void BodyThneedModel::execute() {
  if (!recorded) {
    thneed->record = true;
    float *inputs[1] = {input};
    thneed->copy_inputs(inputs);
    thneed->clexec();
    thneed->copy_output(output);
    thneed->stop();
    recorded = true;
  } else {
    float *inputs[1] = {input};
    thneed->execute(inputs, output);
  }
}
