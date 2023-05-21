#include "selfdrive/modeld/models/body.h"

#include <fcntl.h>
#include <unistd.h>

#include <cassert>
#include <cstring>

#include <eigen3/Eigen/Dense>

#include "common/clutil.h"
#include "common/params.h"
#include "common/timing.h"
#include "common/swaglog.h"

// #define DUMP_YUV

void bodymodel_init(BodyModelState* s, cl_device_id device_id, cl_context context) {
#ifdef USE_THNEED
  s->m = new BodyThneedModel("models/yolov5nn.thneed",
#else
  s->m = new ONNXModel("models/yolov5nn.onnx",
#endif
   &s->output[0], BODY_NET_OUTPUT_SIZE, USE_GPU_RUNTIME, true, false, context);
}

BodyModelResult* bodymodel_eval_frame(BodyModelState* s, VisionBuf* buf) {
  // memcpy(s->net_input_buf, buf->addr, BODY_INPUT_SIZE);
  for (int i=0; i<BODY_INPUT_SIZE; i++) {
    s->net_input_buf[i] = (float)((uint8_t *)buf->addr)[i];
  }

  double t1 = millis_since_boot();
  printf("Adding image...\n");
  s->m->addImage((float*)s->net_input_buf, BODY_INPUT_SIZE);
  printf("Added image\n");
  s->m->execute();
  printf("Executed!\n");
  double t2 = millis_since_boot();

  BodyModelResult *model_res = (BodyModelResult*)&s->output;
  model_res->gpu_execution_time = (t2 - t1) / 1000.;
  return model_res;
}

void bodymodel_publish(PubMaster &pm, uint32_t frame_id, const BodyModelResult &model_res, float execution_time) {
  // make msg
  MessageBuilder msg;
  auto framed = msg.initEvent().initNavModel();
  framed.setFrameId(frame_id);
  framed.setModelExecutionTime(execution_time);
  framed.setDspExecutionTime(model_res.gpu_execution_time);
  framed.setFeatures(to_kj_array_ptr(model_res.preds));

  pm.send("navModel", msg);
}

void bodymodel_free(BodyModelState* s) {
  delete s->m;
}
