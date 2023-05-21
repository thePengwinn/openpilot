#pragma once

#include "cereal/messaging/messaging.h"
#include "cereal/visionipc/visionipc_client.h"
#include "common/util.h"
#include "common/modeldata.h"
#include "selfdrive/modeld/models/commonmodel.h"
#include "selfdrive/modeld/runners/run.h"

constexpr int BODY_INPUT_SIZE = 416 * 640 * 3;
constexpr int BODY_NET_OUTPUT_SIZE = 4;

struct BodyModelResult {
  std::array<float, BODY_NET_OUTPUT_SIZE> preds;
  float gpu_execution_time;
};
static_assert(sizeof(BodyModelResult) == sizeof(float)*BODY_NET_OUTPUT_SIZE + sizeof(float));

constexpr int BODY_OUTPUT_SIZE = sizeof(BodyModelResult) / sizeof(float);

struct BodyModelState {
  RunModel *m;
  uint8_t net_input_buf[BODY_INPUT_SIZE];
  float output[BODY_OUTPUT_SIZE];
};

void bodymodel_init(BodyModelState* s, cl_device_id device_id, cl_context context);
BodyModelResult* bodymodel_eval_frame(BodyModelState* s, VisionBuf* buf);
void bodymodel_publish(PubMaster &pm, uint32_t frame_id, const BodyModelResult &model_res, float execution_time);
void bodymodel_free(BodyModelState* s);
