// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "line_follower_perception/line_follower_perception.h"

#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

#include "dnn_node/util/image_proc.h"

void prepare_nv12_tensor_without_padding(const char *image_data,
                                         int image_height,
                                         int image_width,
                                         hbDNNTensor *tensor) {
  auto &properties = tensor->properties;
  properties.tensorType = HB_DNN_IMG_TYPE_NV12;
  properties.tensorLayout = HB_DNN_LAYOUT_NCHW;
  auto &valid_shape = properties.validShape;
  valid_shape.numDimensions = 4;
  valid_shape.dimensionSize[0] = 1;
  valid_shape.dimensionSize[1] = 3;
  valid_shape.dimensionSize[2] = image_height;
  valid_shape.dimensionSize[3] = image_width;

  auto &aligned_shape = properties.alignedShape;
  aligned_shape = valid_shape;

  int32_t image_length = image_height * image_width * 3 / 2;

  hbSysAllocCachedMem(&tensor->sysMem[0], image_length);
  memcpy(tensor->sysMem[0].virAddr, image_data, image_length);

  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_CLEAN);
}

void prepare_nv12_tensor_without_padding(int image_height,
                                         int image_width,
                                         hbDNNTensor *tensor) {
  auto &properties = tensor->properties;
  properties.tensorType = HB_DNN_IMG_TYPE_NV12;
  properties.tensorLayout = HB_DNN_LAYOUT_NCHW;

  auto &valid_shape = properties.validShape;
  valid_shape.numDimensions = 4;
  valid_shape.dimensionSize[0] = 1;
  valid_shape.dimensionSize[1] = 3;
  valid_shape.dimensionSize[2] = image_height;
  valid_shape.dimensionSize[3] = image_width;

  auto &aligned_shape = properties.alignedShape;
  int32_t w_stride = ALIGN_16(image_width);
  aligned_shape.numDimensions = 4;
  aligned_shape.dimensionSize[0] = 1;
  aligned_shape.dimensionSize[1] = 3;
  aligned_shape.dimensionSize[2] = image_height;
  aligned_shape.dimensionSize[3] = w_stride;

  int32_t image_length = image_height * w_stride * 3 / 2;
  hbSysAllocCachedMem(&tensor->sysMem[0], image_length);
}

LineFollowerPerceptionNode::LineFollowerPerceptionNode(const std::string& node_name,
                      const NodeOptions& options)
  : DnnNode(node_name, options) {
  this->declare_parameter("model_path", "./resnet18_224x224_nv12.bin");
  this->declare_parameter("model_name", "resnet18_224x224_nv12.bin");
  if (GetParams() == false) {
    RCLCPP_ERROR(this->get_logger(), "LineFollowerPerceptionNode GetParams() failed\n\n");
    return;
  }
  
  if (Init() != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("LineFollowerPerceptionNode"), "Init failed!");
  }

  publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 5);
  subscriber_ =
    this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
      "hbmem_img",
      10,
      std::bind(&LineFollowerPerceptionNode::subscription_callback,
      this,
      std::placeholders::_1)); 
}

LineFollowerPerceptionNode::~LineFollowerPerceptionNode() {

}

bool LineFollowerPerceptionNode::GetParams() {
  auto parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(this);
  auto parameters = parameters_client->get_parameters(
    {"model_path", "model_name"});
  return AssignParams(parameters);
}

bool LineFollowerPerceptionNode::AssignParams(
  const std::vector<rclcpp::Parameter> & parameters) {
  for (auto & parameter : parameters) {
    if (parameter.get_name() == "model_path") {
      model_path_ = parameter.value_to_string();
    } else if (parameter.get_name() == "model_name") {
      model_name_ = parameter.value_to_string();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid parameter name: %s",
        parameter.get_name().c_str());
    }
  }
  return true;
}

int LineFollowerPerceptionNode::SetNodePara() {
  if (!dnn_node_para_ptr_) {
    return -1;
  }
  RCLCPP_ERROR(rclcpp::get_logger("LineFollowerPerceptionNode"), "path:%s\n", model_path_.c_str());
  RCLCPP_ERROR(rclcpp::get_logger("LineFollowerPerceptionNode"), "name:%s\n", model_name_.c_str());
  dnn_node_para_ptr_->model_file = model_path_;
  dnn_node_para_ptr_->model_name = model_name_;
  dnn_node_para_ptr_->model_task_type = model_task_type_;
  dnn_node_para_ptr_->task_num = 4;
  return 0;
}

int LineFollowerPerceptionNode::SetOutputParser() {
  auto model_manage = GetModel();
  if (!model_manage) {
    RCLCPP_ERROR(rclcpp::get_logger("LineFollowerPerceptionNode"), "Invalid model");
    return -1;
  }
  int output_index = model_manage->GetOutputCount() - 1;

  std::shared_ptr<OutputParser> line_coordinate_parser =
      std::make_shared<LineCoordinateParser>();
  model_manage->SetOutputParser(output_index, line_coordinate_parser);

  return 0;
}

int LineFollowerPerceptionNode::PostProcess(
  const std::shared_ptr<DnnNodeOutput> &outputs) {
  auto result = dynamic_cast<LineCoordinateResult *>(outputs->outputs[0].get());  
  int x = result->x;
  int y = result->y;
  RCLCPP_INFO(rclcpp::get_logger("LineFollowerPerceptionNode"),
               "post coor x: %d    y:%d", x, y);
  float angular_z = -1.0 * (x - 480) / 300.0;
  auto message = geometry_msgs::msg::Twist();
  message.linear.x = 0.1;
  message.linear.y = 0.0;
  message.linear.z = 0.0;
  message.angular.x = 0.0;
  message.angular.y = 0.0;
  message.angular.z = angular_z;
  publisher_->publish(message);
  return 0;
}

void LineFollowerPerceptionNode::subscription_callback(
    const hbm_img_msgs::msg::HbmMsg1080P::SharedPtr msg) {
  int ret = 0;
  if (!msg || !rclcpp::ok()) {
    return;
  }
  std::stringstream ss;
  ss << "Recved img encoding: "
     << std::string(reinterpret_cast<const char*>(msg->encoding.data()))
     << ", h: " << msg->height << ", w: " << msg->width
     << ", step: " << msg->step << ", index: " << msg->index
     << ", stamp: " << msg->time_stamp.sec << "_"
     << msg->time_stamp.nanosec << ", data size: " << msg->data_size;
  RCLCPP_DEBUG(rclcpp::get_logger("LineFollowerPerceptionNode"), "%s", ss.str().c_str());

  auto model_manage = GetModel();
  if (!model_manage) {
    RCLCPP_ERROR(rclcpp::get_logger("LineFollowerPerceptionNode"), "Invalid model");
    return;
  }

  hbDNNRoi roi;
  roi.left = 0;
  roi.top = 160;
  roi.right = 960 - 1;
  roi.bottom = 384 - 1;
  hbDNNTensor input_tensor;
  prepare_nv12_tensor_without_padding(reinterpret_cast<const char*>(msg->data.data()),
                                      msg->height,
                                      msg->width,
                                      &input_tensor);
  // Prepare output tensor
  hbDNNTensor output_tensor;
  prepare_nv12_tensor_without_padding(224, 224, &output_tensor);

  // resize
  hbDNNResizeCtrlParam ctrl = {
      HB_BPU_CORE_0, 0, HB_DNN_RESIZE_TYPE_BILINEAR, 0, 0, 0, 0};
  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNResize(&task_handle, &output_tensor, &input_tensor, &roi, &ctrl);
  ret = hbDNNWaitTaskDone(task_handle, 0);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("LineFollowerPerceptionNode"), "hbDNNWaitTaskDone failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
  }
  hbDNNReleaseTask(task_handle);
  if (0 != ret) {
    RCLCPP_ERROR(rclcpp::get_logger("LineFollowerPerceptionNode"), "release task failed!");
    hbSysFreeMem(&(input_tensor.sysMem[0]));
    hbSysFreeMem(&(output_tensor.sysMem[0]));
  }

  std::shared_ptr<hobot::easy_dnn::NV12PyramidInput> pyramid = nullptr;
  pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
      reinterpret_cast<const char*>(output_tensor.sysMem[0].virAddr),
      224,
      224,
      224,
      224);
  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("LineFollowerPerceptionNode"), "Get Nv12 pym fail!");
    return;
  }
  std::vector<std::shared_ptr<DNNInput>> inputs;
  auto rois = std::make_shared<std::vector<hbDNNRoi>>();
  roi.left = 0;
  roi.top = 0;
  roi.right = 224;
  roi.bottom = 224;
  rois->push_back(roi);

  for (size_t i = 0; i < rois->size(); i++) {
    for (int32_t j = 0; j < model_manage->GetInputCount(); j++) {
      inputs.push_back(pyramid);
    }
  }

  auto dnn_output = std::shared_ptr<DnnNodeOutput>();

  ret = Predict(inputs, dnn_output, rois);

  ret = hbSysFreeMem(&(input_tensor.sysMem[0]));
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("LineFollowerPerceptionNode"),
                 "Free input_tensor mem failed!");
    hbSysFreeMem(&(output_tensor.sysMem[0]));
  }
  ret = hbSysFreeMem(&(output_tensor.sysMem[0]));
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("LineFollowerPerceptionNode"),
                 "Free output_tensor mem failed!");
  }


}

int LineFollowerPerceptionNode::Predict(
  std::vector<std::shared_ptr<DNNInput>> &dnn_inputs,
  const std::shared_ptr<DnnNodeOutput> &output,
  const std::shared_ptr<std::vector<hbDNNRoi>> rois) {
  RCLCPP_INFO(rclcpp::get_logger("LineFollowerPerceptionNode"), "input size:%d roi size:%d", dnn_inputs.size(), rois->size());
  return Run(dnn_inputs,
             output,
             rois,
             true);
}


int32_t LineCoordinateParser::Parse(
    std::shared_ptr<LineCoordinateResult> &output,
    std::vector<std::shared_ptr<InputDescription>> &input_descriptions,
    std::shared_ptr<OutputDescription> &output_description,
    std::shared_ptr<DNNTensor> &output_tensor) {
  if (!output_tensor) {
    RCLCPP_ERROR(rclcpp::get_logger("LineFollowerPerceptionNode"), "invalid out tensor");
    rclcpp::shutdown();
  }
  std::shared_ptr<LineCoordinateResult> result;
  if (!output) {
    result = std::make_shared<LineCoordinateResult>();
    output = result;
  } else {
    result = std::dynamic_pointer_cast<LineCoordinateResult>(output);
  }
  DNNTensor &tensor = *output_tensor;
  const int32_t *shape = tensor.properties.validShape.dimensionSize;
  RCLCPP_DEBUG(rclcpp::get_logger("LineFollowerPerceptionNode"),
               "PostProcess shape[1]: %d shape[2]: %d shape[3]: %d",
               shape[1],
               shape[2],
               shape[3]);
  hbSysFlushMem(&(tensor.sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  float x = reinterpret_cast<float *>(tensor.sysMem[0].virAddr)[0];
  float y = reinterpret_cast<float *>(tensor.sysMem[0].virAddr)[1];
  result->x = (x * 112 + 112) * 960.0 / 224.0;
  result->y = 224 - (y * 112 + 112) + 272 - 112;
  RCLCPP_INFO(rclcpp::get_logger("LineFollowerPerceptionNode"),
               "coor rawx: %f,  rawy:%f, x: %f    y:%f", x, y, result->x, result->y);
  return 0;
}

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<LineFollowerPerceptionNode>("GetLineCoordinate"));

  rclcpp::shutdown();

  RCLCPP_WARN(rclcpp::get_logger("LineFollowerPerceptionNode"), "Pkg exit.");
  return 0;
}