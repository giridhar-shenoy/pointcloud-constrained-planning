// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
#include "pcp_benchmark/robot_model_files.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <srdfdom/model.h>
#include <urdf/model.h>

namespace pcp_benchmark
{

std::string readFile(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("cannot open " + path);
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

moveit::core::RobotModelPtr loadRobotModel(
  const std::string & urdf_path, const std::string & srdf_path)
{
  auto urdf_model = std::make_shared<urdf::Model>();
  if (!urdf_model->initString(readFile(urdf_path))) {
    throw std::runtime_error("cannot parse URDF " + urdf_path);
  }
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!srdf_model->initString(*urdf_model, readFile(srdf_path))) {
    throw std::runtime_error("cannot parse SRDF " + srdf_path);
  }
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

}  // namespace pcp_benchmark
