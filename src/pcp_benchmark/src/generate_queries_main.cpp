// Copyright (c) 2026 Giridhar J Shenoy. MIT License.
//
// generate_queries --urdf F --srdf F --scene shelf.yaml --out results/queries.json
//                  [--seed 0] [--count 100]
//
// Writes the benchmark query set. Pure C++ (no ROS graph): the output depends only on the
// robot model, the scene YAML and the seed, which is what the determinism test checks.
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include "pcp_benchmark/query_generator.hpp"
#include "pcp_benchmark/robot_model_files.hpp"

int main(int argc, char ** argv)
{
  std::map<std::string, std::string> args = {{"--seed", "0"}, {"--count", "100"}};
  for (int i = 1; i + 1 < argc; i += 2) {
    args[argv[i]] = argv[i + 1];
  }
  for (const char * required : {"--urdf", "--srdf", "--scene", "--out"}) {
    if (args.count(required) == 0) {
      std::cerr << "usage: generate_queries --urdf F --srdf F --scene F --out F [--seed N] "
                   "[--count N]\n";
      return 2;
    }
  }
  try {
    const auto model = pcp_benchmark::loadRobotModel(args["--urdf"], args["--srdf"]);
    const auto shelf = pcp_benchmark::loadShelfScene(args["--scene"]);
    pcp_benchmark::GeneratorConfig cfg;
    cfg.seed = std::stoull(args["--seed"]);
    cfg.count = std::stoul(args["--count"]);
    const auto set = pcp_benchmark::generateQueries(model, shelf, cfg);
    std::ofstream out(args["--out"]);
    out << pcp_benchmark::toJson(set).dump(1) << "\n";
    std::cout << "wrote " << set.queries.size() << " queries (seed " << cfg.seed << ") to "
              << args["--out"] << "; pose samples " << set.pose_samples << ", IK failures "
              << set.ik_failures << ", collision rejects " << set.collision_rejects << "\n";
  } catch (const std::exception & e) {
    std::cerr << "generate_queries: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
