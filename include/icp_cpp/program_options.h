#ifndef PROGRAM_OPTIONS_H
#define PROGRAM_OPTIONS_H

// std
#include <string>

// Third party
#include <cxxopts.hpp>

constexpr int kMaxNumDefaultIterations = 100;
struct ProgramOptions {
  std::string mode{"sequential"};
  std::string input_scan_a_topic{"scan"};
  std::string input_scan_b_topic{"scan_b"};
  std::string input_scan_a_type{"laserscan"};
  std::string input_scan_b_type{"laserscan"};
  std::string output_transform_matrix_topic{"transform_matrix"};
  int max_num_iterations = kMaxNumDefaultIterations;
  std::string frame_id{"laser"};
  bool pub_scan_a{false};
  bool pub_trans_scan_a{false};
  bool pub_scan_b{false};
  std::string output_scan_a_topic{"scan_a_out"};
  std::string output_trans_scan_a_topic{"scan_a_transformed_out"};
  std::string output_scan_b_topic{"scan_b_out"};
};

ProgramOptions ParseArgs(int argc, char** argv) {
  ProgramOptions program_options;
  cxxopts::Options options("Scan Matcher Node", "Match scans sequentially from one topic or from two separate topics.");
  options.add_options()
    ("m,mode", "scan match mode [sequential, a_to_b]",
    cxxopts::value<std::string>(program_options.mode)->default_value("sequential"))
    ("a,in_a", "input topic to receive scan A on",
    cxxopts::value<std::string>(program_options.input_scan_a_topic)->default_value("scan"))
    ("b,in_b", "input topic to receive scan B on",
    cxxopts::value<std::string>(program_options.input_scan_b_topic)->default_value("scan_b"))
    ("a_type", "message type of scan A, ['laserscan' 'pointcloud' or 'pointcloud2']",
    cxxopts::value<std::string>(program_options.input_scan_a_type)->default_value("laserscan"))
    ("b_type", "message type of scan B, ['laserscan' 'pointcloud' or 'pointcloud2']",
    cxxopts::value<std::string>(program_options.input_scan_b_type)->default_value("laserscan"))
    ("transform_topic", "topic to publish transformation matrix on",
    cxxopts::value<std::string>(program_options.output_transform_matrix_topic)->default_value("transform_matrix"))
    ("i,max_num_iterations", "max scan matcher iterations before exiting",
    cxxopts::value<int>(program_options.max_num_iterations)->default_value(std::to_string(kMaxNumDefaultIterations)))
    ("f,frame_id", "tf frame id",
    cxxopts::value<std::string>(program_options.frame_id)->default_value("laser"))
    ("pub_scan_a", "enable publishing of scan A",
    cxxopts::value<bool>(program_options.pub_scan_a))
    ("pub_trans_scan_a", "enable publishing of transformed scan A",
    cxxopts::value<bool>(program_options.pub_trans_scan_a))
    ("pub_scan_b", "enable publishing of scan B",
    cxxopts::value<bool>(program_options.pub_scan_b))
    ("out_a_topic", "Output topic to publish scan A",
    cxxopts::value<std::string>(program_options.output_scan_a_topic)->default_value("scan_a_out"))
    ("out_trans_a_topic", "Output topic to publish transformed scan A",
    cxxopts::value<std::string>(program_options.output_trans_scan_a_topic)->default_value("scan_a_transformed_out"))
    ("out_b_topic", "Output topic to publish scan B",
    cxxopts::value<std::string>(program_options.output_scan_b_topic)->default_value("scan_b_out"))
    ("h,help", "Print usage info");

  auto result = options.parse(argc, argv);
  if (result.count("help")) {
    std::cout <<  options.help()  << std::endl;
    exit(0);
  }

  return program_options;
}

#endif //PROGRAM_OPTIONS_H
