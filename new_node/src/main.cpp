#include <iostream>
#include <csignal>
#include <memory>

#include "behavior_node/core/behavior_node_main.hpp"

// 全局变量用于信号处理
std::shared_ptr<BehaviorNodeMain> g_behavior_node = nullptr;

/**
 * 信号处理函数
 */
void signalHandler(int signal) {
  std::cout << "\nReceived signal " << signal << ", shutting down gracefully..." << std::endl;

  if (g_behavior_node) {
    g_behavior_node->shutdown();
  }

  exit(signal);
}

/**
 * 设置信号处理器
 */
void setupSignalHandlers() {
  std::signal(SIGINT, signalHandler);   // Ctrl+C
  std::signal(SIGTERM, signalHandler);  // Termination request
  std::signal(SIGQUIT, signalHandler);  // Quit signal
}

/**
 * 打印欢迎信息
 */
void printWelcome() {
  std::cout << "\n";
  std::cout << "=================================================\n";
  std::cout << "           Behavior Node v1.0.0                 \n";
  std::cout << "        UAV Behavior Control System             \n";
  std::cout << "=================================================\n";
  std::cout << "\n";
}

/**
 * 打印帮助信息
 */
void printUsage(const char* program_name) {
  std::cout << "Usage: " << program_name << " [OPTIONS]\n";
  std::cout << "\n";
  std::cout << "Options:\n";
  std::cout << "  -h, --help              Show this help message\n";
  std::cout << "  -v, --version          Show version information\n";
  std::cout << "  -c, --config FILE      Specify configuration file\n";
  std::cout << "  -t, --tree-dir DIR     Specify behavior tree directory\n";
  std::cout << "  -n, --node-name NAME   Specify ROS node name\n";
  std::cout << "  --debug                Enable debug mode\n";
  std::cout << "  --no-ros               Run without ROS (testing mode)\n";
  std::cout << "\n";
  std::cout << "Examples:\n";
  std::cout << "  " << program_name << "                           # Run with default settings\n";
  std::cout << "  " << program_name << " -c config.yaml           # Run with custom config\n";
  std::cout << "  " << program_name << " -t ~/trees --debug       # Run with custom tree dir and debug\n";
  std::cout << "\n";
}

/**
 * 打印版本信息
 */
void printVersion() {
  std::cout << "Behavior Node v1.0.0\n";
  std::cout << "Built with:\n";
  std::cout << "  - BehaviorTree.CPP v4.x\n";
  std::cout << "  - ROS2 Humble/Iron\n";
  std::cout << "  - C++20\n";
  std::cout << "\n";
}

/**
 * 解析命令行参数
 */
struct CommandLineArgs {
  std::string config_file;
  std::string tree_directory;
  std::string node_name = "behavior_node";
  bool show_help = false;
  bool show_version = false;
  bool debug_mode = false;
  bool no_ros = false;
};

CommandLineArgs parseArgs(int argc, char** argv) {
  CommandLineArgs args;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];

    if (arg == "-h" || arg == "--help") {
      args.show_help = true;
    }
    else if (arg == "-v" || arg == "--version") {
      args.show_version = true;
    }
    else if (arg == "-c" || arg == "--config") {
      if (i + 1 < argc) {
        args.config_file = argv[++i];
      } else {
        std::cerr << "Error: --config requires a filename\n";
        args.show_help = true;
      }
    }
    else if (arg == "-t" || arg == "--tree-dir") {
      if (i + 1 < argc) {
        args.tree_directory = argv[++i];
      } else {
        std::cerr << "Error: --tree-dir requires a directory path\n";
        args.show_help = true;
      }
    }
    else if (arg == "-n" || arg == "--node-name") {
      if (i + 1 < argc) {
        args.node_name = argv[++i];
      } else {
        std::cerr << "Error: --node-name requires a name\n";
        args.show_help = true;
      }
    }
    else if (arg == "--debug") {
      args.debug_mode = true;
    }
    else if (arg == "--no-ros") {
      args.no_ros = true;
    }
    else {
      std::cerr << "Unknown argument: " << arg << "\n";
      args.show_help = true;
    }
  }

  return args;
}

/**
 * 主函数
 */
int main(int argc, char** argv) {
  // 解析命令行参数
  auto args = parseArgs(argc, argv);

  if (args.show_help) {
    printUsage(argv[0]);
    return 0;
  }

  if (args.show_version) {
    printVersion();
    return 0;
  }

  // 打印欢迎信息
  printWelcome();

  // 设置信号处理器
  setupSignalHandlers();

  try {
    // 创建行为节点主实例
    g_behavior_node = std::make_shared<BehaviorNodeMain>(args.node_name);

    // 初始化节点
    if (!g_behavior_node->initialize(argc, argv)) {
      std::cerr << "Failed to initialize behavior node" << std::endl;
      return 1;
    }

    std::cout << "Behavior node '" << args.node_name << "' initialized successfully" << std::endl;
    std::cout << "Press Ctrl+C to shutdown gracefully" << std::endl;

    // 运行节点
    int exit_code = g_behavior_node->run();

    std::cout << "\nBehavior node shutdown complete" << std::endl;
    return exit_code;

  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "Unknown fatal error occurred" << std::endl;
    return 1;
  }
}