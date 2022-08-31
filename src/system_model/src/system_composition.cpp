#include "rclcpp/rclcpp.hpp"
#include "system_model/sensor_node.hpp"
#include "yaml-cpp/node/node.h"
#include "yaml.h"
#include <chrono>
#include <cstring>
#include <memory>
#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/executors/static_single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <string>
#include <system_model/filter_node.hpp>
#include <system_model/subscription_actuator_node.hpp>
#include <thread>
#include <yaml-cpp/node/detail/iterator_fwd.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

enum Executors {
  single_threaded_executor,
  static_single_threaded_executor,
  multi_threaded_executor
};

static std::map<std::string, Executors> map_string_executor;

void initialize_string_executor_map() {
  map_string_executor["single_threaded_executor"] = single_threaded_executor;
  map_string_executor["static_single_threaded_executor"] =
      static_single_threaded_executor;
  map_string_executor["multi_threaded_executor"] = multi_threaded_executor;
}

enum Nodes {
  filter,
  sensor,
  subscription_actuator,
  subscription_fusion,
  timer_actuator,
  timer_fusion
};

static std::map<std::string, Nodes> map_string_nodes;

void initialize_string_nodes_map() {
  map_string_nodes["filter"] = filter;
  map_string_nodes["sensor"] = sensor;
  map_string_nodes["subscription_actuator"] = subscription_actuator;
  map_string_nodes["subscription_fusion"] = subscription_fusion;
  map_string_nodes["timer_actuator"] = timer_actuator;
  map_string_nodes["timer_fusion"] = timer_fusion;
}

std::vector<std::thread> thread_list;
std::vector<rclcpp::Executor::SharedPtr> executor_list;

int main(int argc, char *argv[]) {
  initialize_string_executor_map();
  initialize_string_nodes_map();

  rclcpp::init(argc, argv);

  std::string yaml_file_path;

  for (int i = 0; i < argc; i++) {
    if (!strcmp("-system", argv[i])) {
      yaml_file_path = argv[i + 1];
      break;
    }
  }

  if (!yaml_file_path.compare("")) {
    yaml_file_path = std::string(getenv("HOME")) +
                     "/ros_system_model/install/system_model/share/"
                     "system_model/config/empty.yaml";
    std::cout << "No YAML file specified. Default system launched instead"
              << std::endl;
  }

  YAML::Node system = YAML::LoadFile(yaml_file_path);
  for (YAML::const_iterator it = system.begin(); it != system.end(); ++it) {
    rclcpp::Executor::SharedPtr executor;
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    switch (map_string_executor[it->first.as<std::string>()]) {
    case single_threaded_executor: {
      executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      break;
    }
    case static_single_threaded_executor: {
      executor =
          std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
      break;
    }
    case multi_threaded_executor: {
      executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      break;
    }
    }
    executor_list.push_back(executor);

    for (YAML::const_iterator it2 = it->second.begin(); it2 != it->second.end();
         ++it2) {

      switch (map_string_nodes[it2->first.as<std::string>()]) {
      case filter: {
        rclcpp::NodeOptions options;
        std::vector<std::string> arguments = {
            it2->second["name"].as<std::string>(),
            it2->second["wcet"].as<std::string>(),
            it2->second["subscription_topic"].as<std::string>(),
            it2->second["publisher_topic"].as<std::string>(),
            it2->second["subscription_buffer_size"].as<std::string>(),
            it2->second["publisher_buffer_size"].as<std::string>()};
        options.arguments(arguments);
        options.arguments(std::vector<std::string>{"node_name"});
        std::shared_ptr<FilterNode> filter_node =
            std::make_shared<FilterNode>(options);
        executor->add_node(filter_node);
        break;
      }
      case sensor: {
        rclcpp::NodeOptions options;
        std::vector<std::string> arguments = {
            it2->second["name"].as<std::string>(),
            it2->second["period"].as<std::string>(),
            it2->second["wcet"].as<std::string>(),
            it2->second["publisher_topic"].as<std::string>(),
            it2->second["publisher_buffer_size"].as<std::string>()};
        options.arguments(arguments);
        std::shared_ptr<SensorNode> sensor_node =
            std::make_shared<SensorNode>(options);
        executor->add_node(sensor_node);
        break;
      }
      case subscription_actuator: {
        rclcpp::NodeOptions options;
        std::vector<std::string> arguments = {
            it2->second["name"].as<std::string>(),
            it2->second["wcet"].as<std::string>(),
            it2->second["subscription_topic"].as<std::string>(),
            it2->second["subscription_buffer_size"].as<std::string>()};
        options.arguments(arguments);
        std::shared_ptr<SubscriptionActuatorNode> subscription_actuator_node =
            std::make_shared<SubscriptionActuatorNode>(options);
        executor->add_node(subscription_actuator_node);
        break;
      }
      case subscription_fusion: {
        std::cout << "UNSUPPORTED NODE TYPE subscription_fusion USED"
                  << std::endl;
        break;
      }
      case timer_actuator: {
        std::cout << "UNSUPPORTED NODE TYPE timer_actuator USED" << std::endl;
        break;
      }
      case timer_fusion: {
        std::cout << "UNSUPPORTED NODE TYPE timer_fusion USED" << std::endl;
        break;
      }
      }
    }
  }

  for (unsigned int i = 0; i < executor_list.size(); ++i) {
    thread_list.push_back(
        std::thread([executor = executor_list[i]]() { executor->spin(); }));
  }
  for (unsigned int i = 0; i < thread_list.size(); ++i) {
    thread_list[i].join();
  }

  rclcpp::shutdown();
  return 0;
}