#include <yaml-cpp/yaml.h>
#include <CanDriver.hpp>
#include "motor.h"
#include "motor_ros/Command.h"
#include "motor_ros/State.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <csignal>
#include <iostream>
#include <map>
#include <memory>

struct MotorConfig {
  std::string name;
  motor::MotorType motor_type;
  uint8_t slave_id;
  uint8_t master_id;
};

struct LegConfig {
  std::string name;
  std::string can_port;
  int num_motors;
  std::vector<std::string> motor_names;
  std::vector<int> directions;
  int zero_position_indices;
  int zero_position_offset;
};

struct MotorCommand {
  double q;
  double dq;
  double tau;
  double kp;
  double kd;
};

class MotorControlSystem {
 private:
  float zero_position_;
  std::map<std::string, MotorConfig> motor_configs_;

  // I need this to keep the order of keys in leg_configs_ for consistent state message ordering
  std::vector<std::string> leg_names_;
  std::map<std::string, LegConfig> leg_configs_;
  
  std::map<std::string, std::unique_ptr<sockcanpp::CanDriver>> leg_handlers_;
  std::map<std::string, std::unique_ptr<motor::Motor_Control>> leg_controllers_;
  std::map<std::string, std::vector<std::shared_ptr<motor::Motor>>> leg_motors_;
  std::map<std::string, std::vector<MotorCommand>> leg_commands_;
  
  std::mutex cmd_mutex_;
  motor_ros::State motor_state_msg_;
  
  // Helper function to convert string motor type to enum
  motor::MotorType stringToMotorType(const std::string& type_str) {
    static const std::map<std::string, motor::MotorType> type_map = {
        {"DM4310", motor::DM4310},
        {"DM4310_48V", motor::DM4310_48V},
        {"DM4340", motor::DM4340},
        {"DM4340_48V", motor::DM4340_48V},
        {"DM6006", motor::DM6006},
        {"DM8006", motor::DM8006},
        {"DM8009", motor::DM8009},
        {"DM10010L", motor::DM10010L},
        {"DM10010", motor::DM10010},
        {"DMH3510", motor::DMH3510},
        {"DMH6215", motor::DMH6215},
        {"DMG6220", motor::DMG6220},
        {"RS00", motor::RS00},
        {"RS01", motor::RS01},
        {"RS02", motor::RS02},
        {"RS03", motor::RS03},
        {"RS04", motor::RS04},
        {"RS05", motor::RS05},
        {"RS06", motor::RS06},
    };
    
    auto it = type_map.find(type_str);
    if (it != type_map.end()) {
      return it->second;
    }
    LOGE("Unknown motor type: %s, defaulting to RS06", type_str.c_str());
    return motor::RS06;
  }
  
 public:
  MotorControlSystem() : zero_position_(1.5707963267948) {}
  
  bool loadConfiguration(const std::string& config_file) {
    try {
      YAML::Node config = YAML::LoadFile(config_file);
      
      if (!config["global"]) {
        LOGE("Missing 'global' section in config file");
        return false;
      }
      zero_position_ = config["global"]["zero_position"].as<float>();
      
      // Load motor configurations
      if (!config["motors"]) {
        LOGE("Missing 'motors' section in config file");
        return false;
      }
      
      for (const auto& motor_node : config["motors"]) {
        std::string motor_name = motor_node.first.as<std::string>();
        YAML::Node motor_data = motor_node.second;
        
        MotorConfig cfg;
        cfg.name = motor_name;
        cfg.motor_type = stringToMotorType(motor_data["motor_type"].as<std::string>());
        cfg.slave_id = motor_data["slave_id"].as<uint8_t>();
        cfg.master_id = motor_data["master_id"].as<uint8_t>();
        
        motor_configs_[motor_name] = cfg;
      }
      
      // Load leg configurations
      if (!config["legs"]) {
        LOGE("Missing 'legs' section in config file");
        return false;
      }
      
      for (const auto& leg_node : config["legs"]) {
        std::string leg_name = leg_node.first.as<std::string>();
        YAML::Node leg_data = leg_node.second;

        LegConfig cfg;
        cfg.name = leg_data["name"].as<std::string>();
        cfg.can_port = leg_data["can_port"].as<std::string>();
        cfg.num_motors = leg_data["num_motors"].as<int>();
        cfg.zero_position_indices = leg_data["zero_position_indices"].as<int>();
        cfg.zero_position_offset = leg_data["zero_position_offset"].as<int>();
        
        // Load motor names
        for (const auto& motor_ref : leg_data["motors"]) {
          cfg.motor_names.push_back(motor_ref.as<std::string>());
        }
        
        // Load directions
        for (const auto& dir : leg_data["directions"]) {
          cfg.directions.push_back(dir.as<int>());
        }
        
        leg_configs_[leg_name] = cfg;
        leg_names_.push_back(leg_name);
      }
      
      LOGI("Configuration loaded successfully");
      return true;
      
    } catch (const YAML::Exception& e) {
      LOGE("Failed to load configuration: %s", e.what());
      return false;
    }
  }
  
  bool initialize() {    
    // Initialize each leg
    // In order to get the order, comment this line.
    // for (const auto& [leg_name, leg_config] : leg_configs_) {
    for (const auto& leg_name : leg_names_) {
      const auto& leg_config = leg_configs_[leg_name];
      LOGI("Initializing leg: %s (CAN port %s)", leg_config.name.c_str(), leg_config.can_port.c_str());
      
      // Get handler for this CAN port
      auto handler = std::make_unique<sockcanpp::CanDriver>(leg_config.can_port, CAN_RAW);
      // Set error filter to ignore non-critical errors
      handler->setErrorFilter();
      leg_handlers_[leg_name] = std::move(handler);
      
      // Create motor controller
      auto controller = std::make_unique<motor::Motor_Control>(leg_handlers_[leg_name].get());
      
      // Create motors and add to controller
      std::vector<std::shared_ptr<motor::Motor>> motors;
      std::vector<std::string> joint_names_for_leg;
      
      for (const auto& motor_name : leg_config.motor_names) {
        if (motor_configs_.find(motor_name) == motor_configs_.end()) {
          LOGE("Motor %s not found in motor configurations", motor_name.c_str());
          return false;
        }
        
        const MotorConfig& mcfg = motor_configs_[motor_name];
        auto motor = std::make_shared<motor::Motor>(
            mcfg.motor_type,
            mcfg.slave_id,
            mcfg.master_id
        );
        motors.push_back(motor);
        joint_names_for_leg.push_back(motor_name);
      }
      
      // Add all motors to controller
      controller->addMotor(motors);
      
      // Store controllers and motors
      leg_controllers_[leg_name] = std::move(controller);
      leg_motors_[leg_name] = motors;
      
      // Initialize commands for this leg
      leg_commands_[leg_name] = std::vector<MotorCommand>(
          leg_config.num_motors,
          {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
      );
      
      // Add joint names to state message
      for (const auto& jname : joint_names_for_leg) {
        motor_state_msg_.joint_names.push_back(jname);
      }
    }
    
    // Enable all controllers
    // for (const auto& [leg_name, controller] : leg_controllers_) {
    for (const auto& leg_name : leg_names_) {
      auto& controller = leg_controllers_[leg_name];
      controller->enable();
      LOGI("Enabled motor control for leg: %s", leg_name.c_str());
    }
    
    // Print initial positions
    ROS_INFO("=================================================");
    LOGI("Motor Initial Positions:");
    int motor_count = 0;
    // For order, also comment this line.
    // for (const auto& [leg_name, motors] : leg_motors_) {
    for (const auto& leg_name : leg_names_) {
      const auto& motors = leg_motors_[leg_name];
      for (int i = 0; i < motors.size(); i++) {
        LOGI("  %s[%d] (ID: 0x%02X): %.4f",
             leg_name.c_str(), i,
             motors[i]->GetSlaveId(),
             motors[i]->Get_Position());
        motor_count++;
      }
    }
    ROS_INFO("Total motors: %d", motor_count);
    ROS_INFO("=================================================");
    
    return true;
  }
  
  void cmdCallback(const motor_ros::CommandConstPtr& msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    
    // Map incoming commands to leg commands
    int cmd_index = 0;
    // for (const auto& [leg_name, leg_config] : leg_configs_) {
    for (const auto& leg_name : leg_names_) {
      const auto& leg_config = leg_configs_[leg_name];
      for (int i = 0; i < leg_config.num_motors; i++) {
        if (cmd_index < msg->pos.size()) {
          leg_commands_[leg_name][i] = {
              msg->pos[cmd_index],
              msg->vel[cmd_index],
              msg->tau[cmd_index],
              msg->kp[cmd_index],
              msg->kd[cmd_index]
          };
          cmd_index++;
        }
      }
    }
  }
  
  void controlStep() {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    
    // Send commands to all legs
    // for (const auto& [leg_name, leg_config] : leg_configs_) {
    for (const auto & leg_name : leg_names_) {
      const auto& leg_config = leg_configs_[leg_name];
      auto& commands = leg_commands_[leg_name];
      auto& motors = leg_motors_[leg_name];
      auto& controller = leg_controllers_[leg_name];
      
      for (int i = 0; i < leg_config.num_motors; i++) {
        float offset = 0;
        // Check if this motor index uses zero position
        if (leg_config.zero_position_indices == i) {
          offset = zero_position_ * leg_config.zero_position_offset;
        }
        
        // Apply direction multiplier and offset
        float q_cmd = commands[i].q * leg_config.directions[i] + offset;
        float dq_cmd = commands[i].dq * leg_config.directions[i];
        float tau_cmd = commands[i].tau * leg_config.directions[i];
        
        controller->control_mit(
            *motors[i],
            commands[i].kp,
            commands[i].kd,
            q_cmd,
            dq_cmd,
            tau_cmd
        );
      }
    }
  }
  
  void updateState() {
    // Collect state from all motors
    motor_state_msg_.pos.clear();
    motor_state_msg_.vel.clear();
    motor_state_msg_.tau.clear();
    
    // For order, also comment this line.
    // for (const auto& [leg_name, leg_config] : leg_configs_) {
    for (const auto& leg_name : leg_names_) {
      const auto& leg_config = leg_configs_[leg_name];
      auto& motors = leg_motors_[leg_name];
      for (int i = 0; i < motors.size(); i++) {
        float pos = motors[i]->Get_Position();
        float vel = motors[i]->Get_Velocity();
        float tau = motors[i]->Get_tau();

        // Apply direction multiplier
        pos *= leg_config.directions[i];
        vel *= leg_config.directions[i];
        tau *= leg_config.directions[i];
        // Check if this motor index uses zero position and apply offset
        if (leg_config.zero_position_indices == i) {
          pos -= zero_position_;
        }
        motor_state_msg_.pos.push_back(pos);
        motor_state_msg_.vel.push_back(vel);
        motor_state_msg_.tau.push_back(tau);
      }
    }
  }
  
  void publishState(ros::Publisher& pub) {
    pub.publish(motor_state_msg_);
  }
  
  void shutdown() {
    LOGI("Shutting down motor controllers...");
    for (const auto& [leg_name, controller] : leg_controllers_) {
      controller->disable();
    }
    
    // Reset all resources
    leg_controllers_.clear();
    leg_motors_.clear();
    leg_handlers_.clear();
  }
  
  const motor_ros::State& getMotorState() const {
    return motor_state_msg_;
  }
};

std::unique_ptr<MotorControlSystem> g_motor_system;

extern "C" void sigint_handler(int signal) {
  LOGI("ROS shutdown signal received");
  if (g_motor_system) {
    g_motor_system->shutdown();
  }
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "motor_node");
  auto nh = std::make_unique<ros::NodeHandle>();
  
  // Get config file path from parameter or use default
  std::string config_file;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--config" && i + 1 < argc) {
      config_file = argv[i+1];
      break;
    }
  }
  LOGI("Using default config file: %s", config_file.c_str());
  
  g_motor_system = std::make_unique<MotorControlSystem>();
  
  if (!g_motor_system->loadConfiguration(config_file)) {
    LOGE("Failed to load motor configuration");
    return 1;
  }
  
  if (!g_motor_system->initialize()) {
    LOGE("Failed to initialize motor control system");
    return 1;
  }
  
  // Register signal handler
  if (std::signal(SIGINT, sigint_handler) == SIG_ERR) {
    LOGE("Failed to register signal handler");
    return 1;
  }
  
  // Setup ROS subscribers and publishers
  ros::Rate loop_rate(500);
  boost::function<void(const motor_ros::CommandConstPtr &)> callback =
      [&](const motor_ros::CommandConstPtr &msg) -> void {
        if (g_motor_system) {
          g_motor_system->cmdCallback(msg);
        }
      };

  ros::Subscriber cmd_sub = nh->subscribe<motor_ros::Command>(
      "/dm_cmd", 10, callback);
  
  ros::Publisher state_pub = nh->advertise<motor_ros::State>("/dm_states", 10);
  
  ros::AsyncSpinner async_spinner(4);
  async_spinner.start();
  
  LOGI("Motor node started, running at 500Hz");
  
  while (ros::ok()) {
    g_motor_system->controlStep();
    g_motor_system->updateState();
    g_motor_system->publishState(state_pub);
    
    loop_rate.sleep();
  }
  
  g_motor_system->shutdown();
  return 0;
}
