-- WebViz Configuration File
-- This file contains all configurable parameters for WebViz
-- Compatible with ROS1, ROS2, C++, and integrates with AMRL lab config-reader

-- WebSocket Server Configuration
websocket = {
  port = 10272;                    -- WebSocket server port
  update_rate_hz = 10.0;           -- Visualization update rate (fps)
  message_timeout_sec = 2.0;       -- Maximum age before messages are dropped
  exit_check_interval_ms = 100;    -- Timer interval for exit signal checking
};

-- ROS Topics Configuration
ros_topics = {
  -- Input topics (subscribers)
  laser_scan = "/Cobot/Laser";       -- Laser scan data
  visualization = "/visualization"; 
  localization = "/localization";
  
  -- Output topics (publishers)
  initial_pose_std = "/initialpose";           -- Standard ROS nav stack
  nav_goal_std = "/move_base_simple/goal";     -- Standard ROS nav stack
  initial_pose_amrl = "/set_pose";             -- AMRL format
  nav_goal_amrl = "/set_nav_target";           -- AMRL format
  reset_nav_goals = "/reset_nav_goals";        -- Reset command
};

-- ROS Node Configuration  
ros_node = {
  name = "websocket";              -- ROS node name
  queue_sizes = {
    laser_scan = 5;                -- Laser scan subscriber queue
    visualization = 10;            -- Visualization subscriber queue  
    localization = 10;             -- Localization subscriber queue
    publishers = 10;               -- All publisher queues
  };
};

-- Frame Configuration
frames = {
  robot_frame = "base_link";       -- Robot-relative frame
  world_frame = "map";             -- World/global frame
};

-- Data Processing Configuration
data_processing = {
  laser_range_scale = 1000.0;      -- Convert meters to millimeters for transmission
  protocol_nonce = 42;             -- Binary protocol identifier
  text_buffer_size = 32;           -- Max characters for text annotations
  map_name_buffer_size = 32;       -- Max characters for map names
};

-- Logging Configuration
logging = {
  verbosity = 0;                   -- Default verbosity level (0=minimal, 1=info, 2=debug)
};
  
-- Performance Tuning
performance = {
  enable_message_aging = true;     -- Drop old messages based on timestamp
  enable_rate_limiting = true;     -- Limit update rate to configured fps
  thread_sleep_usec = 100000;      -- Microseconds to sleep before thread cleanup
}; 