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
  laser_scan = "/scan";
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

-- Image Panel Configuration
-- Two image panels are streamed to the browser. Each panel can subscribe to
-- either sensor_msgs/CompressedImage (msg_type = "compressed", JPEG bytes
-- forwarded as-is) or sensor_msgs/Image (msg_type = "raw", encoded to JPEG
-- via OpenCV before forwarding).
image_panels = {
  left = {
    topic = "/camera/rgb/image_raw/compressed";
    -- topic = "/camera/rgb/image_raw";
    msg_type = "compressed";       -- "compressed" or "raw"
    queue_size = 1;
  };
  -- Right panel shows the planner's 2x2 observation history mosaic
  -- (top-left = oldest, bottom-right = current) published by
  -- legged_deployment on each service call.
  right = {
    topic = "/legged_deployment/observation_mosaic/compressed";
    msg_type = "compressed";
    queue_size = 1;
  };
};

-- Image Streaming Configuration
image_streaming = {
  max_rate_hz = 5.0;               -- Per-panel forwarding cap
  jpeg_quality = 75;               -- JPEG quality (0-100); only used for raw Image topics
};

-- Foresight Planner Topic Configuration
-- The webviz UI exposes a text input that publishes the goal_command as a
-- std_msgs/String on ``command_topic``. graph_navigation orchestrates the
-- mission and republishes per-iteration ForesightPlannerMsg updates on
-- ``status_topic``; webviz subscribes to that topic and forwards the verdict
-- and reason fields to the browser for display.
foresight_planner = {
  command_topic       = "/legged_deployment/foresight_planner/goal_command";
  command_topic_qos   = 10;
  status_topic        = "/legged_deployment/foresight_status";
  status_topic_qos    = 10;
};

