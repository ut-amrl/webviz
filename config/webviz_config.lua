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
  -- Right panel shows memo's plan image: the keyframe views (oldest ->
  -- current), the current one annotated with the plan and route window.
  right = {
    topic = "/memo/plan_image/compressed";
    msg_type = "compressed";
    queue_size = 1;
  };
};

-- Image Streaming Configuration
image_streaming = {
  max_rate_hz = 5.0;               -- Per-panel forwarding cap
  jpeg_quality = 75;               -- JPEG quality (0-100); only used for raw Image topics
};

-- Memo planner: the UI's task box publishes std_msgs/String on task_topic;
-- memo_deployment's JSON status (std_msgs/String) on status_topic is shown in
-- the Robot tab.
memo = {
  task_topic   = "/memo/task";
  status_topic = "/memo/status";
};

-- GPS navigation (Map tab): the robot fix (amrl_msgs/GPSMsg, from
-- gps_phone_interface) and graph_navigation's GraphNavGPSSrv, which plans the
-- OSM route for the GPS goals clicked on the map.
gps = {
  fix_topic     = "/phone/gps_heading";
  route_service = "/graphNavGPSSrv";
  send_rate_hz  = 2.0;
};

-- Sensor health (Sensor Health tab, also published on diagnostics_topic).
-- Parallel lists: one entry per topic. A topic is MISSING until it appears in
-- the ROS graph, STALE when nothing arrived within stale_sec, WARN below half
-- of expected_hz (0 = no rate check), OK otherwise.
sensor_health = {
  publish_rate_hz   = 2.0;
  diagnostics_topic = "/diagnostics";
  names = {
    "Spot odometry", "Spot joint states", "Spot battery",
    "VLP16 points", "VLP16 scan",
    "Kinect RGB", "Kinect depth", "Kinect scan",
    "Phone GPS",
    "VectorNav IMU",
    "Memo status", "Memo plan",
    "Nav status", "Nav cmd_vel", "Robot cmd_vel",
  };
  topics = {
    "/odometry", "/joint_states", "/status/battery_states",
    "/velodyne_points", "/scan",
    "/camera/rgb/image_raw/compressed", "/camera/depth/image_raw", "/kinect_scan",
    "/phone/gps_heading",
    "/vectornav/imu_uncompensated",
    "/memo/status", "/memo/plan",
    "/navigation/mission_status", "/navigation/cmd_vel", "/cmd_vel",
  };
  expected_hz = {
    50.0, 50.0, 0.0,
    10.0, 10.0,
    15.0, 15.0, 15.0,
    1.0,
    200.0,
    0.0, 0.0,
    0.0, 0.0, 0.0,
  };
  stale_sec = {
    0.5, 0.5, 5.0,
    0.5, 0.5,
    0.5, 0.5, 0.5,
    3.0,
    0.5,
    10.0, 10.0,
    5.0, 2.0, 2.0,
  };
};
