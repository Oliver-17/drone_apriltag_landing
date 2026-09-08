cmake_minimum_required(VERSION 3.8)
project(drone_apriltag_landing)

# ROS 2 Humble 用 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 依賴 -------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- 介面（action）----------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PrecisionLand.action"
)

# --- 執行檔 -----------------------------------------------------------------
add_executable(precision_land_node
  src/precision_land_node.cpp
)

ament_target_dependencies(precision_land_node
  rclcpp
  rclcpp_action
  px4_msgs
  apriltag_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ⚠️ 這三行是「同一個套件裡既產 action 又編 C++ 節點」的必要寫法。
#    action 是在這個套件內產生的，還沒被 ament 註冊成外部套件，
#    所以 ament_target_dependencies 抓不到它 —— 一定要手動把 typesupport
#    連進來，否則會編到一半找不到 PrecisionLand 的符號。
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(precision_land_node "${cpp_typesupport_target}")

# --- 安裝 -------------------------------------------------------------------
install(TARGETS precision_land_node
  DESTINATION lib/${PROJECT_NAME}
)

# config 與 launch 要在 share/ 底下，ros2 launch 和參數檔才找得到
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# 測試腳本直接用 python3 跑，不經過 entry_points，
# 這樣改完不用重 build 就能重跑（跟 drone_nav2_apriltag 的做法一致）
install(PROGRAMS
  test/t1_offline_decode.py
  test/t2_pose_check.py
  test/t3_landing_flight.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 個人專案，關掉 copyright/lint 免得一堆警告蓋掉真正的錯誤
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
