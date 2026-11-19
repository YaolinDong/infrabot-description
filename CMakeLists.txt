# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)
project(infrabot_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes config launch rviz docs
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md README.zh-CN.md LICENSE model_manifest.json
  DESTINATION share/${PROJECT_NAME})
ament_package()
