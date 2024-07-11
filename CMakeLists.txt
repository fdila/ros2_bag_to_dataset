cmake_minimum_required(VERSION 3.14)
project(ros2_bag_to_dataset)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package (OpenCV REQUIRED)
find_package (Boost REQUIRED)

ament_auto_add_library(bag_to_dataset SHARED
        src/bag_to_dataset.cpp
        )

target_link_libraries(bag_to_dataset ${PCL_LIBRARIES}
        )

target_include_directories(bag_to_dataset
        SYSTEM PUBLIC
        ${OpenCV_INCLUDE_DIRS}
        )

rclcpp_components_register_node(bag_to_dataset
        PLUGIN "BagToDataset"
        EXECUTABLE bag_to_dataset_node
        )
######

ament_auto_package(INSTALL_TO_SHARE
        launch
        )

