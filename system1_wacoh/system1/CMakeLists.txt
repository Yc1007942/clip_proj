cmake_minimum_required(VERSION 3.10)
project(system1 VERSION 0.0.0)

find_package(ament_cmake          REQUIRED)
find_package(ament_cmake_python   REQUIRED)
find_package(rclpy               REQUIRED)
find_package(geometry_msgs       REQUIRED)
find_package(segment_task_pkg    REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES resource/system1
    DESTINATION share/ament_index/resource_index/packages
)

if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(${PROJECT_NAME}_tests
        tests
        PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE}
    )
endif()

ament_package()