cmake_minimum_required(VERSION 3.10.2)

project(uav_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)

add_subdirectory(thirdparty/yaml-cpp)

add_library(waypoint_navigator
    src/waypoint_navigator.cpp
)

target_include_directories(waypoint_navigator
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(waypoint_navigator
    MAVSDK::mavsdk
    yaml-cpp::yaml-cpp
    Threads::Threads
)

add_library(shiviz_px4_navigation::WaypointNavigator ALIAS waypoint_navigator)

# === Only build executable if this is the top-level project ===
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)

    add_executable(uav_control
        src/main.cpp
    )

    target_include_directories(uav_control
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(uav_control
        PRIVATE waypoint_navigator
    )

endif()
install(FILES config/waypoints.yaml DESTINATION ${CMAKE_BINARY_DIR}/config)

# if(NOT MSVC)
#     target_compile_options(uav_control PRIVATE -Wall -Wextra)
# else()
#     target_compile_options(uav_control PRIVATE /W2)
# endif()

# if(MINGW OR MSVC)
#     target_compile_definitions(uav_control PRIVATE "-D_USE_MATH_DEFINES")
# endif()
