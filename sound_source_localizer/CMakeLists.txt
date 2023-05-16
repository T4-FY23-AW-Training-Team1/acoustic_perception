cmake_minimum_required(VERSION 3.8)
project(sound_source_localizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
#find_package(autoware_cmake REQUIRED)
#autoware_package()
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ALSA REQUIRED)
# find_package(libusb REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
#include_directories(${LIBUSB_INCLUDE_DIRS})
#link_directories(${LIBUSB_LIBRARY_DIRS})
#add_definitions(${LIBUSB_CFLAGS_OTHER})

ament_auto_add_executable(respeaker_driver src/respeaker_publisher.cpp)
#target_link_libraries(respeaker_driver ${LIBUSB_LIBRARIES})
target_link_libraries(respeaker_driver 
                        Eigen3::Eigen
                        ${ALSA_LIBRARIES})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
        INSTALL_TO_SHARE
        launch
)
