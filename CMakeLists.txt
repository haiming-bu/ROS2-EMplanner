cmake_minimum_required(VERSION 3.5)
project(emplanner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hm_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)

#头文件加载路径
include_directories(/usr/local/include/coin-or /usr/local/include/osqp)
link_directories(/usr/local/lib)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


#节点部分
add_executable(emplanner_node 
               src/EMplannerUtils.cpp
               src/osqpSolver.cpp              
               src/emplanner_node.cpp         
              )
target_include_directories(emplanner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(emplanner_node  ipopt coinhsl coinmumps lapack blas Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen   Python3::Python
  Python3::Module Python3::NumPy
)

target_compile_features(emplanner_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  emplanner_node
  "rclcpp"
  "hm_interfaces"
  "sensor_msgs"
  "std_msgs"
  "Eigen3"
  "osqp"
  "OsqpEigen"
  "Python3"
)

# target_link_libraries(emplanner_node  ipopt coinhsl coinmumps lapack blas Eigen3::Eigen osqp::osqp OsqpEigen::OsqpEigen   Python3::Python
#   Python3::Module Python3::NumPy
# )


install(TARGETS emplanner_node
  DESTINATION lib/${PROJECT_NAME})