cmake_minimum_required(VERSION 3.10)
project(rrtstar_sixDoF)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add the main executable
add_executable(rrtstar_planner
    src/rrtstar.cpp
    src/main.cpp
    src/inverse_kinematics.cpp
)

# Add the IK test executable
add_executable(ik_test
    # src/inverse_kinematics.cpp
    src/ivk.cpp
)

# Add the IK test executable
add_executable(rrtstar_6doftest
    # src/inverse_kinematics.cpp
    src/rrt_star_6dof.cpp
)

# Include directories
target_include_directories(rrtstar_planner
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

# Include directories for ik_test
target_include_directories(ik_test
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

# Include directories for rrtstar_6doftest
target_include_directories(rrtstar_6doftest
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

# Find Python and link against it
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Link Python libraries
target_link_libraries(rrtstar_planner 
    PRIVATE 
        Python3::Python
        Eigen3::Eigen
)

# Link Eigen for ik_test
target_link_libraries(ik_test
    PRIVATE
        Eigen3::Eigen
)

# Link Eigen for rrtstar_6doftest
target_link_libraries(rrtstar_6doftest
    PRIVATE
        Eigen3::Eigen
)

# Find and link TBB (Intel Threading Building Blocks)
find_package(TBB REQUIRED)

# Link TBB to the target
target_link_libraries(rrtstar_planner PRIVATE TBB::tbb)

# Compiler options
# if(MSVC)
#     target_compile_options(rrtstar_planner PRIVATE /W4)
# else()
#     target_compile_options(rrtstar_planner PRIVATE -Wall -Wextra -Wpedantic)
# endif()

# Compiler options
if(MSVC)
    target_compile_options(rrtstar_planner PRIVATE /W4)
    target_compile_options(ik_test PRIVATE /W4)
    target_compile_options(rrtstar_6doftest PRIVATE /W4)
else()
    target_compile_options(rrtstar_planner PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ik_test PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(rrtstar_6doftest PRIVATE -Wall -Wextra -Wpedantic)
endif()