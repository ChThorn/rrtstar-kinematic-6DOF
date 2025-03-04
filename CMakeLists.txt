cmake_minimum_required(VERSION 3.10)
project(rrtstar-kinematic-6DOF)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Find Google Test
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include OpenCV directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Add the main executable
add_executable(ivk_test
    src/inverse_kinematics.cpp
    src/forward_kinematics.cpp
    src/main.cpp
)

add_executable(fk_standalone
    src/inverse_kinematics.cpp
    src/fk_test_standalone.cpp
    src/forward_kinematics.cpp
)

add_executable(kinematics_test
    src/kinematics_test.cpp
    src/forward_kinematics.cpp
    src/inverse_kinematics.cpp
)

add_executable(ik_solution_eval_test
    src/ik_solution_evaluator_test.cpp
    src/inverse_kinematics.cpp
    src/ik_solution_evaluator.cpp
    src/obstacle.cpp
)

add_executable(rrt_test
    src/rrtstar.cpp
    src/test_rrtstar.cpp
)

add_executable(rrt_testmodified
    src/rrtstarmodified.cpp
    src/test_rrtstarmodified.cpp
)

# Add the new hand-eye calibration executable
# add_executable(hand_eye_calibration_test
#     src/hand_eye_calibration.cpp
#     src/test_hand_eye_calibration.cpp
# )

# Include directories
target_include_directories(ivk_test
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_include_directories(fk_standalone
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_include_directories(kinematics_test
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_include_directories(rrt_test
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_include_directories(rrt_testmodified
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_include_directories(ik_solution_eval_test
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

# target_include_directories(hand_eye_calibration_test
#     PRIVATE
#         ${PROJECT_SOURCE_DIR}/include
# )

# Find Python and link against it
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Find and link TBB (Intel Threading Building Blocks)
find_package(TBB REQUIRED)

# Link TBB to the target
target_link_libraries(ivk_test PRIVATE TBB::tbb)

# Link Google Test and pthread
target_link_libraries(ivk_test PRIVATE ${GTEST_LIBRARIES} GTest::gtest_main pthread)
target_link_libraries(kinematics_test PRIVATE ${GTEST_LIBRARIES} GTest::gtest_main pthread)
target_link_libraries(rrt_test PRIVATE ${GTEST_LIBRARIES} GTest::gtest_main pthread)
target_link_libraries(rrt_testmodified PRIVATE ${GTEST_LIBRARIES} GTest::gtest_main pthread)
# Link OpenCV libraries to the ik_solution_eval_test target
target_link_libraries(ik_solution_eval_test PRIVATE ${OpenCV_LIBS})
# target_link_libraries(ik_solution_eval_test PRIVATE ${GTEST_LIBRARIES} GTest::gtest_main pthread)
# target_link_libraries(hand_eye_calibration_test PRIVATE ${GTEST_LIBRARIES} GTest::gtest_main pthread)

target_link_libraries(ivk_test
    PRIVATE
        Python3::Python
        Eigen3::Eigen
        TBB::tbb
        ${GTEST_LIBRARIES}
        GTest::gtest_main
        pthread
)
target_link_libraries(fk_standalone
    PRIVATE
        Python3::Python
        Eigen3::Eigen
)
target_link_libraries(kinematics_test
    PRIVATE
        Python3::Python
        Eigen3::Eigen
        ${GTEST_LIBRARIES}
        GTest::gtest_main
        pthread
)
target_link_libraries(ik_solution_eval_test
    PRIVATE
        Python3::Python
        Eigen3::Eigen
        ${GTEST_LIBRARIES}
        GTest::gtest_main
        pthread
)
target_link_libraries(rrt_test
    PRIVATE
        Python3::Python
        Eigen3::Eigen
        ${GTEST_LIBRARIES}
        GTest::gtest_main
        pthread
        OpenMP::OpenMP_CXX # Explicitly link OpenMP for rrt_test
)

target_link_libraries(rrt_testmodified
    PRIVATE
        Python3::Python
        Eigen3::Eigen
        ${GTEST_LIBRARIES}
        GTest::gtest_main
        pthread
        OpenMP::OpenMP_CXX # Explicitly link OpenMP for rrt_test
)

# Link libraries for the new hand-eye calibration executable
# target_link_libraries(hand_eye_calibration_test
#     PRIVATE
#         Python3::Python
#         Eigen3::Eigen
#         ${OpenCV_LIBS}
#         ${GTEST_LIBRARIES}
#         GTest::gtest_main
#         pthread
# )

# Enable testing
enable_testing()

# Compiler options
if(MSVC)
    target_compile_options(ivk_test PRIVATE /W4)
    target_compile_options(fk_standalone PRIVATE /W4)
    target_compile_options(kinematics_test PRIVATE /W4)
    target_compile_options(rrt_test PRIVATE /W4)
    target_compile_options(rrt_testmodified PRIVATE /W4)
    target_compile_options(ik_solution_eval_test PRIVATE /W4)
    # target_compile_options(hand_eye_calibration_test PRIVATE /W4)
else()
    target_compile_options(ivk_test PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(fk_standalone PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(kinematics_test PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(rrt_test PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(rrt_testmodified PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ik_solution_eval_test PRIVATE -Wall -Wextra -Wpedantic)
    # target_compile_options(hand_eye_calibration_test PRIVATE -Wall -Wextra -Wpedantic)
endif()