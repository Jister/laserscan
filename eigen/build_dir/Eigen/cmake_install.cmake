# Install script for directory: /home/chenjie/桌面/eigen/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/home/chenjie/桌面/eigen/Eigen/SPQRSupport"
    "/home/chenjie/桌面/eigen/Eigen/SVD"
    "/home/chenjie/桌面/eigen/Eigen/Sparse"
    "/home/chenjie/桌面/eigen/Eigen/QR"
    "/home/chenjie/桌面/eigen/Eigen/LeastSquares"
    "/home/chenjie/桌面/eigen/Eigen/SparseQR"
    "/home/chenjie/桌面/eigen/Eigen/StdDeque"
    "/home/chenjie/桌面/eigen/Eigen/SuperLUSupport"
    "/home/chenjie/桌面/eigen/Eigen/SparseCore"
    "/home/chenjie/桌面/eigen/Eigen/LU"
    "/home/chenjie/桌面/eigen/Eigen/Eigenvalues"
    "/home/chenjie/桌面/eigen/Eigen/UmfPackSupport"
    "/home/chenjie/桌面/eigen/Eigen/StdVector"
    "/home/chenjie/桌面/eigen/Eigen/SparseCholesky"
    "/home/chenjie/桌面/eigen/Eigen/QtAlignedMalloc"
    "/home/chenjie/桌面/eigen/Eigen/Core"
    "/home/chenjie/桌面/eigen/Eigen/PaStiXSupport"
    "/home/chenjie/桌面/eigen/Eigen/Array"
    "/home/chenjie/桌面/eigen/Eigen/Geometry"
    "/home/chenjie/桌面/eigen/Eigen/Householder"
    "/home/chenjie/桌面/eigen/Eigen/Eigen2Support"
    "/home/chenjie/桌面/eigen/Eigen/SparseLU"
    "/home/chenjie/桌面/eigen/Eigen/Dense"
    "/home/chenjie/桌面/eigen/Eigen/Jacobi"
    "/home/chenjie/桌面/eigen/Eigen/Cholesky"
    "/home/chenjie/桌面/eigen/Eigen/Eigen"
    "/home/chenjie/桌面/eigen/Eigen/IterativeLinearSolvers"
    "/home/chenjie/桌面/eigen/Eigen/PardisoSupport"
    "/home/chenjie/桌面/eigen/Eigen/CholmodSupport"
    "/home/chenjie/桌面/eigen/Eigen/StdList"
    "/home/chenjie/桌面/eigen/Eigen/MetisSupport"
    "/home/chenjie/桌面/eigen/Eigen/OrderingMethods"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/chenjie/桌面/eigen/build_dir/Eigen/src/cmake_install.cmake")

endif()

