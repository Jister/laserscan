# Install script for directory: /home/chenjie/桌面/eigen/Eigen/src/Core/products

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen/src/Core/products" TYPE FILE FILES
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/TriangularMatrixVector.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/TriangularSolverVector.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/TriangularMatrixMatrix_MKL.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/GeneralMatrixMatrixTriangular_MKL.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/GeneralMatrixMatrixTriangular.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/SelfadjointMatrixMatrix_MKL.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/GeneralBlockPanelKernel.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/SelfadjointMatrixMatrix.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/GeneralMatrixMatrix.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/GeneralMatrixMatrix_MKL.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/TriangularMatrixMatrix.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/SelfadjointProduct.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/SelfadjointMatrixVector.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/TriangularSolverMatrix.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/TriangularSolverMatrix_MKL.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/TriangularMatrixVector_MKL.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/Parallelizer.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/GeneralMatrixVector_MKL.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/SelfadjointRank2Update.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/SelfadjointMatrixVector_MKL.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/CoeffBasedProduct.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/products/GeneralMatrixVector.h"
    )
endif()

