# Install script for directory: /home/chenjie/桌面/eigen/Eigen/src/SparseCore

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen/src/SparseCore" TYPE FILE FILES
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseSelfAdjointView.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/CompressedStorage.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseMatrix.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseColEtree.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseUtil.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseMatrixBase.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseView.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseSparseProductWithPruning.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseCwiseUnaryOp.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseDenseProduct.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseDot.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/AmbiVector.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseCwiseBinaryOp.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/TriangularSolver.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseFuzzy.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseTriangularView.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/MappedSparseMatrix.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseRedux.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparsePermutation.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseProduct.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseDiagonalProduct.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseBlock.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseTranspose.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/ConservativeSparseSparseProduct.h"
    "/home/chenjie/桌面/eigen/Eigen/src/SparseCore/SparseVector.h"
    )
endif()

