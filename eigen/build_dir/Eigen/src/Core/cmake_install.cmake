# Install script for directory: /home/chenjie/桌面/eigen/Eigen/src/Core

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen/src/Core" TYPE FILE FILES
    "/home/chenjie/桌面/eigen/Eigen/src/Core/MatrixBase.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Replicate.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Reverse.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/ArrayWrapper.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/TriangularMatrix.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/PlainObjectBase.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Redux.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/NumTraits.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/BandMatrix.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Assign.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/StableNorm.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Array.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/GeneralProduct.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Select.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Stride.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/IO.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/BooleanRedux.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Ref.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Transpose.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Swap.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/CwiseUnaryOp.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/VectorBlock.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/CoreIterators.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Block.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/NestByValue.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/CwiseBinaryOp.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/DiagonalMatrix.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/NoAlias.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/CommaInitializer.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Visitor.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Fuzzy.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/PermutationMatrix.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/GenericPacketMath.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Functors.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Random.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/ArrayBase.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/MathFunctions.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/VectorwiseOp.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Map.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/DenseBase.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Dot.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/DenseStorage.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Diagonal.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/ProductBase.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/CwiseUnaryView.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Transpositions.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/GlobalFunctions.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/DenseCoeffsBase.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/SelfCwiseBinaryOp.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Assign_MKL.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/ForceAlignedAccess.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/SelfAdjointView.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/ReturnByValue.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/DiagonalProduct.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/EigenBase.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Matrix.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/CwiseNullaryOp.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/Flagged.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/SolveTriangular.h"
    "/home/chenjie/桌面/eigen/Eigen/src/Core/MapBase.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/chenjie/桌面/eigen/build_dir/Eigen/src/Core/products/cmake_install.cmake")
  include("/home/chenjie/桌面/eigen/build_dir/Eigen/src/Core/util/cmake_install.cmake")
  include("/home/chenjie/桌面/eigen/build_dir/Eigen/src/Core/arch/cmake_install.cmake")

endif()

