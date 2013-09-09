PROJECT(Nanopositioning_Simulator)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#FIND_PACKAGE( OpenCV REQUIRED )

FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)


#INCLUDE_DIRECTORIES(test)

ADD_EXECUTABLE(Nanopositioning_Simulator Nanopositioning_Simulator.cpp npFeatureLuminance.cpp npFeatureLuminance.h  npImageSimulator.cpp npImageSimulator.h)

