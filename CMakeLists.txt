
PROJECT(muscleDriverCANInterface)


FIND_PACKAGE(Qt4 REQUIRED)


INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

#INCLUDE_DIRECTORIES(../include)
#INCLUDE_DIRECTORIES(../classes)


SET(PROJECT_HEADERS   motorDriverBoardConfiguration/CanInterface.h motorDriverBoardConfiguration/KvaserCanInterface.h userInterface.h controlCycle.h phidget.h log.h)
SET(PROJECT_SOURCES   main.cpp motorDriverBoardConfiguration/can.cpp motorDriverBoardConfiguration/CanInterface.cpp  motorDriverBoardConfiguration/KvaserCanInterface.cpp muscleDriverCANInterface.cpp controlCycle.cpp userInterface.cpp log.cpp)
QT4_WRAP_CPP(QTGENERATED_HEADERS_MOC muscleDriverCANInterface.h userInterface.h)

message(STATUS "MOC FILES ARE: ${QTGENERATED_HEADERS_MOC}")

ADD_EXECUTABLE(muscleDriverCANInterface ${PROJECT_SOURCES} ${PROJECT_HEADERS}  ${QTGENERATED_HEADERS_MOC}  )

#include the Kvaser CAN library
TARGET_LINK_LIBRARIES(muscleDriverCANInterface canlib  )
TARGET_LINK_LIBRARIES(muscleDriverCANInterface ${QT_LIBRARIES})
#include phidget library
TARGET_LINK_LIBRARIES(muscleDriverCANInterface phidget21 )



