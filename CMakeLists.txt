ADD_LIBRARY(hokuyo SHARED hokuyourg.cc)
ADD_EXECUTABLE(hokuyo_test hokuyourg_test.cc)
TARGET_LINK_LIBRARIES(hokuyo_test hokuyo)

