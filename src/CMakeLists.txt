ADD_LIBRARY( Reconstructor SHARED
        dataloader.cpp
        PCbuilder.cpp)

TARGET_LINK_LIBRARIES(Reconstructor ${THIRD_PARTY_LIBRARIES})

ADD_EXECUTABLE(test test.cpp)
TARGET_LINK_LIBRARIES(test Reconstructor)

ADD_EXECUTABLE(test_tiff test_tiff.cpp)
TARGET_LINK_LIBRARIES(test_tiff Reconstructor)
