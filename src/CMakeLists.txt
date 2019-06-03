ADD_LIBRARY( Reconstructor SHARED
        dataloader.cpp
        PCbuilder.cpp)


TARGET_LINK_LIBRARIES(Reconstructor ${THIRD_PARTY_LIBRARIES})
