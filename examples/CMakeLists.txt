ADD_EXECUTABLE(PC_from_xml PC_from_xml.cpp)
TARGET_LINK_LIBRARIES(PC_from_xml Reconstructor)

ADD_EXECUTABLE(PC_from_tiff PC_from_tiff.cpp)
TARGET_LINK_LIBRARIES(PC_from_tiff Reconstructor)


ADD_EXECUTABLE(PC_registration PC_registration.cpp)
TARGET_LINK_LIBRARIES(PC_registration Reconstructor)
