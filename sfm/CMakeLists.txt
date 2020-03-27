add_executable(FeatureExtraction FeatureExtraction.cpp)
target_link_libraries(FeatureExtraction ${THIRD_PARTY_LIBS} ${PROJECT_NAME})

add_executable(ComputeMatches ComputeMatches.cpp)
target_link_libraries(ComputeMatches ${THIRD_PARTY_LIBS} ${PROJECT_NAME})

add_executable(Reconstruction Reconstruction.cpp)
target_link_libraries(Reconstruction ${THIRD_PARTY_LIBS} ${PROJECT_NAME})