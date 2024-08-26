FetchContent_Declare(morton-nd GIT_REPOSITORY "https://github.com/morton-nd/morton-nd.git" GIT_TAG "v4.0.0" GIT_SHALLOW ON)
FetchContent_MakeAvailable(morton-nd)
target_link_libraries(${PROJECT_NAME} PRIVATE morton-nd::MortonND)