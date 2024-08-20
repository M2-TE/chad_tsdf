FetchContent_Declare(phmap GIT_REPOSITORY "https://github.com/greg7mdp/parallel-hashmap.git" GIT_TAG "v1.3.12" GIT_SHALLOW ON)
FetchContent_MakeAvailable(phmap)
target_link_libraries(${PROJECT_NAME} PRIVATE phmap)