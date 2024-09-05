FetchContent_Declare(parallel-hashmap GIT_REPOSITORY "https://github.com/greg7mdp/parallel-hashmap.git" GIT_TAG "v1.3.12" GIT_SHALLOW ON)
FetchContent_MakeAvailable(parallel-hashmap)
target_link_libraries(${PROJECT_NAME} PRIVATE parallel-hashmap)