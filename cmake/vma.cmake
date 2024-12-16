FetchContent_Declare(vulkanmemoryallocator
    GIT_REPOSITORY "https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator.git"
    GIT_TAG "v3.1.0"
    GIT_SHALLOW ON
    GIT_SUBMODULES ""
    SOURCE_SUBDIR "disabled/"
    OVERRIDE_FIND_PACKAGE)
FetchContent_Declare(vulkanmemoryallocator-hpp
    GIT_REPOSITORY "https://github.com/YaaZ/VulkanMemoryAllocator-Hpp.git"
    GIT_TAG "v3.1.0"
    GIT_SHALLOW ON
    GIT_SUBMODULES ""
    SOURCE_SUBDIR "disabled/"
    OVERRIDE_FIND_PACKAGE)
FetchContent_MakeAvailable(vulkanmemoryallocator vulkanmemoryallocator-hpp)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
    "${vulkanmemoryallocator_SOURCE_DIR}/include"
	"${vulkanmemoryallocator-hpp_SOURCE_DIR}/include")
target_compile_definitions(${PROJECT_NAME} PRIVATE 
    "VMA_DYNAMIC_VULKAN_FUNCTIONS" 
	"VMA_STATIC_VULKAN_FUNCTIONS=0")