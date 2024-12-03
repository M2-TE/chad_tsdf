# CHAD TSDF

CHAD TSDF structure for SLAM

## Description

TODO: link to ros implementation

## Getting Started

<details>
    <summary>CMake options</summary>

TODO:
set(CHAD_POPCOUNT_INSTRUCTION std::popcount<uint8_t>)
set(CHAD_NORM_NEIGH_UPPER 20)
set(CHAD_NORM_NEIGH_LOWER 2)
set(CHAD_NORM_MIN_NEIGH 3)
set(CHAD_NORM_RADIUS_MOD 1.5)
set(CHAD_LEAF_RESOLUTION 0.1)
set(CHAD_LEAF_BITS 8)
</details>

<details>
    <summary>CMake Build</summary>
    
#### 1. CMake local build

```
cmake -B build
cmake --build build
TODO: install
TODO: CMake find()
```

#### 2. CMake subdirectory

```cmake
add_subdirectory(${CHAD_DIRECTORY})
target_link_libraries(${PROJECT_NAME} PUBLIC chad::tsdf)
```

#### 3. CMake FetchContent

```cmake
include(FetchContent)
FetchContent_Declare(chad_tsdf
    GIT_REPOSITORY "https://github.com/M2-TE/chad_tsdf.git"
    GIT_TAG "main"
    GIT_SHALLOW ON)
FetchContent_MakeAvailable(chad_tsdf)
target_link_libraries(${PROJECT_NAME} PUBLIC chad::tsdf)
```
</details>


## Authors

Jan Kuhlmann

## Version History

* 0.1
    * Initial Release

## Acknowledgments

* [parallel-hashmap](https://github.com/greg7mdp/parallel-hashmap)
