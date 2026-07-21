// std
#include <array>
#include <mutex>
#include <bitset>
#include <chrono>
#include <random>
#include <string>
#include <thread>
#include <limits>
#include <vector>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <fstream>
#include <algorithm>
#include <execution>
#include <stdexcept>
#include <filesystem>
#include <functional>
#include <string_view>

// ext
#define GLM_FORCE_CXX20
#define GLM_FORCE_INLINE
#define GLM_FORCE_INTRINSICS
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_aligned.hpp>
// better hashmap implementation
#include <gtl/phmap.hpp>
// morton code encoding/decoding
#include <libmorton/morton.h>
// std::print wannabe
#include <fmt/base.h>
#include <fmt/format.h>
