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
// std::print wannabe
#include <fmt/base.h>
#include <fmt/format.h>

namespace fmt {
    template<>
    struct formatter<glm::aligned_ivec3>: formatter<std::string> {
        auto format(const glm::aligned_ivec3& vec, format_context& ctx) const -> format_context::iterator {
            return formatter<std::string>::format(fmt::format("{} {} {}", vec.x, vec.y, vec.z), ctx);
        }
    };

    template<>
    struct formatter<glm::vec3>: formatter<std::string> {
        auto format(const glm::vec3& vec, format_context& ctx) const -> format_context::iterator {
            return formatter<std::string>::format(fmt::format("{:.2f} {:.2f} {:.2f}", vec.x, vec.y, vec.z), ctx);
        }
    };
    template<>
    struct formatter<glm::aligned_vec3>: formatter<std::string> {
        auto format(const glm::aligned_vec3& vec, format_context& ctx) const -> format_context::iterator {
            return formatter<std::string>::format(fmt::format("{:.2f} {:.2f} {:.2f}", vec.x, vec.y, vec.z), ctx);
        }
    };
    template<>
    struct formatter<glm::aligned_dvec3>: formatter<std::string> {
        auto format(const glm::aligned_dvec3& vec, format_context& ctx) const -> format_context::iterator {
            return formatter<std::string>::format(fmt::format("{:.4f} {:.4f} {:.4f}", vec.x, vec.y, vec.z), ctx);
        }
    };

    template<>
    struct formatter<glm::vec4>: formatter<std::string> {
        auto format(const glm::vec4& vec, format_context& ctx) const -> format_context::iterator {
            return formatter<std::string>::format(fmt::format("{:.2f} {:.2f} {:.2f} {:.2f}", vec.x, vec.y, vec.z, vec.w), ctx);
        }
    };
    template<>
    struct formatter<glm::aligned_vec4>: formatter<std::string> {
        auto format(const glm::aligned_vec4& vec, format_context& ctx) const -> format_context::iterator {
            return formatter<std::string>::format(fmt::format("{:.2f} {:.2f} {:.2f} {:.2f}", vec.x, vec.y, vec.z, vec.w), ctx);
        }
    };
    template<>
    struct formatter<glm::aligned_dvec4>: formatter<std::string> {
        auto format(const glm::aligned_dvec4& vec, format_context& ctx) const -> format_context::iterator {
            return formatter<std::string>::format(fmt::format("{:.4f} {:.4f} {:.4f} {:.4f}", vec.x, vec.y, vec.z, vec.w), ctx);
        }
    };
}
