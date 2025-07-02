#pragma once
#include <array>
#include <chrono>
#include <vector>
#include <bitset>
#include <functional>
#if !defined(__BMI2__)
#   error "Requires BMI2 instruction set"
#endif
#include <fmt/format.h>
#include <glm/glm.hpp>
#include <libmorton/morton.h>

namespace chad::detail {
    struct MortonCode {
        MortonCode(uint64_t value): _value(value) {}
        MortonCode(const glm::ivec3& vox_pos) noexcept {
            encode(vox_pos);
        }

        void inline encode(const glm::ivec3& vox_pos) noexcept {
            // truncate from 32-bit int to 21-bit int
            uint32_t x, y, z;
            x = (1 << 20) + uint32_t(vox_pos.x);
            y = (1 << 20) + uint32_t(vox_pos.y);
            z = (1 << 20) + uint32_t(vox_pos.z);
            _value = uint64_t(libmorton::morton3D_64_encode(x, y, z));
        }
        auto inline decode() const noexcept -> glm::ivec3 {
            uint_fast32_t x, y, z;
            libmorton::morton3D_64_decode(_value, x, y, z);
            // expand from 21-bit uint back to 32-bit int
            x -= 1 << 20;
            y -= 1 << 20;
            z -= 1 << 20;
            return { int32_t(x), int32_t(y), int32_t(z) };
        }

        bool inline operator==(const MortonCode& other) const noexcept {
            return _value == other._value;
        }
        bool inline operator<(const MortonCode& other) const noexcept {
            return _value < other._value;
        }
        bool inline operator>(const MortonCode& other) const noexcept {
            return _value > other._value;
        }
        auto friend operator&(MortonCode lhs, const MortonCode& rhs) noexcept -> MortonCode {
            return lhs._value & rhs._value;
        }
        auto friend operator&(MortonCode lhs, const uint64_t& rhs) noexcept -> MortonCode {
            return lhs._value & rhs;
        }
        

        uint64_t _value;
    };
    using MortonVector = std::vector<std::pair<glm::vec3, MortonCode>>;
    auto inline calc_morton_vector(const std::vector<std::array<float, 3>>& points, const float sdf_res) -> MortonVector {
        auto beg = std::chrono::high_resolution_clock::now();

        // calc reciprocal of voxel resolution for later
        const float voxel_reciprocal = float(1.0 / double(sdf_res));

        // generate morton codes from discretized points
        MortonVector points_mc;
        points_mc.reserve(points.size());
        for (const auto& point_arr: points) {
            glm::vec3 point { point_arr[0], point_arr[1], point_arr[2] };
            // convert to voxel coordinate and discretize with floor()
            glm::vec3 point_discretized = glm::floor(point * voxel_reciprocal);
            // create morton code from discretized integer position
            points_mc.emplace_back(point, glm::ivec3(point_discretized));
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("mc  calc {:.2f}", dur);
        return points_mc;
    }
    auto inline sort_morton_vector(MortonVector& points_mc) -> std::vector<glm::vec3> {
        auto beg = std::chrono::high_resolution_clock::now();

        // sort points using morton codes
        auto mc_sorter = [](const auto& a, const auto& b){
            return a.second._value > b.second._value;
        };
        // std::sort(std::execution::par_unseq, points_mc.begin(), points_mc.end(), mc_sorter);
        std::sort(points_mc.begin(), points_mc.end(), mc_sorter);
        
        // store isolated sorted points
        std::vector<glm::vec3> points_sorted;
        points_sorted.reserve(points_mc.size());
        for (const auto& mc_point: points_mc) {
            points_sorted.push_back(mc_point.first);
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("mc  sort {:.2f}", dur);
        return points_sorted;
    }
}

// specialize the std hashing operator for MortonCode
namespace std {
    template<>
    struct hash<chad::detail::MortonCode> {
        size_t inline operator()(const chad::detail::MortonCode& mc) const noexcept {
            return mc._value;
        }
    };
}

// specialize the fmt formatter for MortonCode
namespace fmt {
    template<>
    struct formatter<chad::detail::MortonCode>: formatter<std::string> {
        auto format(const chad::detail::MortonCode& mc, format_context& ctx) const -> format_context::iterator {
            std::string str = std::bitset<63>(mc._value).to_string();
            return formatter<std::string>::format(str, ctx);
        }
    };
}