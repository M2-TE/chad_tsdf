#pragma once
#if !defined(__BMI2__)
#   error "Requires BMI2 instruction set"
#endif

namespace chad::detail {
    struct MortonCode {
        MortonCode(uint64_t value): _value(value) {}
        MortonCode(const glm::aligned_ivec3& vox_pos) {
            encode(vox_pos);
        }
        MortonCode(const glm::aligned_vec3& point, float sdf_res_reciprocal) {
            // convert to voxel coordinate and discretize with floor()
            glm::aligned_vec3 point_discretized = glm::floor(point * sdf_res_reciprocal);
            encode(glm::aligned_ivec3{ point_discretized });
        }
        // TODO: remove this once revamp is done
        [[deprecated]]
        MortonCode(const glm::ivec3& vox_pos) {
            encode(vox_pos);
        }

        void inline encode(const glm::aligned_ivec3& vox_pos) {
            // truncate from 32-bit int to 21-bit int
            uint32_t x, y, z;
            x = (1 << 20) + uint32_t(vox_pos.x);
            y = (1 << 20) + uint32_t(vox_pos.y);
            z = (1 << 20) + uint32_t(vox_pos.z);
            _value = uint64_t(libmorton::morton3D_64_encode(x, y, z));
        }
        auto inline decode() const -> glm::aligned_ivec3 {
            uint_fast32_t x, y, z;
            libmorton::morton3D_64_decode(_value, x, y, z);
            // expand from 21-bit uint back to 32-bit int
            x -= 1 << 20;
            y -= 1 << 20;
            z -= 1 << 20;
            return { int32_t(x), int32_t(y), int32_t(z) };
        }
        void inline print() {
            fmt::println("{}", std::bitset<63>(_value).to_string());
        }

        bool inline operator==(const MortonCode& other) const {
            return _value == other._value;
        }
        bool inline operator<(const MortonCode& other) const {
            return _value < other._value;
        }
        bool inline operator>(const MortonCode& other) const {
            return _value > other._value;
        }
        auto friend operator&(MortonCode lhs, const MortonCode& rhs) -> MortonCode {
            return lhs._value & rhs._value;
        }
        auto friend operator&(MortonCode lhs, const uint64_t& rhs) -> MortonCode {
            return lhs._value & rhs;
        }

        uint64_t _value;
    };

    namespace morton {
        // sort points by their morton code
        void inline sort(std::vector<glm::aligned_vec3>& points, float sdf_res) {
            // reciprocal of voxel resolution for later
            const float sdf_res_reciprocal = static_cast<float>(1.0 / double(sdf_res));

            // create morton codes from XYZ coordinates
            std::vector<MortonCode> morton_codes;
            morton_codes.reserve(points.size());
            for (const auto& point: points) {
                morton_codes.push_back(MortonCode{ point, sdf_res_reciprocal });
            }

            // prepare a set of indices for sorting
            std::vector<std::uint32_t> indices;
            indices.resize(points.size());
            std::iota(indices.begin(), indices.end(), 0);
            std::sort(indices.begin(), indices.end(), [&](std::uint32_t a, std::uint32_t b) -> bool {
                return morton_codes[a] < morton_codes[b];
            });

            // sort using already sorted indices
            const auto points_copy = points;
            for (std::uint32_t i = 0; i < points.size(); i++) {
                std::uint32_t sorted_index = indices[i];
                points[i] = points_copy[sorted_index];
            }
        }
    }
}

// specialize the std hashing operator for MortonCode
namespace std {
    template<>
    struct hash<chad::detail::MortonCode> {
        size_t inline operator()(const chad::detail::MortonCode& mc) const {
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
