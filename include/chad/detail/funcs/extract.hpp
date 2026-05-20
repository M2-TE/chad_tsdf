#pragma once
#include "chad/tsdf_map.hpp"

namespace chad::detail::funcs {
    // copy xyz values from input array into a std::vector of glm::vec3
    template<std::size_t XYZ_BYTES, std::size_t RGB_BYTES>
    auto extract_xyz(const std::uint8_t* data_p, std::size_t data_bytes, chad::PointFlags data_flags) -> std::vector<glm::aligned_vec3> {
        if      (data_flags & PointFlagBits::eXYZW_F32) throw std::logic_error("Not yet implemented");
        else if (data_flags & PointFlagBits::eXYZ_F64)  throw std::logic_error("Not yet implemented");
        else if (data_flags & PointFlagBits::eXYZW_F64) throw std::logic_error("Not yet implemented");

        // constexpr byte sizes for SIMD leverage
        constexpr std::size_t POINT_BYTES = XYZ_BYTES + RGB_BYTES;
        std::vector<glm::aligned_vec3> points;
        points.resize(data_bytes / POINT_BYTES);
        // perform safe bit-wise copy from point array to glm vector
        for (std::size_t i = 0; i < points.size(); i++) {
            glm::aligned_vec3* dst_p = std::next(points.data(), i);
            const std::uint8_t* src_p = std::next(data_p, i * POINT_BYTES);
            std::memcpy(dst_p, src_p, XYZ_BYTES);
        }
        return points;
    }

    template<std::size_t XYZ_BYTES>
    auto inline extract_xyz(const std::uint8_t* data_p, std::size_t data_bytes, chad::PointFlags data_flags) -> std::vector<glm::aligned_vec3> {
        // ensure that no RGB bit is set (not yet implemented)
        constexpr PointFlags rgb_all = PointFlagBits::eRGB_U8 | PointFlagBits::eRGB_U8 | PointFlagBits::eRGB_U8 | PointFlagBits::eRGB_U8;
        if (std::popcount(data_flags & rgb_all) > 0) throw std::runtime_error("TSDFMap: RGB input is not yet supported");
        // forward to next call
        if      (data_flags & PointFlagBits::eRGB_U8)   return extract_xyz<XYZ_BYTES, sizeof(std::uint8_t) * 3>(data_p, data_bytes, data_flags);
        else if (data_flags & PointFlagBits::eRGBA_U8)  return extract_xyz<XYZ_BYTES, sizeof(std::uint8_t) * 4>(data_p, data_bytes, data_flags);
        else if (data_flags & PointFlagBits::eRGB_F32)  return extract_xyz<XYZ_BYTES, sizeof(float) * 3>(data_p, data_bytes, data_flags);
        else if (data_flags & PointFlagBits::eRGBA_F32) return extract_xyz<XYZ_BYTES, sizeof(float) * 4>(data_p, data_bytes, data_flags);
        else                                            return extract_xyz<XYZ_BYTES, 0>(data_p, data_bytes, data_flags);
    }

    auto inline extract_xyz(const std::uint8_t* data_p, std::size_t data_bytes, chad::PointFlags data_flags) -> std::vector<glm::aligned_vec3> {
        // ensure exactly one XYZ bit is set
        constexpr PointFlags xyz_all = PointFlagBits::eXYZ_F32 | PointFlagBits::eXYZ_F64 | PointFlagBits::eXYZW_F32 | PointFlagBits::eXYZW_F64;
        if (std::popcount(data_flags & xyz_all) != 1) throw std::runtime_error("TSDFMap: Exactly one XYZ(W) bit in PointFlags must be set");
        // forward to next call
        if      (data_flags & PointFlagBits::eXYZ_F32)  return extract_xyz<sizeof(float) * 3>(data_p, data_bytes, data_flags);
        else if (data_flags & PointFlagBits::eXYZW_F32) return extract_xyz<sizeof(float) * 4>(data_p, data_bytes, data_flags);
        else if (data_flags & PointFlagBits::eXYZ_F64)  return extract_xyz<sizeof(double) * 3>(data_p, data_bytes, data_flags);
        else if (data_flags & PointFlagBits::eXYZW_F64) return extract_xyz<sizeof(double) * 4>(data_p, data_bytes, data_flags);
        else throw std::logic_error("TSDFMap: Invalid branch in extract_xyz"); // return extract_xyz<0>(data_p, data_bytes, data_flags);
    }
}
