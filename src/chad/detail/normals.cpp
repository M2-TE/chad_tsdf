#include <chrono>
#include <cstdint>
#include <fmt/base.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_aligned.hpp>
#include <gtl/phmap.hpp>
#include "chad/detail/morton.hpp"

namespace {
    // sourced from: https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
    template<typename T, glm::qualifier P>
    auto inline estimate_normal(const std::vector<glm::vec<3, T, P>>& points) -> glm::vec3 {
        // calculate centroid by through coefficient average
        glm::vec<3, T, P> centroid { 0, 0, 0 };
        for (const auto& point: points) {
            centroid += point;
        }
        T recip = 1.0 / (T)points.size();
        centroid *= recip;

        // covariance matrix excluding symmetries
        T xx = 0.0; T xy = 0.0; T xz = 0.0;
        T yy = 0.0; T yz = 0.0; T zz = 0.0;
        for (auto point_it = points.cbegin(); point_it != points.cend(); point_it++) {
            auto r = *point_it - centroid;
            xx += r.x * r.x;
            xy += r.x * r.y;
            xz += r.x * r.z;
            yy += r.y * r.y;
            yz += r.y * r.z;
            zz += r.z * r.z;
        }
        xx *= recip;
        xy *= recip;
        xz *= recip;
        yy *= recip;
        yz *= recip;
        zz *= recip;

        // weighting linear regression based on square determinant
        glm::vec<3, T, P> weighted_dir = { 0, 0, 0 };

        // determinant x
        {
            T det_x = yy*zz - yz*yz;
            glm::vec<3, T, P> axis_dir = {
                det_x,
                xz*yz - xy*zz,
                xy*yz - xz*yy
            };
            T weight = det_x * det_x;
            if (glm::dot(weighted_dir, axis_dir) < 0.0) weight = -weight;
            weighted_dir += axis_dir * weight;
        }
        // determinant y
        {
            T det_y = xx*zz - xz*xz;
            glm::vec<3, T, P> axis_dir = {
                xz*yz - xy*zz,
                det_y,
                xy*xz - yz*xx
            };
            T weight = det_y * det_y;
            if (glm::dot(weighted_dir, axis_dir) < 0.0) weight = -weight;
            weighted_dir += axis_dir * weight;
        }
        // determinant z
        {
            T det_z = xx*yy - xy*xy;
            glm::vec<3, T, P> axis_dir = {
                xy*yz - xz*yy,
                xy*xz - yz*xx,
                det_z
            };
            T weight = det_z * det_z;
            if (glm::dot(weighted_dir, axis_dir) < 0.0) weight = -weight;
            weighted_dir += axis_dir * weight;
        }

        // return normalized weighted direction as surface normal
        weighted_dir = glm::normalize(weighted_dir);
        return { float(weighted_dir.x), float(weighted_dir.y), float(weighted_dir.z) };
    }
}

namespace chad::detail {
    struct MortonNeighbourhood { MortonVector::const_iterator beg, end; };
    using MortonNeighbourhoodMap = gtl::parallel_flat_hash_map<MortonCode, MortonNeighbourhood>;
    
    auto build_neighbourhood_map(const MortonVector& points_mc, const uint32_t level) -> MortonNeighbourhoodMap {
        auto beg = std::chrono::high_resolution_clock::now();

        // create mask to group morton codes up to a certain level
        const uint64_t neigh_mask = std::numeric_limits<uint64_t>::max() << size_t(level * 3);

        // create hash map for neighbourhoods, reserving generous space beforehand
        MortonNeighbourhoodMap neigh_map;
        static constexpr size_t min_points_per_neighbourhood = 8;
        neigh_map.reserve(points_mc.size() / min_points_per_neighbourhood);

        // insert the first neighbourhood
        MortonCode neigh_mc = points_mc[0].second;
        neigh_mc._value &= neigh_mask;
        MortonNeighbourhood neigh { points_mc.cbegin(), points_mc.cbegin() };
        auto neigh_it = neigh_map.emplace(neigh_mc, neigh).first;

        uint32_t neigh_count = 0; // track number of neighbourhoods
        uint32_t neigh_points = 0; // track total points within all neighbourhoods
        static constexpr uint32_t neigh_threshhold_checkup = 256; // perform point count check after this neigh_count
        static constexpr uint32_t neigh_threshhold_abort = neigh_threshhold_checkup * min_points_per_neighbourhood;

        // iterate through all points to build neighbourhoods
        for (auto morton_it = points_mc.cbegin()+1; morton_it != points_mc.cend(); morton_it++) {
            // get morton codes from current point and current neighbourhood
            MortonCode point_mc = morton_it->second._value & neigh_mask;
            MortonCode neigh_mc = neigh_it->second.beg->second;

            // check if the current point doesnt fit the neighbourhood
            if (point_mc != neigh_mc) {
                // set end() iterator for current neighbourhood
                neigh_it->second.end = morton_it;
                // count points and neighbourhoods
                neigh_count++;
                neigh_points += std::distance(neigh_it->second.beg, morton_it);
                if (neigh_count >= neigh_threshhold_checkup) {
                    // ensure enough points were placed in the neighbourhoods
                    if (neigh_points < neigh_threshhold_abort) {
                        fmt::println("nei    {} (aborted)", level);
                        return {};
                    }
                }
                // create a new neighbourhood starting at current point
                MortonNeighbourhood neigh { morton_it, morton_it };
                neigh_it = neigh_map.emplace(point_mc, neigh).first;
            }
        }
        // set end() iterator for final neighbourhood
        neigh_it->second.end = points_mc.cend();

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("nei    {} {:.2f}", level, dur);
        return neigh_map;
    }
    auto estimate_normals(const MortonVector& points_mc, const glm::vec3 position) -> std::vector<glm::vec3> {
        auto beg = std::chrono::high_resolution_clock::now();

        // build morton code neighbourhood maps starting from the leaf level
        uint32_t neigh_level = 0;
        MortonNeighbourhoodMap neigh_map;
        for (; neigh_level < 8; neigh_level++) {
            neigh_map = build_neighbourhood_map(points_mc, neigh_level);
            if (!neigh_map.empty()) break;
        }
        // verify that it found a fitting neighbourhood map
        if (neigh_map.empty()) fmt::println("Too few points inserted for this voxel resolution");

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("nei calc {:.2f}", dur);
        
        beg = std::chrono::high_resolution_clock::now();
        std::vector<glm::vec3> normals;
        normals.resize(points_mc.size());
        
        // iterate over all neighbourhoods
        for (const auto& neigh_entry: neigh_map) {
            MortonCode neigh_mc = neigh_entry.first;
            MortonNeighbourhood neigh = neigh_entry.second;
            glm::ivec3 neigh_pos = neigh_mc.decode();

            // gather adjacent neighbourhoods
            uint32_t point_count = 0;
            std::vector<MortonNeighbourhood> neigh_adj;
            for (int32_t z = -1; z <= +1; z++) {
            for (int32_t y = -1; y <= +1; y++) {
            for (int32_t x = -1; x <= +1; x++) {
                // construct neighbourhood position with offset
                glm::ivec3 offset { x, y, z };
                offset *= 1 << neigh_level;
                MortonCode mc_adj { neigh_pos + offset };
                // attempt to find this neighbourhood in the map
                auto neigh_adj_it = neigh_map.find(mc_adj._value);
                if (neigh_adj_it != neigh_map.cend()) {
                    MortonNeighbourhood neigh = neigh_adj_it->second;
                    neigh_adj.push_back(neigh);
                    // add point count from this neighbourhood
                    point_count += std::distance(neigh.beg, neigh.end);
                }
            }}}

            // supersample in case there are too many points
            static constexpr uint32_t max_points = 64;
            uint32_t step = point_count / max_points + 1;

            // gather points of all adjacent neighbourhoods
            std::vector<glm::aligned_vec3> points_adj;
            points_adj.reserve(max_points);
            uint32_t counter = 0;
            for (const auto& neigh: neigh_adj) {
                for (auto it = neigh.beg; it != neigh.end; it++) {
                    if (counter++ >= step) {
                        points_adj.push_back(it->first);
                        counter = 0;
                    }
                }
            }

            if (points_adj.size() > 8) {
                // estimate normal of points when theres sufficient data
                glm::vec3 normal = estimate_normal(points_adj);

                // assign this normal to all center neighbourhood points
                for (auto point_it = neigh.beg; point_it != neigh.end; point_it++) {
                    // flip normal if needed
                    float normal_dot = glm::dot(normal, glm::normalize(position - point_it->first));
                    if (normal_dot < 0.0f) normal = -normal;

                    // store normal
                    size_t point_index = std::distance(points_mc.cbegin(), point_it);
                    normals[point_index] = normal;
                }
            }
            else {
                // otherwise use normalized vector from point to position as normal
                for (auto point_it = neigh.beg; point_it != neigh.end; point_it++) {
                    glm::vec3 normal = glm::normalize(position - point_it->first);

                    // store normal
                    size_t point_index = std::distance(points_mc.cbegin(), point_it);
                    normals[point_index] = normal;
                }
            }
        }

        end = std::chrono::high_resolution_clock::now();
        dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("norm est {:.2f}", dur);
        return normals;
    }
}