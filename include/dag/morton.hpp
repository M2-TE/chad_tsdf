#pragma once
#include "glm/geometric.hpp"
#include <cstddef>
#include <vector>
#include <chrono>
#include <utility>
#include <cstdint>
#include <execution>
#include <fmt/base.h>
#include <glm/glm.hpp>
#include <parallel_hashmap/phmap.h>
#include <morton-nd/mortonND_BMI2.h>
#include "dag/normals.hpp"

struct MortonCode {
    MortonCode(glm::ivec3 vox_pos): _code(encode(vox_pos)) {}
    MortonCode(uint64_t code): _code(code) {}
    
    auto static inline encode(glm::ivec3 vox_pos) -> uint64_t {
        // truncate from two's complement 32-bit to 21-bit integer
        uint32_t x, y, z;
        x = (1 << 20) + (uint32_t)vox_pos.x;
        y = (1 << 20) + (uint32_t)vox_pos.y;
        z = (1 << 20) + (uint32_t)vox_pos.z;
        return mortonnd::MortonNDBmi_3D_64::Encode(x, y, z);
    }
    auto static inline decode(uint64_t code) -> glm::ivec3{
        auto [x, y, z] = mortonnd::MortonNDBmi_3D_64::Decode(code);
        x -= 1 << 20;
        y -= 1 << 20;
        z -= 1 << 20;
        return { (int32_t)x, (int32_t)y, (int32_t)z };
    }
    auto inline decode() -> glm::ivec3 {
        return decode(_code);
    }
    bool inline operator==(const MortonCode& other) const { return _code == other._code; }
    bool inline operator!=(const MortonCode& other) const { return _code != other._code; }
    bool inline operator<(const MortonCode& other) const { return _code < other._code; }
    bool inline operator>(const MortonCode& other) const { return _code > other._code; }
    uint64_t _code;
};

static auto morton_code_calc(std::vector<glm::vec3>& points) -> std::vector<std::pair<MortonCode, glm::vec3>> {
    auto beg = std::chrono::high_resolution_clock::now();

    // create a vector to hold sortable morton codes alongside position
    std::vector<std::pair<MortonCode, glm::vec3>> morton_codes;
    morton_codes.reserve(points.size());

    // iterate through all points to populate morton code vector
    for (auto points_it = points.cbegin(); points_it != points.cend(); points_it++) {
        glm::vec3 position = *points_it;
        // convert to voxel coordinate
        glm::vec3 position_chunk = position * (float)(1.0 / LEAF_RESOLUTION);
        // discretize using floor()
        position_chunk = glm::floor(position_chunk);
        // create morton code and insert into vector
        morton_codes.emplace_back((glm::ivec3)position_chunk, position);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("mort calc {:.2f}", dur);
    return morton_codes;
}
static void morton_code_sort(std::vector<glm::vec3>& points, std::vector<std::pair<MortonCode, glm::vec3>>& morton_codes) {
    auto beg = std::chrono::steady_clock::now();
    
    // sort morton code vector
    auto sorter = [](const auto& a, const auto& b){
        return std::get<0>(a) > std::get<0>(b);
    };
    std::sort(std::execution::par_unseq, morton_codes.begin(), morton_codes.end(), sorter);
    
    // insert sorted morton code points back into points vector
    auto points_it = points.begin();
    for (auto morton_it = morton_codes.cbegin(); morton_it != morton_codes.cend(); morton_it++, points_it++) {
        *points_it = std::get<1>(*morton_it);
    }
    
    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("pnts sort {:.2f}", dur);
}
static auto morton_code_normals(std::vector<std::pair<MortonCode, glm::vec3>>& morton_codes, glm::vec3 pose_pos) -> std::vector<glm::vec3> {
    auto beg = std::chrono::steady_clock::now();
    
    typedef std::vector<std::pair<MortonCode, glm::vec3>>::const_iterator MortonIt;
    struct Neighbourhood {
        MortonIt beg_it;
        MortonIt end_it;
    };
    // lambda function to get a map of neighbourhoods
    auto fnc_get_neigh_map = [&morton_codes](std::size_t neigh_level) {
        // the level up to which is checked to see if two morton codes belong to the same neighbourhood
        phmap::flat_hash_map<typeof(MortonCode::_code), Neighbourhood> neigh_map;

        // create mask to group morton codes up to a certain level
        const std::size_t mask = std::numeric_limits<std::size_t>::max() << neigh_level * 3;
        MortonCode neigh_mc = morton_codes.front().first;
        neigh_mc._code &= mask;
        // insert the first neighbourhood
        auto neigh_it = neigh_map.emplace(neigh_mc._code, Neighbourhood{morton_codes.cbegin(), morton_codes.cbegin()}).first;

        // iterate through all points to build neighbourhoods
        for (auto morton_it = morton_codes.cbegin() + 1; morton_it != morton_codes.cend(); morton_it++) {
            // get morton codes from current point and current neighbourhood
            MortonCode point_mc = std::get<0>(*morton_it);
            MortonCode neigh_mc = std::get<0>(*neigh_it->second.beg_it);
            point_mc._code &= mask;
            neigh_mc._code &= mask;

            // check if the current point doesnt fit the neighbourhood
            if (point_mc != neigh_mc) {
                // set past-the-end iterator for current neighbourhood
                neigh_it->second.end_it = morton_it;
                // create a new neighbourhood starting at current point
                neigh_it = neigh_map.emplace(point_mc._code, Neighbourhood(morton_it, morton_it)).first;
            }
        }
        // set end() iterator for final neighbourhood
        neigh_it->second.end_it = morton_codes.cend();
        return neigh_map;
    };

    // use chunk at level 2 (relative from leaves) for neighbourhoods by default
    static std::size_t neigh_level = 2;
    auto neigh_map = fnc_get_neigh_map(neigh_level);
    // rough approximation of points per neighbourhood
    std::size_t pts_per_neigh = morton_codes.size() / neigh_map.size();
    // decrease neigh level if too many points per neighbourhood are present
    while (pts_per_neigh > 50) {
        neigh_level--;
        neigh_map = fnc_get_neigh_map(neigh_level);
        // rough approximation of points per neighbourhood
        pts_per_neigh = morton_codes.size() / neigh_map.size();
        fmt::println("decreased normal neighbourhood level to: {}", neigh_level);
    }
    // increase neigh level until the desired amount of points per neighbourhood is reached
    while (pts_per_neigh < 6) {
        neigh_level++;
        neigh_map = fnc_get_neigh_map(neigh_level);
        // rough approximation of points per neighbourhood
        pts_per_neigh = morton_codes.size() / neigh_map.size();
        fmt::println("increased normal neighbourhood level to: {}", neigh_level);
    }

    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("neig calc {:.2f}", dur);
    beg = std::chrono::steady_clock::now();

    // approximate normals using local neighbourhoods
    const float dist_max = LEAF_RESOLUTION * (1 << neigh_level);
    std::vector<glm::vec3> normals { morton_codes.size() };
    // create lambda function for norm approximations
    typedef decltype(neigh_map)::const_iterator NeighIt;
    auto fnc_approx_norms = [&normals, &neigh_map, &morton_codes, pose_pos, dist_max](NeighIt beg, NeighIt end) {
        for (auto neigh_it = beg; neigh_it != end; neigh_it++) {
            // get morton code for current neighbourhood
            MortonCode neigh_mc = neigh_it->first;
            const Neighbourhood& neigh = neigh_it->second;
            glm::ivec3 neigh_pos = neigh_mc.decode();

            // gather adjacent neighbourhoods
            std::vector<Neighbourhood*> adj_neighs;
            for (int32_t z = -1; z <= +1; z++) {
                for (int32_t y = -1; y <= +1; y++) {
                    for (int32_t x = -1; x <= +1; x++) {\
                        // offset based on neighbourhood level
                        glm::ivec3 offset { x, y, z };
                        offset *= 1 << neigh_level;
                        MortonCode near_mc { neigh_pos + offset };
                        // attempt to find adjacent neighbourhood in map
                        auto near_it = neigh_map.find(near_mc._code);
                        if (near_it != neigh_map.cend()) {
                            adj_neighs.push_back(&near_it->second);	
                        }
                    }
                }
            }

            // count total points in all adjacent neighbourhoods
            std::size_t point_count = 0;
            for (auto neigh_it = adj_neighs.cbegin(); neigh_it != adj_neighs.cend(); neigh_it++) {
                point_count += (**neigh_it).end_it - (**neigh_it).beg_it;
            }

            // collect all points in all adjacent neighbourhoods
            std::vector<glm::vec3> points_local { point_count };
            auto points_local_it = points_local.begin();
            for (auto it_neigh = adj_neighs.cbegin(); it_neigh != adj_neighs.cend(); it_neigh++) {
                // go over points in this neighbourhood
                for (auto it_point = (**it_neigh).beg_it; it_point != (**it_neigh).end_it; it_point++) {
                    *points_local_it = it_point->second;
                    points_local_it++;
                }
            }

            //  reserve some generous space for nearest points
            std::vector<glm::dvec4> nearest_points;
            nearest_points.reserve(point_count);
            // for every point within the central neighbourhood, find its nearest neighbours for normal calc
            for (auto point_it = neigh.beg_it; point_it != neigh.end_it; point_it++) {
                // go over all points and store the nearest ones (within bounds)
                for (auto other_it = points_local.cbegin(); other_it != points_local.cend(); other_it++) {
                    glm::vec3 diff = *other_it - point_it->second;
                    float dist_sqr = glm::dot(diff, diff);
                    // add point if it falls within bounds
                    if (dist_sqr <= dist_max * dist_max) {
                        nearest_points.emplace_back(other_it->x, other_it->y, other_it->z, 0.0);
                    }
                }

                // use these filtered nearest points to approximate the normal
                glm::vec3 normal;
                if (nearest_points.size() >= 3) {
                    normal = approximate_normal(nearest_points);
                    // flip normal if needed
                    float normal_dot = glm::dot(normal, point_it->second - pose_pos);
                    if (normal_dot < 0.0f) normal = -normal;
                }
                else normal = glm::normalize(point_it->second - pose_pos);

                // figure out the index of the normal and write to shared vector
                std::size_t normal_idx = point_it - morton_codes.cbegin();
                normals[normal_idx] = normal;
            }
        }
    };
    // balance load across several threads using lambda invocation
    std::vector<std::thread> threads;
    std::size_t thread_count = std::thread::hardware_concurrency();
    threads.reserve(thread_count);
    for (std::size_t thread_i = 0; thread_i < threads.capacity(); thread_i++) {
        // neighbours per thread
        std::size_t neighs_per_thread = neigh_map.size() / threads.capacity();
        // prep iterators
        NeighIt beg = neigh_map.cbegin();
        std::advance(beg, thread_i * neighs_per_thread);
        NeighIt end;
        if (thread_i == threads.capacity() - 1) {
            end = neigh_map.cend();
        }
        else {
            end = beg;
            std::advance(end, neighs_per_thread);
        }
        threads.emplace_back(fnc_approx_norms, beg, end);
    }

    // join all threads
    for (auto& thread: threads) thread.join();

    end = std::chrono::steady_clock::now();
    dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("norm calc {:.2f}", dur);
    return normals;
}