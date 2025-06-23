#include <chrono>
#include <fmt/base.h>
#include "chad/tsdf.hpp"
#include "chad/detail/lvr2.hpp"
#include "chad/detail/levels.hpp"
#include "chad/detail/morton.hpp"
#include "chad/detail/octree.hpp"
#include "chad/detail/submap.hpp"
#include "chad/detail/normals.hpp"

namespace chad::detail {
    void inline print_vec(glm::vec3 vec) {
        fmt::println("{:.2f} {:.2f} {:.2f}", vec.x, vec.y, vec.z);
    }
    void inline print_vec(glm::aligned_vec3 vec) {
        fmt::println("{:.4f} {:.4f} {:.4f}", vec.x, vec.y, vec.z);
    }
    void inline print_vec(glm::ivec3 vec) {
        fmt::println("{:5} {:5} {:5}", vec.x, vec.y, vec.z);
    }
    void inline print_vec(glm::aligned_ivec3 vec) {
        fmt::println("{:5} {:5} {:5}", vec.x, vec.y, vec.z);
    }
    // TODO: make an iterator
}

namespace chad {
    TSDFMap::TSDFMap(float sdf_res, float sdf_trunc): _sdf_res(sdf_res), _sdf_trunc(sdf_trunc) {
        _active_octree_p = new detail::Octree();
        _active_submap_p = new detail::Submap();
        _node_levels_p = new detail::NodeLevels();
    }
    TSDFMap::~TSDFMap() {
        delete _node_levels_p;
        delete _active_octree_p;
        for (detail::Submap* submap_p: _submaps) {
            delete submap_p;
        }
    }
    void TSDFMap::insert(const std::vector<std::array<float, 3>>& points, const std::array<float, 3>& position) {
        using namespace detail;
        auto beg = std::chrono::high_resolution_clock::now();

        // turn float array into usable vector
        const glm::vec3 position_vec { position[0], position[1], position[2] };

        // either update submap or create a new one
        auto& positions = _active_submap_p->positions;
        if (positions.empty()) positions.push_back(position_vec);
        else {
            // finalize active submap once traversed far enough
            glm::vec3 start = positions.front();
            if (glm::distance(position_vec, start) > 5.0f) {
                _active_submap_p->finalize(*_active_octree_p, *_node_levels_p, _sdf_trunc);
                _submaps.push_back(_active_submap_p);
                _active_submap_p = new Submap();
                _active_submap_p->positions.push_back(position_vec);
                _active_octree_p->clear();
            }
            // else just update active submap
            else positions.push_back(position_vec);
        }

        // sort points by their morton code, discretized to the voxel resolution
        MortonVector points_mc = calc_morton_vector(points, _sdf_res);
        std::vector<glm::vec3> points_sorted = sort_morton_vector(points_mc);
        // estimate the normal of every point
        std::vector<glm::vec3> normals = estimate_normals(points_mc, position_vec);

        // insert points into octree with signed distances
        _active_octree_p->insert(points_sorted, normals, position_vec, _sdf_res, _sdf_trunc);

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("total    {:.2f}", dur);
    }
    void TSDFMap::save(const std::string& filename) {
        // finalize current active submap
        if (!_active_submap_p->positions.empty()) {
            _active_submap_p->finalize(*_active_octree_p, *_node_levels_p, _sdf_trunc);
            _submaps.push_back(_active_submap_p);
        }

        // reconstruct 3D mesh using LVR2
        fmt::println("reconstructing the first submap");
        detail::reconstruct(*_submaps.front(), *_node_levels_p, _sdf_res, _sdf_trunc, filename);
    }
}