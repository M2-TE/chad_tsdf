#pragma once
#include "chad/detail/morton.hpp"

namespace chad::detail {
    // dense spatially hashed octree
    struct Octree3 {
        struct Leaf { float _sd = 0.0f; uint32_t _weight = 0; };
        struct LeafCluster {
            std::array<Leaf, 64> _leaves;
        };

        void clear() {
            _lc_map.clear();
        }
        // insert leaf and/or return it
        auto inline insert(MortonCode mc) -> Leaf& {
            // discretize morton code to match lookup depth to find leaf cluster
            auto [lc_it, lc_emplaced] = _lc_map.try_emplace(mc._value & _LOOKUP_MASK);
            
        }

        // todo: benchmark performance benefits/regressions using "node" version
        gtl::parallel_node_hash_map<uint64_t, LeafCluster> _lc_map;

        private:
        static constexpr uint32_t _LC_MAP_DEPTH = 18;
        static constexpr uint64_t _LOOKUP_SHIFT = (20 - _LC_MAP_DEPTH) * 3; // assuming max depth of 20 (21 levels) and 3 bits per depth
        static constexpr uint64_t _LOOKUP_MASK = ((0xffff'ffff'ffff'ffff - 1) >> _LOOKUP_SHIFT) << _LOOKUP_SHIFT;
    };
}