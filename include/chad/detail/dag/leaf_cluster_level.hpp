#pragma once
#include "chad/cluster.hpp"
#include "chad/detail/dag/node.hpp"
#include "chad/detail/misc/virtual_array.hpp"

namespace chad::detail::dag {
    struct LeafClusterLevel {
        struct FncHash {
            FncHash(const VirtualArray<LeafCluster>& leaf_clusters): _leaf_clusters(leaf_clusters) {}
            auto inline operator()(ADDR_T addr) const noexcept -> std::uint64_t {
                // use the 64-bit value of the leaf cluster as the hash
                return _leaf_clusters[addr]._value;
            }
            const VirtualArray<LeafCluster>& _leaf_clusters;
        };
        struct FncEq {
            FncEq(const VirtualArray<LeafCluster>& leaf_clusters): _leaf_clusters(leaf_clusters) {}
            auto inline operator()(ADDR_T addr_a, ADDR_T addr_b) const noexcept -> bool {
                // compare leaf cluster values directly
                return _leaf_clusters[addr_a]._value == _leaf_clusters[addr_b]._value;
            }
            const VirtualArray<LeafCluster>& _leaf_clusters;
        };

        LeafClusterLevel(): _addr_set(0, FncHash(_leaf_clusters), FncEq(_leaf_clusters)) {
            clear();
        }
        void clear() {
            _dupes_n = 0;
            _uniques_n = 0;
            _addr_set.clear();
            _leaf_clusters.clear();
            // reserve first index
            _leaf_clusters.push_back(LeafCluster{});
        }

        std::uint32_t _uniques_n, _dupes_n; // for statistics
        VirtualArray<LeafCluster> _leaf_clusters; // leaf cluster data
        gtl::parallel_flat_hash_set<ADDR_T, FncHash, FncEq> _addr_set; // set of addresses
    };
}
