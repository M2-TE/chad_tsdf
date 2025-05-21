#pragma once
#include <cstdint>
#include <gtl/phmap.hpp>
#include "chad/leaf_cluster.hpp"
#include "chad/virtual_array.hpp"

namespace chad{
    using NodeAddress = uint32_t;
    struct NodeLevel {
        struct FncHash {
            FncHash(const VirtualArray<uint32_t>& node_data): _node_data(node_data) {
            }
            auto inline operator()(const NodeAddress addr) const noexcept -> uint64_t {
                // count children
                uint8_t child_count = std::popcount(uint8_t(_node_data[addr]));
                // hash entire node
                uint64_t hash = 0;
                for (uint8_t i = 1; i <= child_count; i++) {
                    hash = gtl::HashState::combine(hash, _node_data[addr + i]);
                }
                return hash;
            }
            const VirtualArray<uint32_t>& _node_data;
        };
        struct FncEq {
            FncEq(const VirtualArray<uint32_t>& node_data): _node_data(node_data) {
            }
            bool inline operator()(const NodeAddress addr_a, const NodeAddress addr_b) const noexcept {
                // compare child masks
                uint8_t child_mask_a = uint8_t(_node_data[addr_a]);
                uint8_t child_mask_b = uint8_t(_node_data[addr_b]);
                if (child_mask_a != child_mask_b) return false;

                // count children
                uint8_t child_count_a = std::popcount(child_mask_a);
                uint8_t child_count_b = std::popcount(child_mask_b);
                if (child_count_a != child_count_b) return false;

                // compare entire nodes
                int cmp = std::memcmp(
                    &_node_data[addr_a + 1],
                    &_node_data[addr_b + 1],
                    child_count_a * sizeof(NodeAddress));
                return cmp == 0;
            }
            const VirtualArray<uint32_t>& _node_data;
        };

        NodeLevel():
                _raw_data(), _count_uniques(0), _count_dupes(0),
                _addr_set(0, FncHash(_raw_data), FncEq(_raw_data)) {
            // reserve first index
            _raw_data.push_back(0);
        }

        VirtualArray<uint32_t> _raw_data; // raw unaligned node data
        uint32_t _count_uniques;
        uint32_t _count_dupes;
        gtl::parallel_flat_hash_set<NodeAddress, FncHash, FncEq> _addr_set; // set of addresses
    };
    struct LeafClusterLevel {
        struct FncHash {
            FncHash(const VirtualArray<LeafCluster>& lc_data): _lc_data(lc_data) {
            }
            auto inline operator()(const NodeAddress addr) const noexcept -> uint64_t {
                // use the 64-bit value of the leaf cluster as the hash
                return _lc_data[addr]._value;
            }
            const VirtualArray<LeafCluster>& _lc_data;
        };
        struct FncEq {
            FncEq(const VirtualArray<LeafCluster>& lc_data): _lc_data(lc_data) {
            }
            bool inline operator()(const NodeAddress addr_a, const NodeAddress addr_b) const noexcept {
                // compare leaf clusters values directly
                return _lc_data[addr_a]._value == _lc_data[addr_b]._value;
            }
            const VirtualArray<LeafCluster>& _lc_data;
        };

        LeafClusterLevel():
                _raw_data(), _count_uniques(0), _count_dupes(0),
                _addr_set(0, FncHash(_raw_data), FncEq(_raw_data)) {
            // reserve first index
            _raw_data.push_back(LeafCluster{});
        }

        VirtualArray<LeafCluster> _raw_data; // leaf cluster data
        uint32_t _count_uniques;
        uint32_t _count_dupes;
        gtl::parallel_flat_hash_set<NodeAddress, FncHash, FncEq> _addr_set; // set of addresses
    };
}