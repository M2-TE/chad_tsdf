#pragma once
#include <cstdint>
#include <gtl/phmap.hpp>
#include "chad/detail/cluster.hpp"
#include "chad/detail/virtual_array.hpp"

namespace chad::detail {
    class NodeLevel {
        struct FncHash {
            FncHash(const VirtualArray<uint32_t>& node_data): _node_data(node_data) {
            }
            auto inline operator()(const uint32_t addr) const noexcept -> uint64_t {
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
            bool inline operator()(const uint32_t addr_a, const uint32_t addr_b) const noexcept {
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
                    child_count_a * sizeof(uint32_t));
                return cmp == 0;
            }
            const VirtualArray<uint32_t>& _node_data;
        };

    public:
        NodeLevel():
                _uniques_n(0), _dupes_n(0), _raw_data(),
                _addr_set(0, FncHash(_raw_data), FncEq(_raw_data)) {
            // reserve first index
            _raw_data.push_back(0);
        }

        uint32_t _uniques_n, _dupes_n;
        VirtualArray<uint32_t> _raw_data; // raw unaligned node data
        gtl::parallel_flat_hash_set<uint32_t, FncHash, FncEq> _addr_set; // set of addresses
    };
    class LeafClusterLevel {
        struct FncHash {
            FncHash(const VirtualArray<LeafCluster>& lc_data): _lc_data(lc_data) {
            }
            auto inline operator()(const uint32_t addr) const noexcept -> uint64_t {
                // use the 64-bit value of the leaf cluster as the hash
                return _lc_data[addr]._value;
            }
            const VirtualArray<LeafCluster>& _lc_data;
        };
        struct FncEq {
            FncEq(const VirtualArray<LeafCluster>& lc_data): _lc_data(lc_data) {
            }
            bool inline operator()(const uint32_t addr_a, const uint32_t addr_b) const noexcept {
                // compare leaf clusters values directly
                return _lc_data[addr_a]._value == _lc_data[addr_b]._value;
            }
            const VirtualArray<LeafCluster>& _lc_data;
        };

    public:
        LeafClusterLevel():
                _uniques_n(0), _dupes_n(0), _raw_data(),
                _addr_set(0, FncHash(_raw_data), FncEq(_raw_data)) {
            // reserve first index
            _raw_data.push_back(LeafCluster{});
        }
        auto add_leaf_cluster(LeafCluster lc) -> uint32_t {
            // append a placeholder node
            uint32_t new_addr = _uniques_n;
            if (_raw_data.size() <= new_addr + 1) {
                _raw_data.push_back(lc);
            }
            else {
                _raw_data.back() = lc;
            }

            // emplace placeholder node if it is a new one
            auto [old_addr_it, is_new] = _addr_set.emplace(new_addr);
            if (is_new) {
                _uniques_n++;
                return new_addr;
            }
            else {
                _dupes_n++;
                return *old_addr_it;
            }
        }

        uint32_t _uniques_n, _dupes_n;
        VirtualArray<LeafCluster> _raw_data; // leaf cluster data
        gtl::parallel_flat_hash_set<uint32_t, FncHash, FncEq> _addr_set; // set of addresses
    };
}