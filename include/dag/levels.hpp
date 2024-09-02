#pragma once
#include <bit>
#include <vector>
#include <cstdint>
#include <cstring>
#include <parallel_hashmap/phmap.h>
#include "dag/leaf_cluster.hpp"

class NodeLevel {
    struct HashFnc {
        HashFnc(std::vector<uint32_t>& raw_data): _raw_data(raw_data) {}
        inline uint64_t operator()(uint32_t key) const noexcept {
            // count children
#           ifdef CHAD_POPCOUNT_INSTRUCTION
                uint8_t nChildren = CHAD_POPCOUNT_INSTRUCTION(_raw_data[key]);
#           else
                static_assert(false, "popcount instruction not available");
#           endif
            // hash entire node
            uint64_t hash = 0;
            for (uint8_t i = 1; i <= nChildren; i++) {
                // TODO: adjust hash, this is a much more heavy weight combiner than is necessary
                hash = phmap::HashState::combine(hash, _raw_data[key + i]);
            }
            return hash;
        }
        std::vector<uint32_t>& _raw_data; // non-owning pointer to raw data array
    };
    struct CompFnc {
        CompFnc(std::vector<uint32_t>& raw_data): _raw_data(raw_data) {}
        inline bool operator()(uint32_t key_a, uint32_t key_b) const noexcept {
            // compare child masks
            if ((uint8_t)_raw_data[key_a] != (uint8_t)_raw_data[key_b]) return false;

            // count children
            // TODO: count children of a vs b? could be a performance improvement
#           ifdef CHAD_POPCOUNT_INSTRUCTION
                uint8_t nChildren = CHAD_POPCOUNT_INSTRUCTION(_raw_data[key_a]);
#           else
                static_assert(false, "popcount instruction not available");
#           endif
            // compare entire node
            int cmp = std::memcmp(
                &_raw_data[key_a + 1],
                &_raw_data[key_b + 1],
                nChildren * sizeof(uint32_t));
            return cmp == 0;
        }
        std::vector<uint32_t>& _raw_data; // non-owning pointer to raw data array
    };
public:
    NodeLevel(): 
        _lookup_set(0, HashFnc(_raw_data), CompFnc(_raw_data)),
        _raw_data(1),
        _occupied_count(1),
        _unique_count(0),
        _dupe_count(0) {}
    // TODO: 8 separate hash sets for different node sizes?
    phmap::flat_hash_set<uint32_t, HashFnc, CompFnc> _lookup_set;
    std::vector<uint32_t> _raw_data;
    uint32_t _occupied_count;
    // debug trackers
    uint32_t _unique_count;
    uint32_t _dupe_count;
};
struct LeafLevel {
    LeafLevel(): 
        _lookup_map(),
        _raw_data(1),
        _unique_count(0),
        _dupe_count(0) {}
#if LEAF_BITS == 8
    typedef uint64_t ClusterValue;
#elif LEAF_BITS == 4
    typedef uint32_t ClusterValue;
#endif
    phmap::flat_hash_map<ClusterValue, uint32_t> _lookup_map;
    std::vector<ClusterValue> _raw_data;
    // debug trackers
    uint32_t _unique_count;
    uint32_t _dupe_count;
};