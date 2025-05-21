#if defined(linux)
#   include <sys/mman.h>
#   include <errno.h>
#elif defined(_WIN32) || defined(_WIN64)
#   error "not yet implemented for windows"
#endif
#include <cstdint>
#include <iostream>
#include "chad/virtual_array.hpp"
#include "chad/leaf_cluster.hpp"

namespace chad {
    namespace {
        auto allocate_virt_mem(size_t virtual_capacity) -> void* {
            int prot = PROT_READ | PROT_WRITE;
            int flags = MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE /*| MAP_HUGETLB | MAP_UNINITIALIZED*/;
            void* mem_p = mmap(nullptr, virtual_capacity, prot, flags, -1, 0);
            if (mem_p == MAP_FAILED) std::cout << "virtual memory mapping failed with errno: " << errno << std::endl;
            return mem_p;
        }
    }
    template<> VirtualArray<uint32_t>::VirtualArray(size_t virtual_capacity): _size(0), _capacity(virtual_capacity) {
        _virt_mem_p = reinterpret_cast<uint32_t*>(allocate_virt_mem(virtual_capacity));
    }
    template<> VirtualArray<LeafCluster>::VirtualArray(size_t virtual_capacity): _size(0), _capacity(virtual_capacity) {
        _virt_mem_p = reinterpret_cast<LeafCluster*>(allocate_virt_mem(virtual_capacity));
    }
    template<> VirtualArray<uint32_t>::~VirtualArray() {
        int res = munmap(_virt_mem_p, _capacity);
        if (res == -1) std::cout << "virtual memory unmapping failed with errno: " << errno << std::endl;
    }
    template<> VirtualArray<LeafCluster>::~VirtualArray() {
        int res = munmap(_virt_mem_p, _capacity);
        if (res == -1) std::cout << "virtual memory unmapping failed with errno: " << errno << std::endl;
    }
}