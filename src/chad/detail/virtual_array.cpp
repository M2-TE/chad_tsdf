#include <iostream>
#if defined(linux)
#   include <sys/mman.h>
#   include <errno.h>
#elif defined(_WIN32) || defined(_WIN64)
#   error "not yet implemented for windows"
#endif
#include "chad/detail/virtual_array.hpp"

namespace chad::detail {
    auto allocate_virtual(size_t virtual_capacity) -> void* {
        int prot = PROT_READ | PROT_WRITE;
        int flags = MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE /*| MAP_HUGETLB | MAP_UNINITIALIZED*/;
        void* mem_p = mmap(nullptr, virtual_capacity, prot, flags, -1, 0);
        if (mem_p == MAP_FAILED) std::cout << "virtual memory mapping failed with errno: " << errno << std::endl;
        return mem_p;
    }
    void deallocate_virtual(void* virt_mem_p, size_t virtual_capacity) {
        int res = munmap(virt_mem_p, virtual_capacity);
        if (res == -1) std::cout << "virtual memory unmapping failed with errno: " << errno << std::endl;
    }
}