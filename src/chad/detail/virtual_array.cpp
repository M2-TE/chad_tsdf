#include <string>
#include <cstring>
#include <stdexcept>
#if defined(linux)
#   include <sys/mman.h>
#   include <errno.h>
#elif defined(_WIN32) || defined(_WIN64)
#   error "not yet implemented for windows"
#endif

namespace chad::detail {
    auto allocate_virtual(size_t virtual_capacity) -> void* {
        int prot = PROT_READ | PROT_WRITE;
        int flags = MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE /*| MAP_HUGETLB*/;
        void* mem_p = mmap(nullptr, virtual_capacity, prot, flags, -1, 0);
        if (mem_p == MAP_FAILED) {
            std::string err_message = "chad::detail::allocate_virtual -> mmap failed: ";
            err_message += std::strerror(errno);
            throw std::runtime_error(err_message);
        }
        return mem_p;
    }
    void deallocate_virtual(void* virt_mem_p, size_t virtual_capacity) {
        int res = munmap(virt_mem_p, virtual_capacity);
        if (res == -1) {
            std::string err_message = "chad::detail::deallocate_virtual -> munmap failed: ";
            err_message += std::strerror(errno);
            throw std::runtime_error(err_message);
        }
    }
}