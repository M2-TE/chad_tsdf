namespace chad::detail {
    auto allocate_virtual(size_t bytes) -> void*;
    void deallocate_virtual(void* virtual_p, size_t bytes);
    void prefault_virtual(void* virtual_p, size_t bytes);
}

#if defined (__unix__)
#   include <sys/mman.h>
#   include <unistd.h>
#   include <errno.h>
    namespace chad::detail {
        auto allocate_virtual(size_t bytes) -> void* {
            // allocate memory pages without file-backing via mmap, but do not populate them
            int prot = PROT_READ | PROT_WRITE;
            int flags = MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE /*| MAP_HUGETLB*/;
            void* virtual_p = mmap(nullptr, bytes, prot, flags, -1, 0);
            if (virtual_p == MAP_FAILED) {
                std::string err_message = "chad::detail::allocate_virtual -> mmap failed: ";
                err_message += std::strerror(errno);
                throw std::runtime_error(err_message);
            }
            // MADV_DONTDUMP: these gigantic memory regions should not be included in core dumps
            // MADV_HUGEPAGE: allow use of transparent huge pages
            int res = madvise(virtual_p, bytes, MADV_DONTDUMP | MADV_HUGEPAGE); // TODO: benchmark MADV_SEQUENTIAL against MADV_RANDOM
            if (res == -1) {
                std::string err_message = "chad::detail::allocate_virtual -> madvise with MADV_DONTDUMP and MADV_HUGEPAGE failed: ";
                err_message += std::strerror(errno);
                throw std::runtime_error(err_message);
            }
            return virtual_p;
        }
        void deallocate_virtual(void* virtual_p, size_t bytes) {
            int res = munmap(virtual_p, bytes);
            if (res == -1) {
                std::string err_message = "chad::detail::deallocate_virtual -> munmap failed: ";
                err_message += std::strerror(errno);
                throw std::runtime_error(err_message);
            }
        }
        void prefault_virtual(void* virtual_p, size_t bytes) {
            int res = madvise(virtual_p, bytes, MADV_POPULATE_READ | MADV_POPULATE_WRITE);
            if (res == -1) {
                std::string err_message = "chad::detail::prefault_virtual -> madvise with MADV_POPULATE_* failed: ";
                err_message += std::strerror(errno);
                throw std::runtime_error(err_message);
            }
        }
        // TODO: the idea is to release pages that are no longer needed, not the entire memory range like with deallocate_virtual()
        // void free_virtual(void* virtual_p, size_t bytes) {
        //     size_t page_size = getpagesize();

        //     int res = madvise(virtual_p, bytes, MADV_FREE);
        //     if (res == -1) {
        //         std::string err_message = "chad::detail::free_virtual -> madvise with MADV_FREE failed: ";
        //         err_message += std::strerror(errno);
        //         throw std::runtime_error(err_message);
        //     }
        // }
    }
#elif defined(_WIN32) || defined(_WIN64)
#   error "Windows is not yet supported. Implement chad::detail::allocate_virtual(size_t)->void*, chad::detail::deallocate_virtual(void*, size_t)->void and chad::detail::prefault_virtual(void*, size_t)->void."
#elif
#   error "Unknown platform. Implement chad::detail::allocate_virtual(size_t)->void*, chad::detail::deallocate_virtual(void*, size_t)->void and chad::detail::prefault_virtual(void*, size_t)->void."
#endif
