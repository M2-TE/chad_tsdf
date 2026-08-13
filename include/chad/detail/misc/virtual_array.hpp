#pragma once

namespace chad::detail {
    auto get_page_size() -> size_t;
    auto allocate_virtual(size_t bytes) -> void*;
    void deallocate_virtual(void* virtual_p, size_t bytes);
    void prefault_virtual(void* virtual_p, size_t bytes);
    void free_virtual(void* virtual_p, size_t bytes);
}

namespace chad::detail {
    static constexpr std::size_t KiB = 1024;
    static constexpr std::size_t MiB = 1024 * KiB;
    static constexpr std::size_t GiB = 1024 * MiB;

    template<typename T> struct VirtualArray {
        using const_iterator = const T*;
        using iterator = T*;

        // defaults to a maximum capacity of 16 GiB
        VirtualArray(std::size_t virtual_capacity = 16 * GiB / sizeof(T)):
            _virtual_p(static_cast<T*>(allocate_virtual(virtual_capacity * sizeof(T)))),
            _virtual_capacity(virtual_capacity),
            _virtual_page_size(get_page_size()),
            _size(0),
            _capacity(0) {
            #ifndef NDEBUG
            // check if T is aligned to page size
            if (_virtual_page_size % sizeof(T) > 0) {
                std::string msg = fmt::format("VirtualArray: sizeof(T) == {} bytes is not aligned to page size of {} bytes", sizeof(T), _virtual_page_size);
                throw std::runtime_error(msg);
            }
            #endif
        }
        ~VirtualArray() {
            deallocate_virtual(_virtual_p, _virtual_capacity * sizeof(T));
        }

        auto inline operator[](std::size_t index) const -> const T& {
            #ifndef NDEBUG
            if (index >= _size) throw std::runtime_error("VirtualArray: Index out of bounds.");
            #endif
            return _virtual_p[index];
        }
        auto inline operator[](std::size_t index) -> T& {
            #ifndef NDEBUG
            if (index >= _size) throw std::runtime_error("VirtualArray: Index out of bounds.");
            #endif
            return _virtual_p[index];
        }
        auto inline push_back(const T& value) -> T& {
            _virtual_p[_size] = value;
            return _virtual_p[_size++];
        }
        auto inline push_back(T&& value) -> T& {
            _virtual_p[_size] = value;
            return _virtual_p[_size++];
        }

        void inline insert_back(std::initializer_list<T> ilist) {
            std::memcpy(_virtual_p + _size, ilist.data(), ilist.size() * sizeof(T));
            _size += ilist.size();
        }

        auto inline front() const -> const T& {
            return _virtual_p[0];
        }
        auto inline front() -> T& {
            return _virtual_p[0];
        }
        auto inline back() const -> const T& {
            return _virtual_p[_size - 1];
        }
        auto inline back() -> T& {
            return _virtual_p[_size - 1];
        }
        auto inline data() const -> const T* {
            return _virtual_p;
        }
        auto inline data() -> T* {
            return _virtual_p;
        }

        auto inline cbegin() const -> const_iterator {
            return _virtual_p;
        }
        auto inline cend() const -> const_iterator {
            return _virtual_p + _size;
        }
        auto inline begin() -> iterator {
            return _virtual_p;
        }
        auto inline end() -> iterator {
            return _virtual_p + _size;
        }

        // returns the number of stored elements
        auto inline size() const -> std::size_t {
            return _size;
        }
        // returns the element capacity (based on populated memory pages)
        auto inline capacity() const -> std::size_t {
            // since _capacity does not get updated on every push/insert,
            // we may need to use _size instead to know how much physical memory is currently populated
            if (_capacity > _size) return _capacity;
            else return align(_size * sizeof(T), _virtual_page_size) / sizeof(T);
        }
        // returns the maximum element capacity that cannot grow
        auto inline capacity_virtual() const -> std::size_t {
            return _virtual_capacity;
        }

        // resize to given element count and optionally prefault memory pages
        void inline resize(std::size_t new_size, bool prefault_pages = false) {
            if (prefault_pages && new_size > _capacity) {
                reserve(new_size);
            }
            _size = new_size;
        }
        // reserve space for a number of elements, prefaulting pages if necessary
        void inline reserve(std::size_t new_capacity) {
            // calc start and end of last populated memory page in bytes
            std::size_t last_page_end = capacity() * sizeof(T);
            std::size_t last_page_beg = (last_page_end > 0) ? (last_page_end - _virtual_page_size) : (0);

            // check whether we need to prefault or free pages
            std::size_t new_page_end = align(new_capacity * sizeof(T), _virtual_page_size);
            if (new_page_end >= last_page_end) {
                prefault_virtual(_virtual_p + last_page_beg, new_page_end - last_page_beg);
            }
            _capacity = new_page_end / sizeof(T);
        }
        // free memory pages to fit current size
        void inline shrink_to_fit() {
            // calc end of last populated memory page in bytes
            std::size_t last_page_end = capacity() * sizeof(T);
            std::size_t new_page_end = align(_size * sizeof(T), _virtual_page_size);
            // free the leftover pages
            free_virtual(_virtual_p + new_page_end, last_page_end - new_page_end);
            _capacity = new_page_end;
        }
        // reset the array without and optionally free allocated pages
        void inline clear(bool free_pages = false) {
            if (free_pages) {
                free_virtual(_virtual_p, capacity());
                _capacity = 0;
            }
            _size = 0;
        }

    private:
        auto inline static align(std::size_t size, std::size_t alignment) noexcept -> std::size_t {
            return (size + (alignment - 1)) & ~(alignment - 1);
        }

    private:
        T* const _virtual_p;
        const std::size_t _virtual_capacity; // maximum virtual capacity for T (not bytes!)
        const std::size_t _virtual_page_size; // page size in bytes // TODO: respect page size for resizes/reserves, TODO2: use resize/reserve during pushback functions
        std::size_t _size;
        std::size_t _capacity; // only updated during reserve() or capacity(), not in every push/emplace function
    };
}
