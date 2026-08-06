#pragma once

namespace chad::detail {
    auto get_page_size() -> size_t;
    auto allocate_virtual(size_t bytes) -> void*;
    void deallocate_virtual(void* virtual_p, size_t bytes);
    void prefault_virtual(void* virtual_p, size_t bytes);

    auto inline align(std::size_t size, std::size_t alignment) -> std::size_t {
        return (size + (alignment - 1)) & ~(alignment - 1);
    }
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

        template<class InputIt>
        void inline insert_back(InputIt first, InputIt last) {
            for (InputIt cur = first; cur != last; std::next(cur)) {
                _virtual_p = *cur;
            }
            _size += std::distance(first, last);
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

        auto inline size() const -> std::size_t {
            return _size;
        }
        auto inline capacity() const -> std::size_t {
            // since _capacity does not get updated on every push/insert,
            // we may need to use _size instead to know how much physical memory is currently populated
            if (_capacity > _size) return _capacity;
            else return align(_size * sizeof(T), _virtual_page_size) / sizeof(T);
        }
        auto inline capacity_virtual() const -> std::size_t {
            return _virtual_capacity;
        }
        void inline resize(std::size_t new_size) {
            // if (_capacity < new_size) reserve(new_size);
            _size = new_size;
        }
        // TODO: actually free the pages if smaller than before? (shrink_to_fit)
        // TODO: and also needs a virtual reserve variant
        // prefault memory pages for later use
        void inline reserve(std::size_t new_capacity) {
            // calc start and end of last populated memory page
            std::size_t page_end = capacity() * sizeof(T);
            std::size_t page_beg = (page_end > 0) ? (page_end - _virtual_page_size) : 0;

            // check if new_capacity will actually require more pages to be populated
            std::size_t page_end_new = align(new_capacity * sizeof(T), _virtual_page_size);
            if (page_end_new > page_end) {
                prefault_virtual(_virtual_p + page_beg, page_end_new - page_beg);
                _capacity = page_end_new / sizeof(T);
            }
            else {
                // effectively just update _capacity to what the current last populated page is
                // _capacity = page_end / sizeof(T);
            }
        }
        // TODO: actually free the pages?
        void inline clear() {
            // MADV_FREE stuff
            _size = 0;
        }

    private:
        T* const _virtual_p;
        const std::size_t _virtual_capacity; // maximum virtual capacity for T (not bytes!)
        const std::size_t _virtual_page_size; // page size in bytes // TODO: respect page size for resizes/reserves, TODO2: use resize/reserve during pushback functions
        std::size_t _size;
        std::size_t _capacity; // only updated during reserve() or capacity(), not in every push/emplace function
    };
}
