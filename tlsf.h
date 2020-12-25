/// Object-oriented, C++ implementation of the TLSF allocator
/// http://www.gii.upv.es/tlsf/main/docs
/// conforms with std::allocator_traits
#pragma once

#include <new>
#include <stdexcept>

namespace mem {

template <typename T, size_t DefaultPoolSize = 1024 * 1024>
class tlsf_allocator {
    public:
        // using size_type = size_t;
        // using difference_type = ptrdiff_t;
        // using pointer = T*;
        // using const_pointer = const T*;
        // using reference = T&;
        // using const_reference = const T&;
        using value_type = T;

        explicit tlsf_allocator(size_t size) {
            initialize(size);
        }

        template <class U> tlsf_allocator(const tlsf_allocator<U>& alloc) noexcept
        : memory_pool(alloc.memory_pool), pool_size(alloc.pool_size) {} 

        template <class U, size_t OtherDefaultSize> tlsf_allocator(const tlsf_allocator<U>& alloc) noexcept
        : memory_pool(alloc.memory_pool), pool_size(alloc.pool_size) {} 

        tlsf_allocator() noexcept: memory_pool(nullptr) {
            initialize(DefaultPoolSize);
        }

        size_t initialize(size_t size) {
            pool_size = size;
            if (!memory_pool) {
                memory_pool = new char[pool_size];
                memset(memory_pool, 0, pool_size);
                init_memory_pool(pool_size, memory_pool);
            }
            return pool_size;
        }

        ~tlsf_allocator(){
            if (memory_pool){
                destroy_memory_pool(memory_pool);
                memory_pool = nullptr;
            }
        }


        T* allocate(size_t size){
            T* ptr = static_cast<T*>(malloc(size * sizeof(T)));
            if (ptr == NULL && size > 0){
                throw std::bad_alloc();
            }
            return ptr;
        }

        void deallocate(T* ptr, size_t){
            tlsf_free(ptr);
        }

        template<typename U>
        struct rebind{
            using other = tlsf_allocator<U>;
        };

        template<class U, class... Args>
        void construct(U* p, Args&&... args);

        template<class U>
        void destroy(U* p);

        char* memory_pool;
        size_t pool_size;

    private:
        void* malloc(size_t size);
        void* tlsf_memalign(size_t align, size_t bytes);
        void* tlsf_realloc(T* ptr, size_t size);
        void tlsf_free(T* ptr);
        void tlsf_coalesce();

        size_t align_size;
        size_t block_size_min;
        size_t block_size_max;
        size_t pool_overhead;
        size_t tlsf_alloc_overhead;

        void init_memory_pool(size_t size, char* pool);
        void destroy_memory_pool(char* pool);

        
        
};

template<typename T, typename U>
constexpr bool operator==(
    const tlsf_allocator<T>& a,
    const tlsf_allocator<U>& b) noexcept {
        return a.memory_pool == b.memory_pool;
    }

template <typename T, typename U>
constexpr bool operator!=(
    const tlsf_allocator<T>& a,
    const tlsf_allocator<U>& b) noexcept {
        return a.memory_pool != b.memory_pool;
    }
    
template<typename T, typename U, size_t X, size_t Y>
constexpr bool operator==(
    const tlsf_allocator<T,X>& a,
    const tlsf_allocator<U,Y>& b) noexcept {
        return a.memory_pool == b.memory_pool;
    }

template <typename T, typename U, size_t X, size_t Y>
constexpr bool operator!=(
    const tlsf_allocator<T,X>& a,
    const tlsf_allocator<U,Y>& b) noexcept {
        return a.memory_pool != b.memory_pool;
    }
} //namespace mem