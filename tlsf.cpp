#include "tlsf.h"
#include <utility>
#include <cstdint>
#include <cstddef>
#include <cstdlib>
#include <climits>
#include <cassert>
#include <cstring>

/**
 * Implementation of TLSF allocator.
 * NOTE: no automatic type deduction used here, for maximum explicitness due to 
 * copious amounts of pointer casting. 
 */


/**
 * Determine architecture bitness.
 */
#if INTPTR_MAX == INT64_MAX
// 64-bit 
#define TLSF_64BIT
#elif INTPTR_MAX == INT32_MAX
//32 bit 
#else
#error Unsupported bitness architecture for TLSF allocator.
#endif

namespace mem {

    #define TLSF_CAST(t, exp) ((t)(exp))
    #define TLSF_MIN(a,b) ((a) < (b) ? (a) : (b))
    #define TLSF_MAX(a,b) ((a) > (b) ? (a) : (b))

    //define assert if not defined elsewhere
    #ifndef TLSF_ASSERT
    #define TLSF_ASSERT assert
    #endif

    // /**
    //  * Macro for static assert
    //  */
    // #define _TLSF_GLUE_IMPL(x,y) x ## y
    // #define _TLSF_GLUE(x,y) _TLSF_GLUE_IMPL(x,y)
    // #define TLSF_STATIC_ASSERT(exp) \
    //     typedef char _TLSF_GLUE(static_assert_at_line_, __LINE__) [(exp)? 1 : -1]


    /**
     * use builtin function to count leading zeroes of a bitmap.
     * also known as "find first bit set" or "ffs"
     * also "find last bit set" or "fls"
     */
    #if defined(__GNUC__) && (__GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 4)) \
        && defined(__GNUC_PATCHLEVEL__)

    static inline int tlsf_ffs(unsigned int word){
        return __builtin_clz(word)-1;
    }

    static inline int tlsf_fls(unsigned int word){
        const int bit = word ? 32 - __builtin_clz(word) : 0; // if word is 0, then clz is undefined
        return bit-1;
    }
    #else 
    //generic implementation
    static inline int tlsf_fls_generic(unsigned int word){
        int bit = 32;

        if (!word) bit -= 1;
        if (!(word & 0xffff0000)) { word <<= 16; bit -= 16;}
        if (!(word & 0xff000000)) { word <<= 8; bit -= 8;}
        if (!(word & 0xf0000000)) { word <<= 4; bit -= 4;}
        if (!(word & 0xc0000000)) { word <<= 2; bit -= 2;}
        if (!(word & 0x80000000)) { word <<= 1; bit -= 1;}

        return bit;
    }

    static inline int tlsf_ffs(unsigned int word){
        return tlsf_fls_generic(word & (~word +1)) -1;
    }

    static inline int tlsf_fls(unsigned int word){
        return tlsf_fls_generic(word)-1;
    }

    #endif 
    
    /**
     * 64-bit version of TLSF fls
     */
    #ifdef TLSF_64BIT
    static inline int tlsf_fls_sizet(size_t size){
        int high = (int)(size >> 32);
        int bits = 0;
        if (high) {
            bits = 32 + tlsf_fls(high);
        } 
        else {
            bits = tlsf_fls((int)size & 0xffffffff);
        }
        return bits; 
    }
    #else 
    #define tlsf_fls_sizet tlsf_fls
    #endif

    /**
     * Parameters of the TLSF data structure
     * TODO: incorporate these into the main class somehow
     * These values are used to find the correct list/block given the requested memory size. 
     */

    /**
     * log2 of number of linear subdivisions of block sizes
     * values of 4-5 typical.
     */
    const int SL_INDEX_COUNT_LOG2 = 5; 
    
    // these values should be private
    #ifdef TLSF_64BIT
    // all allocation sizes are aligned to 8 bytes
    const int ALIGN_SIZE_LOG2 = 3;
    const int FL_INDEX_MAX = 32; //note this means the largest block we can allocate is 2^32 bytes
    #else 
    // all allocation sizes are aligned to 4 bytes
    const int ALIGN_SIZE_LOG2 = 2;
    const int FL_INDEX_MAX = 30;
    #endif
    constexpr int ALIGN_SIZE = (1 << ALIGN_SIZE_LOG2);

    /**
     * Allocations of sizes up to (1 << FL_INDEX_MAX) are supported. 
     * Because we linearly subdivide the second-level lists and the minimum size block 
     * is N bytes, it doesn't make sense to create first-level lists for sizes smaller than
     * SL_INDEX_COUNT * N or (1 << (SL_INDEX_COUNT_LOG2 + log2(N))) bytes, as we will be trying 
     * to split size ranges into more slots than we have available.
     * We calculate the minimum threshold size, and place all blocks below that size into 
     * the 0th first-level list. 
     */

    constexpr int SL_INDEX_COUNT = (1 << SL_INDEX_COUNT_LOG2);
    constexpr int FL_INDEX_SHIFT = (SL_INDEX_COUNT_LOG2+ALIGN_SIZE_LOG2);
    constexpr int FL_INDEX_COUNT = (FL_INDEX_MAX - FL_INDEX_SHIFT + 1);
    constexpr int SMALL_BLOCK_SIZE = (1 << FL_INDEX_SHIFT);


    static_assert(sizeof(int) * CHAR_BIT == 32);
    static_assert(sizeof(size_t) * CHAR_BIT >= 32);
    static_assert(sizeof(size_t) * CHAR_BIT <= 64);

    using tlsfptr_t = ptrdiff_t; 
    
    /**
     * Block header.
     * According to the TLSF specification:
     * - the prev_phys_block field is only valid if the previous block is free.
     * - the prev_phys_block is actually stored at the end of the previous block. This arrangement is to simplify the implementation.
     * - The next_free and prev_free are only valid if the block is free. 
     */
    struct block_header {
        block_header* prev_phys_block;
        size_t size; 
        block_header* next_free;
        block_header* prev_free;

        size_t get_size() const {
            return size & ~(block_header_free_bit | block_header_prev_free_bit);
        }

        void set_size(size_t new_size){
            const size_t oldsize = size;
            
            size = new_size | (oldsize & (block_header_free_bit | block_header_prev_free_bit));
        }

        int is_last() const {
            return get_size() == 0;
        }

        int is_free() const {
            return TLSF_CAST(int, size & block_header_free_bit);
        }

        
        int is_prev_free() const {
            return TLSF_CAST(int, size & block_header_prev_free_bit);
        }

        static block_header* from_void_ptr(const void* ptr){
            return TLSF_CAST(block_header*, TLSF_CAST(unsigned char*, ptr)-block_start_offset);
        }

        void* to_void_ptr() const {
            return TLSF_CAST(void*, TLSF_CAST(unsigned char*, this) + block_start_offset);
        }

        static block_header* offset_to_block( const void* ptr, size_t blk_size) { //could remove ptr and use block->to_void_ptr() instead.
            return TLSF_CAST(block_header*, TLSF_CAST(tlsfptr_t, ptr) + blk_size);
        } 

        block_header* get_next() {
            block_header* next = offset_to_block(to_void_ptr(), get_size()-block_header_overhead);
            TLSF_ASSERT(!is_last());
            return next;
        }

        block_header* link_next(){
            block_header* next = get_next();
            next->prev_phys_block = this;
            return next;
        }

        void mark_as_free(){
            //link to the next block
            block_header* next = link_next();
            next->set_prev_free();
            this->set_free();            
        }

        void mark_as_used(){
            block_header* next = get_next();
            next->set_prev_used();
            this->set_used();
        }

        void set_free() {
            size |= block_header_free_bit;
        }

        void set_used() {
            size &= ~block_header_free_bit;
        }

        void set_prev_free() {
            size |= block_header_prev_free_bit;
        }

        void set_prev_used(){
            size &= block_header_prev_free_bit;
        }
        private:


    };

    /**
     * block sizes are always a multiple of 4.
     * the two least significant bits of the size field are used to store the block status
     * bit 0: whether the block is busy or free
     * bit 1: whether the previous block is busy or free
     * 
     * Block overhead: 
     * The only overhead exposed during usage is the size field. The previous_phys_block field is technically stored
     * inside the previous block. 
     */
    static constexpr size_t block_header_free_bit = 1 << 0;
    static constexpr size_t block_header_prev_free_bit = 1 << 1; 
    static constexpr size_t block_header_overhead = sizeof(size_t);

    /* User data starts after the size field in a used block */
    static constexpr size_t block_start_offset = offsetof(block_header, size) + sizeof(size_t);

    /**  
     * A free block needs to store its header minus the size of the prev_phys_block field,
     * and cannot be larger than the number of addressable bits for FL_INDEX. 
     */
    static constexpr size_t block_size_min = sizeof(block_header)- sizeof(block_header*);
    static constexpr size_t block_size_max = TLSF_CAST(size_t, 1) << FL_INDEX_MAX;

    /** 
     * Control structure for TLSF allocator
     */
    class tlsf_control_block {
        public:
            /* empty lists point to this block to indicate they are free.*/
            static block_header block_null;

            /*bitmaps*/
            unsigned int fl_bitmap;
            unsigned int sl_bitmap[FL_INDEX_COUNT];

            /*head of free lists*/
            block_header* blocks[FL_INDEX_COUNT][SL_INDEX_COUNT];

            tlsf_control_block() {
                int i, j;
                block_null.next_free = &block_null;
                block_null.prev_free = &block_null;

                fl_bitmap = 0;
                for (i = 0; i < FL_INDEX_COUNT; ++i){
                    sl_bitmap[i] = 0;
                    for (j=0; i<SL_INDEX_COUNT; ++j){
                        blocks[i][j] = &block_null;
                    }
                }
            }

            // tlsf_control_block(void* mem) {
            //     if (((tlsfptr_t)mem % ALIGN_SIZE) != 0){
            //         printf("tlsf_create: Memory must be aligned to %u bytes.\n",
            //             (unsigned int)ALIGN_SIZE);
            //         return 0;
            //     }
            //     tlsf_control_block();
            // }


            block_header* search_suitable_block(int* fli, int* sli){
                int fl = *fli;
                int sl = *sli;

                /**
                 * Search for a block in the list associated with the given fl/sl index
                 */
                unsigned int sl_map = sl_bitmap[fl] & (~0U << sl);
                if (!sl_map) {
                    const unsigned int fl_map = fl_bitmap & (~0U << (fl+1));
                    if (!fl_map){
                        /* no free blocks available, memory has been exhausted. */
                        return nullptr;
                    }

                    fl = tlsf_ffs(fl_map);
                    *fli = fl;
                    sl_map = sl_bitmap[fl];

                }
                TLSF_ASSERT(sl_map && "internal error - second level bitmap is null");
                sl = tlsf_ffs(sl_map);
                *sli = sl;

                return blocks[fl][sl];
            }
            
            /*Removes a block from the free-list and updates the bitmaps. */
            void remove_free_block(block_header* block, int fl, int sl){
                block_header* prev = block->prev_free;
                block_header* next = block->next_free;
                TLSF_ASSERT(prev && "prev_free field cannot be null");
                TLSF_ASSERT(next && "next_free field cannot be null");
                next->prev_free = prev;
                prev->next_free = next;

                // if block is head of the free list, set new head
                if (blocks[fl][sl] == block){
                    blocks[fl][sl] = next;

                    //if the new head is null, clear the bitmap
                    if (next == &block_null) {
                        sl_bitmap[fl] &= ~(1U << sl);
                        // if the second bitmap is empty, clear the fl bitmap
                        if (!sl_bitmap[fl]) {
                            fl_bitmap &= ~(1U << fl);
                        }
                    }

                }
            }
            /* Given the fl and sl indices, adds a block to the free-list and updates the bitmaps. */
            void insert_free_block(block_header* block, int fl, int sl){
                block_header* current = blocks[fl][sl];
                TLSF_ASSERT(current && "free list cannot have a null entry");
                TLSF_ASSERT(block && "cannot insert a null entry into the free list");
                block->next_free = current;
                block->prev_free = &block_null;
                current->prev_free = block;

                TLSF_ASSERT(block->to_void_ptr() == align_ptr(block->to_void_ptr(), ALIGN_SIZE) && "block not aligned properly");


                //add block to head of list and update bitmaps
                blocks[fl][sl] = block;
                fl_bitmap |= (1U << fl);
                sl_bitmap[fl] |= (1U << sl);
            }

            void block_remove(block_header* block){
                int fl, sl;
                mapping_insert(block->get_size(), &fl, &sl);
                remove_free_block(block, fl, sl);
            }

            void block_insert(block_header* block) {
                int fl, sl;
                mapping_insert(block->get_size(), &fl, &sl);
                insert_free_block(block, fl, sl);
            }


            static bool block_can_split(block_header* block, size_t size){
                return block->get_size() >= sizeof(block_header)+size;
            }

            /* split a block into two; the second one is free. */
            static block_header* block_split(block_header* block, size_t size){
                block_header* remaining = block->offset_to_block(block->to_void_ptr(), size-block_header_overhead);

                const size_t remain_size = block->get_size() - (size+block_header_overhead);
                TLSF_ASSERT(remaining->to_void_ptr() == align_ptr(remaining->to_void_ptr(), ALIGN_SIZE) 
                    && "remaining block not aligned properly");
                
                TLSF_ASSERT(block->get_size() == remain_size + size + block_header_overhead);
                remaining->set_size(remain_size);
                TLSF_ASSERT(remaining->get_size() >= block_size_min && "block split with invalid (too small) size");

                block->set_size(size);
                remaining->mark_as_free();
                
                return remaining;
            }


            block_header* merge_prev(block_header* block){
                int fl, sl;
                if (block->is_prev_free()){
                    block_header* prev = block->prev_phys_block;
                    TLSF_ASSERT(prev && "prev physical block cannot be null.");
                    TLSF_ASSERT(prev->is_free() && "prev block is not free even though marked as such.");
                    block_remove(prev);
                    block = block_coalesce(prev, block);
                }
                return block;
            }

            block_header* merge_next(block_header* block){
                int fl, sl;
                block_header* next = block->get_next();
                TLSF_ASSERT(next && "next physical block cannot be null.");
                if (next->is_free()){
                    TLSF_ASSERT(!block->is_last() && "previous block cannot be last.");
                    block_remove(next);
                    block = block_coalesce(block, next);
                }
                return block;
            }

            /* Trims off any trailing block space over size, and returns it to the pool. */
            void trim_free(block_header* block, size_t size){
                TLSF_ASSERT(block->is_free() && "block must be free");
                if (block_can_split(block, size)) {
                    block_header* remaining_block = block_split(block, size);
                    block->link_next();
                    remaining_block->set_prev_free();
                    block_insert(remaining_block);
                }
            }

            /*Trims trailing block space off the end of a used block, returns to pool*/
            void trim_used(block_header* block, size_t size){
                TLSF_ASSERT(!block->is_free() && "block must be used.");
                if (block_can_split(block, size)) {

                    // if the next block is free, we must coalesce
                    block_header* remaining_block = block_split(block, size);
                    remaining_block->set_prev_used();
                    remaining_block = merge_next(remaining_block);
                    block_insert(remaining_block);
                }
            }

            block_header* trim_free_leading(block_header* block, size_t size){
                block_header* remaining_block = block;
                if (block_can_split(block, size)){
                    //we want the second block
                    remaining_block = block_split(block, size-block_header_overhead);
                    remaining_block->set_prev_free();

                    block->link_next();
                    block_insert(block);
                }

                return remaining_block;
            }

            block_header* locate_free(size_t size){
                int fl = 0, sl = 0;
                block_header* block = 0;
                if (size){
                    mapping_search(size, &fl, &sl);
                    if (fl <FL_INDEX_COUNT)
                        block = search_suitable_block(&fl, &sl);
                }
                if (block) {
                    TLSF_ASSERT(block->get_size() >= size);
                    remove_free_block(block, fl, sl);
                }

                return block;
            }

            void* prepare_used(block_header* block, size_t size){
                void* p = 0;
                if (block){
                    TLSF_ASSERT(size && "size must be non-zero");
                    trim_free(block, size);
                    block->mark_as_used();
                    p = block->to_void_ptr();
                }
                return p;
            }

            void* malloc(size_t size){
                // int fl, sl;
                // mapping_search(bytes, &fl, &sl);
                // block_header* block = search_suitable_block(&fl, &sl);
                // TLSF_ASSERT(block->is_free() && "Block must be free.");
                // block_remove(block);
                // if (block_can_split(block, bytes)){
                //     block_header* remaining_block = block_split(block, bytes);
                //     block_insert(remaining_block);
                // }

                const size_t adjust = adjust_request_size(size, ALIGN_SIZE);
                block_header* block = locate_free(size);

                return prepare_used(block, adjust);
            }

            /*Deallocates the block and returns it to the pool.*/
            void free(void* ptr){
                if (ptr){
                    block_header* block = block->from_void_ptr(ptr);
                    TLSF_ASSERT(!block->is_free() && "block already marked as free");
                    block->mark_as_free();
                    block = merge_prev(block);
                    block = merge_next(block);
                    block_insert(block);
                }
            }

            void* realloc(void* ptr, size_t size){
                void* p = 0;

                //zero-size requests are treated as freeing the block.
                if(ptr && size == 0){
                    free(ptr);
                }
                // nullptrs are treated as malloc
                else if (!ptr){
                    p = this->malloc(size);
                }
                else {
                    block_header* block = block->from_void_ptr(ptr);
                    block_header* next = block->get_next();
                    
                    const size_t cursize = block->get_size();
                    const size_t combined = cursize + next->get_size() + block_header_overhead;
                    const size_t adjust = adjust_request_size(size, ALIGN_SIZE);

                    TLSF_ASSERT(!block->is_free() && "Block is already marked as free.");

                    /**
                     * If the next block is used, or when combined with the current block, does not 
                     * offer enough space, we must reallocate and copy.
                     */
                    if (adjust > cursize && (!next->is_free() || adjust > combined)) {
                        p = this->malloc(size);
                        if (p) {
                            const size_t minsize = TLSF_MIN(cursize, size);
                            memcpy(p, ptr, minsize);
                            this->free(ptr);
                        }
                    }
                    else {
                        if (adjust > cursize) {
                            merge_next(block);
                            block->mark_as_used();
                        }

                        trim_used(block, adjust);
                        p = ptr;
                    }
                }
                
                return p;
            }

            void* init_memory_pool(size_t bytes, char* pool){
                block_header* block;
                block_header* next;

                constexpr size_t pool_overhead = tlsf_pool_overhead();
                const size_t pool_bytes = align_down(bytes-pool_overhead, ALIGN_SIZE);
                
                if (((ptrdiff_t)pool % ALIGN_SIZE) != 0) {
                    printf("tlsf init pool: Memory size must be aligned by %u bytes.\n", (unsigned int)ALIGN_SIZE);
                    return nullptr;
                }

                if (pool_bytes < block_size_min || pool_bytes > block_size_max){
            #ifdef TLSF_64BIT
                        printf("Init pool: Memory size must be between 0x%x and 0x%x00 bytes.\n",
                            (unsigned int)(pool_overhead+block_size_min),
                            (unsigned int)(pool_overhead+block_size_max));
            #else
                        printf("Init pool: Memory size must be between %u and %u bytes.\n",
                            (unsigned int)(pool_overhead+block_size_min),
                            (unsigned int)(pool_overhead+block_size_max));
            #endif
                    return nullptr;
                }

                /**
                 * Create the main free block. Offset the start of the block slightly
                 * so that the prev_phys_free_block field falls outside of the pool - 
                 * it will never be used.
                 */

                block = block->offset_to_block((void*)pool,-(tlsfptr_t)block_header_overhead);
                block->set_size(pool_bytes);
                block->set_free();
                block->set_prev_used();
                //insert block into control structure linked list

                next = block->link_next();
                next->set_size(0);
                next->set_used();
                next->set_prev_free();

                return (void*)pool;
            }


        private:
        
        /*Rounds up to the next block size for allocations */
        static void mapping_search(size_t size, int* fli, int* sli){
            if (size >= SMALL_BLOCK_SIZE){
                const size_t round = (1 << (tlsf_fls_sizet(size)-SL_INDEX_COUNT_LOG2))-1;
                size += round;
            }
            mapping_insert(size, fli, sli);
        }

        /* mapping insert: computes first level index (fl) and second level index (sl) */
        static void mapping_insert(size_t size, int* fli, int* sli){
            int fl, sl;
            if (size < SMALL_BLOCK_SIZE){
                fl = 0;
                sl = TLSF_CAST(int, size)/ (SMALL_BLOCK_SIZE/SL_INDEX_COUNT);
            }
            else {
                fl = tlsf_fls_sizet(size);
                sl = TLSF_CAST(int, size >> (fl-SL_INDEX_COUNT_LOG2))^(1 << SL_INDEX_COUNT_LOG2);
                fl -= (FL_INDEX_SHIFT-1);
            }
            *fli = fl;
            *sli = sl;
        }    

        static size_t adjust_request_size(size_t size, size_t align){
            size_t adjust = 0;
            if (size){
                const size_t aligned = align_up(size, align);

                // aligned size must not exceed block_max_size or we'll go out of bounds on sl_bitmap 
                if (aligned < block_size_max){
                    adjust = TLSF_MAX(aligned, block_size_min);
                }
            }
            return adjust; 
            }

        static block_header* block_coalesce(block_header* prev, block_header* block){
            TLSF_ASSERT(!prev->is_last() && "previous block can't be last");
            // leaves flags untouched
            prev->size += block->get_size() + block_header_overhead;
            prev->link_next();
            return prev;
        }

        /* rounds up to power of two size */
        static constexpr size_t align_up(size_t x, size_t align){
            TLSF_ASSERT(0 == (align & (align-1)) && "must align to a power of two");
            return (x + (align -1)) & ~(align -1);
        }

        /* rounds down to power of two size */
        static constexpr size_t align_down(size_t x, size_t align){
            TLSF_ASSERT(0 == (align & (align-1)) && "must align to a power of two");
            return x - (x & (align - 1));
        }

        /* aligns pointer to machine word */
        static void* align_ptr(const void* ptr, size_t align){
            const tlsfptr_t aligned = 
                (TLSF_CAST(tlsfptr_t, ptr)+ (align -1)) & ~(align-1);
            TLSF_ASSERT(0 == (align & (align-1)) && "must align to a power of two");
            return TLSF_CAST(void*, aligned);
        }

    };


    /**
     * TLSF utility functions 
     * Based on the implementation described in this paper:
     * http://www.gii.upv.es/tlsf/files/spe_2008.pdf
     */

    
    
    constexpr size_t tlsf_size(void){
        return sizeof(tlsf_control_block);
    }

    constexpr size_t tlsf_align_size(){
        return ALIGN_SIZE;
    }

    constexpr size_t tlsf_block_size_min(){
        return block_size_min;
    }

    constexpr size_t tlsf_block_size_max(){
        return block_size_max;
    }

    constexpr size_t tlsf_pool_overhead(){
        return 2*block_header_overhead;
    }

    constexpr size_t tlsf_alloc_overhead(){
        return block_header_overhead;
    }

} //namespace mem

