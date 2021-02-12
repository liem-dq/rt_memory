This is a C++ implementation of the TLSF allocator. It is mostly a wrapper of the C implementation found [here](https://github.com/mattconte/tlsf), with some minor changes and adaptations to make it conform to `std::allocator_traits`.  

This is, however, a stateful allocator, which will make it difficult to use in situations where a stateless allocator is expected. 
