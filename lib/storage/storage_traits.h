#ifndef STORAGE_TRAITS_H
#define STORAGE_TRAITS_h

#include <stdint.h>
#include <type_traits>

// Primary template defaults to false
template <typename, typename = void> struct is_storage_impl : std::false_type
{
};

// Specialization: true if required methods exist with correct types
template <typename T>
struct is_storage_impl<
    T,
    typename std::enable_if<
        std::is_same<decltype(std::declval<T>().loadDeviceId()), uint16_t>::value &&
        std::is_same<decltype(std::declval<T>().saveDeviceId(uint16_t())), void>::value &&
        std::is_same<decltype(std::declval<T>().loadSequenceNumber()), uint32_t>::value &&
        std::is_same<decltype(std::declval<T>().saveSequenceNumber(uint32_t())), void>::value>::type>
    : std::true_type
{
};

// Helper: equivalent of variable template
template <typename T> struct is_storage : is_storage_impl<T>
{
};

#endif // STORAGE_TRAITS_H: