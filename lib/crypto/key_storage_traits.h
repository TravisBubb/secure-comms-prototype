#ifndef KEY_STORAGE_TRAITS_H
#define KEY_STORAGE_TRAITS_H

#include <stddef.h>
#include <stdint.h>
#include <type_traits>

/*
  - Key storage must implement:
    - bool loadKey(uint8_t *out, size_t maxLen, size_t &actualLen);
    - bool saveKey(const uint8_t *in, size_t len);
    - bool eraseKey(void);

  - loadKey returns true if a valid key was read.
  - saveKey atomically persists the key and returns true on success.
*/

template <typename, typename = void>
struct is_key_storage : std::false_type {};

template <typename T>
struct is_key_storage<
    T,
    std::void_t<
        decltype(std::declval<T>().loadKey((uint8_t*)nullptr, (size_t)0, std::declval<size_t&>())),
        decltype(std::declval<T>().saveKey((const uint8_t*)nullptr, (size_t)0)),
        decltype(std::declval<T>().eraseKey())
    >
> : std::true_type {};

#endif // KEY_STORAGE_TRAITS_H