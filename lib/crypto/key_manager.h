#ifndef KEY_MANAGER_H
#define KEY_MANAGER_H

#include "key_storage_traits.h"
#include "key_types.h"
#include <array>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>
#include <string.h>

enum class KMError : int
{
  OK = 0,
  INVALID_ARG,
  NO_KEY,
  STORAGE_FAIL,
  RNG_FAIL,
  BAD_LEN
};

/*
  KeyManager<TStorage>
  --------------------
  - Template storage that conforms to is_key_storage<TStorage>.
  - No heap/no virtuals. Single fixed-size key buffer inside the object.
  - Basic operations:
    - bool init();
    - bool loadOrGenerate();
    - bool generate();
    - bool save();
    - bool rotate();
    - void zeroize();
    - const std::array<uint8_t, 32> &getKey() const noexcept;
    - size_t keyLen() const noexcept;
*/
template <typename TStorage> class KeyManager
{
  static_assert(is_key_storage<TStorage>::value,
                "Storage type does not satisfy key storage traits.");

public:
  // Construct with a storage instance (must outlive KeyManager).
  // Storage can be a static or global object; no ownership is taken.
  explicit KeyManager(TStorage &storage) noexcept : _storage(storage)
  {
    memset(_key.data(), 0, _key.size());
    _isLoaded = false;

    // init drbg contexts but do not seed yet
    mbedtls_entropy_init(&_entropy);
    mbedtls_ctr_drbg_init(&_drbg);
  }

  // Must call init() before generate/load operations.
  // Seeds the DRBG using mbedtls_entropy_func.
  // Returns KMError::OK on success.
  KMError init() noexcept
  {
    const unsigned char pers[] = "km_ctr_drbg_v1";
    int r = mbedtls_ctr_drbg_seed(&_drbg, mbedtls_entropy_func, &_entropy, pers, sizeof(pers) - 1);
    if (r != 0) return KMError::RNG_FAIL;
    return KMError::OK;
  }

  // Attempt to load key from storage. Returns KMError::OK if successful.
  KMError load() noexcept
  {
    size_t actual = 0;
    if (!_storage.loadKey(_key.data(), _key.size(), actual))
    {
      _isLoaded = false;
      _keyLen = 0;
      return KMError::STORAGE_FAIL;
    }

    if (actual != 16 && actual != 32)
    {
      _isLoaded = false;
      _keyLen = 0;
      return KMError::BAD_LEN;
    }

    _keyLen = actual;
    _isLoaded = true;
    return KMError::OK;
  }

  // If no key present, generate new and save immediately.
  KMError loadOrGenerate() noexcept
  {
    KMError ret = load();
    if (ret == KMError::OK) return KMError::OK;
    ret = generate();
    if (ret != KMError::OK) return ret;

    ret = save();
    if (ret != KMError::OK)
    {
      zeroize();
      return ret;
    }

    return KMError::OK;
  }

  // Generate a new key of given length (16 or 32). Leaves key in RAM but does not persist.
  KMError generate(size_t len = 32) noexcept
  {
    if (len != 16 && len != 32) return KMError::BAD_LEN;
    int r = mbedtls_ctr_drbg_random(&_drbg, _key.data(), len);
    if (r != 0) return KMError::RNG_FAIL;
    _keyLen = len;
    _isLoaded = true;
    return KMError::OK;
  }

  // Save current in-RAM key to storage (atomic as provided by TStorage).
  KMError save() noexcept
  {
    if (!_isLoaded || (_keyLen != 16 && _keyLen != 32)) return KMError::BAD_LEN;
    if (!_storage.saveKey(_key.data(), _keyLen)) return KMError::STORAGE_FAIL;
    return KMError::OK;
  }

  // Generate new key and save atomically; zeroize old key immediately.
  KMError rotate(size_t newLen = 32) noexcept
  {
    if (newLen != 16 && newLen != 32) return KMError::BAD_LEN;

    // generate into temp local buffer on stack to avoid overwriting existing key in case
    // generation fails
    uint8_t temp[KM_MAX_KEY_BYTES];
    memset(temp, 0, sizeof(temp));
    if (mbedtls_ctr_drbg_random(&_drbg, temp, newLen) != 0)
    {
      // zero temp before return
      volatile uint8_t *p = temp;
      for (size_t i = 0; i < sizeof(temp); ++i)
        p[i] = 0;
      return KMError::RNG_FAIL;
    }

    // Attempt to save new key
    bool ok = _storage.saveKey(temp, newLen);

    // zero temp immediately
    volatile uint8_t *p = temp;
    for (size_t i = 0; i < sizeof(temp); ++i)
      p[i] = 0;

    if (!ok) return KMError::STORAGE_FAIL;

    // if saved ok, zero old key and load new key into RAM (read back)
    zeroize();
    size_t actual = 0;
    if (!_storage.loadKey(_key.data(), _key.size(), actual))
    {
      _isLoaded = false;
      _keyLen = 0;
      return KMError::STORAGE_FAIL;
    }

    _keyLen = actual;
    _isLoaded = true;
    return KMError::OK;
  }

  // Get const reference to key storage. Caller must not mutate or copy key.
  const std::array<uint8_t, KM_MAX_KEY_BYTES> &getKey() const noexcept { return _key; }
  size_t keyLen() const noexcept { return _keyLen; }
  bool isLoaded() const noexcept { return _isLoaded; }

  // Overwrite in-RAM key, mark as unloaded. Does not attempt to update storage.
  void zeroize() noexcept
  {
    volatile uint8_t *p = _key.data();
    for (size_t i = 0; i < _key.size(); ++i)
      p[i] = 0;
    _isLoaded = false;
    _keyLen = 0;
  }

  // Erase persisted key from storage and zeroize RAM
  KMError erase() noexcept
  {
    KMError ret = _storage.eraseKey();
    zeroize();
    return ret;
  }

private:
  TStorage &_storage;
  std::array<uint8_t, KM_MAX_KEY_BYTES> _key;
  size_t _keyLen = 0;
  bool _isLoaded = false;
  mbedtls_entropy_context _entropy;
  mbedtls_ctr_drbg_context _drbg;
};

#endif // KEY_MANAGER_H