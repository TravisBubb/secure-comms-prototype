#ifndef FLASH_KEY_STORAGE_H
#define FLASH_KEY_STORAGE_H

#include "key_types.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <Preferences.h>

// Magic value
static constexpr uint32_t KM_SLOT_MAGIC = 0x4B4D4752U; // 'KMGR'

// Slot layout on flash: fixed-size to simplify storage.
// Keep it packed and predictable to match reads/writes exactly.
struct FlashKeySlot
{
  uint32_t magic;   // KM_SLOT_MAGIC
  uint32_t version; // mono-increasing on rotate
  uint8_t keyLen;   // 16 or 32
  uint8_t flags;    // bit0 = non-exportable (1)
  uint8_t reserved[2];
  uint8_t key[KM_MAX_KEY_BYTES];
  uint32_t crc32; // CRC32 over slot with crc32 field zeroed
} __attribute__((packed));

// Simple CRC32 table & function
static uint32_t crc32_table_flash[256];
static bool crc32_table_inited_flash = false;
static void init_crc32_table_flash()
{
  if (crc32_table_inited_flash) return;
  crc32_table_inited_flash = true;
  for (uint32_t i = 0; i < 256; ++i)
  {
    uint32_t c = i;
    for (int j = 0; j < 8; ++j)
    {
      if (c & 1)
        c = 0xEDB88320U ^ (c >> 1);
      else
        c = c >> 1;
    }
    crc32_table_flash[i] = c;
  }
}

static uint32_t crc32_compute_flash(const void *data, size_t len)
{
  init_crc32_table_flash();
  uint32_t c = 0xFFFFFFFFU;
  const uint8_t *p = reinterpret_cast<const uint8_t *>(data);
  while (len--)
    c = crc32_table_flash[(c ^ *p++) & 0xFF] ^ (c >> 8);
  return c ^ 0xFFFFFFFFU;
}

/*
  FlashKeyStorage
  ---------------
  - Preferences-backed two-slot atomic store:
      keys "km_slot0", "km_slot1", and index "km_idx" (0 or 1).
  - saveKey() writes the inactive slot fully, then flips index to point to it.
  - loadKey() reads both slots, validates CRC+magic, and returns the highest-version valid slot.
  - eraseKey() invalidates inactive slot and flips index to invalid.
  - No dynamic memory; small stack copies only.

  Security notes:
    - CEC32 protects against accidental corruption and incomplete writes but NOT against a 
      bad actor with raw flash access. For tamper-resistance, an HMAC using a device-unique 
      secret could be used, or the key could be moved to a secure element.
*/
class FlashKeyStorage 
{
public:
  // namespace - keep short (<= 15 chars) to save flash
  explicit FlashKeyStorage(const char *ns = "keymgr")
  {
    strncpy(_ns, ns, sizeof(ns) - 1);
    _ns[sizeof(_ns) - 1] = '\0';
  }

  bool loadKey(uint8_t *out, size_t maxLen, size_t &actualLen)
  {
    FlashKeySlot a, b;
    bool aOk = readSlot(0, &a);
    bool bOk = readSlot(1, &b);

    if (!aOk && !bOk) return false;

    FlashKeySlot *pick = nullptr;
    if (aOk && bOk) pick = (a.version >= b.version) ? &a : &b;
    else if (aOk) pick = &a;
    else if (bOk) pick = &b;
    else return false;

    // only support keys of length 16 or 32
    if (pick->keyLen != 16 && pick->keyLen != 32) return false;
    if (maxLen < pick->keyLen) return false;
    
    memcpy(out, pick->key, pick->keyLen);
    actualLen = pick->keyLen;
    return true;
  }

  // Atomically save the key. Returns true on success.
  // Accepts only 16 or 32 byte keys. Sets version = previous version + 1.
  bool saveKey(const uint8_t *in, size_t len)
  {
    if (len != 16 && len != 32) return false;

    // read active index
    uint8_t idx = readIndex();
    if (idx > 1) idx = 0; // treat invalid as 0

    // read slots to get previous version
    FlashKeySlot prev;
    bool ok = false;
    if (readSlot(0, &prev) == true || readSlot(1, &prev) == true)
      ok = true;
    uint32_t newVersion = ok ? (prev.version + 1) : 1;

    // initialize new flash slot
    FlashKeySlot newSlot;
    memset(&newSlot, 0, sizeof(newSlot));
    newSlot.magic = KM_SLOT_MAGIC;
    newSlot.version = newVersion;
    newSlot.keyLen = static_cast<uint8_t>(len);
    newSlot.flags = 1; // default non-exportable for safety
    memcpy(newSlot.key, in, len);

    // compute crc with crc32 field zeroed
    newSlot.crc32 = 0;
    newSlot.crc32 = crc32_compute_flash(&newSlot, sizeof(newSlot));

    uint8_t inactive = (idx == 0) ? 1 : 0;

    // write inactive slot fully
    if (!writeRawSlot(inactive, &newSlot, sizeof(newSlot))) return false;

    // flip index to point to inactive (now active)
    if (!writeIndex(inactive)) return false;

    // Done. Currently, the old slot is not deleted.
    return true;
  }

  bool eraseKey(void)
  {
    uint8_t idx = readIndex();
    uint8_t inactive = (idx == 0) ? 1 : 0;
    FlashKeySlot bad;
    memset(&bad, 0xFF, sizeof(bad));
    if (!writeRawSlot(inactive, &bad, sizeof(bad))) return false;
    if (!writeIndex(inactive)) return false;
    return true;
  }

private:
  char _ns[16];

  bool readSlot(int which, FlashKeySlot *out)
  {
    Preferences prefs;

    // load flash slot
    if (!prefs.begin(_ns, true)) return false;
    char keyname[12];
    snprintf(keyname, sizeof(keyname), "km_slot%d", which);
    size_t got = prefs.getBytes(keyname, out, sizeof(FlashKeySlot));
    prefs.end();

    // verify integrity of loaded key
    if (got != sizeof(FlashKeySlot)) return false;
    if (out->magic != KM_SLOT_MAGIC) return false;
    uint32_t savedCrc = out->crc32;
    out->crc32 = 0;
    uint32_t comp = crc32_compute_flash(out, sizeof(FlashKeySlot));
    out->crc32 = savedCrc;
    if (comp != savedCrc) return false;
    if (out->keyLen != 16 && out->keyLen != 32) return false;
    return true;
  }

  bool writeRawSlot(int which, const void *data, size_t len)
  {
    Preferences prefs;
    if (!prefs.begin(_ns, false)) return false;
    char keyname[12];
    snprintf(keyname, sizeof(keyname), "km_slot%d", which);
    bool ok = prefs.putBytes(keyname, data, len);
    prefs.end();
    return ok;
  }

  uint8_t readIndex()
  {
    Preferences prefs;
    if (!prefs.begin(_ns, true)) return false;
    uint8_t v = prefs.getUChar("km_idx", 0);
    prefs.end();
    return v;
  }

  bool writeIndex(uint8_t idx)
  {
    Preferences prefs;
    if (!prefs.begin(_ns, false)) return false;
    bool ok = prefs.putUChar("km_idx", idx);
    prefs.end();
    return ok;
  }
};

#endif // FLASH_KEY_STORAGE_H