#include "frame.h"
#include <string.h>
#include <unity.h>

void test_frame_reset_defaults()
{
  Frame f;

  // Fill the frame with garbage first
  memset(&f, 0xFF, sizeof(Frame));

  frame_reset(f);

  TEST_ASSERT_EQUAL_UINT8(0, f.ver);
  TEST_ASSERT_EQUAL_UINT8(0, f.flags);
  TEST_ASSERT_EQUAL_UINT16(0, f.dev_id);
  TEST_ASSERT_EQUAL_UINT32(0, f.seq);
  TEST_ASSERT_EQUAL_UINT8(0, f.payload_len);

  // Verify payload and auth_tag are all zeroed
  for (int i = 0; i < MAX_PAYLOAD_LEN; ++i)
    TEST_ASSERT_EQUAL_UINT8(0, f.payload[i]);

  for (int i = 0; i < 16; ++i)
    TEST_ASSERT_EQUAL_UINT8(0, f.auth_tag[i]);

  TEST_ASSERT_EQUAL_UINT16(0, f.crc);
}

void test_frame_serialize_deserialize()
{
  Frame f{};
  f.ver = 2;
  f.flags = 0x5A;
  f.dev_id = 0xABCD;
  f.seq = 0x12345678;
  f.payload_len = 2;
  f.payload[0] = 0x11;
  f.payload[1] = 0x22;
  memset(f.auth_tag, 0x77, sizeof(f.auth_tag));
  f.crc = 0xBEEF;

  uint8_t buf[256];
  size_t len;
  TEST_ASSERT_TRUE(frame_serialize(f, buf, &len));

  Frame out{};
  TEST_ASSERT_TRUE(frame_deserialize(out, buf, len));

  TEST_ASSERT_EQUAL_MEMORY(&f, &out, sizeof(Frame));
}

void test_frame_serialize_empty_payload()
{
  Frame f{};
  f.ver = 1;
  f.payload_len = 0;
  uint8_t buffer[512];
  size_t len;

  TEST_ASSERT_TRUE(frame_serialize(f, buffer, &len));
  TEST_ASSERT_EQUAL_UINT8(9 + 16 + 2, len); // 9 header + 16 auth_tag + 2 crc
}

void test_frame_serialize_max_payload()
{
  Frame f{};
  f.ver = 3;
  f.payload_len = MAX_PAYLOAD_LEN;
  for (int i = 0; i < MAX_PAYLOAD_LEN; ++i)
    f.payload[i] = (uint8_t)i;
  memset(f.auth_tag, 0xAA, sizeof(f.auth_tag));
  f.crc = 0x1234;

  uint8_t buf[1024];
  size_t len;
  TEST_ASSERT_TRUE(frame_serialize(f, buf, &len));
  TEST_ASSERT_TRUE(frame_deserialize(f, buf, len));
}

void test_frame_serialize_invalid_input()
{
  Frame f{};
  uint8_t buffer[10];
  size_t len;
  f.payload_len = MAX_PAYLOAD_LEN + 1;

  TEST_ASSERT_FALSE(frame_serialize(f, buffer, &len));
}

void test_frame_deserialize_truncated_buffer()
{
  Frame f{};
  f.ver = 1;
  f.payload_len = 5;
  uint8_t buf[256];
  size_t len;

  TEST_ASSERT_TRUE(frame_serialize(f, buf, &len));

  // Corrupt the buffer by truncating it
  Frame out{};
  TEST_ASSERT_FALSE(frame_deserialize(out, buf, len - 3)); // simulate missing CRC
}

void test_frame_deserialize_too_short()
{
  uint8_t bad_buf[4] = {0x01, 0x02, 0x03, 0x04};
  Frame f{};
  TEST_ASSERT_FALSE(frame_deserialize(f, bad_buf, sizeof(bad_buf)));
}

void test_frame_deserialize_corrupted_payload()
{
  Frame f{};
  f.payload_len = 4;
  f.payload[0] = 0xAA;
  f.payload[1] = 0xBB;
  f.payload[2] = 0xCC;
  f.payload[3] = 0xDD;

  uint8_t buf[256];
  size_t len;
  TEST_ASSERT_TRUE(frame_serialize(f, buf, &len));

  buf[10] ^= 0xFF; // flip a payload bit
  Frame out{};
  TEST_ASSERT_TRUE(frame_deserialize(out, buf, len)); // should still “succeed” structurally
}

void test_frame_endianness_consistency()
{
  Frame f{};
  f.dev_id = 0x1122;
  f.seq = 0x33445566;

  uint8_t buf[sizeof(Frame)];
  size_t len;
  TEST_ASSERT_TRUE(frame_serialize(f, buf, &len));

  // Expected big-endian order
  TEST_ASSERT_EQUAL_UINT8(0x11, buf[2]);
  TEST_ASSERT_EQUAL_UINT8(0x22, buf[3]);
  TEST_ASSERT_EQUAL_UINT8(0x33, buf[4]);
  TEST_ASSERT_EQUAL_UINT8(0x44, buf[5]);
  TEST_ASSERT_EQUAL_UINT8(0x55, buf[6]);
  TEST_ASSERT_EQUAL_UINT8(0x66, buf[7]);
}

void test_frame_fuzz_roundtrip()
{
  for (int i = 0; i < 5000; ++i)
  {
    Frame f{};
    f.ver = rand() & 0xFF;
    f.flags = rand() & 0xFF;
    f.dev_id = rand() & 0xFFFF;
    f.seq = rand();
    f.payload_len = rand() % (MAX_PAYLOAD_LEN + 1);
    for (int j = 0; j < f.payload_len; ++j)
      f.payload[j] = rand() & 0xFF;
    for (int j = 0; j < 16; ++j)
      f.auth_tag[j] = rand() & 0xFF;
    f.crc = rand() & 0xFFFF;

    uint8_t buf[512];
    size_t len;
    TEST_ASSERT_TRUE(frame_serialize(f, buf, &len));
    TEST_ASSERT(len <= 9 + MAX_PAYLOAD_LEN + 16 + 2);

    Frame out{};
    TEST_ASSERT_TRUE(frame_deserialize(out, buf, len));

    TEST_ASSERT_EQUAL_MEMORY(&f, &out, sizeof(Frame));
  }
}

void setup()
{
  UNITY_BEGIN();
  RUN_TEST(test_frame_reset_defaults);
  RUN_TEST(test_frame_serialize_deserialize);
  RUN_TEST(test_frame_serialize_empty_payload);
  RUN_TEST(test_frame_serialize_max_payload);
  RUN_TEST(test_frame_serialize_invalid_input);
  RUN_TEST(test_frame_deserialize_truncated_buffer);
  RUN_TEST(test_frame_deserialize_too_short);
  RUN_TEST(test_frame_deserialize_corrupted_payload);
  RUN_TEST(test_frame_endianness_consistency);
  RUN_TEST(test_frame_fuzz_roundtrip);
  UNITY_END();
}

int main()
{
  setup();
  return 0;
}