#include <unity.h>
#include "bytes.h"

void test_write_read_u16_be() {
    uint8_t buf[2];
    write_u16_be(0xABCD, buf);
    TEST_ASSERT_EQUAL_UINT8(0xAB, buf[0]);
    TEST_ASSERT_EQUAL_UINT8(0xCD, buf[1]);
    TEST_ASSERT_EQUAL_UINT16(0xABCD, read_u16_be(buf));
}

void test_write_read_u32_be() {
    uint8_t buf[4];
    write_u32_be(0x12345678, buf);
    TEST_ASSERT_EQUAL_UINT8(0x12, buf[0]);
    TEST_ASSERT_EQUAL_UINT8(0x34, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(0x56, buf[2]);
    TEST_ASSERT_EQUAL_UINT8(0x78, buf[3]);
    TEST_ASSERT_EQUAL_UINT32(0x12345678, read_u32_be(buf));
}

void setup()
{
  UNITY_BEGIN();
  RUN_TEST(test_write_read_u16_be);
  RUN_TEST(test_write_read_u32_be);
  UNITY_END();
}

int main() {
  setup();
  return 0;
}