#include <gtest/gtest.h>

int Add(int a, int b) {
    return a + b;
}

TEST(SampleTest, Addition) {
    EXPECT_EQ(Add(2, 2), 4);
}
