/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 * Modifications Copyright (c) 2018 LG Electronics, Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/common/math/math_utils.h"

#include "gtest/gtest.h"

namespace apollo {
namespace common {
namespace math {

TEST(MathUtilsTest, CrossProd) {
  EXPECT_NEAR(CrossProd({0, 0}, {0, 1}, {1, 0}), -1.0, 1e-5);
  EXPECT_NEAR(CrossProd({0, 0}, {1, 0}, {0, 1}), 1.0, 1e-5);
  EXPECT_NEAR(CrossProd({0, 1}, {0, 0}, {1, 0}), 1.0, 1e-5);
  EXPECT_NEAR(CrossProd({1, 2}, {3, 4}, {5, 6}), 0.0, 1e-5);
  EXPECT_NEAR(CrossProd({1, 2}, {3, 4}, {6, 5}), -4.0, 1e-5);
  EXPECT_NEAR(CrossProd({2, 2}, {7, 5}, {3, 4}), 7.0, 1e-5);
}

TEST(MathUtilsTest, InnerProd) {
  EXPECT_NEAR(InnerProd({0, 0}, {0, 1}, {1, 0}), 0.0, 1e-5);
  EXPECT_NEAR(InnerProd({0, 0}, {1, 0}, {0, 1}), 0.0, 1e-5);
  EXPECT_NEAR(InnerProd({0, 1}, {0, 0}, {1, 0}), 1.0, 1e-5);
  EXPECT_NEAR(InnerProd({1, 2}, {3, 4}, {5, 6}), 16.0, 1e-5);
  EXPECT_NEAR(InnerProd({1, 2}, {3, 4}, {6, 5}), 16.0, 1e-5);
  EXPECT_NEAR(InnerProd({2, 2}, {7, 5}, {3, 4}), 11.0, 1e-5);
  EXPECT_NEAR(InnerProd({2, 2}, {0, 0}, {3, 4}), -6.0, 1e-5);
}

TEST(MathUtilsTest, WrapAngle) {
  EXPECT_NEAR(WrapAngle(-1.2), -1.2 + M_PI * 2.0, 1e-6);
  EXPECT_NEAR(WrapAngle(3.4), 3.4, 1e-6);
  EXPECT_NEAR(WrapAngle(5.6), 5.6, 1e-6);
  EXPECT_NEAR(WrapAngle(7.8), 7.8 - M_PI * 2.0, 1e-6);
  EXPECT_NEAR(WrapAngle(12.4), std::fmod(12.4, M_PI * 2.0), 1e-6);
  EXPECT_NEAR(WrapAngle(-12.4), std::fmod(-12.4, M_PI * 2.0) + M_PI * 2.0,
              1e-6);
}

TEST(MathUtilsTest, NormalizeAngle) {
  EXPECT_DOUBLE_EQ(1.5, NormalizeAngle(1.5));
  EXPECT_DOUBLE_EQ(1.5 - M_PI, NormalizeAngle(1.5 + M_PI));
  EXPECT_DOUBLE_EQ(1.5, NormalizeAngle(1.5 + M_PI * 2));
  EXPECT_DOUBLE_EQ(1.5, NormalizeAngle(1.5 - M_PI * 2));
  EXPECT_DOUBLE_EQ(-1.5, NormalizeAngle(-1.5));
  EXPECT_DOUBLE_EQ(-9.0 + M_PI * 2, NormalizeAngle(-9.0));
  EXPECT_DOUBLE_EQ(-M_PI, NormalizeAngle(-M_PI));
  EXPECT_DOUBLE_EQ(-M_PI, NormalizeAngle(M_PI));
  EXPECT_DOUBLE_EQ(-M_PI, NormalizeAngle(-M_PI * 3));
  EXPECT_DOUBLE_EQ(-M_PI, NormalizeAngle(M_PI * 3));
  EXPECT_DOUBLE_EQ(0.0, NormalizeAngle(M_PI * 4));
}

TEST(MathUtilsTest, Square) {
  EXPECT_DOUBLE_EQ(121.0, Square(11.0));
  EXPECT_FLOAT_EQ(144.0f, Square(-12.0f));
  EXPECT_EQ(169, Square(-13));
  EXPECT_EQ(2147395600, Square(46340));
  EXPECT_EQ(-2147479015, Square(46341));  // Overflow!
}

TEST(MathUtilsTest, Sqr) {
  EXPECT_DOUBLE_EQ(121.0, Sqr(11.0));
  EXPECT_DOUBLE_EQ(0.25, Sqr(0.5));
  EXPECT_DOUBLE_EQ(169.0, Sqr(-13.0));
}

TEST(MathUtilsTest, Sigmoid) {
  EXPECT_DOUBLE_EQ(0.5, Sigmoid(0.0));
}

TEST(MathUtilsTest, Clamp) {
  EXPECT_EQ(1, Clamp(1, 0, 6));
  EXPECT_EQ(6, Clamp(7, 0, 6));
  EXPECT_EQ(0, Clamp(-1, 0, 6));
  EXPECT_EQ(0, Clamp(0, 0, 6));  // test lower bound as input
  EXPECT_EQ(6, Clamp(6, 0, 6));  // test upper bound as input
}

TEST(MathUtilsTest, RotateAxis) {
  double x, y;
  double expected_x, expected_y;
  expected_x = sqrt(2);
  expected_y = 0;
  RotateAxis(M_PI / 4, 1.0, 1.0, &x, &y);
  EXPECT_DOUBLE_EQ(expected_x, x);
  EXPECT_NEAR(expected_y, y, 1e-5);

  expected_x = 1;
  expected_y = 0;
  RotateAxis(M_PI / 2, 0.0, 1.0, &x, &y);
  EXPECT_DOUBLE_EQ(expected_x, x);
  EXPECT_NEAR(expected_y, y, 1e-5);

  expected_x = -1;
  expected_y = 0;
  RotateAxis(M_PI, 1.0, 0.0, &x, &y);
  EXPECT_DOUBLE_EQ(expected_x, x);
  EXPECT_NEAR(expected_y, y, 1e-5);
}

}  // namespace math
}  // namespace common
}  // namespace apollo
