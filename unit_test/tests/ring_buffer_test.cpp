#include "peripheral//ring_buffer.h"

#include <gtest/gtest.h>

namespace fs
{

  TEST(common_ring_buffer, general)
  {
    RingBuffer<int, 4> rb;
    rb.pushBack(3);
    rb.pushFront(1);
    RingBuffer<int, 4> rb1(std::move(rb));
    ASSERT_TRUE(rb.empty());
    rb.pushBack(5);
    EXPECT_EQ(5, rb.back());
    EXPECT_EQ(1u, rb.size());

    RingBuffer<int, 4> rb2;
    rb2 = std::move(rb1);
    EXPECT_FALSE(rb2.full());
    EXPECT_EQ(2u, rb2.size());
    EXPECT_EQ(1, rb2[0]);
    rb2.pushBack(2);
    EXPECT_TRUE(rb2.full());

    int& elem1 = *rb2.pushBackForce();
    elem1      = 5;
    EXPECT_EQ(5, rb2.back());
    int& elem2 = *rb2.pushFrontForce();
    elem2      = 6;
    EXPECT_EQ(6, rb2.front());

    RingBuffer<std::string, 4> sb;
    std::string                s = "abc";
    ASSERT_TRUE(sb.pushBack(std::move(s)));
    EXPECT_EQ(3u, sb[0].size());
    EXPECT_EQ(0u, s.size());
  }

  TEST(common_ring_buffer, iterator)
  {
    // -----------------------------------------------------------------
    // set rb = [0, 1, end, empty]
    // -----------------------------------------------------------------
    RingBuffer<int, 4> rb;
    rb.pushBack(0);
    rb.pushBack(1);

    // -----------------------------------------------------------------
    // read value of rb
    // rb = [0, 1, end, empty]
    // -----------------------------------------------------------------
    int i = 0;
    for(const auto& item : rb)
    {
      EXPECT_EQ(i++, item);
    }

    // -----------------------------------------------------------------
    // set rb over flowed to check correctness of iterator
    // rb = [4, 5, end, 3]
    // -----------------------------------------------------------------
    rb.pushBack(2);
    rb.pushBackForce(3);
    rb.pushBackForce(4);
    rb.pushBackForce(5);
    EXPECT_EQ(3u, rb.size());
    EXPECT_EQ(4, *rb.head());
    EXPECT_EQ(3, *rb.tail());

    // -----------------------------------------------------------------
    // set value of rb
    // rb = [4, 3, end, 5]
    // -----------------------------------------------------------------
    i = 5;
    for(auto& item : rb)
    {
      item = i--;
    }
    EXPECT_EQ(5, rb.front());
    EXPECT_EQ(3, rb.back());

    // -----------------------------------------------------------------
    // check begin() of iterator
    // set rb = [1, 3, end, 5]
    // -----------------------------------------------------------------
    auto iter = rb.begin();
    *++iter   = 1;
    EXPECT_EQ(1, rb[1]);

    // -----------------------------------------------------------------
    // check find() and operator-(iterator) function of iterator
    // rb = [1, 3, end, 5]
    // -----------------------------------------------------------------
    const auto iter1 = std::find(rb.cbegin(), rb.cend(), 1);
    EXPECT_EQ(1, iter1 - rb.cbegin());
    EXPECT_EQ(-2, iter1 - rb.cend());

    // -----------------------------------------------------------------
    // check operator> and operator<() function of iterator
    // rb = [1, 3, end, 5]
    // -----------------------------------------------------------------
    EXPECT_TRUE(iter1 > rb.cbegin());
    EXPECT_TRUE(iter1 < rb.cend());

    // -----------------------------------------------------------------
    // check min_element() and minmax_element() function of iterator
    // check separately for compilation reason
    // rb = [1, 3, end, 5]
    // -----------------------------------------------------------------
    const auto [minIter, maxIter] = std::minmax_element(rb.cbegin(), rb.cend());
    EXPECT_EQ(1, *minIter);
    EXPECT_EQ(5, *maxIter);
    EXPECT_EQ(minIter, std::min_element(rb.cbegin(), rb.cend()));

    // -----------------------------------------------------------------
    // check operator+(int) and operator-(int) function of iterator
    // rb = [1, 3, end, 5]
    // -----------------------------------------------------------------
    EXPECT_EQ(5, *(rb.cbegin() + 0));
    EXPECT_EQ(1, *(rb.cbegin() + 1));
    EXPECT_EQ(3, *(rb.cbegin() + 2));
    EXPECT_EQ(rb.cend(), rb.cbegin() + 3);
    EXPECT_EQ(3, *(rb.cend() - 1));
    EXPECT_EQ(1, *(rb.cend() - 2));
    EXPECT_EQ(5, *(rb.cend() - 3));
    EXPECT_EQ(rb.cend(), rb.cend() - 4);

    // -----------------------------------------------------------------
    // check conversion to reverse iterator
    // rb = [1, 3, end, 5]
    // -----------------------------------------------------------------
    const auto reverse_iter = iter1.toReverseIterator();
    EXPECT_EQ(1, *reverse_iter);
    EXPECT_EQ(5, *(reverse_iter + 1));
  }

  TEST(common_ring_buffer, reverse_iterator)
  {
    // -----------------------------------------------------------------
    // set rb = [0, 1, 2 end]
    // -----------------------------------------------------------------
    RingBuffer<int, 4> rb;
    rb.pushBack(0);
    rb.pushBack(1);
    rb.pushBack(2);
    EXPECT_TRUE(rb.crbegin() < rb.crend());

    // -----------------------------------------------------------------
    // set rb = [0, 1, 2 end]
    // read value of rb
    // -----------------------------------------------------------------
    int i = 2;
    for(auto iter = rb.crbegin(); iter != rb.crend(); ++iter)
    {
      EXPECT_EQ(i--, *iter);
    }
    ASSERT_EQ(-1, i);

    std::find_if(rb.crbegin(),rb.crend(),[](const auto& x){return x ==5;});

    // -----------------------------------------------------------------
    // set rb over flowed to check correctness of reverse iterator
    // rb = [4, 5, end, 3]
    // -----------------------------------------------------------------
    rb.pushBackForce(3);
    EXPECT_EQ(rb.empty(), false);
    rb.pushBackForce(4);
    rb.pushBackForce(5);
    EXPECT_EQ(rb.empty(), false);
    EXPECT_EQ(3u, rb.size());
    EXPECT_EQ(4, *rb.head());
    EXPECT_EQ(3, *rb.tail());

    // -----------------------------------------------------------------
    // test rbegin() and rend() of reverse iterator
    // rb = [4, 5, end, 3]
    // -----------------------------------------------------------------
    EXPECT_EQ(5, *rb.crbegin());
    EXPECT_EQ(4, *(++rb.crbegin()));
    EXPECT_EQ(3, *(--rb.crend()));

    // -----------------------------------------------------------------
    // check find() and operator-(iterator) function of reverse iterator
    // rb = [4, 5, end, 3]
    // -----------------------------------------------------------------
    const auto iter = std::find(rb.crbegin(), rb.crend(), 4);
    EXPECT_EQ(4, *iter);
    EXPECT_EQ(1, iter - rb.crbegin());
    EXPECT_EQ(-2, iter - rb.crend());

    // -----------------------------------------------------------------
    // check conversion to iterator
    // rb = [4, 5, end, 3]
    // -----------------------------------------------------------------
    const auto normal_iter = iter.toIterator();
    EXPECT_EQ(4, *normal_iter);
    EXPECT_EQ(5, *(normal_iter + 1));

    // -----------------------------------------------------------------
    // check operator> and operator<() function of reverse iterator
    // b = [4, 5, end, 3]
    // -----------------------------------------------------------------
    EXPECT_TRUE(iter > rb.crbegin());
    EXPECT_TRUE(iter < rb.crend());

    // -----------------------------------------------------------------
    // check operator+(int) and operator-(int) function of iterator
    // rb = [4, 5, end, 3]
    // -----------------------------------------------------------------
    EXPECT_EQ(5, *(rb.crbegin() + 0));
    EXPECT_EQ(4, *(rb.crbegin() + 1));
    EXPECT_EQ(3, *(rb.crbegin() + 1002));
    EXPECT_EQ(rb.crend(), rb.crbegin() + 3);
    EXPECT_EQ(3, *(rb.crend() - 1));
    EXPECT_EQ(4, *(rb.crend() - 2));
    EXPECT_EQ(5, *(rb.crend() - 1003));
    EXPECT_EQ(rb.crend(), rb.crend() - 4);

    // -----------------------------------------------------------------
    // check min_element() and minmax_element() function of reverse iterator
    // check separately for compilation reason
    // set rb = [4, end, 3, 3]
    // -----------------------------------------------------------------
    rb.pushFrontForce(3);
    const auto [minIter, maxIter] = std::minmax_element(rb.crbegin(), rb.crend());
    EXPECT_EQ(4, *(minIter - 1));
    EXPECT_EQ(3, *(minIter + 1));
    EXPECT_EQ(4, *maxIter);
    EXPECT_EQ(minIter, std::min_element(rb.crbegin(), rb.crend()));
  }

} // namespace fs
