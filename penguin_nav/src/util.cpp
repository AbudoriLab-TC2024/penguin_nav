#include "util.hpp"

#include "doctest.h"

namespace penguin_nav {

void into(const geometry_msgs::msg::Point &src, tf2::Vector3 &dst) {
  dst = {src.x, src.y, src.z};
}

void into(const tf2::Vector3 &src, geometry_msgs::msg::Point &dst) {
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
}

void into(const geometry_msgs::msg::Quaternion &src, tf2::Quaternion &dst) {
  dst = {src.x, src.y, src.z, src.w};
}

void into(const tf2::Quaternion &src, geometry_msgs::msg::Quaternion &dst) {
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
  dst.w = src.w();
}

TEST_CASE("testing into()") {
  SUBCASE("msg::Point -> tf2::Vector3") {
    geometry_msgs::msg::Point m_p;
    m_p.x = 1;
    m_p.y = 2;
    m_p.z = 3;
    tf2::Vector3 t_p;

    into(m_p, t_p);
    CHECK(t_p.x() == 1);
    CHECK(t_p.y() == 2);
    CHECK(t_p.z() == 3);
  }

  SUBCASE("tf2::Vector3 -> msg::Point") {
    geometry_msgs::msg::Point m_p;
    tf2::Vector3 t_p(1, 2, 3);

    into(t_p, m_p);
    CHECK(m_p.x == 1);
    CHECK(m_p.y == 2);
    CHECK(m_p.z == 3);
  }

  SUBCASE("msg::Quaternion -> tf2::Quaternion") {
    geometry_msgs::msg::Quaternion m_p;
    m_p.x = 1;
    m_p.y = 2;
    m_p.z = 3;
    m_p.w = 4;
    tf2::Quaternion t_p;

    into(m_p, t_p);
    CHECK(t_p.x() == 1);
    CHECK(t_p.y() == 2);
    CHECK(t_p.z() == 3);
    CHECK(t_p.w() == 4);
  }

  SUBCASE("tf2::Quaternion -> msg::Quaternion") {
    geometry_msgs::msg::Quaternion m_p;
    tf2::Quaternion t_p(1, 2, 3, 4);

    into(t_p, m_p);
    CHECK(m_p.x == 1);
    CHECK(m_p.y == 2);
    CHECK(m_p.z == 3);
    CHECK(m_p.w == 4);
  }
}

} // namespace penguin_nav
