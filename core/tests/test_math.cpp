#include "metaport/math.hpp"
#include "test.hpp"

using namespace metaport;

TEST(quat_rotate_axis_angle) {
    Quat q = Quat::fromAxisAngle({0, 1, 0}, 3.14159265f / 2);
    Vec3 v = q.rotate({0, 0, -1});  // forward turned 90° left
    EXPECT_NEAR(v.x, -1, 1e-5);
    EXPECT_NEAR(v.z, 0, 1e-5);
}

TEST(quat_rotation_vector_roundtrip) {
    Vec3 rv{0.3f, -0.2f, 0.9f};
    Vec3 back = Quat::fromRotationVector(rv).toRotationVector();
    EXPECT_NEAR(back.x, rv.x, 1e-5);
    EXPECT_NEAR(back.y, rv.y, 1e-5);
    EXPECT_NEAR(back.z, rv.z, 1e-5);
}

TEST(quat_look_rotation) {
    Vec3 fwd{1, 0, -1};
    Quat q = Quat::lookRotation(fwd, {0, 1, 0});
    Vec3 f = q.rotate({0, 0, -1});
    Vec3 u = q.rotate({0, 1, 0});
    EXPECT_NEAR(f.dot(fwd.normalized()), 1, 1e-5);
    EXPECT_NEAR(u.y, 1, 1e-5);
}

TEST(quat_from_basis_identity) {
    Quat q = Quat::fromBasis({1, 0, 0}, {0, 1, 0}, {0, 0, 1});
    EXPECT_NEAR(q.w, 1, 1e-6);
}

TEST(quat_twist_extracts_yaw) {
    Quat yaw = Quat::fromAxisAngle({0, 1, 0}, 0.7f);
    Quat pitch = Quat::fromAxisAngle({1, 0, 0}, 0.4f);
    Quat t = (yaw * pitch).twist({0, 1, 0});
    // Twist around Y of yaw*pitch is not exactly yaw, but the heading of -Z must match.
    Vec3 f1 = (yaw * pitch).rotate({0, 0, -1});
    Vec3 f2 = t.rotate({0, 0, -1});
    f1.y = 0;
    EXPECT_NEAR(f1.normalized().dot(f2.normalized()), 1, 1e-4);
}

TEST(pose_inverse) {
    Pose p{Quat::fromAxisAngle({1, 2, 3}, 0.8f), {0.5f, -1, 2}};
    Pose id = p * p.inverse();
    EXPECT_NEAR(id.position.length(), 0, 1e-5);
    EXPECT_NEAR(std::fabs(id.orientation.w), 1, 1e-5);
}
