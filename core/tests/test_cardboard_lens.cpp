#include <cmath>

#include "metaport/cardboard_lens.hpp"
#include "test.hpp"

using namespace metaport;

TEST(lens_fov_is_mirrored_between_eyes) {
    CardboardLens lens{ViewerParams{}, ScreenParams{}};
    EyeFovTan l = lens.fov(0), r = lens.fov(1);
    EXPECT_NEAR(l.left, r.right, 1e-5);
    EXPECT_NEAR(l.right, r.left, 1e-5);
    EXPECT_NEAR(l.up, r.up, 1e-6);
    // Cardboard v2 is limited to 60° per side: tan(60°) = 1.732.
    EXPECT_TRUE(l.left <= std::tan(60.0f * 3.14159265f / 180) + 1e-4f);
}

TEST(lens_mesh_maps_lens_center_to_axis) {
    ViewerParams v;
    ScreenParams sp;
    CardboardLens lens{v, sp};
    std::vector<MeshVertex> verts;
    std::vector<unsigned short> idx;
    lens.buildMesh(0, 40, 40, &verts, &idx);
    EXPECT_TRUE(verts.size() == 41u * 41u);
    EXPECT_TRUE(idx.size() == 40u * 40u * 6u);
    // The vertex nearest to the lens centre must sample the eye's optical axis.
    float cx = (sp.widthM * 0.5f - v.interLensDistanceM * 0.5f) / sp.widthM * 2 - 1;
    float cy = (v.trayToLensCenterM - sp.bezelM) / sp.heightM * 2 - 1;
    const MeshVertex* best = &verts[0];
    for (const auto& m : verts)
        if (std::hypot(m.x - cx, m.y - cy) < std::hypot(best->x - cx, best->y - cy)) best = &m;
    EyeFovTan f = lens.fov(0);
    EXPECT_NEAR(best->u, f.left / (f.left + f.right), 0.03);
    EXPECT_NEAR(best->v, f.down / (f.down + f.up), 0.03);
}
