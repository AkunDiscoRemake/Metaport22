// Cardboard viewer optics: field of view and the distortion mesh used to pre-warp each eye.
//
// Model (same polynomial as the Cardboard viewer profiles): a point on the screen at lens-space
// tangent t_s is perceived through the lens at t_v = t_s * (1 + k1 r^2 + k2 r^4), r = |t_s|.
// The mesh maps each screen vertex to the texture coordinate of the perceived direction, so a
// rectilinear eye image appears undistorted through the lens.
#pragma once

#include <vector>

namespace metaport {

struct ViewerParams {
    // Defaults: Google Cardboard v2.
    float interLensDistanceM = 0.064f;
    float screenToLensDistanceM = 0.039f;
    float trayToLensCenterM = 0.035f;   // from the tray (bottom edge of the phone) to lens centres
    float k1 = 0.34f, k2 = 0.55f;
    float maxFovDeg[4] = {60, 60, 60, 60};  // left, right, bottom, top limits per eye
};

struct ScreenParams {
    float widthM = 0.135f;   // landscape width of the visible area
    float heightM = 0.065f;
    float bezelM = 0.003f;   // distance from the tray to the bottom of the visible area
    int widthPx = 2400, heightPx = 1080;
};

// Half-angle tangents, all positive: left, right, down, up.
struct EyeFovTan {
    float left, right, down, up;
};

struct MeshVertex {
    float x, y;    // NDC of the whole screen, -1..1
    float u, v;    // eye texture coordinates for this lens's own FOV, 0..1
    float tx, ty;  // perceived view-direction tangents (used for reprojection)
};

class CardboardLens {
public:
    CardboardLens(const ViewerParams& viewer, const ScreenParams& screen) : viewer_(viewer), screen_(screen) {}

    float distortFactor(float r2) const { return 1.0f + viewer_.k1 * r2 + viewer_.k2 * r2 * r2; }

    // eye: 0 = left, 1 = right.
    EyeFovTan fov(int eye) const;

    // Grid of (cols+1)*(rows+1) vertices over the eye's half of the screen, plus triangle indices.
    void buildMesh(int eye, int cols, int rows, std::vector<MeshVertex>* vertices, std::vector<unsigned short>* indices) const;

    // Recommended per-eye render resolution keeping ~1:1 pixel density at the lens centre.
    void recommendedEyeSize(int eye, int* width, int* height) const;

private:
    // Lens centre for the eye, in metres from the screen's bottom-left corner.
    void lensCenter(int eye, float* x, float* y) const;
    // Eye viewport bounds on screen in metres: [x0, x1] x [y0, y1].
    void viewport(int eye, float* x0, float* x1, float* y0, float* y1) const;

    ViewerParams viewer_;
    ScreenParams screen_;
};

}  // namespace metaport
