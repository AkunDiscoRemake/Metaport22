#include "metaport/cardboard_lens.hpp"

#include <algorithm>
#include <cmath>

namespace metaport {

namespace {
constexpr float kDegToRad = 3.14159265358979f / 180.0f;
}

void CardboardLens::lensCenter(int eye, float* x, float* y) const {
    float sign = eye == 0 ? -1.0f : 1.0f;
    *x = screen_.widthM * 0.5f + sign * viewer_.interLensDistanceM * 0.5f;
    *y = viewer_.trayToLensCenterM - screen_.bezelM;
}

void CardboardLens::viewport(int eye, float* x0, float* x1, float* y0, float* y1) const {
    *x0 = eye == 0 ? 0.0f : screen_.widthM * 0.5f;
    *x1 = eye == 0 ? screen_.widthM * 0.5f : screen_.widthM;
    *y0 = 0.0f;
    *y1 = screen_.heightM;
}

EyeFovTan CardboardLens::fov(int eye) const {
    float cx, cy, x0, x1, y0, y1;
    lensCenter(eye, &cx, &cy);
    viewport(eye, &x0, &x1, &y0, &y1);
    float d = viewer_.screenToLensDistanceM;
    auto perceived = [&](float meters) {
        float t = meters / d;
        return t * distortFactor(t * t);
    };
    // Screen edges seen through the lens, capped by the viewer's physical FOV.
    float l = perceived(cx - x0), r = perceived(x1 - cx), dn = perceived(cy - y0), up = perceived(y1 - cy);
    // For the left eye the "outer" limit is left; the viewer limits are given as left, right,
    // bottom, top and mirror for the right eye.
    const float* m = viewer_.maxFovDeg;
    float maxL = std::tan((eye == 0 ? m[0] : m[1]) * kDegToRad);
    float maxR = std::tan((eye == 0 ? m[1] : m[0]) * kDegToRad);
    return {std::min(l, maxL), std::min(r, maxR), std::min(dn, std::tan(m[2] * kDegToRad)),
            std::min(up, std::tan(m[3] * kDegToRad))};
}

void CardboardLens::buildMesh(int eye, int cols, int rows, std::vector<MeshVertex>* vertices,
                              std::vector<unsigned short>* indices) const {
    float cx, cy, x0, x1, y0, y1;
    lensCenter(eye, &cx, &cy);
    viewport(eye, &x0, &x1, &y0, &y1);
    EyeFovTan f = fov(eye);
    float d = viewer_.screenToLensDistanceM;

    vertices->clear();
    indices->clear();
    for (int j = 0; j <= rows; ++j) {
        for (int i = 0; i <= cols; ++i) {
            float sx = x0 + (x1 - x0) * i / cols;
            float sy = y0 + (y1 - y0) * j / rows;
            float tx = (sx - cx) / d, ty = (sy - cy) / d;
            float k = distortFactor(tx * tx + ty * ty);
            float vx = tx * k, vy = ty * k;
            MeshVertex v;
            v.x = sx / screen_.widthM * 2.0f - 1.0f;
            v.y = sy / screen_.heightM * 2.0f - 1.0f;
            v.u = (vx + f.left) / (f.left + f.right);
            v.v = (vy + f.down) / (f.down + f.up);
            v.tx = vx;
            v.ty = vy;
            vertices->push_back(v);
        }
    }
    for (int j = 0; j < rows; ++j) {
        for (int i = 0; i < cols; ++i) {
            unsigned short a = static_cast<unsigned short>(j * (cols + 1) + i);
            unsigned short b = static_cast<unsigned short>(a + 1);
            unsigned short c = static_cast<unsigned short>(a + cols + 1);
            unsigned short e = static_cast<unsigned short>(c + 1);
            indices->insert(indices->end(), {a, b, c, b, e, c});
        }
    }
}

void CardboardLens::recommendedEyeSize(int eye, int* width, int* height) const {
    EyeFovTan f = fov(eye);
    // Pixels per tangent unit at the lens centre (distortion factor is 1 there).
    float pxPerM = screen_.widthPx / screen_.widthM;
    float pxPerTan = pxPerM * viewer_.screenToLensDistanceM;
    *width = static_cast<int>(std::ceil((f.left + f.right) * pxPerTan));
    *height = static_cast<int>(std::ceil((f.down + f.up) * pxPerTan));
}

}  // namespace metaport
