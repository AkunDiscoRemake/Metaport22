// Synthetic right hand, back of the hand facing the camera, fingers up, 40 cm in front.
// Coordinates in the OpenXR camera frame (x right, y up, camera looks along -z).
#pragma once

#include <array>

#include "metaport/hand.hpp"

namespace fixture {

using namespace metaport;

inline std::array<Vec3, kLandmarkCount> rightHandXr(float z = -0.4f, float x0 = 0.0f, float y0 = 0.0f) {
    std::array<Vec3, kLandmarkCount> p{};
    auto at = [&](int i, float x, float y) { p[i] = {x0 + x, y0 + y, z}; };
    at(kWrist, 0, 0);
    at(kThumbCmc, -0.03f, 0.02f); at(kThumbMcp, -0.05f, 0.04f); at(kThumbIp, -0.065f, 0.06f); at(kThumbTip, -0.075f, 0.08f);
    at(kIndexMcp, -0.03f, 0.08f); at(kIndexPip, -0.035f, 0.12f); at(kIndexDip, -0.037f, 0.145f); at(kIndexTip, -0.039f, 0.165f);
    at(kMiddleMcp, -0.01f, 0.09f); at(kMiddlePip, -0.01f, 0.135f); at(kMiddleDip, -0.01f, 0.16f); at(kMiddleTip, -0.01f, 0.185f);
    at(kRingMcp, 0.01f, 0.085f); at(kRingPip, 0.012f, 0.125f); at(kRingDip, 0.013f, 0.15f); at(kRingTip, 0.014f, 0.17f);
    at(kPinkyMcp, 0.03f, 0.075f); at(kPinkyPip, 0.035f, 0.105f); at(kPinkyDip, 0.037f, 0.125f); at(kPinkyTip, 0.039f, 0.14f);
    return p;
}

// Builds the landmark observation (metric around the hand centre + normalised image
// coordinates) that a camera with intrinsics K would produce for the XR-frame points.
inline HandObservation observe(const std::array<Vec3, kLandmarkCount>& xr, const CameraIntrinsics& K, Side side) {
    HandObservation o;
    o.side = side;
    o.score = 0.95f;
    Vec3 mean;
    for (const Vec3& v : xr) mean += v;
    mean = mean / kLandmarkCount;
    for (int i = 0; i < kLandmarkCount; ++i) {
        Vec3 cv{xr[i].x, -xr[i].y, -xr[i].z};
        Vec3 meanCv{mean.x, -mean.y, -mean.z};
        o.metric[i] = cv - meanCv;
        o.image[i] = {(K.fx * cv.x / cv.z + K.cx) / K.width, (K.fy * cv.y / cv.z + K.cy) / K.height, 0};
    }
    return o;
}

}  // namespace fixture
