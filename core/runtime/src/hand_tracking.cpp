// XR_EXT_hand_tracking backed by the hand joints the Metaport service publishes. The joint source is
// pluggable (see hand.hpp); the intended source is the hand tracking component of the user's own
// Horizon OS firmware once its compatibility shim exists.
#include "runtime.hpp"

using namespace mpxr;

XRAPI_ATTR XrResult XRAPI_CALL xrCreateHandTrackerEXT(XrSession session, const XrHandTrackerCreateInfoEXT* info,
                                                      XrHandTrackerEXT* out) {
    Session* s = fromHandle<Session>(session);
    if (!sessions().valid(s)) return XR_ERROR_HANDLE_INVALID;
    if (!info || !out || info->type != XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT) return XR_ERROR_VALIDATION_FAILURE;
    if (info->hand != XR_HAND_LEFT_EXT && info->hand != XR_HAND_RIGHT_EXT) return XR_ERROR_VALIDATION_FAILURE;
    if (info->handJointSet != XR_HAND_JOINT_SET_DEFAULT_EXT) return XR_ERROR_FEATURE_UNSUPPORTED;
    auto* h = new HandTracker;
    h->session = s;
    h->side = info->hand == XR_HAND_LEFT_EXT ? metaport::kLeft : metaport::kRight;
    handTrackers().add(h);
    s->handTrackers.insert(h);
    *out = toHandle<XrHandTrackerEXT>(h);
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrDestroyHandTrackerEXT(XrHandTrackerEXT tracker) {
    HandTracker* h = fromHandle<HandTracker>(tracker);
    if (!handTrackers().valid(h)) return XR_ERROR_HANDLE_INVALID;
    h->session->handTrackers.erase(h);
    handTrackers().remove(h);
    delete h;
    return XR_SUCCESS;
}

XRAPI_ATTR XrResult XRAPI_CALL xrLocateHandJointsEXT(XrHandTrackerEXT tracker, const XrHandJointsLocateInfoEXT* info,
                                                     XrHandJointLocationsEXT* locations) {
    HandTracker* h = fromHandle<HandTracker>(tracker);
    if (!handTrackers().valid(h)) return XR_ERROR_HANDLE_INVALID;
    if (!info || !locations || info->type != XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT ||
        locations->type != XR_TYPE_HAND_JOINT_LOCATIONS_EXT)
        return XR_ERROR_VALIDATION_FAILURE;
    if (locations->jointCount != XR_HAND_JOINT_COUNT_EXT) return XR_ERROR_VALIDATION_FAILURE;
    if (info->time <= 0) return XR_ERROR_TIME_INVALID;
    Space* base = fromHandle<Space>(info->baseSpace);
    if (!spaces().valid(base)) return XR_ERROR_HANDLE_INVALID;
    Session* s = h->session;

    XrHandJointVelocitiesEXT* velocities = nullptr;
    for (auto* n = static_cast<XrBaseOutStructure*>(locations->next); n; n = n->next)
        if (n->type == XR_TYPE_HAND_JOINT_VELOCITIES_EXT) velocities = reinterpret_cast<XrHandJointVelocitiesEXT*>(n);

    const metaport::HandState& hs = s->frame.hands[h->side];
    Pose baseWorld;
    bool tracked;
    bool ok = hs.active && locateInWorld(s, base, info->time, &baseWorld, &tracked);
    locations->isActive = ok ? XR_TRUE : XR_FALSE;
    for (uint32_t i = 0; i < XR_HAND_JOINT_COUNT_EXT; ++i) {
        XrHandJointLocationEXT& j = locations->jointLocations[i];
        if (velocities) velocities->jointVelocities[i].velocityFlags = 0;
        if (!ok) {
            j.locationFlags = 0;
            continue;
        }
        // Hand frames are ~30 Hz and not extrapolated: fingers overshooting looks worse than lag.
        Pose world{hs.joints[i].orientation, hs.joints[i].position};
        j.pose = toXr(baseWorld.inverse() * world);
        j.radius = hs.joints[i].radius;
        j.locationFlags = XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_VALID_BIT |
                          XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT | XR_SPACE_LOCATION_POSITION_TRACKED_BIT;
    }
    return XR_SUCCESS;
}
