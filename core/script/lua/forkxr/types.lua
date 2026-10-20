-- FFI mirror of the native structs (shared_state.hpp, cardboard_lens.hpp, script_host.hpp).
-- forkxr.types.verify() checks every size/offset against the native build.
local ffi = require("ffi")

ffi.cdef[[
typedef struct { float x, y, z; } fx_vec3;
typedef struct { float x, y, z, w; } fx_quat;

typedef struct {
    fx_quat orientation;
    fx_vec3 position;
    fx_vec3 linearVelocity;
    fx_vec3 angularVelocity;
    int64_t timeNs;
} fx_pose_sample;

typedef struct {
    fx_pose_sample grip;
    uint32_t connected, orientationValid, positionSource, poseFromHand, inputFromHand, buttons;
    float trigger, squeeze, thumbX, thumbY;
} fx_controller;

typedef struct { fx_quat orientation; fx_vec3 position; float radius; } fx_joint;

typedef struct {
    uint32_t active, holdingController;
    float pinchStrength, confidence;
    int64_t timeNs;
    fx_joint joints[26];
    fx_pose_sample aim;
} fx_hand;

typedef struct {
    float interLensDistanceM, screenToLensDistanceM, trayToLensCenterM, k1, k2;
    float maxFovDeg[4];
} fx_viewer;

typedef struct { float widthM, heightM, bezelM; int widthPx, heightPx; } fx_screen;

typedef struct {
    float ipdMeters;
    float fov[2][4];
    uint32_t eyeWidth, eyeHeight;
    float refreshHz;
    fx_viewer viewer;
    fx_screen screen;
} fx_display;

typedef struct {
    fx_pose_sample head;
    uint32_t headTracking6Dof;
    float floorHeight;
    fx_controller controllers[2];
    fx_hand hands[2];
    fx_display display;
} fx_frame;

typedef struct {
    uint32_t kind, panel;
    float x0, y0, x1, y1;
    float gray, alpha, width;
    char text[48];
} fx_draw;

typedef struct {
    fx_quat orientation;
    fx_vec3 position;
    float widthM, heightM, pixelsPerMeter;
    uint32_t visible;
} fx_panel;

typedef struct {
    const fx_draw* commands;
    uint32_t commandCount;
    const fx_panel* panels;
    uint32_t panelCount;
    float worldYawRad, worldHeightM, vignette, hapticLeft, hapticRight;
} fx_output;
]]

local M = {}

M.DRAW_RECT, M.DRAW_OUTLINE, M.DRAW_LINE, M.DRAW_TEXT = 1, 2, 3, 4
M.LEFT, M.RIGHT = 0, 1

-- Button bits (shared_state.hpp ButtonBits).
M.BTN_A, M.BTN_B, M.BTN_MENU, M.BTN_SYSTEM, M.BTN_STICK = 1, 2, 4, 8, 16

-- Returns true, or false plus a description of the first mismatch.
function M.verify(native)
    native = native or forkxr_native_layout
    if not native then return false, "no native layout table" end
    local checks = {
        {"TrackingFrame", ffi.sizeof("fx_frame")},
        {"PoseSample", ffi.sizeof("fx_pose_sample")},
        {"ControllerState", ffi.sizeof("fx_controller")},
        {"HandState", ffi.sizeof("fx_hand")},
        {"DisplayInfo", ffi.sizeof("fx_display")},
        {"DrawCmd", ffi.sizeof("fx_draw")},
        {"PanelDesc", ffi.sizeof("fx_panel")},
        {"ScriptOutput", ffi.sizeof("fx_output")},
        {"off_controllers", ffi.offsetof("fx_frame", "controllers")},
        {"off_hands", ffi.offsetof("fx_frame", "hands")},
        {"off_display", ffi.offsetof("fx_frame", "display")},
        {"off_ctrl_trigger", ffi.offsetof("fx_controller", "trigger")},
        {"off_hand_joints", ffi.offsetof("fx_hand", "joints")},
        {"off_hand_aim", ffi.offsetof("fx_hand", "aim")},
        {"off_display_screen", ffi.offsetof("fx_display", "screen")},
    }
    for _, c in ipairs(checks) do
        if native[c[1]] ~= c[2] then
            return false, string.format("%s: native %s, Lua %s", c[1], tostring(native[c[1]]), tostring(c[2]))
        end
    end
    return true
end

return M
