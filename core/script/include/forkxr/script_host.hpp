// ForkXR script host: runs the Lua (LuaJIT, Lua 5.1 API) layer that implements Metaport's shell,
// UI and XR behaviours. Scripts read the tracking frame in place through the LuaJIT FFI and
// return a draw list that the native renderer turns into panels.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "metaport/shared_state.hpp"

struct lua_State;

namespace forkxr {

enum DrawKind : uint32_t {
    kDrawRect = 1,     // filled rectangle
    kDrawOutline = 2,  // rectangle outline, `width` px
    kDrawLine = 3,     // segment (x0,y0)-(x1,y1), `width` px
    kDrawText = 4,     // text at (x0,y0) baseline-left, `width` = font size px
};

// One draw command in panel pixel coordinates (origin top-left). Mirrored in forkxr/types.lua.
struct DrawCmd {
    uint32_t kind;
    uint32_t panel;  // index into the panel list
    float x0, y0, x1, y1;
    float gray;      // 0 = black, 1 = white: ForkXR's UI is strictly monochrome
    float alpha;
    float width;
    char text[48];
};

// A UI panel placed in the world. Mirrored in forkxr/types.lua.
struct PanelDesc {
    metaport::Quat orientation;
    metaport::Vec3 position;
    float widthM, heightM;
    float pixelsPerMeter;
    uint32_t visible;
};

// What the scripts ask the native side to do this frame. Mirrored in forkxr/types.lua.
struct ScriptOutput {
    const DrawCmd* commands;
    uint32_t commandCount;
    const PanelDesc* panels;
    uint32_t panelCount;
    float worldYawRad;     // comfort snap-turn applied on top of tracking
    float worldHeightM;    // seated-mode height offset
    float vignette;        // 0..1 comfort vignette strength
    float hapticLeft, hapticRight;
};

class ScriptHost {
public:
    ScriptHost();
    ~ScriptHost();
    ScriptHost(const ScriptHost&) = delete;
    ScriptHost& operator=(const ScriptHost&) = delete;

    // Directory containing the `forkxr/` Lua package and `main.lua`.
    void addSearchPath(const std::string& dir);
    bool runFile(const std::string& path, std::string* error);
    bool runString(const std::string& code, const std::string& chunkName, std::string* error);

    // Calls forkxr.frame(frame, dt) and collects its output. The frame must stay alive until the
    // next call (scripts may keep pointers into it during the frame only).
    bool frame(const metaport::TrackingFrame& frame, double dtSeconds, ScriptOutput* out, std::string* error);

    lua_State* state() { return L_; }

private:
    void publishLayout();
    lua_State* L_;
};

}  // namespace forkxr
