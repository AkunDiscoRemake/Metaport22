#include "forkxr/script_host.hpp"

#include <cstddef>

extern "C" {
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>
}

namespace forkxr {

namespace {

bool report(lua_State* L, int status, std::string* error) {
    if (status == 0) return true;
    if (error) *error = lua_isstring(L, -1) ? lua_tostring(L, -1) : "unknown Lua error";
    lua_pop(L, 1);
    return false;
}

int traceback(lua_State* L) {
    const char* msg = lua_tostring(L, 1);
    lua_getglobal(L, "debug");
    lua_getfield(L, -1, "traceback");
    lua_pushstring(L, msg ? msg : "(error object)");
    lua_pushinteger(L, 2);
    lua_call(L, 2, 1);
    return 1;
}

void setInt(lua_State* L, const char* key, size_t v) {
    lua_pushinteger(L, static_cast<lua_Integer>(v));
    lua_setfield(L, -2, key);
}

}  // namespace

ScriptHost::ScriptHost() : L_(luaL_newstate()) {
    luaL_openlibs(L_);
    publishLayout();
}

ScriptHost::~ScriptHost() { lua_close(L_); }

// Sizes and offsets of the shared structs, so Lua can verify its FFI declarations match.
void ScriptHost::publishLayout() {
    using namespace metaport;
    lua_newtable(L_);
    setInt(L_, "TrackingFrame", sizeof(TrackingFrame));
    setInt(L_, "PoseSample", sizeof(PoseSample));
    setInt(L_, "ControllerState", sizeof(ControllerState));
    setInt(L_, "HandState", sizeof(HandState));
    setInt(L_, "DisplayInfo", sizeof(DisplayInfo));
    setInt(L_, "DrawCmd", sizeof(DrawCmd));
    setInt(L_, "PanelDesc", sizeof(PanelDesc));
    setInt(L_, "ScriptOutput", sizeof(ScriptOutput));
    setInt(L_, "off_controllers", offsetof(TrackingFrame, controllers));
    setInt(L_, "off_hands", offsetof(TrackingFrame, hands));
    setInt(L_, "off_display", offsetof(TrackingFrame, display));
    setInt(L_, "off_ctrl_trigger", offsetof(ControllerState, trigger));
    setInt(L_, "off_hand_joints", offsetof(HandState, joints));
    setInt(L_, "off_hand_aim", offsetof(HandState, aim));
    setInt(L_, "off_display_screen", offsetof(DisplayInfo, screen));
    lua_setglobal(L_, "forkxr_native_layout");
}

void ScriptHost::addSearchPath(const std::string& dir) {
    lua_getglobal(L_, "package");
    lua_getfield(L_, -1, "path");
    std::string path = dir + "/?.lua;" + dir + "/?/init.lua;" + (lua_tostring(L_, -1) ? lua_tostring(L_, -1) : "");
    lua_pop(L_, 1);
    lua_pushstring(L_, path.c_str());
    lua_setfield(L_, -2, "path");
    lua_pop(L_, 1);
}

bool ScriptHost::runFile(const std::string& path, std::string* error) {
    lua_pushcfunction(L_, traceback);
    int handler = lua_gettop(L_);
    bool ok = report(L_, luaL_loadfile(L_, path.c_str()), error) && report(L_, lua_pcall(L_, 0, 0, handler), error);
    lua_remove(L_, handler);
    return ok;
}

bool ScriptHost::runString(const std::string& code, const std::string& chunkName, std::string* error) {
    lua_pushcfunction(L_, traceback);
    int handler = lua_gettop(L_);
    bool ok = report(L_, luaL_loadbuffer(L_, code.data(), code.size(), chunkName.c_str()), error) &&
              report(L_, lua_pcall(L_, 0, 0, handler), error);
    lua_remove(L_, handler);
    return ok;
}

bool ScriptHost::frame(const metaport::TrackingFrame& f, double dt, ScriptOutput* out, std::string* error) {
    lua_pushcfunction(L_, traceback);
    int handler = lua_gettop(L_);
    lua_getglobal(L_, "forkxr");
    if (!lua_istable(L_, -1)) {
        lua_pop(L_, 2);
        if (error) *error = "global `forkxr` table not defined (main.lua not loaded?)";
        return false;
    }
    lua_getfield(L_, -1, "frame");
    lua_remove(L_, -2);
    lua_pushlightuserdata(L_, const_cast<metaport::TrackingFrame*>(&f));
    lua_pushnumber(L_, dt);
    if (!report(L_, lua_pcall(L_, 2, 1, handler), error)) {
        lua_remove(L_, handler);
        return false;
    }
    // forkxr.frame returns a ScriptOutput* cdata owned by the Lua side.
    const void* p = lua_topointer(L_, -1);
    bool ok = p != nullptr;
    if (ok) *out = *static_cast<const ScriptOutput*>(p);
    else if (error) *error = "forkxr.frame did not return a ScriptOutput";
    lua_pop(L_, 1);
    lua_remove(L_, handler);
    return ok;
}

}  // namespace forkxr
