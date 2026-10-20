-- XR input for scripts: reads the native TrackingFrame in place (zero copy) and turns
-- controllers and hands into the same "pointer" abstraction: a ray plus select/squeeze.
local ffi = require("ffi")
local T = require("forkxr.types")
local V = require("forkxr.vmath")

local M = {}

local frame = nil      -- const fx_frame*
local selectHeld = {false, false}
local time = 0

-- Select uses hysteresis so a trembling trigger or pinch never double-clicks.
local SELECT_ON, SELECT_OFF = 0.75, 0.45

function M.setFrame(ptr, dt)
    frame = ffi.cast("const fx_frame*", ptr)
    time = time + (dt or 0)
end

function M.frame() return frame end
function M.time() return time end

function M.head()
    local h = frame.head
    return {orientation = V.fromFfiQuat(h.orientation), position = V.fromFfiVec(h.position),
            angularVelocity = V.fromFfiVec(h.angularVelocity), sixDof = frame.headTracking6Dof ~= 0}
end

function M.controller(side) return frame.controllers[side] end
function M.hand(side) return frame.hands[side] end

-- Pointer for one side: {active, origin, dir, select, pressed, released, squeeze, stickX, stickY,
-- buttons, source}. `pressed`/`released` are edges since the previous call for that side.
local lastSelect = {false, false}
function M.pointer(side)
    local c = frame.controllers[side]
    local h = frame.hands[side]
    local p = {active = false, side = side}
    if c.connected ~= 0 then
        local q, o
        if c.poseFromHand ~= 0 and h.active ~= 0 then
            q, o = V.fromFfiQuat(h.aim.orientation), V.fromFfiVec(h.aim.position)
            p.source = c.inputFromHand ~= 0 and "hand" or "hand+joycon"
        else
            q, o = V.fromFfiQuat(c.grip.orientation), V.fromFfiVec(c.grip.position)
            o = V.add(o, V.rotate(q, {x = 0, y = 0, z = -0.06}))
            p.source = "joycon"
        end
        p.active = true
        p.origin = o
        p.orientation = q
        p.dir = V.rotate(q, {x = 0, y = 0, z = -1})
        local v = c.trigger
        local held = selectHeld[side + 1]
        if not held and v >= SELECT_ON then held = true elseif held and v <= SELECT_OFF then held = false end
        selectHeld[side + 1] = held
        p.select = held
        p.squeeze = c.squeeze
        p.stickX, p.stickY = c.thumbX, c.thumbY
        p.buttons = c.buttons
    else
        selectHeld[side + 1] = false
        p.select = false
    end
    p.pressed = p.select and not lastSelect[side + 1]
    p.released = (not p.select) and lastSelect[side + 1]
    lastSelect[side + 1] = p.select
    return p
end

local band = require("bit").band
function M.buttonDown(side, mask)
    local c = frame.controllers[side]
    return c.connected ~= 0 and band(c.buttons, mask) ~= 0
end

M.LEFT, M.RIGHT = T.LEFT, T.RIGHT

return M
