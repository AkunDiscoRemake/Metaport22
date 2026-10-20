-- Hand gestures on top of the 26 OpenXR joints.
--   palm menu: turn a palm up toward your face and hold ~0.35 s -> opens the quick menu there
--   double pinch: two quick pinches -> recenter
local V = require("forkxr.vmath")
local xr = require("forkxr.xr")

local M = {}

local PALM, THUMB_TIP, INDEX_TIP = 0, 5, 10
local HOLD_S = 0.35
local DOUBLE_PINCH_S = 0.45

local state = {
    {palmTime = 0, palmFired = false, pinchDown = false, lastPinchRelease = -1},
    {palmTime = 0, palmFired = false, pinchDown = false, lastPinchRelease = -1},
}

-- Palm normal (out of the palm, i.e. -Y of the palm joint) and position.
function M.palm(side)
    local h = xr.hand(side)
    if h.active == 0 then return nil end
    local j = h.joints[PALM]
    local q = V.fromFfiQuat(j.orientation)
    return V.fromFfiVec(j.position), V.rotate(q, {x = 0, y = -1, z = 0})
end

-- Returns a list of events: {type = "palm_menu", side, position} / {type = "double_pinch", side}.
function M.update(dt)
    local events = {}
    local head = xr.head()
    for side = 0, 1 do
        local s = state[side + 1]
        local h = xr.hand(side)
        local pos, normal = M.palm(side)
        local palmUp = false
        if pos and not (h.holdingController ~= 0) then
            local toHead = V.normalize(V.sub(head.position, pos))
            palmUp = normal.y > 0.55 and V.dot(normal, toHead) > 0.35 and V.length(V.sub(head.position, pos)) < 0.7
        end
        if palmUp then
            s.palmTime = s.palmTime + dt
            if s.palmTime >= HOLD_S and not s.palmFired then
                s.palmFired = true
                events[#events + 1] = {type = "palm_menu", side = side, position = pos}
            end
        else
            s.palmTime, s.palmFired = 0, false
        end

        local pinching = h.active ~= 0 and h.pinchStrength > 0.9
        local released = h.active ~= 0 and h.pinchStrength < 0.5
        if pinching and not s.pinchDown then
            s.pinchDown = true
        elseif released and s.pinchDown then
            s.pinchDown = false
            local now = xr.time()
            if s.lastPinchRelease >= 0 and now - s.lastPinchRelease < DOUBLE_PINCH_S then
                events[#events + 1] = {type = "double_pinch", side = side}
                s.lastPinchRelease = -1
            else
                s.lastPinchRelease = now
            end
        end
    end
    return events
end

return M
