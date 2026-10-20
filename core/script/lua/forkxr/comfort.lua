-- Comfort: snap turn on the thumbstick, a vignette that fades in with fast rotation, and seated
-- mode (raises the world so a seated user gets standing eye height).
local xr = require("forkxr.xr")
local V = require("forkxr.vmath")

local M = {
    snapAngle = math.rad(30),
    snapEnabled = true,
    vignetteEnabled = true,
    seated = false,
    seatedEyeHeight = 1.65,
    yaw = 0,
    heightOffset = 0,
    vignette = 0,
}

local armed = {true, true}
local snapFlash = 0

function M.update(dt)
    if M.snapEnabled then
        for side = 0, 1 do
            local c = xr.controller(side)
            if c.connected ~= 0 and c.inputFromHand == 0 then
                local x = c.thumbX
                if armed[side + 1] and math.abs(x) > 0.7 then
                    M.yaw = M.yaw - (x > 0 and 1 or -1) * M.snapAngle
                    armed[side + 1] = false
                    snapFlash = 1
                elseif math.abs(x) < 0.3 then
                    armed[side + 1] = true
                end
            end
        end
    end

    if M.seated then
        local h = xr.head()
        local floor = xr.frame().floorHeight
        local current = h.position.y - floor
        -- Slew instead of jumping so posture changes don't yank the view.
        local target = M.seatedEyeHeight - current
        M.heightOffset = M.heightOffset + (target - M.heightOffset) * math.min(1, dt * 2)
    else
        M.heightOffset = M.heightOffset * math.max(0, 1 - dt * 4)
    end

    if M.vignetteEnabled then
        local w = V.length(xr.head().angularVelocity)
        local target = math.max(0, math.min(1, (w - 1.2) / 2.0))
        snapFlash = math.max(0, snapFlash - dt * 6)
        target = math.max(target, snapFlash * 0.8)
        local rate = target > M.vignette and 12 or 3
        M.vignette = M.vignette + (target - M.vignette) * math.min(1, dt * rate)
    else
        M.vignette = 0
    end
end

return M
