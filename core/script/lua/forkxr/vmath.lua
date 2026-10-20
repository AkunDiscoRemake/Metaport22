-- Small vector/quaternion library on plain numbers, written so LuaJIT can keep everything in
-- registers (no table allocation in the hot paths that matter).
local sqrt, sin, cos, atan2, acos = math.sqrt, math.sin, math.cos, math.atan2, math.acos

local M = {}

function M.vec(x, y, z) return {x = x or 0, y = y or 0, z = z or 0} end
function M.add(a, b) return {x = a.x + b.x, y = a.y + b.y, z = a.z + b.z} end
function M.sub(a, b) return {x = a.x - b.x, y = a.y - b.y, z = a.z - b.z} end
function M.scale(a, s) return {x = a.x * s, y = a.y * s, z = a.z * s} end
function M.dot(a, b) return a.x * b.x + a.y * b.y + a.z * b.z end
function M.cross(a, b) return {x = a.y * b.z - a.z * b.y, y = a.z * b.x - a.x * b.z, z = a.x * b.y - a.y * b.x} end
function M.length(a) return sqrt(a.x * a.x + a.y * a.y + a.z * a.z) end
function M.normalize(a)
    local l = M.length(a)
    if l < 1e-9 then return {x = 0, y = 0, z = 0} end
    return {x = a.x / l, y = a.y / l, z = a.z / l}
end

function M.quat(x, y, z, w) return {x = x or 0, y = y or 0, z = z or 0, w = w or 1} end
function M.axisAngle(ax, ay, az, angle)
    local s = sin(angle * 0.5)
    return {x = ax * s, y = ay * s, z = az * s, w = cos(angle * 0.5)}
end
function M.qmul(a, b)
    return {
        x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    }
end
function M.conj(q) return {x = -q.x, y = -q.y, z = -q.z, w = q.w} end
function M.rotate(q, v)
    -- v + 2w(u x v) + 2u x (u x v)
    local ux, uy, uz, w = q.x, q.y, q.z, q.w
    local tx = 2 * (uy * v.z - uz * v.y)
    local ty = 2 * (uz * v.x - ux * v.z)
    local tz = 2 * (ux * v.y - uy * v.x)
    return {
        x = v.x + w * tx + (uy * tz - uz * ty),
        y = v.y + w * ty + (uz * tx - ux * tz),
        z = v.z + w * tz + (ux * ty - uy * tx),
    }
end

-- Heading (yaw around +Y) of a rotation, using its -Z (forward) axis.
function M.yaw(q)
    local f = M.rotate(q, {x = 0, y = 0, z = -1})
    return atan2(-f.x, -f.z)
end

function M.angleBetween(a, b)
    local d = M.dot(M.normalize(a), M.normalize(b))
    if d > 1 then d = 1 elseif d < -1 then d = -1 end
    return acos(d)
end

-- Copies an FFI pose (fx_quat/fx_vec3) into plain tables.
function M.fromFfiQuat(q) return {x = q.x, y = q.y, z = q.z, w = q.w} end
function M.fromFfiVec(v) return {x = v.x, y = v.y, z = v.z} end

-- Ray (origin o, unit dir d) against the plane through p with normal n. Returns distance or nil.
function M.rayPlane(o, d, p, n)
    local denom = M.dot(d, n)
    if math.abs(denom) < 1e-6 then return nil end
    local t = M.dot(M.sub(p, o), n) / denom
    if t < 0 then return nil end
    return t
end

return M
