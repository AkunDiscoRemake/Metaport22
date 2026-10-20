-- Performance HUD data: frame time average, 99th percentile and dropped-frame count.
local M = {}

local N = 240
local samples, idx, count = {}, 0, 0
M.dropped = 0

function M.record(dt, refreshHz)
    idx = idx % N + 1
    samples[idx] = dt
    if count < N then count = count + 1 end
    local budget = 1 / (refreshHz > 1 and refreshHz or 60)
    if dt > budget * 1.5 then M.dropped = M.dropped + 1 end
end

function M.stats()
    if count == 0 then return 0, 0 end
    local sorted, sum = {}, 0
    for i = 1, count do sorted[i] = samples[i]; sum = sum + samples[i] end
    table.sort(sorted)
    local p99 = sorted[math.max(1, math.ceil(count * 0.99))]
    return sum / count, p99
end

function M.reset() samples, idx, count, M.dropped = {}, 0, 0, 0 end

return M
