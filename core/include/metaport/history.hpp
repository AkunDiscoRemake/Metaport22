// Fixed-size timestamped history used for latency compensation: camera-based measurements
// (ARCore, hand tracking) arrive 30-80 ms late, so their error is computed against the
// state we *had* at the measurement time, not the current one.
#pragma once

#include <array>
#include <cstdint>

namespace metaport {

template <typename T, int N>
class History {
public:
    void push(int64_t timeNs, const T& value) {
        head_ = (head_ + 1) % N;
        times_[head_] = timeNs;
        values_[head_] = value;
        if (count_ < N) ++count_;
    }

    void clear() { count_ = 0; }
    bool empty() const { return count_ == 0; }

    // Entry closest to `timeNs` (no interpolation; IMU-rate entries are ~5 ms apart).
    bool lookup(int64_t timeNs, T* out) const {
        if (count_ == 0) return false;
        int best = head_;
        int64_t bestDiff = absdiff(times_[head_], timeNs);
        for (int i = 1; i < count_; ++i) {
            int idx = (head_ - i + N) % N;
            int64_t d = absdiff(times_[idx], timeNs);
            if (d < bestDiff) { bestDiff = d; best = idx; }
            if (times_[idx] < timeNs) break;  // entries are chronological, older from here
        }
        *out = values_[best];
        return true;
    }

    // Shifts every stored value by `fn` so later lookups see corrected history.
    template <typename F>
    void applyAll(F fn) {
        for (int i = 0; i < count_; ++i) fn(values_[(head_ - i + N) % N]);
    }

private:
    static int64_t absdiff(int64_t a, int64_t b) { return a > b ? a - b : b - a; }

    std::array<int64_t, N> times_{};
    std::array<T, N> values_{};
    int head_ = N - 1;
    int count_ = 0;
};

}  // namespace metaport
