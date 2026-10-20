#include "test.hpp"

namespace mptest {
std::vector<Case>& registry() {
    static std::vector<Case> r;
    return r;
}
int failures = 0;
}  // namespace mptest

int main() {
    int failedCases = 0;
    for (auto& c : mptest::registry()) {
        int before = mptest::failures;
        c.fn();
        bool ok = mptest::failures == before;
        if (!ok) ++failedCases;
        std::printf("[%s] %s\n", ok ? " OK " : "FAIL", c.name);
    }
    std::printf("%zu cases, %d failed\n", mptest::registry().size(), failedCases);
    return failedCases == 0 ? 0 : 1;
}
