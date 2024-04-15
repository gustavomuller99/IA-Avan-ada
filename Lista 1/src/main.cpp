#include <bits/stdc++.h>

#include "solvers/solvers.cpp"

int main(int argc, char **argv) {
    std::vector<int> testState = {0, 6, 1, 7, 4, 2, 3, 8, 5};

    State8 state = State8(testState);

    ret_info sol = bfs_graph<State8>(state);

    std::cout << "\n";
    std::cout << sol.expanded << ",";
    std::cout << sol.sol.size() - 1 << ",";
    std::cout << sol.time << std::setprecision(20) << ",";
    std::cout << sol.avg_h << ",";
    std::cout << sol.start_h << "\n\n";

    for (std::shared_ptr<Node> &i : sol.sol) {
        i->print();
        std::cout << "\n";
    }
}



