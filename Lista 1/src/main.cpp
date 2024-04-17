#include <bits/stdc++.h>

#include "solvers/solvers.cpp"
#include "state/state8.cpp"

int main(int argc, char **argv) {
    std::vector<int> testState = {2, 4, 7, 0, 3, 6, 8, 1, 5};

    State8 state = State8(testState);

    ret_info sol = greedy_best_first<State8>(state);

    std::cout << "\n";
    std::cout << sol.expanded << ",";
    std::cout << sol.sol.size() - 1 << ",";
    std::cout << sol.time << std::setprecision(5) << ",";
    std::cout << (float) sol.avg_h / sol.expanded << std::setprecision(5) << ",";
    std::cout << sol.start_h << "\n\n";

    /*for (std::shared_ptr<Node> &i : sol.sol) {
        i->print();
        std::cout << "\n";
    }*/
}



