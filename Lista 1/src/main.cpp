#include <bits/stdc++.h>

#include "solvers/solvers.cpp"
#include "state/state8.cpp"
//#include "state/state16.cpp"

int main(int argc, char **argv) {
    std::vector<int> testState = {0, 6, 1, 7, 4, 2, 3, 8, 5};
    //std::vector<int> testState = {7, 11, 8, 3, 14, 0, 6, 15, 1, 4, 13, 9, 5, 12, 2, 10};

    State8 state = State8(testState);
    //State16 state = State16(testState);

    ret_info sol = idastar<State8>(state);
    //ret_info sol = astar<State16>(state);

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



