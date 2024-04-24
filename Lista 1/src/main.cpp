#include <bits/stdc++.h>

#include "solvers/solvers.cpp"
#include "state/state8.cpp"

template<typename State>
void solve(std::string alg, State s);

std::vector<std::vector<int>> parse_states(int argc, char **argv);

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "Invalid solver choice\n";
        return 0;
    } else {
        for (std::vector<int> state : parse_states(argc, argv)) {
            if (state.size() == 9)
                solve(argv[1], State8(state));
            // else solve(argv[1], State16(state));
        }
    }
}

template<typename State>
void solve(std::string alg, State s) {
    std::function<ret_info(State)> f;

    if (alg == "-bfs")
        f = bfs_graph<State>;
    if (alg == "-idastar")
        {}
    if (alg == "-idfs")
        f = iter_deep<State>;
    if (alg == "-gbfs")
        f = greedy_best_first<State>;
    if (alg == "-astar")
        f = astar<State>;

    ret_info sol = f(s);

    std::cout << sol.expanded << ",";
    std::cout << sol.sol.size() - 1 << ",";
    std::cout << sol.time << std::setprecision(5) << ",";
    std::cout << (float) sol.avg_h / sol.expanded << std::setprecision(5) << ",";
    std::cout << sol.start_h << "\n";

    /*for (std::shared_ptr<Node> &i : sol.sol) {
        i->print();
        std::cout << "\n";
    }*/
}

std::vector<std::vector<int>> parse_states(int argc, char **argv) {
    std::vector<std::vector<int>> ret;
    std::vector<int> curr;

    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        curr.push_back(arg[0] - '0');
        if (arg.size() == 2) {
            ret.push_back(curr);
            curr.clear();
        }
    }
    ret.push_back(curr);

    return ret;
}

