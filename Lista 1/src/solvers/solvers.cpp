#ifndef SOLVERS_H
#define SOLVERS_H

#include "../state/state8.cpp"
#include "../state/state16.cpp"
#include <deque>
#include <map>

struct ret_info {
    int expanded = 0;
    double time = 0.;
    std::vector<std::shared_ptr<Node>> sol = {};
    int start_h = 0;
    int avg_h = 0;
};

template<typename State>
ret_info bfs_graph(State state) {
    ret_info ret;
    clock_t start, end;
    start = clock();

    std::shared_ptr<Node> n = state.init();
    ret.start_h = state.h_value(n);
    if (state.is_goal(n)) {
        ret.sol = state.extract_path(n);
        end = clock();
        ret.time = double(end - start) / double(CLOCKS_PER_SEC);
        return ret;
    }

    std::deque<std::shared_ptr<Node>> open = std::deque<std::shared_ptr<Node>>();
    open.push_back(n);

    std::map<std::string, bool> closed = std::map<std::string, bool>();
    closed[n->toString()] = true;

    while (!open.empty()) {

        std::shared_ptr<Node> front = open.front();
        open.pop_front();
        ret.expanded += 1;

        for (std::shared_ptr<Node> next: state.succ(front)) {
            if (state.is_goal(next)) {
                ret.sol = state.extract_path(next);
                end = clock();
                ret.time = double(end - start) / double(CLOCKS_PER_SEC);
                return ret;
            }
            if (!closed[next->toString()]) {
                closed[next->toString()] = true;
                open.push_back(next);
            }
        }
    }

    end = clock();
    ret.time = double(end - start) / double(CLOCKS_PER_SEC);
    return ret;
}

#endif