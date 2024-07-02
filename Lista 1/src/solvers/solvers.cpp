#ifndef SOLVERS_H
#define SOLVERS_H

#include "../state/node.cpp"
#include <deque>
#include <map>

const int MAX_DEPTH = 1e7;
const int MAX_H = 100;
const float MAX_TIME = 180.0;

struct ret_info {
    int expanded = 0;
    double time = 0.;
    std::vector<std::shared_ptr<Node>> sol = {};
    int start_h = 0;
    int avg_h = 0;
};

/** BFS GRAPH */
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


/** ITERATIVE DEEPENING */
template<typename State>
ret_info iter_deep(State state) {
    ret_info ret;
    clock_t start, end;
    start = clock();

    std::shared_ptr<Node> n = state.init();
    ret.start_h = state.h_value(n);

    for (int i = 0; i < MAX_DEPTH; ++i) {
        dfs_iter_deep(n, &state, &ret, i);
        if (!ret.sol.empty())
            break;
    }

    end = clock();
    ret.time = double(end - start) / double(CLOCKS_PER_SEC);
    return ret;
}

template<typename State>
void dfs_iter_deep(std::shared_ptr<Node> n, State *state, ret_info *ret, int depth) {

    std::string v = n->toString(); v.push_back((char) depth);

    if (state->is_goal(n)) {
        ret->sol = state->extract_path(n);
        return;
    }

    if (depth > 0) {
        ret->expanded += 1;
        for (std::shared_ptr<Node> next: state->succ(n)) {
            dfs_iter_deep(next, state, ret, depth - 1);
            if (!ret->sol.empty())
                return;
        }
    }
}


/** GREEDY BEST FIRST SEARCH */
struct QueueNode {
    std::shared_ptr<Node> node;
    int h;
    int g;
    int pos;
};

class GreedyCompare {
public:
    bool operator()(QueueNode f, QueueNode s) {
        if (f.h > s.h)
            return true;
        if (f.h == s.h && f.g < s.g)
            return true;
        if (f.h == s.h && f.g == s.g && f.pos < s.pos)
            return true;
        return false;
    }
};

template<typename State>
ret_info greedy_best_first(State state) {
    ret_info ret;
    clock_t start, end;
    start = clock();

    std::shared_ptr<Node> n = state.init();
    ret.start_h = state.h_value(n);

    int cur_pos = 0;

    std::priority_queue<QueueNode, std::deque<QueueNode>, GreedyCompare> open;
    open.push({
                      n,
                      state.h_value(n),
                      n->path_cost,
                      ++cur_pos
              });

    std::map<std::string, bool> closed = std::map<std::string, bool>();

    while (!open.empty()) {
        QueueNode front = open.top();
        open.pop();

        if (!closed[front.node->toString()]) {
            closed[front.node->toString()] = true;
            if (state.is_goal(front.node)) {
                ret.sol = state.extract_path(front.node);
                end = clock();
                ret.time = double(end - start) / double(CLOCKS_PER_SEC);
                return ret;
            }

            ret.expanded += 1;
            ret.avg_h += state.h_value(front.node);

            for (std::shared_ptr<Node> next: state.succ(front.node)) {
                open.push(QueueNode{
                        next,
                        state.h_value(next),
                        next->path_cost,
                        ++cur_pos
                });
            }
        }
    }

    end = clock();
    ret.time = double(end - start) / double(CLOCKS_PER_SEC);
    return ret;
}


/** A* */
class AstarCompare {
public:
    bool operator()(QueueNode f, QueueNode s) {
        if (f.h > s.h)
            return true;
        if (f.h == s.h && f.g < s.g)
            return true;
        if (f.h == s.h && f.g == s.g && f.pos < s.pos)
            return true;
        return false;
    }
};

template<typename State>
ret_info astar(State state) {
    ret_info ret;
    clock_t start, end;
    start = clock();

    std::shared_ptr<Node> n = state.init();
    ret.start_h = state.h_value(n);

    int cur_pos = 0;

    std::priority_queue<QueueNode, std::deque<QueueNode>, AstarCompare> open;
    open.push({
                      n,
                      state.h_value(n) + n->path_cost,
                      n->path_cost,
                      ++cur_pos
              });

    std::map<std::string, bool> closed = std::map<std::string, bool>();

    while (!open.empty()) {
        QueueNode front = open.top();
        open.pop();

        end = clock();
        if (double(end - start) / double(CLOCKS_PER_SEC) >= MAX_TIME) return {};

        if (!closed[front.node->toString()]) {
            closed[front.node->toString()] = true;
            if (state.is_goal(front.node)) {
                ret.sol = state.extract_path(front.node);
                end = clock();
                ret.time = double(end - start) / double(CLOCKS_PER_SEC);
                return ret;
            }

            ret.expanded += 1;
            ret.avg_h += state.h_value(front.node);

            for (std::shared_ptr<Node> next: state.succ(front.node)) {
                open.push(QueueNode{
                        next,
                        state.h_value(next) + next->path_cost,
                        next->path_cost,
                        ++cur_pos
                });
            }
        }
    }

    end = clock();
    ret.time = double(end - start) / double(CLOCKS_PER_SEC);
    return ret;
}


/** IDA* */
template<typename State>
int dfs_idastar(std::shared_ptr<Node> n, State *state, ret_info *ret, int f_limit) {

    int f = state->h_value(n) + n->path_cost;
    if (f > f_limit) {
        return f;
    }

    if (state->is_goal(n)) {
        ret->sol = state->extract_path(n);
        return 0;
    }

    ret->avg_h += state->h_value(n);
    ret->expanded += 1;

    int next_limit = INT_MAX;
    for (std::shared_ptr<Node> next: state->succ(n)) {
        int rec_limit = dfs_idastar(next, state, ret, f_limit);
        if (!ret->sol.empty())
            return 0;
        next_limit = std::min(next_limit, rec_limit);
    }
    
    return next_limit;
}

template<typename State>
ret_info idastar(State state) {
    ret_info ret;
    clock_t start, end;
    start = clock();

    std::shared_ptr<Node> n = state.init();
    ret.start_h = state.h_value(n);
    int f_limit = state.h_value(n);

    while(f_limit <= MAX_H) {
        f_limit = dfs_idastar(n, &state, &ret, f_limit);
        if (!ret.sol.empty())
            break;
    }

    end = clock();
    ret.time = double(end - start) / double(CLOCKS_PER_SEC);
    return ret;
}

#endif