#ifndef STATE8_H
#define STATE8_H

#include <memory>
#include <utility>
#include <vector>
#include "node.cpp"

enum Action {
    up = 0,
    down = 1,
    right = 2,
    left = 3
};

class State8 {
public:
    State8(std::vector<int> state);

    std::shared_ptr<Node> init();

    static bool is_goal(std::shared_ptr<Node> node);

    std::vector<std::shared_ptr<Node>> succ(std::shared_ptr<Node>);

    int cost(int action);

    int h_value(std::shared_ptr<Node> node);

    std::vector<std::shared_ptr<Node>> extract_path(std::shared_ptr<Node> node);

private:
    std::vector<int> s_state;

};

State8::State8(std::vector<int> state) {
    this->s_state = std::move(state);
}

std::shared_ptr<Node> State8::init() {
    return std::make_shared<Node>(s_state, nullptr, 0, -1);
}

bool State8::is_goal(std::shared_ptr<Node> node) {
    for (int i = 0; i < 9; ++i) {
        if (node->state[i] != i) return false;
    }
    return true;
}

std::vector<std::shared_ptr<Node>> State8::succ(std::shared_ptr<Node> node) {
    std::vector<std::shared_ptr<Node>> ret;
    int idx = -1;
    for (long unsigned int i = 0; i < node->state.size(); ++i) {
        if (node->state[i] == 0) idx = (int) i;
    }

    if (idx / 3 > 0) {
        std::vector<int> n_state = node->state;
        n_state[idx] = n_state[idx - 3];
        n_state[idx - 3] = 0;
        ret.emplace_back(std::make_shared<Node>(n_state,
                                                node,
                                                node->path_cost + cost(Action::up),
                                                Action::up));
    }

    if (idx % 3 > 0) {
        std::vector<int> n_state = node->state;
        n_state[idx] = n_state[idx - 1];
        n_state[idx - 1] = 0;
        ret.emplace_back(std::make_shared<Node>(n_state,
                                                node,
                                                node->path_cost + cost(Action::left),
                                                Action::left));
    }

    if (idx % 3 < 2) {
        std::vector<int> n_state = node->state;
        n_state[idx] = n_state[idx + 1];
        n_state[idx + 1] = 0;
        ret.emplace_back(std::make_shared<Node>(n_state,
                                                node,
                                                node->path_cost + cost(Action::right),
                                                Action::right));
    }

    if (idx / 3 < 2) {
        std::vector<int> n_state = node->state;
        n_state[idx] = n_state[idx + 3];
        n_state[idx + 3] = 0;
        ret.emplace_back(std::make_shared<Node>(n_state,
                                                node,
                                                node->path_cost + cost(Action::down),
                                                Action::down));
    }

    return ret;
}

int State8::cost(int action) {
    return 1;
}

int State8::h_value(std::shared_ptr<Node> node) {
    int h = 0;
    for (int i = 0; i < (int) node->state.size(); ++i) {
        if (node->state[i] == 0) continue;
        int cidx = node->state[i];
        h += abs(i % 3 - cidx % 3) + abs(i / 3 - cidx / 3);
    }
    return h;
}

std::vector<std::shared_ptr<Node>> State8::extract_path(std::shared_ptr<Node> node) {
    std::vector<std::shared_ptr<Node>> ret;
    while (true) {
        ret.push_back(node);
        if (node->parent == nullptr)
            break;
        else
            node = node->parent;
    }
    return ret;
}

#endif

