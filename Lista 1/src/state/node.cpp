#ifndef NODE_H
#define NODE_H

#include <utility>
#include <vector>

enum Action {
    up = 0,
    down = 1,
    right = 2,
    left = 3
};

class Node {
public:
    Node(std::vector<int> state, std::shared_ptr<Node> parent, int cost, int action);

    void print();

    std::string toString();

    std::vector<int> state;
    std::shared_ptr<Node> parent = nullptr;
    int path_cost;
    int action;
private:
};

Node::Node(std::vector<int> state, std::shared_ptr<Node> parent, int cost, int action) {
    this->state = std::move(state);
    this->parent = std::move(parent);
    this->path_cost = cost;
    this->action = action;
}

void Node::print() {
    std::cout << "State: ";
    for (int i : state) {
        std::cout << i << " ";
    }
    std::cout << "\nCost: " << path_cost;
    std::cout << "\nAction: " << action << "\n";
}

std::string Node::toString() {
    std::string ret;
    for (int i : state) {
        ret.push_back(i + '0');
    }
    return ret;
}

#endif