#include <bits/stdc++.h>

#include "solvers/solvers.cpp"
#include "state/state8.cpp"
#include "state/state15.cpp"

#define FILE_RUN 1
#define INPUT_USED 0
#ifdef FILE_RUN
#include <fstream>
#include <string>
#include <sstream>
#endif

template<typename State>
void solve(std::string alg, State s, ret_info *solver_out=NULL);


std::vector<std::vector<int>> parse_states(int argc, char **argv);

int main(int argc, char **argv) {
    if (FILE_RUN == 0){
        if (argc < 2) {
            std::cout << "Invalid solver choice\n";
            return 0;
        } else {
            for (std::vector<int> state : parse_states(argc, argv)) {
                if (state.size() == 9)
                    solve(argv[1], State8(state), NULL);
                else solve(argv[1], State15(state), NULL);
            }
        }
    } else {
        int instance_count = 0;
        std::ifstream in_state8("15puzzle_instances.txt");
        std::ofstream out_state8("astar_15puzzle_output.txt");
        long unsigned int avg_expanded = 0;
        long double avg_time = 0.0;
        long int avg_start_h = 0;
        long int avg_avg_h = 0;
        long int avg_sol_size = 0;
        std::string line, out_line;
        while(std::getline(in_state8, line)) {
            //std::cout << "here" << std::endl;
            ret_info solver_out;
            int aux;
            std::istringstream iss(line);
            std::vector<int> state;
            while ((iss >> aux)) {
                state.push_back(aux);
            }
            //std::cout << "solve" << std::endl;
            solve(argv[1], State15(state), &solver_out);
            ++instance_count;
            avg_expanded += solver_out.expanded;
            avg_time += solver_out.time;
            avg_start_h += solver_out.start_h;
            avg_avg_h += solver_out.avg_h;
            avg_sol_size += solver_out.sol.size()-1;
            out_line = std::to_string(solver_out.expanded) + " " + std::to_string(solver_out.time) +
            " " + std::to_string(solver_out.start_h) + " " + std::to_string(solver_out.avg_h) + " " + std::to_string(solver_out.sol.size());
            out_state8 << out_line << std::endl;
            //printf("\n###############################################################################\n");
        }
        avg_expanded /= instance_count;
        avg_time /= instance_count;
        avg_start_h /= instance_count;
        avg_avg_h /= instance_count;
        avg_sol_size /= instance_count;
        out_state8 << "###############################################################################" << std::endl;
        out_line = std::to_string(avg_expanded)+" "+std::to_string(avg_time)+" "+std::to_string(avg_start_h)+" "+std::to_string(avg_avg_h)+" "+std::to_string(avg_sol_size);
        out_state8 << out_line << std::endl;
    }
}

template<typename State>
void solve(std::string alg, State s, ret_info *solver_out) {
    std::function<ret_info(State)> f;

    if (alg == "-bfs")
        f = bfs_graph<State>;
    if (alg == "-idastar")
        f = idastar<State>;
    if (alg == "-idfs")
        f = iter_deep<State>;
    if (alg == "-gbfs")
        f = greedy_best_first<State>;
    if (alg == "-astar")
        f = astar<State>;

    ret_info sol = f(s);
    *solver_out = sol;

    /* std::cout << sol.expanded << ",";
    std::cout << sol.sol.size() - 1 << ",";
    std::cout << sol.time << std::setprecision(5) << ",";
    std::cout << (float) sol.avg_h / sol.expanded << std::setprecision(5) << ",";
    std::cout << sol.start_h << "\n"; */
}

std::vector<std::vector<int>> parse_states(int argc, char **argv) {
    std::vector<std::vector<int>> ret;
    std::vector<int> curr;
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg.size() == 2 && arg[1] != ',') || arg.size() == 3) {
            curr.push_back(stoi(arg.substr(0, 2)));
        } else {
            curr.push_back(arg[0] - '0');
        }

        if (arg[arg.size() - 1] == ',') {
            ret.push_back(curr);
            curr.clear();
        }
    }
    ret.push_back(curr);

    return ret;
}

