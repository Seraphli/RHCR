#include "MAPFSystem.h"
#include "ECBS.h"
#include "LRAStar.h"
#include "PBS.h"
#include "WHCAStar.h"
#include <boost/tokenizer.hpp>
#include <regex>
#include <stdlib.h>
#include <utility>

MAPFSystem::MAPFSystem(MAPFGraph &G, MAPFSolver &solver)
    : BasicSystem(G, solver), G(G) {}

MAPFSystem::~MAPFSystem() {}

static const std::regex r_instance =
    std::regex(R"(\d+\t.+\.map\t\d+\t\d+\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t.+)");

bool MAPFSystem::load_scen(string scen_filename) {
  // load start-goal pairs
  std::ifstream file(scen_filename);
  if (!file) {
    return false;
  }
  std::string line;
  std::smatch results;

  int k = 0;
  int orientation = -1;
  initial_locations.resize(num_of_drives);
  task_sequences.resize(num_of_drives);
  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end() - 1) == 0x0d)
      line.pop_back();

    if (std::regex_match(line, results, r_instance)) {
      auto x_s = std::stoi(results[1].str());
      auto y_s = std::stoi(results[2].str());
      auto x_g = std::stoi(results[3].str());
      auto y_g = std::stoi(results[4].str());
      if (x_s < 0 || G.get_cols() <= x_s || x_g < 0 || G.get_cols() <= x_g)
        continue;
      if (y_s < 0 || G.get_rows() <= y_s || y_g < 0 || G.get_rows() <= y_g)
        continue;
      int s = G.get_cols() * y_s + x_s;
      // starts[k] = State(s, 0, orientation);
      // paths[k].emplace_back(starts[k]);
      initial_locations[k] = s;
      int g = G.get_cols() * y_g + x_g;
      task_sequences[k] = g;
      k += 1;
      if (k == num_of_drives)
        break;
    }
  }
  return true;
}

void MAPFSystem::update_goal_locations() {}

void MAPFSystem::simulate(int simulation_time) {
  std::cout << "*** Simulating " << seed << " ***" << std::endl;
  this->simulation_time = simulation_time;
  initialize();
  update_start_locations();
  update_goal_locations();
  solve();

  int min_timestep = simulation_time;
  for (int k = 0; k < num_of_drives; k++) {
    min_timestep = min(min_timestep, (int)paths[k].size() - 1);
  }
  std::cout << "Minimal timestep " << min_timestep << std::endl;

  for (; timestep < min_timestep; timestep += simulation_window) {
    std::cout << "Timestep " << timestep << std::endl;

    // move drives
    auto new_finished_tasks = move();
    std::cout << new_finished_tasks.size() << " tasks has been finished"
              << std::endl;

    // update tasks
    for (auto task : new_finished_tasks) {
      int id, loc, t;
      std::tie(id, loc, t) = task;
      finished_tasks[id].emplace_back(loc, t);
      num_of_tasks++;
    }

    // if (congested()) {
    //   cout << "***** Too many traffic jams ***" << endl;
    //   break;
    // }
  }

  update_start_locations();
  std::cout << std::endl << "Done!" << std::endl;
  save_results();
}

// move all agents from start_timestep to end_timestep
// return a list of finished tasks
list<tuple<int, int, int>> MAPFSystem::move() {
  int start_timestep = timestep;
  int end_timestep = timestep + simulation_window;

  list<tuple<int, int, int>> finished_tasks; // <agent_id, location, timestep>

  for (int t = start_timestep; t <= end_timestep; t++) {
    for (int k = 0; k < num_of_drives; k++) {
      // Agents waits at its current locations if no future paths are assigned
      while ((int)paths[k].size() <= t) { // This should not happen?
        State final_state = paths[k].back();
        paths[k].emplace_back(final_state.location, final_state.timestep + 1,
                              final_state.orientation);
      }
    }
  }

  for (int t = start_timestep; t <= end_timestep; t++) {
    for (int k = 0; k < num_of_drives; k++) {
      State curr = paths[k][t];

      /* int wait_times = 0; // wait time at the current location
      while (wait_times < t && paths[k][t - wait_times] != curr)
      {
          wait_times++;
      }*/

      // remove goals if necessary
      if ((!hold_endpoints || paths[k].size() == t + 1) &&
          !goal_locations[k].empty() &&
          curr.location == goal_locations[k].front().first &&
          curr.timestep >= goal_locations[k]
                               .front()
                               .second) // the agent finish its current task
      {
        goal_locations[k].erase(goal_locations[k].begin());
        finished_tasks.emplace_back(k, curr.location, t);
      }

      // check whether the move is valid
      if (t > 0) {
        State prev = paths[k][t - 1];

        if (curr.location == prev.location) {
          if (G.get_rotate_degree(prev.orientation, curr.orientation) == 2) {
            cout << "Drive " << k << " rotates 180 degrees from " << prev
                 << " to " << curr << endl;
            save_results();
            exit(-1);
          }
        } else if (consider_rotation) {
          if (prev.orientation != curr.orientation) {
            cout << "Drive " << k << " rotates while moving from " << prev
                 << " to " << curr << endl;
            save_results();
            exit(-1);
          } else if (!G.valid_move(prev.location, prev.orientation) ||
                     prev.location + G.move[prev.orientation] !=
                         curr.location) {
            cout << "Drive " << k << " jump from " << prev << " to " << curr
                 << endl;
            save_results();
            exit(-1);
          }
        } else {
          int dir = G.get_direction(prev.location, curr.location);
          if (dir < 0 || !G.valid_move(prev.location, dir)) {
            cout << "Drive " << k << " jump from " << prev << " to " << curr
                 << endl;
            save_results();
            exit(-1);
          }
        }
      }

      // Check whether this move has conflicts with other agents
      if (G.types[curr.location] != "Magic") {
        for (int j = k + 1; j < num_of_drives; j++) {
          for (int i = max(0, t - k_robust);
               i <= min(t + k_robust, end_timestep); i++) {
            if ((int)paths[j].size() <= i)
              break;
            if (paths[j][i].location == curr.location) {
              cout << "Drive " << k << " at " << curr
                   << " has a conflict with drive " << j << " at "
                   << paths[j][i] << endl;
              save_results(); // TODO: write termination reason to files
              exit(-1);
            }
          }
        }
      }
    }
  }
  return finished_tasks;
}

void MAPFSystem::initialize() {
  initialize_solvers();

  starts.resize(num_of_drives);
  goal_locations.resize(num_of_drives);
  paths.resize(num_of_drives);
  finished_tasks.resize(num_of_drives);
  timestep = 0;

  new_agents.clear();
  for (int k = 0; k < num_of_drives; k++) {
    int orientation = -1;
    if (consider_rotation) {
      orientation = rand() % 4;
    }
    int loc = initial_locations[k];
    starts[k] = State(loc, 0, orientation);
    paths[k].emplace_back(starts[k]);
    finished_tasks[k].emplace_back(loc, 0);

    auto next = task_sequences[k];
    goal_locations[k].emplace_back(next, 0);
    new_agents.emplace_back(k);
  }
}

void MAPFSystem::save_results() {
  std::cout << "*** Saving " << seed << " ***" << std::endl;
  clock_t t = std::clock();
  std::ofstream output;

  // settings
  output.open(outfile + "/config.txt", std::ios::out);
  output << "map: " << G.map_name << std::endl
         << "#drives: " << num_of_drives << std::endl
         << "seed: " << seed << std::endl
         << "solver: " << solver.get_name() << std::endl
         << "time_limit: " << time_limit << std::endl
         << "simulation_window: " << simulation_window << std::endl
         << "planning_window: " << planning_window << std::endl
         << "simulation_time: " << simulation_time << std::endl
         << "robust: " << k_robust << std::endl
         << "rotate: " << consider_rotation << std::endl
         << "use_dummy_paths: " << useDummyPaths << std::endl
         << "hold_endpoints: " << hold_endpoints << std::endl;

  output.close();

  // paths
  output.open(outfile + "/paths.txt", std::ios::out);
  output << paths.size() << std::endl;
  for (const auto &path : paths) {
    for (auto p : path) {
      if (p.timestep <= timestep)
        output << p << ";";
    }
    output << std::endl;
  }
  output.close();
  double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
  std::cout << "Done! (" << runtime << " s)" << std::endl;
}