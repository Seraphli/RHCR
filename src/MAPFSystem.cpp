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
  if (std_out)
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
  if (std_out)
    std::cout << "Minimal timestep " << min_timestep << std::endl;

  for (; timestep < min_timestep; timestep += simulation_window) {
    if (std_out)
      std::cout << "Timestep " << timestep << std::endl;

    // move drives
    auto new_finished_tasks = move();
    if (std_out)
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
  if (std_out)
    std::cout << std::endl << "Done!" << std::endl;
  save_results();
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
  if (std_out)
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
  if (std_out)
    std::cout << "Done! (" << runtime << " s)" << std::endl;
}