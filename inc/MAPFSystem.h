#pragma once
#include "BasicSystem.h"
#include "MAPFGraph.h"

class MAPFSystem : public BasicSystem {
public:
  MAPFSystem(MAPFGraph &G, MAPFSolver &solver);
  ~MAPFSystem();

  bool load_scen(string fname);
  void simulate(int simulation_time);

private:
  MAPFGraph &G;
  vector<int> initial_locations; // one task sequence per agent
  vector<int> task_sequences; // one task sequence per agent

  void initialize();
  void update_goal_locations();
  void save_results();
  list<tuple<int, int, int>> move();
};
