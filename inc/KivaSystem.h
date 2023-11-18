#pragma once
#include "BasicSystem.h"
#include "KivaGraph.h"

class KivaSystem :
	public BasicSystem
{
public:
	KivaSystem(KivaGrid& G, MAPFSolver& solver);
	~KivaSystem();

	void simulate(int simulation_time);


private:
	KivaGrid& G;
	unordered_set<int> held_endpoints;

	void initialize();
	void initialize_start_locations();
	void initialize_goal_locations();
	void update_goal_locations();
};

