#ifndef SRC_PARTICLESWARM_H
#define SRC_PARTICLESWARM_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <set>
#include <time.h>

#include "../common/utility.h"
#include "../common/dubins.h"
#include "../common/parameter.h"
#include "../aco_ws/aco.h"
#include "../pso_ws/pso.h"

using namespace std;


class MultipleSearchAndTrack
{
public:
	MultipleSearchAndTrack(){};
	~MultipleSearchAndTrack(){};
	void run();
	void init();
	void destory();
	void show(utility::UAV *uav, utility::TARGET *target); //return the current state of uav swarm and target group
private:

	void updateUAVStatesInDubinsState(utility::UAV * uav);
	void updateArtificalPotentialFieldStateImpl(utility::UAV * uav, utility::State& goal, utility::State&next);
	utility::point2D computeRepulsion(utility::point2D& start, utility::point2D& obstacle);
	double computeEnage(utility::State& start, utility::point2D& obstacle,double zero_dist);
	double computeAttEnage(utility::State& start, utility::State& end, double zero_dist);
	utility::point2D computeAttraction(utility::point2D& start, utility::point2D& goal);
	void updateTargetStates();
	void updateUAVStates();
	void informationShare();
	void updateMission();

	float computeUncetanty(utility::State& state, double search_r, int id);
	double sigmod(double x){ return 1.0 / (1 + exp(-x)); };
	double computeAngle(utility::State& start, utility::State& goal);

	vector<utility::MAP> global_map_;

	vector<utility::TARGET> target_;
	vector<utility::UAV> uav_;
	utility::RADAR *radar_;
	utility::OBSTACLE *obs_;
	ACO aco_;
	PSO pso_;
	//PIO pio_;
	//output
	ofstream output_uav;
	ofstream output_target;
	ofstream output_traj_Point;
	ofstream output_area_Point;
	ofstream output_coverage_rate;
	ofstream output_time;

	//others
	utility::nion union_;//Disjoint Set
	vector<vector<double>> *target_state_;
	vector<bool> *tracked_;
	HybridAStar::DubinsPath *path_;
	int cunt = 0;
	int opt_cunt = 0;
	double  duration_=0;
};

#endif
