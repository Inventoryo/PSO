#ifndef SRC_ACO_H
#define SRC_ACO_H


#include "../common/utility.h"
#include "../common/parameter.h"
#include <vector>
#include <fstream>
#include <time.h>
#include <string.h>
using namespace std;
struct ANT {
	utility::point2I position;
	int dir;
	bool alive =  false;
	bool reach_bord =false ;
	utility::point2I path[5000];
	int path_len = 0;
	double global_gain;
};

struct info_grid
{
	double info=1.0;
	double cover_rate=0;
	bool is_bord=false;
	bool occupy = false;
};

class ACO
{
public:
	ACO();
	~ACO();
	void resetAntColony();
	void initInfoMap( utility::MAP &global_map);
	void updateInfoMap();
	void transitionState();
	void transitionState_ones(int idx);
	void setParam(int ant_num, int max_iteration, double alpha, double beta, double initial_ph,double rio);
	utility::point2I run( utility::MAP &global_map, utility::point2I cur_uav_pose, int cur_uav_direction, int R, int uav_idx,int cur_T);
private:
	info_grid *info_map_;
	int height_, width_;
	
	char *visited_;

	ANT *ant_colony_;
	utility::point2I shift_;
	utility::point2I nextWayPoints_;
	utility::point2I prevWayPoints_;
	utility::point2I uav_pose_;
	int cur_uav_direction_ ;
	double shortest_path_cov_;
	int search_R_;
	int uav_idx_;
	int T_;
	
	//param
	double alpha_, beta_, initial_ph_, rio_;
	int ant_num_, max_iteration_;

	//out_put
	ofstream output_ants;
	ofstream output_time;
	double  duration=0;
};

#endif