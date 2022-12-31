#ifndef SRC_PSO_H
#define SRC_PSO_H

#include <time.h>
#include "../common/parameter.h"
#include "../common/utility.h"
#include <fstream>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

struct acc_grid
{
	double cover_rate = 0;
	bool is_bord = false;
	bool occupy = false;
};

class particle
{
public:
	particle() {};
	~particle() {};
	utility::State  state;
	double Pbest_fitness;
	utility::State Pbest_state;
};

class PSO
{
public:
	PSO(){};
	~PSO(){};
	void creatParticleSwarm(int particle_num);
	void destoryParticleSwarm();
	void getNextPoint(utility::MAP &global_map, utility::RADAR *radar,
								  utility::OBSTACLE *obs, vector<utility::TARGET> &target, 
								  vector<utility::UAV> &uav, int cur_T, int uav_idx);

private:
	void spreadSwarm();
	void updateSwarmStates();
	void updateParticleStates(particle* particle, int iteration);
	double calculateFitness(particle* particle);

	particle *swarm_;
	int particle_num_ ;
	int border_ ;

	double Gbest_fitness_;
	utility::State Gbest_state_;

	vector<utility::UAV> *uav_;
	utility::UAV *cur_uav_;
	utility::MAP global_map_;
	utility::RADAR *radar_;
	vector<utility::TARGET> *target_;
	utility::OBSTACLE *obs_;
	double min_R_;
	int cur_T_;
	utility::point2D area_guider_;
	double base_angle_;

	ofstream output_particle_state;
	ofstream output_uav_state;
	ofstream output_best_state;
	vector<cv::Mat> layers;
	
	
};

#endif

