
#include "pso.h"

using namespace std;
#define DEBUG 1
#define DEBUG_IDX 0

int Fdrections[9][2] = { { -1, -1 }, { 0, -1 }, { 1, -1 },
						 { -1,  0 }, { 0,  0 }, { 1,  0 },
						 { -1,  1 }, { 0,  1 }, { 1,  1 }, };

utility::point2D findBestPoints(const vector<cv::Mat> &layers, const utility::State & uav_state, int border) {
	int layer_idx = MAX_LAYER_NUM - 2;
	int x_idx = uav_state.position.x / RESOLUTION;
	x_idx = min(max(0, x_idx), WIDTH - 1) + border;
	x_idx >>= layer_idx ;

	int y_idx = uav_state.position.y / RESOLUTION;
	y_idx = min(max(0, y_idx), HEIGHT - 1) + border;
	y_idx >>= layer_idx ;
	
	int uav_dir = 4 * (uav_state.velocity.y + 3 * PI / 4) / PI;
	uav_dir = uav_dir % 8;

	int max_val = 0;
	int best_x, best_y;
	int temp_x, temp_y;

	for (int dir_idx = 0; dir_idx < 8; dir_idx++) {
		temp_x = x_idx + DIRECTION[dir_idx][0];
		temp_x = min(max(0, temp_x), (int)layers[layer_idx].cols - 1);
		temp_y = y_idx + DIRECTION[dir_idx][1];
		temp_y = min(max(0, temp_y), (int)layers[layer_idx].rows - 1);
		int dir_gain = (8 - abs(uav_dir - dir_idx));
		if (max_val <= dir_gain * layers[layer_idx].at<uint16_t>(temp_y, temp_x)) {
			max_val = dir_gain * layers[layer_idx].at<uint16_t>(temp_y, temp_x);
			best_x = temp_x;
			best_y = temp_y;
		}
	}
	x_idx = best_x ;//top layer best
	y_idx = best_y ;

	/*
	while (layer_idx--) {
		max_val = 0;
		for (int j = 0; j < 4; j++) {
			temp_x = x_idx + Fdrections[j][0];
			temp_x = min(max(0, temp_x), (int)layers[layer_idx].cols - 1);
			temp_y = y_idx + Fdrections[j][1];
			temp_y = min(max(0, temp_y), (int)layers[layer_idx].rows - 1);
			if (max_val <= layers[layer_idx].at<uint16_t>(temp_y, temp_x)) {
				max_val = layers[layer_idx].at<uint16_t>(temp_y, temp_x);
				best_x = temp_x;
				best_y = temp_y;
			}
		}
		x_idx = best_x << 1 ;
		y_idx = best_y << 1 ;
	}
	*/
	x_idx = (best_x + 0.5) * pow(2, layer_idx) - border;
	y_idx = (best_y + 0.5) * pow(2, layer_idx) - border;
	return utility::point2D(x_idx * RESOLUTION, y_idx * RESOLUTION);
}

double getUncertanty(const vector<cv::Mat> &layers, const double x, const double y, int border){
	double res = 0 ;
	int x_idx = x / RESOLUTION;
	x_idx = x_idx < 0 ? 0 : x_idx >= WIDTH ? WIDTH - 1 : x_idx;
	x_idx += border;
	int y_idx = y / RESOLUTION;
	y_idx = y_idx < 0 ? 0 : y_idx >= HEIGHT ? HEIGHT - 1 : y_idx;
	y_idx += border;
	int layer_num = USED_LAYERS;
	int weight = 1;// << layer_num;
	double sum_w = 0;
	for(int i = 0; i < layer_num; i++){
		sum_w += weight;
		res += weight * layers[i].at<uint16_t>(y_idx, x_idx);
		x_idx >>= 1;
		y_idx >>= 1;
		weight <<= 1;
	}
	return res / sum_w;
}

void PSO::creatParticleSwarm(int particle_num)
{
	particle_num_ = particle_num;
	border_ = 64;
	swarm_ = (particle*)malloc(particle_num_ * sizeof(particle));
	int height = HEIGHT + 2 * border_;
	int width  = WIDTH  + 2 * border_;
	cout << "height" << height << "width" << width;
	for(uint8_t layer = 0 ;layer < MAX_LAYER_NUM;layer++){
		cv::Mat mat(height, width, CV_16UC1, cv::Scalar(0));
		layers.push_back(mat);
		height = (height + 1) / 2;//ceil
		width  = (width + 1) / 2;
	}
	
	cout << "height" << height << "width" << width;
	output_particle_state.open(particle_state_path, ios::binary);
	output_uav_state.open(uav_state_path, ios::binary);
	output_best_state.open(best_state_path, ios::binary);

	return;
};

void PSO::destoryParticleSwarm()
{
	output_particle_state.close();
	output_uav_state.close();
	output_best_state.close();
	free(swarm_);
	return;
};

void PSO::getNextPoint(utility::MAP &global_map, utility::RADAR *radar, utility::OBSTACLE *obs, vector<utility::TARGET> &target, vector<utility::UAV> &uav, int cur_T, int uav_idx)
{
	global_map_ = global_map;
	radar_ = radar;
	obs_ = obs;
	uav_ = &uav;
	cur_T_ = cur_T;
	target_ = &target;
	cur_uav_ = &(uav[uav_idx]);
	min_R_ = pow(cur_uav_->state.velocity.x, 2) / cur_uav_->Acc_limite_y[1];
	
	for(int i = 0; i < HEIGHT; i++){
		for(int j = 0; j < WIDTH; j++){
			layers[0].at<uint16_t>(i + border_, j + border_) = cur_T > (global_map.map_[i * WIDTH + j].search_time + forget_time) ? forget_time : (cur_T - global_map.map_[i * WIDTH + j].search_time);
		}
	}
	
	for (int layer_idx = 0; layer_idx < layers.size() - 1; layer_idx++) 
		cv::pyrDown(layers[layer_idx], layers[layer_idx + 1], cv::Size(layers[layer_idx + 1].cols, layers[layer_idx + 1].rows));
	
		//cv::resize(layers[layer_idx], layers[layer_idx + 1], cv::Size(layers[layer_idx + 1].cols, layers[layer_idx + 1].rows), 0, 0, CV_INTER_LINEAR);
	
	if (uav_idx == DEBUG_IDX)
		printf("debug points\n");

	area_guider_ = findBestPoints(layers, cur_uav_->state, border_);
	base_angle_ = atan2(area_guider_.y - cur_uav_->state.position.y, area_guider_.x - cur_uav_->state.position.x);//-pi~pi
	base_angle_ = base_angle_ >= 0 ? base_angle_ : 2 * PI + base_angle_;//0~2*pi

#if DEBUG
	cv::Mat temp;
	temp.copySize(layers[0]);
	cv::convertScaleAbs(layers[0], temp, 0.2);
    cv::namedWindow("Ô­Í¼Ïñ", cv::WINDOW_AUTOSIZE);

    cv::imshow("Ô­Í¼Ïñ", temp);
	cv::waitKey(10);
#endif

	spreadSwarm();
	updateSwarmStates();

	return;
};

void PSO::spreadSwarm() {
	srand((unsigned)time(NULL));
	particle* temp_particle = swarm_;
	Gbest_fitness_ = 0;
	int j = 0;
	while (j < particle_num_) {
		
		double length = 1.1 * cur_uav_->search_r + (cur_uav_->state.velocity.x) * (rand() % 1000) / 1000.0;
		double theta = 2 * PI * ((rand() % 1000) / 1000.0);
		temp_particle->state.position.x = length * cos(theta) + cur_uav_->state.position.x;
		temp_particle->state.position.y = length * sin(theta) + cur_uav_->state.position.y;
		
		if(!global_map_.isInBound(temp_particle->state.position) || radar_->isInRader(temp_particle->state.position) || obs_->isInObstacle(temp_particle->state.position))
			continue;

		temp_particle->state.velocity.x = cur_uav_->Vel_limite[0];//+ (cur_uav_->Vel_limite[1] - cur_uav_->Vel_limite[0]) * ((rand() % 1000) / 1000.0);//ï¿½ï¿½ï¿½Ù¶ï¿½
		temp_particle->state.velocity.y   = 2 * PI*((rand() % 1000) / 1000.0);
		temp_particle->state.accelerate.x = 0;
		temp_particle->state.accelerate.y = 0;
		temp_particle->Pbest_state        = temp_particle->state;
		temp_particle->Pbest_fitness      = 0;
		if (Gbest_fitness_ <= temp_particle->Pbest_fitness) {
			Gbest_state_   = temp_particle->state;
			Gbest_fitness_ = temp_particle->Pbest_fitness;
		}
		temp_particle++;
		j++;
	}
	return;
}

double PSO::calculateFitness(particle* particle){

	if (radar_->isInRader(particle->state.position))
		return -100000;
	if(obs_->isInObstacle(particle->state.position))
		return -100000;

	//fitness = (1-w)*fitness_1 + w*fitness_2
	double fitness = 0;
	double fitness_1 = 0;
	double fitness_2 = 0;
	double fitness_3 = 0;
	//fitness1
	//double t = global_map_->computeUncetanty(particle->state, cur_uav_->search_r, cur_T_);
	double t = getUncertanty(layers, particle->state.position.x, particle->state.position.y, border_) / forget_time;
	//double p  = (1 - exp(-tao * t)) ;
	double d = utility::dubinsDistance(cur_uav_->state, particle->state, min_R_);
	d = (d - cur_uav_->search_r) / RESOLUTION ;
	fitness_1 = t / (1.0 + d * d );

	double c;
	/*
	for (int target_id = 0; target_id < TARGET_NUM; target_id++) {//target influence
		if (cur_T_ - cur_uav_->target_state[target_id].second < forget_time) {//target has been spotted
			c = 1 + (*target_)[target_id].Vel_limite[1]/(cur_uav_->Vel_limite[1] + 1);
			d = utility::dist(cur_uav_->target_state[target_id].first.position, particle->state.position);
			d /= 5000;
			if(d < 1.1)
			fitness_2 += 1 * c / (10 + d * d );
		}
	}
	*/

	int cur_uav_num = cur_T_ < ADD_TIME ? uav_num - extra_uav_num : uav_num;
	for (int uav_id = 0; uav_id < cur_uav_num; uav_id++) {//uav avoidance
		c = cur_uav_->Vel_limite[1] /((*uav_)[uav_id].Vel_limite[1] + 1);
		d = utility::dist((*uav_)[uav_id].state.position, particle->state.position);
		d = d / ((*uav_)[uav_id].search_r + cur_uav_->search_r);
		if( d < 1.1 && uav_id != cur_uav_->id)
			fitness_2 -= 1 * c / (1.0 + d * d );
	}

	//fitness3
	double angle2 = atan2(particle->state.position.y - cur_uav_->state.position.y , particle->state.position.x - cur_uav_->state.position.x);//-pi~pi
	angle2 = angle2 >= 0 ? angle2 : 2 * PI + angle2;//0~2pi
	angle2 = abs(utility::Mod2pi(angle2 - base_angle_));
	angle2 = angle2 <= PI ? angle2 : 2 * PI - angle2;//0~pi
	fitness_3 = 1.0/(1.0 + exp(angle2));

	fitness = W1 * fitness_1 + W2 * fitness_2 + W3 * fitness_3;

	return fitness;
};

void PSO::updateParticleStates(particle* particle, int iteration)
{
	double r1 = rand() % 1000 / 1000.0;
	double r2 = rand() % 1000 / 1000.0;
	double line_X = particle->state.velocity.x * cos(particle->state.velocity.y);//
	double line_Y = particle->state.velocity.x * sin(particle->state.velocity.y);//
	double temp_weight = (double)(MAX_ITERATION - iteration) / MAX_ITERATION * WEIGHT + 0.5 * WEIGHT;
	//double temp_weight = 1.0;
	line_X = temp_weight * line_X + c1 * r1 *(particle->Pbest_state.position.x - particle->state.position.x) + c2 * r2 * (Gbest_state_.position.x - particle->state.position.x) ;//+ F_sum.x*DT;
	line_Y = temp_weight * line_Y + c1 * r1 *(particle->Pbest_state.position.y - particle->state.position.y) + c2 * r2 * (Gbest_state_.position.y - particle->state.position.y) ;//+ F_sum.y*DT;

	particle->state.velocity.x = sqrt(line_X * line_X + line_Y * line_Y);//linear velocity
	if(particle->state.velocity.x <= 0.0001 )//stoped
		particle->state.velocity.y = 0;
	else 
		particle->state.velocity.y = atan2(line_Y, line_X) >= 0 ? atan2(line_Y, line_X) : 2*PI + atan2(line_Y, line_X);

	particle->state.velocity.x = min(cur_uav_->Vel_limite[1], max((double)cur_uav_->Vel_limite[0], (double)particle->state.velocity.x));

	particle->state.position.x += particle->state.velocity.x * DT * cos(particle->state.velocity.y);
	particle->state.position.y += particle->state.velocity.x * DT * sin(particle->state.velocity.y);

	if (particle->state.position.x < 0) {
		particle->state.position.x = 0;
		particle->state.velocity.y = PI * (0.5 - (rand() % 1000) / 1000.0);//
	}
	else if (particle->state.position.x >= WIDTH * RESOLUTION) {
		particle->state.position.x = WIDTH * RESOLUTION;
		particle->state.velocity.y = PI * (0.5 + (rand() % 1000) / 1000.0);//
	}
	if (particle->state.position.y < 0) {
		particle->state.position.y = 0;
		particle->state.velocity.y = PI * ((rand() % 1000) / 1000.0);//
	}
	else if (particle->state.position.y > HEIGHT * RESOLUTION) {
		particle->state.position.y = HEIGHT * RESOLUTION;
		particle->state.velocity.y = PI * (1 + (rand() % 1000) / 1000.0);//
	}
	return;
};

void PSO::updateSwarmStates() {
	Gbest_fitness_ = 0;

	double temp_fitness;
	
	int iteration = 0;
	int p_idx = 0;
	for (iteration = 0; iteration < MAX_ITERATION; iteration++) {
		for (p_idx = 0; p_idx < particle_num_; p_idx++) {
			particle *temp_part = swarm_ + p_idx;
			updateParticleStates(temp_part, iteration);
			temp_fitness = calculateFitness(temp_part);

			//update local best
			if (temp_fitness >= temp_part->Pbest_fitness) {
				temp_part->Pbest_fitness = temp_fitness;
				temp_part->Pbest_state = temp_part->state;
			}

			//update global best
			if (temp_fitness >= Gbest_fitness_) {
				Gbest_fitness_ = temp_fitness;
				Gbest_state_   = temp_part->state;
			}

			if (cur_uav_->id == DEBUG_IDX && (particle_num_ % 5) ==0 && (iteration % 10) == 0 && cur_T_ <= 500)
				output_particle_state << temp_part->state.position.x << " " << temp_part->state.position.y << " ";
		}
		if (cur_uav_->id == DEBUG_IDX && (iteration % 10) == 0 && cur_T_ <= 500) {
			double d = utility::dubinsDistance(cur_uav_->state, Gbest_state_, min_R_);

			output_particle_state << endl;
			output_uav_state  << cur_uav_->state.position.x << " " << cur_uav_->state.position.y << " " << cur_uav_->state.velocity.y << endl;
			output_best_state << Gbest_state_.position.x << " " << Gbest_state_.position.y << " " 
							  << area_guider_.x << " " << area_guider_.y << " "
							  << Gbest_fitness_ << " " << d << endl;
		}
	}
	
	printf("Gbest_fitness_ = %lf\n", Gbest_fitness_);
	cur_uav_->traj_Point = Gbest_state_;
	cur_uav_->area_guider = area_guider_;
	//uav_->updatePrevPoses();
	return;
};
