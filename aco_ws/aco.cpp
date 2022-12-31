#include "aco.h"

ACO::ACO() {
	info_map_ = (info_grid*)malloc(100000 * sizeof(info_grid));
	visited_ = (char*)malloc(300000 * sizeof(char));
	ant_colony_ = (ANT*)malloc(30 * sizeof(ANT));
}

ACO::~ACO() {
	free(info_map_);
	free(visited_);
	free(ant_colony_);
}

void ACO::setParam(int ant_num, int max_iteration, double alpha, double beta, double initial_ph, double rio) {
	ant_num_ = ant_num;
	max_iteration_ = max_iteration;
	alpha_ = alpha;
	beta_ = beta;
	initial_ph_ = initial_ph;
	rio_ = rio;
	return;
};

void ACO::initInfoMap( utility::MAP &global_map) {
	int height = global_map.height_;
	int width  = global_map.width_;
	int start_row = uav_pose_.y - search_R_ - 1 > 0 ? uav_pose_.y - search_R_ - 1 : 0;
	int end_row = uav_pose_.y + search_R_ + 1 < height ? uav_pose_.y + search_R_ + 1 : height;
	int start_col = uav_pose_.x - search_R_ - 1 > 0 ? uav_pose_.x - search_R_ - 1 : 0;
	int end_col = uav_pose_.x + search_R_ + 1 < width ? uav_pose_.x + search_R_ + 1 : width;
	height_ = end_row - start_row;
	width_  = end_col - start_col;
	shift_.x = start_col;
	shift_.y = start_row;
	info_grid* grid_ptr = info_map_;
	for (int row = start_row; row < end_row; row++) {
		for (int col = start_col; col < end_col; col++) {
			grid_ptr->info = initial_ph_;

			utility::point2I temp_p(col, row);
			if (utility::dist(temp_p, uav_pose_) >= search_R_)//
				grid_ptr->is_bord = true;
			else
				grid_ptr->is_bord = false;

			int start_row_cov = row - search_R_ > 0 ? row - search_R_ : 0;
			int end_row_cov = row + search_R_ < height ? row + search_R_ : height;
			int start_col_cov = col - search_R_ > 0 ? col - search_R_ : 0;
			int end_col_cov = col + search_R_ < width ? col + search_R_ : width;
			grid_ptr->cover_rate = 0;
			//printf("start_row_cov = %d;end_row_cov = %d;start_col_cov = %d;end_col_cov = %d;\n", start_row_cov, end_row_cov, start_col_cov, end_col_cov);
			for (int row_cov = start_row_cov; row_cov < end_row_cov; row_cov++) {
				for (int col_cov = start_col_cov; col_cov < end_col_cov; col_cov++) {
					grid_ptr->cover_rate += T_ - global_map(col_cov, row_cov).search_time;
				}
			}
			grid_ptr->cover_rate /= search_R_*search_R_*forget_time;
			//printf("grid_ptr[%d,%d].cover_rate =%f\n", row, col, grid_ptr->cover_rate);
			grid_ptr++;
		}
	}
	uav_pose_ = uav_pose_ - shift_;;
	return;
}

void ACO::resetAntColony() {
	ANT* ant_ptr = ant_colony_;
	memset(visited_, 0, height_*width_*ant_num_);
	for (int ant_idx = 0; ant_idx < ant_num_; ant_idx++, ant_ptr++) {
		ant_ptr->position = uav_pose_;
		ant_ptr->alive = true;
		ant_ptr->reach_bord = false;
		ant_ptr->path_len = 0;
		ant_ptr->dir = (rand() % 3+ cur_uav_direction_+7)%8;
		visited_[uav_pose_.y*width_*ant_num_ + uav_pose_.x*ant_num_  + ant_idx] = 1;
	}
	return;
};

utility::point2I ACO::run( utility::MAP &global_map, utility::point2I cur_uav_pose, int cur_uav_direction, int R, int uav_idx, int cur_T) {
	uav_pose_ = cur_uav_pose;
	cur_uav_direction_ = cur_uav_direction;
	search_R_ = R;
	uav_idx_ = uav_idx;
	T_ = cur_T;

	//output_ants.open("data/ants.txt", ios::app | ios::binary);

	initInfoMap(global_map);
	
	nextWayPoints_.x = -1;
	nextWayPoints_.y = -1;
	prevWayPoints_.x = -1;
	prevWayPoints_.y = -1;

	
	int cunt = 0;
	int iterator = max_iteration_;//��ʼ����ֵ�أ�
	while (--iterator > 0) {
		resetAntColony();
		bool success_one_iterator=false;
		while (1) {
			transitionState();
			success_one_iterator = false;
			bool is_all_died = true;
			ANT* ant_ptr = ant_colony_;
			for (int ant_idx = 0; ant_idx < ant_num_; ant_idx++, ant_ptr++) {
				if (ant_ptr->reach_bord)
					success_one_iterator = true;
				if (ant_ptr->alive) {
					is_all_died = false;
					break;
				}
			} 
			if (is_all_died)
				break;
		}
		updateInfoMap();

		if (success_one_iterator&&prevWayPoints_ == nextWayPoints_) {
			//printf("prevWayPoints_=[%d,%d]\n", prevWayPoints_.x, prevWayPoints_.y);
			cunt++;
		}
		else
		{
			prevWayPoints_ = nextWayPoints_;
			cunt = 0;
		}
		
		if (cunt > 20) {
			printf("prevWayPoints_=[%d,%d]\n", prevWayPoints_.x, prevWayPoints_.y);
			break;
		}
			
	}
	printf("iterator=%d\n", iterator);
	nextWayPoints_ = nextWayPoints_ + shift_;
	//output_ants.close();
	return nextWayPoints_;
};


void ACO::transitionState() {
	for (int ant_idx = 0; ant_idx < ant_num_; ant_idx++) {
		//if(uav_idx_==1)
			//output_ants << ant_colony_[ant_idx].position.x << " " << ant_colony_[ant_idx].position.y << " ";
		transitionState_ones(ant_idx);
	}
	//if (uav_idx_ == 1)
		//output_ants << uav_pose_.x << " " << uav_pose_.y << endl;
	return;
}

void ACO::transitionState_ones(int idx) {
	ANT* cur_ant = ant_colony_+idx;

	if (!cur_ant->alive)
		return;

	double sum_val = 0;
	for (int temp_dix = cur_ant->dir-1; temp_dix <= cur_ant->dir + 1; temp_dix++) {
		int dir_idx = (temp_dix + 8) % 8;
		int x = cur_ant->position.x + DIRECTION[dir_idx][0];
		int y = cur_ant->position.y + DIRECTION[dir_idx][1];
		//int dir_gain = (8 - abs(cur_ant->dir - dir_idx));
		double dir_gain = dir_idx == cur_ant->dir ? 1.414 : 1;
		if (0 <= x && x < width_ && 0 <= y && y < height_&&visited_[y*width_*ant_num_ + x * ant_num_ + idx] == 0 ) {
			sum_val += pow(info_map_[y*width_ + x].info, alpha_)*pow(info_map_[y*width_ + x].cover_rate, beta_)*dir_gain;
			//printf("dsum=%f\n", pow(info_map_[y*width_ + x].info, alpha_)*pow(info_map_[y*width_ + x].cover_rate, beta_));
		}
	}
	if (sum_val == 0) {
		//printf("no grid avaliable for ant %d\n", idx);
		cur_ant->alive = false;
		return;
	}

	double lun_val = 0;
	while(lun_val==0)
		lun_val = rand() % 1000 / 1000.0*sum_val;
	for (int temp_dix = cur_ant->dir - 1; temp_dix <= cur_ant->dir + 1; temp_dix++) {
		int dir_idx = (temp_dix + 8) % 8;
		int x = cur_ant->position.x + DIRECTION[dir_idx][0];
		int y = cur_ant->position.y + DIRECTION[dir_idx][1];
		//int dir_gain = (8 - abs(cur_ant->dir - dir_idx));
		double dir_gain = dir_idx == cur_ant->dir ? 1.414 : 1;
		if (0 <= x && x < width_ && 0 <= y && y < height_&&visited_[y*width_*ant_num_ + x * ant_num_ + idx] == 0) {
			sum_val -= pow(info_map_[y*width_ + x].info, alpha_)*pow(info_map_[y*width_ + x].cover_rate, beta_)*dir_gain;
		}
		if (sum_val < lun_val) {
			cur_ant->position.x = x;
			cur_ant->position.y = y;
			cur_ant->dir = dir_idx;
			visited_[y*width_*ant_num_ + x * ant_num_ + idx] = 1;
			//info_map_[y*height_ + x].info = (1 - rio_)*info_map_[y*height_ + x].info + rio_ * initial_ph_;
			cur_ant->path[cur_ant->path_len] = cur_ant->position;
			cur_ant->path_len++;
			if (info_map_[y*width_ + x].is_bord) {
				cur_ant->alive = false;
				cur_ant->reach_bord = true;;
				//printf("ant %d reach_bord\n", idx);
			}
			break;
		}
	}

	return;
}


void ACO::updateInfoMap() {

	for (int row = 0; row < height_; row++) {
		for (int col = 0; col < width_; col++) {
			info_map_[row*width_+col].info *= 1 - rio_;
		}
	}

	ANT* ant_ptr = ant_colony_;
	shortest_path_cov_ = 100000;
	ANT* best_ant = ant_colony_;
	for (int ant_idx = 0; ant_idx < ant_num_; ant_idx++, ant_ptr++) {
		float cov = info_map_[ant_ptr->position.y*width_ + ant_ptr->position.x].cover_rate;
		int lg = ant_ptr->path_len;
		if (ant_ptr->reach_bord) {
			if (shortest_path_cov_ >= lg) {
				nextWayPoints_ = ant_ptr->position;
				best_ant = ant_ptr;
			}
			utility::point2I* point_ptr = ant_ptr->path;
			for (int i = 0; i < lg; i++) {
				info_map_[point_ptr->y*width_ + point_ptr->x].info += rio_ *initial_ph_ / lg;
				point_ptr++;
			}
		}
	}
	if (shortest_path_cov_ != 100000) {
		int lg = best_ant->path_len;
		utility::point2I* point_ptr = best_ant->path;
		for (int i = 0; i < lg; i++) {
			info_map_[point_ptr->y*width_ + point_ptr->x].info += rio_ * initial_ph_ / lg;
			point_ptr++;
		}
	}
};