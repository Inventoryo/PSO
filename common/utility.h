
// Created by bb on 2020/3/9.
//

#ifndef SRC_UTILITY_H
#define SRC_UTILITY_H

#include <vector>
#include <map>
#include <queue>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include "../common/dubins.h"
#include "../common/parameter.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace utility 
{

inline double Mod2pi(double x)
{
	return x - 2 * PI * floor(x / (2 * PI));
}

struct point2I 
{
	point2I() {};
	point2I(int a, int b):x(a), y(b) {};

	~point2I() {};
	int distance() { return sqrt(x * x + y * y); }

	double angle() {return y >= 0 ? atan2(y, x) : 2 * PI + atan2(y, x);};

	point2I operator+(const point2I& b) {
		point2I temp;
		temp.x = this->x + b.x;
		temp.y = this->y + b.y;
		return temp;
	}
	point2I operator-(const point2I& b) {
		point2I temp;
		temp.x = this->x - b.x;
		temp.y = this->y - b.y;
		return temp;
	}
	point2I operator-() {
		this->x = -this->x;
		this->y = -this->y;
		return *this;
	}
	point2I operator*(int T) {
		point2I temp;
		temp.x = this->x * T;
		temp.y = this->y * T;
		return temp;
	}

	bool operator==(const point2I& b) { return this->x == b.x && this->y == b.y; }

	int x;
	int y;
};

struct point2D 
{
public:
	point2D() {};
	point2D(double a, double b):x(a), y(b) {};
	~point2D() {};

	double distance() {return sqrt(x*x+y*y);}

	double angle() { return y >= 0 ? atan2(y, x) : 2 * PI + atan2(y, x); };

	point2D operator+(const point2D& b) {
		point2D temp;
		temp.x = this->x + b.x;
		temp.y = this->y + b.y;
		return temp;
	}
	point2D operator-(const point2D& b) {
		point2D temp;
		temp.x = this->x - b.x;
		temp.y = this->y - b.y;
		return temp;
	}
	point2D operator-() {
		this->x = -this->x;
		this->y = -this->y;
		return *this;
	}
	point2D operator*(double T) {
		point2D temp;
		temp.x = this->x*T;
		temp.y = this->y*T;
		return temp;
	}

	point2D operator/(double T) {
		point2D temp;
		if(abs(T) < 0.000001)
			T += 0.00001;
		temp.x = this->x/T;
		temp.y = this->y/T;
		return temp;
	}

	double x;
	double y;
};

inline double dist(const point2D& a, const point2D& b)
{
	return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}
inline int dist(point2I& a, point2I& b) 
{
	return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}


struct State 
{
	point2D position;
	point2D velocity;
	point2D accelerate;
};

inline double dubinsDistance(State& state1, State& state2, double min_R)
{
	
	double l = dist(state1.position, state2.position);
	double theTa1 = atan2(state2.position.y - state1.position.y, state2.position.x - state1.position.x);
	theTa1 = theTa1 >= 0 ? theTa1 : 2 * PI + theTa1;
	theTa1 = abs(utility::Mod2pi(theTa1 - state1.velocity.y));
	theTa1 = theTa1 > PI ? 2 * PI - theTa1 : theTa1 ;
	double res = l;
	res += abs(theTa1) * min_R;
	return res;
	/*
	HybridAStar::DubinsPath *path = new HybridAStar::DubinsPath();
	double theTa1 = acos((state2.position.x - state1.position.x) / l);
	theTa1 = (state2.position.y - state1.position.y)>0 ? theTa1 : 2 * PI - theTa1;
	double theTa2 = state1.velocity.y - theTa1;
	if (theTa2 > PI)
		theTa2 -= 2 * PI;
	else if (theTa2 <-PI)
		theTa2 += 2 * PI;
	double theTi, d;

	if (theTa2 < -PI / 2) {
		theTa2 += PI;
		d = sqrt((l - min_R * sin(theTa2))*(l - min_R * sin(theTa2)) + min_R * cos(theTa2)*min_R*cos(theTa2));
		theTi = asin(min_R*cos(theTa2) / d) + asin(min_R / d) + theTa1;
	}
	else if (theTa2 < 0) {
		theTa2 *= -1;
		d = sqrt((l - min_R * sin(theTa2))*(l - min_R * sin(theTa2)) + min_R * cos(theTa2)*min_R*cos(theTa2));
		theTi = asin(min_R / d) - asin(min_R*cos(theTa2) / d) + theTa1;
	}
	else if (theTa2 > PI / 2) {
		theTa2 = PI - theTa2;
		d = sqrt((l - min_R * sin(theTa2))*(l - min_R * sin(theTa2)) + min_R * cos(theTa2)*min_R*cos(theTa2));
		theTi = -asin(min_R / d) - asin(min_R*cos(theTa2) / d) + theTa1;
	}
	else {
		d = sqrt((l - min_R * sin(theTa2))*(l - min_R * sin(theTa2)) + min_R * cos(theTa2)*min_R*cos(theTa2));
		theTi = -asin(min_R / d) + asin(min_R*cos(theTa2) / d) + theTa1;
	}

	if (std::isnan(theTi))
		return 100000.0;
	
	
	double q1[3] = { state1.position.x,state1.position.y,state1.velocity.y };//start point state
	double q2[3] = { state2.position.x,state2.position.y, theTa1 };//end point state;
	HybridAStar::dubins_init(q1, q2, min_R, path);
	double res = HybridAStar::dubins_path_length(path);
	delete(path);
	return res;
	*/
}

class TARGET
{
public:
	TARGET() {};
	~TARGET() {};
	void setDefaultValue(
		float vel_min, float vel_max, 
		float acc_x_min, float acc_x_max, float acc_y_min, float acc_y_max,
		float pose_x, float pose_y, 
		float vel_x, float vel_y,
		float acc_x, float acc_y
		){
		Vel_limite[0]      = vel_min;
		Vel_limite[1]      = vel_max;
		Acc_limite_x[0]    = acc_x_min;
		Acc_limite_x[1]    = acc_x_max;
		Acc_limite_y[0]    = acc_y_min;
		Acc_limite_y[1]    = acc_y_max;
		state.position.x   = pose_x;
		state.position.y   = pose_y;
		state.velocity.x   = vel_x;
		state.velocity.y   = vel_y;
		state.accelerate.x = acc_x;
		state.accelerate.y = acc_y;
	}
	State  state;
	double Vel_limite[2];
	double Acc_limite_x[2];
	double Acc_limite_y[2];
};

class particle
{
public:
	particle() {};
	~particle() {};
	State  state;

	double p[3];

	double fitness;
	State Pbest_state;
};

class RADAR 
{
public:
	RADAR() {};
	RADAR(double x, double y, double r) {
		center_point.x = x;
		center_point.y = y;
		R = r;
	};
	~RADAR() {};
	point2D center_point;
	double R;

	bool isInRader(utility::point2D& position) {
		return utility::dist(position, center_point) <= R;
	}
};

class UAV
{
public:
	UAV() {};
	~UAV() {};

	int id;
	State  state;
	double Vel_limite[2];
	double Acc_limite_x[2];
	double Acc_limite_y[2];

	double h;
	double search_r;
	double coverd_area_cnt;

	State  Gbest_state;
	State  traj_Point;
	double Gbest_fitness;
	utility::point2D area_guider;
	std::queue<State> path_;//states
	std::vector<std::pair<State, int>> target_state;//
	std::map<int, int> Tj;//
	int track_target_num;
	std::vector<State> prev_poses;

	double k_att;
	double k_rep;
	double k_d;

	void updatePrevPoses() {
		if (prev_poses.size() < 5) {
			prev_poses.push_back(traj_Point);
		}else {
			prev_poses.erase(prev_poses.begin());
			prev_poses.push_back(traj_Point);
		}
	}

};

class grid
{
public:
	grid() {};
	grid(int t, bool occ):search_time(), occupy(occ) {};
	~grid() {};
	int search_time;
	bool occupy;
};

class MAP
{
public:
	MAP(){};
	~MAP(){};
	void init(int width, int height, int resulution, int time_init){
		search_time_ = cv::Mat(height, width, CV_16UC1, time_init);
		width_  = width;
		height_ = height;
		resolution_ = resulution;
		map_ = (utility::grid *)malloc(width_ * height_ * sizeof(utility::grid));
		int map_size = width * height;
		for(int i = 0;i < map_size; i++){
			map_[i].search_time = time_init;
			map_[i].occupy = 0;
		}
	}
	void updateMap(utility::UAV * uav, int cunt){
		unsigned int row_min = std::max((int)(uav->state.position.y - uav->search_r) / resolution_, 0);
		unsigned int row_max = std::min((int)(uav->state.position.y + uav->search_r) / resolution_, height_ - 1);
		unsigned int col_min = std::max((int)(uav->state.position.x - uav->search_r) / resolution_, 0);
		unsigned int col_max = std::min((int)(uav->state.position.x + uav->search_r) / resolution_, width_ - 1);
		for (unsigned int row_idx = row_min; row_idx <= row_max; row_idx++) {
			for (unsigned int col_idx = col_min; col_idx <= col_max; col_idx++) {
				utility::point2D grid_pose((col_idx + 0.5) * resolution_, (row_idx + 0.5) * resolution_);
				if (dist(grid_pose, uav->state.position) <= uav->search_r + 0.5 * resolution_)
					map_[row_idx * width_ + col_idx].search_time = cunt;
					this->search_time_.at<unsigned short>(row_idx, col_idx) = cunt;
			}
		}
	};

	bool isInBound(utility::point2D& position)
	{
		return !(position.x <= 0 || position.x >= width_  * resolution_ ||
			     position.y <= 0 || position.y >= height_ * resolution_);
	}

	void destory(){
		free(map_);
	}

	utility::grid& operator()(int x, int y){ return map_[y * width_ + x]; };

	//data 
	utility::grid *map_;
	int height_;
	int width_;
	int resolution_;
	cv::Mat search_time_;
};

	

	class nion {
	public:
		nion() {};
		~nion() {};
		std::vector<int> parent;
		std::vector<std::vector<int>> barrel;
		int unionsearch(int root) //
		{
			int son, tmp;
			son = root;
			while (root != parent[root]) //
				root = parent[root];
			while (son != root) //
			{
				tmp = parent[son];
				parent[son] = root;
				son = tmp;
			}
			return root;
		}

		void join(int root1, int root2) //
		{
			int x, y;
			x = unionsearch(root1);
			y = unionsearch(root2);
			if (x != y) //
				parent[y] = x;
		}
		void setup_barrel() {
			barrel.clear();
			for (int i = 0; i<parent.size(); i++) {
				if (i == parent[i]) {
					std::vector<int> temp;
					temp.push_back(i);
					barrel.push_back(temp);
				}
				else {
					for (int j = 0; j<barrel.size(); j++) {
						if (barrel[j].front() == parent[i]) {
							barrel[j].push_back(i);
							break;
						}
					}
				}
			}
		};
	};

class OBSTACLE {
public:
	OBSTACLE() {};
	~OBSTACLE() {};
	std::vector<point2D> point_lists;
	bool isInObstacle(utility::point2D& position) 
	{
		return point_lists[0].x <= position.x && position.x <= point_lists[2].x &&
		   	   point_lists[1].y <= position.y && position.y <= point_lists[3].y;
	}
};


}


#endif //SRC_UTILITY_H
