/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <chrono>
#include <time.h>
#include <queue>
#include <list>
#include <set>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fstream>
#include <vector>


/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654
#define INF 0x3f3f3f3f
#define DOF 5
//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10
using namespace std;
typedef double* Config;
int call_count_rrt = 0;
int call_count_prm = 0;

typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size)
{
    double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
  params->UsingYIndex = 0;

  if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
    (params->UsingYIndex)++;

  if (params->UsingYIndex)
    {
      params->Y1=p1x;
      params->X1=p1y;
      params->Y2=p2x;
      params->X2=p2y;
    }
  else
    {
      params->X1=p1x;
      params->Y1=p1y;
      params->X2=p2x;
      params->Y2=p2y;
    }

   if ((p2x - p1x) * (p2y - p1y) < 0)
    {
      params->Flipped = 1;
      params->Y1 = -params->Y1;
      params->Y2 = -params->Y2;
    }
  else
    params->Flipped = 0;

  if (params->X2 > params->X1)
    params->Increment = 1;
  else
    params->Increment = -1;

  params->DeltaX=params->X2-params->X1;
  params->DeltaY=params->Y2-params->Y1;

  params->IncrE=2*params->DeltaY*params->Increment;
  params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
  params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

  params->XIndex = params->X1;
  params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
  if (params->UsingYIndex)
    {
      *y = params->XIndex;
      *x = params->YIndex;
      if (params->Flipped)
        *x = -*x;
    }
  else
    {
      *x = params->XIndex;
      *y = params->YIndex;
      if (params->Flipped)
        *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params)
{
  if (params->XIndex == params->X2)
    {
      return 0;
    }
  params->XIndex += params->Increment;
  if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
    params->DTerm += params->IncrE;
  else
    {
      params->DTerm += params->IncrNE;
      params->YIndex += params->Increment;
    }
  return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double* map,int x_size,int y_size)
{
	bresenham_param_t params;
	int nX, nY;
    short unsigned int nX0, nY0, nX1, nY1;

    //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);

	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

    //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
            return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
		   int x_size, int y_size)
{
    double x0,y0,x1,y1;
    int i;

 	//iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
    y1 = 0;
	for(i = 0; i < numofDOFs; i++)
	{
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
				return 0;
	}
    return 1;
}

//Point class - creates points in the space, gets config distance
class Point
{
public:
    Point(Config config)
    {   config_ = config;
        parent_id = -1;
        DOF_ = DOF;
        goal_thresh_joint_space_ = 10;
        cost_ = 0;}

    Point(Config config, int parent_idx)
    {   DOF_ = DOF;
        config_ = config;
        parent_id = parent_idx;
        cost_ = 0;}
    
    Point(Config config, int parent_idx, double cost)
    {
        DOF_ = DOF;
        config_ = config;
        parent_id = parent_idx;
        cost_ = cost;
    }

    double config_dist(Config config_2)
    {
        double total_dist = 0.0;
        double curr_diff = 0.0;
        double min_joint_angle = 0.0;
        for (int joint_idx=0; joint_idx<DOF_; joint_idx++)
        {
            curr_diff = config_[joint_idx] - config_2[joint_idx];
            min_joint_angle = std::min(curr_diff, 2*PI - curr_diff);
            total_dist += pow(std::fabs(min_joint_angle), 2);
        }
        return sqrt(total_dist);
    }
    int get_parent_idx()
    {
        return parent_id;
    }
    Config get_config()
    {
        return config_;
    }
    ~Point(){};
    Config config_;
    int parent_id;
    int DOF_;
    double goal_thresh_joint_space_;
    double cost_;
};

//Graph Formation Class
class Graph
{
public:
   Graph(const Config &start_config,
        const Config &goal_config,
        double goal_thresh_cartesian,
        double epsilon,
        int num_steps_interp,
        int numofDOFs,
        double sample_goal_bias,
        double* map,
        int x_size,
        int y_size,
        double gamma,
        double epsilon_rrt_star,
        int tree_id)
    {
        start_config_ = start_config;
        goal_config_ = goal_config;
        num_samples_ = 0;
        is_goal_reached_ = false;
        goal_thresh_cartesian_ = goal_thresh_cartesian;
        epsilon_ = epsilon;
        DOF_ = numofDOFs;
        sample_goal_bias_ = sample_goal_bias;
        num_steps_interp_ = num_steps_interp;
        map_ = map;
        map_x_size_ = x_size;
        map_y_size_ = y_size;
        goal_cartesian_ = new double[2];
        start_cartesian_ = new double[2];
        FK(goal_config_, goal_cartesian_);//FK=Forward Kinematics
        FK(start_config_, start_cartesian_);
        std::cout << "start config " << start_config_[0] << " " << start_config_[1]<< " " << start_config_[2]<< " " << start_config_[3] << std::endl;
        std::cout << "goal_config_ " << goal_config_[0] << " " << goal_config_[1]<< " " << goal_config_[2]<< " " << goal_config_[3] << std::endl;
        std::cout << "start_cartesian_ " << start_cartesian_[0] << ", " << start_cartesian_[1] << std::endl;
        std::cout << "goal_cartesian_ " << goal_cartesian_[0] << ", " << goal_cartesian_[1] << std::endl;
        tree_id_ = tree_id;
        insert_vertex(start_config);
        delta_ = ball_vol();
        gamma_ = gamma;
        epsilon_rrt_star_ = epsilon_rrt_star;
    }

    ~Graph(){};

    int get_nearest_index(const Config &config)
    {
        int idx_min = -1;
        double min_dist = std::numeric_limits<double>::infinity();
        for (int node_idx=0; node_idx < nodes_.size(); ++node_idx)
        {   
           if (nodes_[node_idx].config_dist(config) < min_dist)
            {
                min_dist = nodes_[node_idx].config_dist(config);
                idx_min = node_idx;
                 }
        }
        return idx_min;
    }
//http://home.ustc.edu.cn/~hyx/0423/Volume_of_an_n-ball.pdf
//https://www.britannica.com/science/gamma-function
    double ball_vol()
    {
        return pow(PI, 2.5)/3.32335;
    }

    void update_rrt_star_radius()
    {
        if (nodes_.size() > 1)
            radius_rrt_star_ = std::min(pow( gamma_ / delta_ * log(nodes_.size()) / nodes_.size() , 1.0/(double)DOF_),
                                epsilon_rrt_star_);
        return;
    }
    void update_nearest_nodes_and_dist(Config sample_config)
    {
        nearest_nodes_dist_.clear();
        nearest_nodes_indices_.clear();
        closest_node_idx_ = 0;
        double min_dist = std::numeric_limits<double>::infinity();
        double curr_dist = 0.0;

        for (int node_idx=0; node_idx < nodes_.size(); node_idx++)
        {
            curr_dist = nodes_[node_idx].config_dist(sample_config);
            if (curr_dist < min_dist)
            {
                closest_node_idx_ = node_idx;
                min_dist = curr_dist;
            }
            if (curr_dist <= radius_rrt_star_)
            {
                nearest_nodes_indices_.push_back(node_idx);
                nearest_nodes_dist_.push_back(curr_dist);
            }
        }
        dist_closest_node = min_dist;
        return;
    }

    bool transition_validity(Config curr_config, Config sample_config)
    {
        Config interm_config = new double[DOF_];
        for (int idx_step=0; idx_step < num_steps_interp_; idx_step++)
        {
            for (int joint_idx=0; joint_idx < DOF_; joint_idx++)
            {
                interm_config[joint_idx] = curr_config[joint_idx] +
                            ((double)idx_step*(double)epsilon_*(sample_config[joint_idx] - curr_config[joint_idx])/(double)num_steps_interp_);
            }
            if (!IsValidArmConfiguration(interm_config, DOF_, map_, map_x_size_, map_y_size_))
            { return false; }
        }
        return true;
    }

    bool extend_tree_rrt_star(Config sample_config)
    {
        if (nodes_.size()==0)
        {
            Point start_node(start_config_, -1);
            nodes_.push_back(start_node);
            std::cout << "inserted first node" << std::endl;
            return true;
        }
        int nearest_id = get_nearest_index(sample_config);
        Config interm_config = new double[DOF_];
        for (int idx_step=0; idx_step < num_steps_interp_; idx_step++)
        {
            for (int joint_idx=0; joint_idx<DOF_; joint_idx++)
            {
                interm_config[joint_idx] = nodes_[nearest_id].get_config()[joint_idx] +
                            ((double)idx_step*(double)epsilon_*(sample_config[joint_idx] - nodes_[nearest_id].get_config()[joint_idx])/(double)num_steps_interp_);
            }

            if (!IsValidArmConfiguration(interm_config, DOF_, map_, map_x_size_, map_y_size_))
            { return false; }
        }

        sample_config = interm_config;
        update_rrt_star_radius();
        update_nearest_nodes_and_dist(sample_config);
        if(transition_validity(nodes_[closest_node_idx_].get_config(), sample_config))
        {
            valid_nearest_nodes_vec_.clear();
            int min_node_idx = closest_node_idx_;
            double min_cost = nodes_[closest_node_idx_].cost_ + dist_closest_node;
            double curr_cost = 0;
            bool transition_validity_curr = false;

            for (int loop_idx=0; loop_idx<nearest_nodes_indices_.size(); loop_idx++)
            {
                transition_validity_curr = transition_validity(nodes_[nearest_nodes_indices_[loop_idx]].get_config(),sample_config);
                valid_nearest_nodes_vec_.push_back(transition_validity_curr);
                if (transition_validity_curr)
                {
                    curr_cost = nodes_[nearest_nodes_indices_[loop_idx]].cost_ +
                                    nearest_nodes_dist_[loop_idx];
                    if (curr_cost < min_cost)
                    {
                        min_node_idx = nearest_nodes_indices_[loop_idx];
                        min_cost = curr_cost;
                    }
                }
            }
            Point node_to_insert(sample_config, min_node_idx, min_cost);
            nodes_.push_back(node_to_insert);

            if (in_goal_region(node_to_insert))
            {
                is_goal_reached_ = true;
                construct_path();
            }

            for (int loop_idx=0; loop_idx<nearest_nodes_indices_.size(); loop_idx++)
            {
                if (loop_idx == min_node_idx)
                    continue;
                curr_cost = nodes_[nearest_nodes_indices_[loop_idx]].cost_
                                + nearest_nodes_dist_[loop_idx];
                if (valid_nearest_nodes_vec_[loop_idx] && curr_cost < nearest_nodes_dist_[loop_idx])
                {
                    nodes_[nearest_nodes_indices_[loop_idx]].cost_ = curr_cost;
                    nodes_[nearest_nodes_indices_[loop_idx]].parent_id = loop_idx;
                }
            }
        }
    }
    // forward Kinematics
    void FK(const Config &config, double* cartesian)
    {
        double x0,y0,x1,y1;
        int i;
        x1 = ((double)map_x_size_)/2.0;
        y1 = 0;
        for(i = 0; i < DOF_; i++)
        {
            x0 = x1;
            y0 = y1;
            x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-config[i]);
            y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-config[i]);}
        cartesian[0] = x1;
        cartesian[1] = y1;
    }

    bool in_goal_region(Point& node)
    {
        double* node_cartesian = new double[DOF_];
        FK(node.config_, node_cartesian);
        double dist = pow(node_cartesian[0]-goal_cartesian_[0], 2) +
                      pow(node_cartesian[1]-goal_cartesian_[1], 2);
        dist = sqrt(dist);

        if (dist < goal_thresh_cartesian_)
        {
            return true;
        }
        return false;
    }

    bool closer_region(Config &config_1, Config &config_2)
    {
        double* cartesian_1 = new double[DOF_];
        double* cartesian_2 = new double[DOF_];
        FK(config_1, cartesian_1);
        FK(config_2, cartesian_2);
        double dist = pow(cartesian_1[0]-cartesian_2[0], 2) +
                      pow(cartesian_1[1]-cartesian_2[1], 2);
        dist = sqrt(dist);
        if (dist < goal_thresh_cartesian_)
        {
            return true;
        }
        return false;
    }


    bool insert_vertex(Config sample_config)
    {
        if (nodes_.size()==0)
        {
            Point start_node(start_config_, -1);
            nodes_.push_back(start_node);
            std::cout << "inserted first node" << std::endl;
            call_count_rrt++;
            return true;
        }

        int nearest_id = get_nearest_index(sample_config);
        Config interm_config = new double[DOF_];
        for (int idx_step=0; idx_step < num_steps_interp_; idx_step++)
        {
            for (int joint_idx=0; joint_idx<DOF_; joint_idx++)
            {

                interm_config[joint_idx] = nodes_[nearest_id].get_config()[joint_idx] +
                            ((double)idx_step*(double)epsilon_*(sample_config[joint_idx] - nodes_[nearest_id].get_config()[joint_idx])/(double)num_steps_interp_);
              }
            if (!IsValidArmConfiguration(interm_config, DOF_, map_, map_x_size_, map_y_size_))
            { return false; }
        }

        Point new_node(interm_config, nearest_id);
        nodes_.push_back(new_node);
        nearest_id = nodes_.size()-1;
        if (in_goal_region(new_node))
        {
            is_goal_reached_ = true;
            construct_path();
            call_count_rrt++;
            return true;
        }
        call_count_rrt++;
        return true;
        
        
    }

    bool extend_tree(Config sample_config)
    {
        if (nodes_.size()==0)
        {
            Point start_node(start_config_, -1);
            nodes_.push_back(start_node);
            std::cout << "inserted first node" << std::endl;
            return true;
        }

        int nearest_id = get_nearest_index(sample_config);
        Config interm_config = new double[DOF_];
        for (int idx_step=0; idx_step < num_steps_interp_; idx_step++)
        {
            for (int joint_idx=0; joint_idx<DOF_; joint_idx++)
            {
                interm_config[joint_idx] = nodes_[nearest_id].get_config()[joint_idx] +
                            ((double)idx_step*(double)epsilon_*(sample_config[joint_idx] - nodes_[nearest_id].get_config()[joint_idx])/(double)num_steps_interp_);

            }

            if (IsValidArmConfiguration(interm_config, DOF_, map_, map_x_size_, map_y_size_))
            {
                Point new_node(interm_config, nearest_id);
                nodes_.push_back(new_node);
                nearest_id = nodes_.size()-1;
                if (closer_region(interm_config, sample_config))
                {
                    construct_path();
                    return true;
                }
            }
            else
            {
                return false;
            }
        }
        return true;
    }

    void construct_path()
    {
        path_.clear();
        int idx_waypt = nodes_.size()-1;
        while (idx_waypt!=-1)
        {
            path_.push_back(nodes_[idx_waypt].get_config());
            idx_waypt = nodes_[idx_waypt].get_parent_idx();
        }
        std::reverse(path_.begin(), path_.end());
    }

    Config get_random_sample()
    {
         num_samples_++;

        if ((double) rand()/(double) RAND_MAX < sample_goal_bias_)
        {
            return goal_config_;
        }
        else
        {
            Config sample_config = new double[DOF_];

            for(int i = 0; i < DOF_; ++i)
            {
                sample_config[i] = (double)rand()*(2*PI)/(double) RAND_MAX;
            }
            return sample_config;
        }
    }

    std::vector<Config> get_path()
    {
        return path_;
    }

    Config start_config_;
    Config goal_config_;
    double* goal_cartesian_;
    double* start_cartesian_;
    std::vector<Point> nodes_;
    std::vector<Config> path_;
    int num_samples_;
    bool is_goal_reached_;
    int DOF_;
    double* map_;
    int map_x_size_;
    int map_y_size_;
    double sample_goal_bias_;
    double epsilon_;
    double goal_thresh_cartesian_;
    int num_steps_interp_;
    int tree_id_;
    double delta_;
    double gamma_;
    double epsilon_rrt_star_;
    double radius_rrt_star_;
    int closest_node_idx_;
    std::vector<int> nearest_nodes_indices_;
    std::vector<double> nearest_nodes_dist_;
    std::vector<int> valid_nearest_nodes_vec_;
    double dist_closest_node;

};

static double calc_path_quality(double*** plan, int* planlength, int numofDOFs)
{
    double total_dist = 0.0;
    for (int i = 0; i < *planlength - 1; i++)
    {
        Config curr_config = (*plan)[i];
        Config next_config = (*plan)[i+1];
        double curr_dist = 0.0;
        double curr_diff = 0.0;
        double min_joint_angle = 0.0;

         for (int joint_idx=0; joint_idx<numofDOFs; joint_idx++)
        {
            curr_diff = curr_config[joint_idx] - next_config[joint_idx];
            min_joint_angle = std::min(curr_diff, 2*PI - curr_diff);
            curr_dist += pow(std::fabs(min_joint_angle), 2);
        }
        total_dist += sqrt(curr_dist);
    }
    return total_dist;
}

static void planner_rrt(
    double* map,
    int x_size,
    int y_size,
    double* armstart_anglesV_rad,
    double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength)
{
    *plan = NULL;
    *planlength = 0;
    double epsilon = 0.15;
    int num_max_iters = (int)1e7; //1e6
    double sample_goal_bias = 0.0;
    double goal_thresh_cartesian = 5;
    int num_steps_interp = 50 ;//20
    int tree_id = 0 ;
    Graph graph(armstart_anglesV_rad,
            armgoal_anglesV_rad,
            goal_thresh_cartesian,
            epsilon,
            num_steps_interp,
            numofDOFs,
            sample_goal_bias,
            map,
            x_size,
            y_size,
            0,
            0,
            tree_id);
    bool got_path=false;
    int num_samples = 0;

    for (int iter=0; iter < num_max_iters; iter++)
    {
        if (graph.is_goal_reached_)
        {
            std::cout << " SOLUTION FOUND  " << std::endl;
            got_path = true;
            break;
        }

        Config sample_config = graph.get_random_sample();
        if(!IsValidArmConfiguration(sample_config, numofDOFs, map, x_size, y_size))
            continue;
        num_samples++;
        graph.insert_vertex(sample_config);
    }

    if (got_path)
    {
        std::vector<Config> path_vector = graph.get_path();
        *planlength = (int)path_vector.size();
        *plan = (double**) malloc(path_vector.size()*sizeof(double*));

        for (int i = 0; i < path_vector.size(); i++)
        {
            (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
            for(int j = 0; j < numofDOFs; j++)
            {
                (*plan)[i][j] = path_vector[i][j];
            }
        }
    }
    std::cout << "num_samples " << num_samples << "\n";
    

    return;
}

static void planner_rrt_connect(
    double* map,
    int x_size,
    int y_size,
    double* armstart_anglesV_rad,
    double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength)
{
    *plan = NULL;
    *planlength = 0;
    double epsilon = 0.4;
    double sample_goal_bias = 0.1;
    double goal_thresh_cartesian = 3;
    int num_steps_interp = 80;

    Graph start_tree(armstart_anglesV_rad,
            armgoal_anglesV_rad,
            goal_thresh_cartesian,
            epsilon,
            num_steps_interp,
            numofDOFs,
            sample_goal_bias,
            map,
            x_size,
            y_size,
            0,
            0,
            0);
   Graph goal_tree(armgoal_anglesV_rad,
            armstart_anglesV_rad,
            goal_thresh_cartesian,
            epsilon,
            num_steps_interp,
            numofDOFs,
            sample_goal_bias,
            map,
            x_size,
            y_size,
            0,
            0,
            100);

    Graph* current_tree_ptr;
    current_tree_ptr = &start_tree;
    bool is_start_tree = true;
    Config current_config;
    Point* nearest_neighbour;
    bool sample_was_valid = false;
    bool path_found = false;
    bool trees_connect = false;
    int num_max_iters = 1e6;
    int num_samples = 0;

    for (int iter=0; iter < num_max_iters; iter++)
    {
        if (path_found)
        {
            break;
        }
        Config sample_config = current_tree_ptr->get_random_sample();
        if(!IsValidArmConfiguration(sample_config, numofDOFs, map, x_size, y_size))
            continue;
        num_samples++;
        sample_was_valid = current_tree_ptr->insert_vertex(sample_config);

        if (current_tree_ptr->is_goal_reached_)
        {
            std::cout << "current_graph_ptr reached goal" << std::endl;
            path_found = true;
        }
        if (is_start_tree)
        {
            current_tree_ptr = &goal_tree;
            is_start_tree = false;
        }
        else
        {
            current_tree_ptr = &start_tree;
            is_start_tree = true;
        }
        if (sample_was_valid)
        {
            trees_connect = current_tree_ptr->extend_tree(sample_config);
            if ((start_tree.nodes_.size()>0) && (goal_tree.nodes_.size()>0))
            {
                if (trees_connect)
                {
                    path_found = true;
                    break;
                }
            }
        }
    }
    

    if (path_found)
    {
        start_tree.construct_path();
        goal_tree.construct_path();
        std::vector<Config> path_start_tree = start_tree.get_path();
        std::vector<Config> path_goal_tree = goal_tree.get_path();
        std::reverse(path_goal_tree.begin(), path_goal_tree.end());
        *planlength = (int)path_start_tree.size() + (int)path_goal_tree.size();
        *plan = (double**) malloc(*planlength*sizeof(double*));
        if (path_start_tree.size() > 0)
        {
            for (int i = 0; i < path_start_tree.size(); i++)
            {
                (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
                for(int j = 0; j < numofDOFs; j++)
                {
                    (*plan)[i][j] = path_start_tree[i][j];
                }
            }
        }
        if (path_goal_tree.size() > 0)
        {
            for (int i = 0; i < path_goal_tree.size(); i++)
            {
                (*plan)[i+path_start_tree.size()] = (double*) malloc(numofDOFs*sizeof(double));
                for(int j = 0; j < numofDOFs; j++)
                {
                    (*plan)[i+path_start_tree.size()][j] = path_goal_tree[i][j];
                }
            }
        }
    }
    std::cout << "num_samples " << num_samples << std::endl;
    return;
}

static void planner_rrt_star(
    double* map,
    int x_size,
    int y_size,
    double* armstart_anglesV_rad,
    double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength)
{
    *plan = NULL;
    *planlength = 0;
    double epsilon = 0.2;
    double sample_goal_bias = 0.3;
    double goal_thresh_cartesian = 5;
    int num_steps_interp = 50;
    int tree_id = 0;
    double gamma = 100;
    double epsilon_rrt_star = 0.8;
    double radius;
    int num_max_iters = 1e7;

    Graph graph(armstart_anglesV_rad,
            armgoal_anglesV_rad,
            goal_thresh_cartesian,
            epsilon,
            num_steps_interp,
            numofDOFs,
            sample_goal_bias,
            map,
            x_size,
            y_size,
            0,
            0,
            tree_id);

    bool got_path=false;
    int num_samples = 0;

    for (int iter=0; iter < num_max_iters; iter++)
    {
        if (graph.is_goal_reached_)
        {
            std::cout << " SOLUTION FOUND  " << std::endl;
            got_path = true;
            break;
        }

        Config sample_config = graph.get_random_sample();
        if(!IsValidArmConfiguration(sample_config, numofDOFs, map, x_size, y_size))
            continue;
        num_samples++;
        graph.extend_tree_rrt_star(sample_config);
    }

    if (got_path)
    {
        std::vector<Config> path_vector = graph.get_path();
        *planlength = (int)path_vector.size();
        *plan = (double**) malloc(path_vector.size()*sizeof(double*));

        for (int i = 0; i < path_vector.size(); i++)
        {
            (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
            for(int j = 0; j < numofDOFs; j++)
            {
                (*plan)[i][j] = path_vector[i][j];
            }
        }
    }
    std::cout << "num_samples " << num_samples << "\n";

    return;
}

/////////////////////////////////////////////////////////PRM//////////////////////////////////////////////////////////////////

struct kd_node_t{
  int index;
  double x[DOF];
  struct kd_node_t *left, *right;
};

void initialNode(struct kd_node_t *node, double* value){
  for(int i = 0; i < DOF; i++){
    node->x[i] = value[i];
  }
}

class kdtree{
  public:
  inline double dist(struct kd_node_t *a, struct kd_node_t *b, int dim)
  {
    double t, d = 0;
    while (dim--) {
      t = a->x[dim] - b->x[dim];
      d += t * t;
    }
    return d;
  }

  inline void swapping(struct kd_node_t *x, struct kd_node_t *y)
  {
    int index_temp;
    index_temp = x->index;
    x->index = y->index;
    y->index = index_temp;

    double tmp[DOF];
    memcpy(tmp, x->x, sizeof(tmp));
    memcpy(x->x, y->x, sizeof(tmp));
    memcpy(y->x, tmp, sizeof(tmp));
  }

  struct kd_node_t* finding_median(struct kd_node_t *start, struct kd_node_t *end, int idx)
  {
    if (end <= start) return NULL;
    if (end == start + 1)
      return start;

    kd_node_t *p, *store, *md = start + (end - start)/2;
    double pivot;
    while (1) {
      pivot = md->x[idx];

      swapping(md, end - 1);
      for(store = p = start; p < end; p++){
        if (p->x[idx] < pivot){
          if (p != store)
            swapping(p, store);
          store++;
        }
      }
      swapping(store, end - 1);

      if (store->x[idx] == md->x[idx])
        return md;

      if (store > md)
        end = store;
      else
        start = store;
    }
  }

  struct kd_node_t* make_tree(struct kd_node_t *t, int len, int i, int dim)
  {
    struct kd_node_t *n;

    if (!len)
      return 0;
    if ((n = finding_median(t, t + len, i))){
      i = (i + 1) % dim;
      n->left = make_tree(t, n - t, i, dim);
      n->right = make_tree(n + 1, t + len - (n + 1), i, dim);
    }
    return n;
  }
  void nearest(struct kd_node_t *root, struct kd_node_t *nd, int i, int dim, struct kd_node_t **best, double *best_dist)
  {
    double d, diffx , diff2x ;

    if(!root)
      return;
    d = dist(root, nd, dim);
    diffx  = root->x[i] - nd->x[i];
    diff2x  = diffx  * diffx ;

    if(!*best || d < *best_dist){
      *best_dist = d;
      *best = root;
    }

    if (!*best_dist)
      return;

    if (++i >= dim)
      i = 0;

    nearest(diffx  > 0 ? root->left : root->right, nd, i, dim, best, best_dist);
    if (diff2x  >= *best_dist)
      return;
    nearest(diffx  > 0 ? root->right : root->left, nd, i, dim, best, best_dist);
  }

  void near(struct kd_node_t *root, struct kd_node_t *nd, int i, int dim, vector<struct kd_node_t *>& neighbor, double best_dist)
  {
    double d, diffx , diff2x ;

    if(!root)
      return;
    d = sqrt(dist(root, nd, dim));
    diffx  = root->x[i] - nd->x[i];
    diff2x  = fabs(diffx );

    if( d < best_dist){
      neighbor.push_back(root);
    }

    if (++i >= dim)
      i = 0;

    near(diffx  > 0 ? root->left : root->right, nd, i, dim, neighbor, best_dist);
    if (diff2x  >= best_dist)
      return;
    near(diffx  > 0 ? root->right : root->left, nd, i, dim, neighbor, best_dist);
  }

};

struct prmNode{
  int index;
  double x[DOF];
};
void initialPrmNode(prmNode* node, double *value){
  node->index = 0;
  for(int i = 0; i < DOF; i++)
    node->x[i] = value[i];
}

bool comp(const pair<prmNode*, double> &a, const pair<prmNode*, double> &b)
{
    return a.second < b.second;
}

class AstarPoint{
public:
  int index;
  double g;
  double h;
  double f;
  AstarPoint* parent;

public:
  AstarPoint(int indexIn, double gIn){
    index = indexIn;
    g = gIn;
    parent = NULL;
  };

  ~AstarPoint(){
    delete parent;
  };

  bool operator<(const AstarPoint &a)const
  {
    return a.g < g;
  };

};

class prm: public kdtree{
public:
  int V;
  vector< list< pair<prmNode*, double> > > graph;
  struct kd_node_t *kdTree;
  struct kd_node_t *root;

public:
  prm(int VIn){
    V = VIn;
    kdTree = (struct kd_node_t*) calloc((V-2), sizeof(struct kd_node_t));
    root = NULL;
  };

  ~prm(){
    if(!graph.empty())
      graph.clear();

    free(kdTree);
    if(root)
     delete root;
  }
  void addVertice(prmNode *v){
    list< pair<prmNode*, double> > adj;
    adj.push_back(make_pair(v, 0));
    graph.push_back(adj);
    call_count_prm++;
  };

  void addKdNode(struct kd_node_t *node){
    kdTree[node->index] = *node;
  }
  void addEdge(prmNode *u, prmNode *v, double weight){
    graph[u->index].push_back(make_pair(v, weight));
    graph[v->index].push_back(make_pair(u, weight));
  };
  double dist(prmNode *u, prmNode *v){
    double dist = 0;
    for(int i = 0; i < DOF; i++)
      dist += (u->x[i] - v->x[i])*(u->x[i] - v->x[i]);
    dist = sqrt(dist);
    return dist;
  };
  bool checkConnected(int u, int v){
    bool *visited = new bool[V];
    printf("break point 0\n");

    for(int i = 0; i < V; i++)
      visited[i] = false;

    int cnt = 0;
    dfstil(u, visited, cnt);

    if(visited[v]){
      delete [] visited;
      return true;
    }
    else{
      delete [] visited;
      return false;
    }

  };

  void dfstil(int u, bool visited[], int cnt){
    printf("before ");
    if(u > (V - 1) || u < 0)
      printf("big error\n");
    printf(" %d\n", visited[u]);
    visited[u] = true;
    cnt++;
    printf("cnt %d u %d \n", cnt, u);

    list< pair<prmNode*, double> >::iterator iter0;
    for(iter0 = graph[u].begin(); iter0 != graph[u].end(); iter0++){
      printf(" %d  %d ", (*iter0).first->index, visited[(*iter0).first->index]);
    }
    printf("\n");

    list< pair<prmNode*, double> >::iterator iter;
    for(iter = graph[u].begin(); iter != graph[u].end(); iter++){
      if(!visited[(*iter).first->index]){
        printf(" dfs %d %d\n", (*iter).first->index, visited[(*iter).first->index]);
        dfstil((*iter).first->index, visited, cnt);
      }
    }

    return;

  };


  bool Connection_check_backup(int u, int v){

    vector<bool> visited(V, false);

    dfstil_backup(u, visited);

    if(visited[v]){
      return true;
    }
    else{
      return false;
    }

  };

  void dfstil_backup(int u, vector<bool> &visited){

    visited[u] = true;

    list< pair<prmNode*, double> >::iterator iter;
    for(iter = graph[u].begin(); iter != graph[u].end(); iter++){
      if(!visited[(*iter).first->index]){
        dfstil_backup((*iter).first->index, visited);
      }
    }

    return;

  };

  bool checkValid(int u, int v, double espln, double* map, int x_size, int y_size){
    prmNode* start = graph[u].front().first;
    prmNode* goal = graph[v].front().first;
    double dist_temp = dist(start, goal);
    bool result = true;

    double lamda = espln/dist_temp;
    double base = lamda;

    while(result && lamda < 1 ){
      double *node_new = new double[5];
      for(int i = 0; i < DOF; i++){
          node_new[i] = lamda*goal->x[i] + (1-lamda)*start->x[i];
      }
      if(!IsValidArmConfiguration(node_new, DOF, map, x_size, y_size)){
        result = false;
      }

      delete[] node_new;
      lamda = lamda + base;
    }

    return result;
  };

  void process(double radius, int maxLimit, double espln, double *map, int x_size, int y_size){

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    root = make_tree(kdTree, (V-2), 0, DOF);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    double kdtree_time = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    std::cout << "kdtree generation time " << kdtree_time << std::endl;
    bool debugPrint = true;
    for(int number = 0; number < (V-2); number++){

      struct kd_node_t *node = new kd_node_t;
      node->index = number;
      initialNode(node, &(graph[number].front().first->x[0]));

      vector<struct kd_node_t *> neighbor;
      near(root, node, 0, DOF, neighbor, radius);
      delete node;

      if(neighbor.size() > 1){
        vector<pair<prmNode*, double> > nearVertice;
        vector<struct kd_node_t *>::iterator iter;
        for(iter = neighbor.begin(); iter != neighbor.end(); iter++){
          if((*iter)->index != number){
            double dist_temp = dist(graph[(*iter)->index].front().first, graph[number].front().first);
            nearVertice.push_back(make_pair(graph[(*iter)->index].front().first, dist_temp));
          }
        }

        std::sort(nearVertice.begin(), nearVertice.end(), comp);
        int neighborSize;
        if(neighbor.size() <= maxLimit)
          neighborSize = nearVertice.size();
        else
          neighborSize = maxLimit;

        for(int i = 0; i < neighborSize; i++){
          bool alreadyIn = false;
          list<pair<prmNode*, double>>::iterator iter;
          for(iter = graph[number].begin(); iter != graph[number].end(); iter++){
            if(nearVertice[i].first->index == (*iter).first->index)
              alreadyIn = true;
          }

          if(!alreadyIn){
            bool valid = checkValid(number, nearVertice[i].first->index, espln, map, x_size, y_size);
            if(valid)
              addEdge(nearVertice[i].first, graph[number].front().first, nearVertice[i].second);
          }
        }
      }
    }

    return;

  };

  void addStartGoal(prmNode* node_start, kd_node_t* kdNode_start, prmNode* node_goal, kd_node_t* kdNode_goal, double radius, double espln, double *map, int x_size, int y_size){
    assert(node_start->index == graph.size());
    addVertice(node_start);
    assert(node_goal->index == graph.size());
    addVertice(node_goal);

    for(int j = 0; j < 2; j++){

      struct kd_node_t *node = new kd_node_t;

      if(j == 0)
        node->index = node_start->index;
      else
        node->index = node_goal->index;

      initialNode(node, &(graph[node->index].front().first->x[0]));

      vector<struct kd_node_t *> neighbor;
      neighbor.clear();
      near(root, node, 0, DOF, neighbor, radius);

      if(neighbor.size() > 1){
        vector<pair<prmNode*, double> > nearVertice;
        vector<struct kd_node_t *>::iterator iter;
        for(iter = neighbor.begin(); iter != neighbor.end(); iter++){
          if((*iter)->index != node->index){
            double dist_temp = dist(graph[(*iter)->index].front().first, graph[node->index].front().first);
            nearVertice.push_back(make_pair(graph[(*iter)->index].front().first, dist_temp));
          }
        }

        std::sort(nearVertice.begin(), nearVertice.end(), comp);
        for(int i = 0; i < nearVertice.size(); i++){
          bool valid = checkValid(node->index, nearVertice[i].first->index, espln, map, x_size, y_size);
          if(valid)
            addEdge(nearVertice[i].first, graph[node->index].front().first, nearVertice[i].second);
        }
      }
      else{
        double best_dist;
        struct kd_node_t *node_near = 0;
        nearest(root, node, 0, DOF, &node_near, &best_dist);
        double dist_temp = dist(graph[node_near->index].front().first, graph[node->index].front().first);
        addEdge(graph[node_near->index].front().first, graph[node->index].front().first, dist_temp);
      }

      delete node;

    }

    return;

  };

  AstarPoint *getLeastNode(list<AstarPoint *> open)
  {if(!open.empty())
      {
        auto resNode = open.front();
        for(auto &node:open)
            if(node->f < resNode->f)
                resNode = node;
        return resNode;
      }
      return NULL;
  };

  double computeH(AstarPoint* tp, prmNode* node_goal){
    int index_temp = tp->index;
    double h;
    h = dist(graph[index_temp].front().first, node_goal);
    return h;
  }
  void best_path(prmNode* node_start, prmNode* node_goal, double*** plan, int* planlength){

    bool result = Connection_check_backup(node_start->index, node_goal->index);

    if(result){
      double weight = 2;
      double *cost = new double[V];
      bool *vst = new bool[V];
      for(int i = 0; i < V; i++)
        {cost[i] = INF;
         vst[i] = false;
        }

      list<AstarPoint *> open;
      vector<AstarPoint *> close;
      AstarPoint *start = new AstarPoint(node_start->index, 0);
      cost[node_start->index] = 0;
      start->h = computeH(start, node_goal);
      start->f = start->g + weight * start->h;
      open.push_back(start);

      int cnt = 0;
      while(!open.empty())
      {
        AstarPoint *expand = getLeastNode(open);
        open.remove(expand);
        close.push_back(expand);

        if(vst[expand->index])
          continue;
        else
          vst[expand->index] = true;

        if(vst[node_goal->index]){
          break;
        }

        cnt++;
        if(cnt % 1000 == 0)
           std::cout << cnt << std::endl;

        list<pair<prmNode*, double>>::iterator iter;
        for(iter = graph[expand->index].begin(), iter++; iter != graph[expand->index].end(); iter++){
          double newG = expand->g + (*iter).second;
          if(!vst[(*iter).first->index] && cost[(*iter).first->index] > newG){
            AstarPoint* tp = new AstarPoint((*iter).first->index, newG);
            tp->parent = expand;
            cost[tp->index] = tp->g;
            tp->h = computeH(tp, node_goal);
            tp->f = tp->g + weight * tp->h;
            open.push_back(tp);
          }
        }
      }

      delete[] vst;
      vst = NULL;
      delete[] cost;
      cost = NULL;


      AstarPoint* goal = close.back();
      assert(goal->index == node_goal->index);
      list<int> path;
      while(goal != NULL){
        path.push_front(goal->index);
        goal = goal->parent;
      }

      *planlength = path.size();

      list<int>::iterator iter = path.begin();
      *plan = (double**) malloc((path.size())*sizeof(double*));
      for(int i = 0; i < *planlength; i++, iter++)
      {
         (*plan)[i] = (double*) malloc(DOF*sizeof(double));
         memcpy((*plan)[i], &(graph[(*iter)].front().first->x[0]), sizeof(graph[(*iter)].front().first->x));
      }

    }
    else{
      std::cout << " fail " << std::endl;
    }
  };

};

static void planner_PRM(
       double*  map,
       int x_size,
       int y_size,
       double* armstart_anglesV_rad,
       double* armgoal_anglesV_rad,
       int numofDOFs,
       double*** plan,
       int* planlength)
{
  //no plan by default
  *plan = NULL;
  *planlength = 0;


  double radius = 0.85;
  double V = 15000;
  double espln = 0.16;
  int maxLimit = 15;
  prm* myPRM = new prm(V);

  int cnt = 0;
  while(cnt < (V-2)){

    double node_rand[5] = { 2*PI*(rand()/double(RAND_MAX)), \
                            2*PI*(rand()/double(RAND_MAX)), \
                            2*PI*(rand()/double(RAND_MAX)), \
                            2*PI*(rand()/double(RAND_MAX)), \
                            2*PI*(rand()/double(RAND_MAX))};

    if(IsValidArmConfiguration(&node_rand[0], DOF, map, x_size, y_size)){
        struct prmNode *node_new = new prmNode;
        initialPrmNode(node_new, &node_rand[0]);
        node_new->index = cnt;
        myPRM->addVertice(node_new);
        struct kd_node_t *kdNode_new = new kd_node_t;
        kdNode_new->index = node_new->index;
        initialNode(kdNode_new, &node_rand[0]);
        myPRM->addKdNode(kdNode_new);
        cnt++;
    }
  }

  struct prmNode *node_start = new prmNode;
  initialPrmNode(node_start, armstart_anglesV_rad);
  node_start->index = myPRM->graph.size();
  struct kd_node_t *kdNode_start = new kd_node_t;
  kdNode_start->index = node_start->index;
  initialNode(kdNode_start, armstart_anglesV_rad);
  struct prmNode *node_goal = new prmNode;
  initialPrmNode(node_goal, armgoal_anglesV_rad);
  node_goal->index = myPRM->graph.size() + 1;
  struct kd_node_t *kdNode_goal = new kd_node_t;
  kdNode_goal->index = node_goal->index;
  initialNode(kdNode_goal, armgoal_anglesV_rad);

  myPRM->process(radius, maxLimit, espln, map, x_size, y_size);

  int edges = 0;
  vector<int> result;
  for(int i = 0; i < V-2; i++){
    edges += (myPRM->graph[i].size() - 1);
    result.push_back(myPRM->graph[i].size() - 1);
  }
  sort(result.begin(), result.end());
  std::cout << "PRM generated " << edges  << " edges, "<< " the average neighbors per node is " << result[V/2 -1] << std::endl;
  
//   chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
//     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//     double kdtree_time = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
//     std::cout << "kdtree generation time " << kdtree_time << std::endl;
  myPRM->addStartGoal(node_start, kdNode_start, node_goal, kdNode_goal, radius, espln, map, x_size, y_size);
  myPRM->best_path(node_start, node_goal, plan, planlength);
  return;
}


//The function below is used to output the cout values in matlab console:
class mystream : public std::streambuf
{
protected:
virtual std::streamsize xsputn(const char *s, std::streamsize n) { mexPrintf("%.*s", n, s); return n; }
virtual int overflow(int c=EOF) { if (c != EOF) { mexPrintf("%.1s", &c); } return 1; }
};
class scoped_redirect_cout
{
public:
  scoped_redirect_cout() { old_buf = std::cout.rdbuf(); std::cout.rdbuf(&mout); }
  ~scoped_redirect_cout() { std::cout.rdbuf(old_buf); }
private:
  mystream mout;
  std::streambuf *old_buf;
};
static scoped_redirect_cout mycout_redirect;

//prhs contains input parameters (3):
//1st is matrix with all the obstacles
//2nd is a row vector of start angles for the arm
//3nd is a row vector of goal angles for the arm
//plhs should contain output parameters (2):
//1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the ith step of the plan
//(there are D DoF of the arm (that is, D angles). So, j can take values from 0 to D-1
//2nd is planlength (int)
void mexFunction( int nlhs, mxArray *plhs[],
		  int nrhs, const mxArray*prhs[])

{

    /* Check for proper number of arguments */
    if (nrhs != 4) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Four input arguments required.");
    } else if (nlhs != 2) {
	    mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }

    /* get the dimensions of the map and the map matrix itself*/
    int x_size = (int) mxGetM(MAP_IN);
    int y_size = (int) mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);

    /* get the start and goal angles*/
    int numofDOFs = (int) (MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
    if(numofDOFs <= 1){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "it should be at least 2");
    }
    double* armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
    if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN))){
        	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidnumofdofs",
                "numofDOFs in startangles is different from goalangles");
    }
    double* armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);

    //get the planner id
    int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
    if(planner_id < 0 || planner_id > 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidplanner_id",
                "planner id should be between 0 and 3 inclusive");
    }

    //call the planner
    double** plan = NULL;
    int planlength = 0;

    if (planner_id==RRT)
    {
        planner_rrt(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
        double path_quality = calc_path_quality(&plan, &planlength, numofDOFs);
        std::cout << "path_quality " << path_quality << std::endl;
    }
    if (planner_id==RRTCONNECT)
    {
        planner_rrt_connect(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
        double path_quality = calc_path_quality(&plan, &planlength, numofDOFs);
        std::cout << "path_quality " << path_quality << std::endl;
 
    }
    if (planner_id==RRTSTAR)
    {
        planner_rrt_star(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
        double path_quality = calc_path_quality(&plan, &planlength, numofDOFs);
        std::cout << "path_quality " << path_quality << std::endl;
         
    }
   if (planner_id == PRM)
    {
       planner_PRM(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
       double path_quality = calc_path_quality(&plan, &planlength, numofDOFs);
       std::cout << "path_quality " << path_quality << std::endl;

   }

    printf("planner returned plan of length=%d\n", planlength);

    /* Create return values */
    if(planlength > 0)
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL);
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int i,j;
        for(i = 0; i < planlength; i++)
        {
            for (j = 0; j < numofDOFs; j++)
            {
                plan_out[j*planlength + i] = plan[i][j];
            }
        }
    }
    else
    {
        PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL);
        double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values
        int j;
        for(j = 0; j < numofDOFs; j++)
        {
                plan_out[j] = armstart_anglesV_rad[j];
        }
    }
    PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL);
    int* planlength_out = (int*) mxGetPr(PLANLENGTH_OUT);
    *planlength_out = planlength;


    return;

}





