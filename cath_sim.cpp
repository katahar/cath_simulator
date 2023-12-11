#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#include <tuple>


#include <iostream>
#include "fssimplewindow.h"
// #include "Eigen/Core"
// #include <Eigen3/Dense>


#define PI 3.14159265
class node;
class joint;
class collision_detector;
class catheter;

class vector
{
	public:
		int dims;
		double* vec; //allocated based on constructor

		vector()
		{
			dims = -1;
			// needs to be instantiated!
		}

		vector(double x, double y, double z)
		{
			dims = 3; 
			vec = new double[dims];
			vec[0] = x;
			vec[1] = y;
			vec[2] = z;
		}

		vector(double x, double y)
		{
			dims = 2; 
			vec = new double[dims];
			vec[0] = x;
			vec[1] = y;
		}

		vector(const vector& other)
		{
			this->dims = other.dims;
			vec = new double[dims];
			for(int i = 0; i < dims; i++)
			{
				this->vec[i] = other.vec[i];
			}
		}

		// @TODO: update for 3d
		//finds the perpendicular of the current vector facing toward the target vector, normalized
		vector get_perpen_toward(vector target)
		{
			vector perpendicular = vector(-vec[1], vec[0]);
			if(target.dot(perpendicular) > 0) //obtuse angle, facing toward target
			{
				return perpendicular.normalize();
			}
			perpendicular = vector(vec[1], -vec[0]);
			return perpendicular.normalize();

		}

		double dot(vector other_vec)
		{
			if(is_compatible(other_vec))
			{
				if(3 == dims)
				{
					return (other_vec.vec[0]*this->vec[0])+(other_vec.vec[1]*this->vec[1])+(other_vec.vec[2]*this->vec[2]);
				}
				if(2 == dims)
				{
					return (other_vec.vec[0]*this->vec[0])+(other_vec.vec[1]*this->vec[1]);
				}
			}
			return 0;
		}

		double at(int ind)
		{
			if (ind<dims)
			{
				return vec[ind];
			}
			else if(-1 == dims)
			{
				std::cout << "ERROR: vector not instantiated."  << std::endl;
			}
			else
			{
				std::cout << "ERROR: vector indexing out of bounds for : " << ind << std::endl;
				return -10000;
			}
		}

		void update(int index, double value)
		{
			vec[index] = value;
		}

		double get_length()
		{
			if(3 == dims)
				return sqrt( (vec[0]*vec[0]) + (vec[1]*vec[1]) +(vec[2]*vec[2]));
			else
				return sqrt( (vec[0]*vec[0]) + (vec[1]*vec[1]) );
		}

		vector normalize()
		{
			vector ret_vec = *this;
			double length = ret_vec.get_length();
			for (int i = 0; i < dims; i++)
			{
				ret_vec.vec[i] = ret_vec.vec[i]/length; 
			} 
			return ret_vec;
		}

		//returns a vector where input component (eg. wall NOT-normal) is removed from this vector (eg velocity)
		vector remove_component(vector input_vec)
		{
			// normalize input
			vector input_norm = input_vec.normalize();

		    // Compute the projection of velocity onto the wall normal
			vector projection = input_norm*(this->dot(input_vec));

			return projection;
			// to complete
			vector ret_vec = (*this)-projection;

			return ret_vec;
		}

		std::string to_string()
		{
			std::string ret_str = "(";
			for(int i =0; i < dims; i++)
			{
				ret_str = ret_str + std::to_string(vec[i]) + ", ";
			}
			ret_str = ret_str + ")";
			return ret_str;
		}

		//returns a vector where input component direction  (eg. joint tangent) is in this vector (eg velocity)
		vector keep_component(vector input_vec)
		{
			// normalize input
			vector input_norm = input_vec.normalize();

		    // Compute the projection of velocity onto the joint tangent
			vector projection = input_norm*(this->dot(input_norm));
			
			return projection;
		}

		vector operator*(double scalar_input)
		{
			vector ret_vec = *this;
			for (int i = 0; i < dims; i++)
			{
				ret_vec.vec[i] = ret_vec.vec[i]*scalar_input; 
			} 		
			return ret_vec;
		}

		vector operator/(double scalar_input)
		{
			vector ret_vec = *this;
			for (int i = 0; i < dims; i++)
			{
				ret_vec.vec[i] = ret_vec.vec[i]/scalar_input; 
			} 		
			return ret_vec;
		}

		vector operator+(vector input)
		{
			vector ret_vec = *this;
			if(is_compatible(input))
			{
				for (int i = 0; i < dims; i++)
				{
					ret_vec.vec[i] = ret_vec.vec[i]+input.vec[i]; 
				} 				
			}
			return ret_vec;
		}

		vector operator-(vector input)
		{
			vector ret_vec = *this;
			if(is_compatible(input))
			{
				for (int i = 0; i < dims; i++)
				{
					ret_vec.vec[i] = ret_vec.vec[i]-input.vec[i]; 
				} 				
			}
			return ret_vec;
		}

		void operator=(const vector& input)
		{
			this->dims = input.dims;
			vec = new double[dims];
			for(int i = 0; i < dims; i++)
			{
				this->vec[i] = input.vec[i];
			}
		}
		
		~vector()
		{
			delete[] vec;
			vec = nullptr;
		}

		// @TODO: equality operator

	private:
		bool is_compatible(vector other_vec)
		{
			if(other_vec.dims != this->dims)
				{
					std::cout << "ERROR: vector dimensions not compatible" << std::endl;	
					return false;
				}
			return true;
		}

};



class render_entity
{
	public:
		const int X = 0;
		const int Y = 1;

		double scale = 10;

		// @TODO: modify to be able to shift view
		void PhysicalCoordToScreenCoord(int &sx,int &sy,double px,double py)
		{
			sx=(int)(px*scale);
			sy=600-(int)(py*scale);
		}

		void PhysicalCoordToScreenDim(int &sdim,double pdim)
		{
			sdim = int(pdim*scale);
		}
		

		void DrawCircle(int cx,int cy,int rad,int fill)
		{
			const double YS_PI=3.1415927;

			if(0!=fill)
			{
				glBegin(GL_POLYGON);
			}
			else
			{
				glBegin(GL_LINE_LOOP);
			}

			int i;
			for(i=0; i<64; i++)
			{
				double angle=(double)i*YS_PI/32.0;
				double x=(double)cx+cos(angle)*(double)rad;
				double y=(double)cy+sin(angle)*(double)rad;
				glVertex2d(x,y);
			}

			glEnd();
		}

		void DrawRect(int x1,int y1,int x2,int y2,int fill)
		{
			if(0!=fill)
			{
				glBegin(GL_QUADS);
			}
			else
			{
				glBegin(GL_LINE_LOOP);
			}

			glVertex2i(x1,y1);
			glVertex2i(x2,y1);
			glVertex2i(x2,y2);
			glVertex2i(x1,y2);

			glEnd();
		}

		void DrawLine(int x1,int y1,int x2,int y2)
		{
			glBegin(GL_LINES);

			glVertex2i(x1,y1);
			glVertex2i(x2,y2);

			glEnd();
		}

		void DrawLineThick(int x1,int y1,int x2,int y2, int thickness)
		{
			int x_bl = x1-(thickness/2);
			int x_br = x1+(thickness/2);
			int x_tl = x2-(thickness/2);
			int x_tr = x2+(thickness/2);
			int y_t = y2;
			int y_b = y1;
			glBegin(GL_QUADS);
			glVertex2i(x_tl,y_t);
			glVertex2i(x_tr,y_t);
			glVertex2i(x_br,y_b);
			glVertex2i(x_bl,y_b);

			glEnd();

		}
};


class node: public render_entity
{
	public:

		// node* node1;
		// node* node2;
		double neutral_angle = PI; //angle between adjacent nodes
		double current_angle;
		double spring_constant = 20; 
		double joint_distance = 0; //distance between two bodies. Body to joint "center" is joint_distance/2
		double dist_tol = 0.05; //meters
		double angle_tol = 2*PI/180; //meters

		vector pos;
		vector vel = vector(0,0);
		vector acc = vector(0,0);

		double mass;
		bool fixed;

		double radius;
		std::vector<node*> connected_nodes;


		node(double x, double y, double radius, double neutral_angle, double spring_const, double joint_dist)
		{
			pos = vector(x,y);
			// pos[0] = x;
			this->radius = radius;
			this->neutral_angle = neutral_angle;
			this->spring_constant = spring_const;
			// this->joint_distance = hypot(node1->get_pos(node1->X)- node2->get_pos(node2->X), node1->get_pos(node1->Y)- node2->get_pos(node2->Y));
			this->joint_distance = joint_dist;
			this->reset();
		}

		
		bool connected_to(node* other_node)
		{
			if(!connected_nodes.empty())
			{
				//already connected
				if(std::find(connected_nodes.begin(), connected_nodes.end(), other_node) != connected_nodes.end()) 
				{
					return true; 
				}
			}
			return false;
		}

		// bidirectional connection
		void connect_node(node* other_node)
		{
			if(!this->connected_to(other_node))
			{
				this->connected_nodes.push_back(other_node);
				
				if(!other_node->connected_to(this))
				{
					other_node->connected_nodes.push_back(this);
				}
				
			}
		}


		double get_pos(int index)
		{
			return pos.at(index);
		}

		vector get_pos()
		{
			return pos;
		}

		double get_vel(int index)
		{
			return vel.at(index);
		}

		double get_acc(int index)
		{
			return acc.at(index);
		}
		
		vector get_acc()
		{
			return acc;
		}
		double get_rad()
		{
			return this->radius;
		}

		// using dot product definition
		double update_angle()
		{
			if(!this->is_terminal())
			{
				vector A = connected_nodes[0]->get_pos() - this->get_pos();
				vector B = connected_nodes[1]->get_pos() - this->get_pos();
				double numer = A.dot(B);
				double denom = A.get_length() * B.get_length();
				current_angle = acos(numer/denom);
			}
			else
			{
				current_angle = 0;
			}
			return current_angle;
		}


		void draw()
		{
			glColor3ub(255,0,0);

			int sRad, sx, sy;
			render_entity::PhysicalCoordToScreenDim(sRad, radius);
			render_entity::PhysicalCoordToScreenCoord(sx,sy,pos.at(render_entity::X), pos.at(render_entity::Y));
			render_entity::DrawCircle(sx,sy,sRad,true);
		}
		// @TODO: String

		// should only be used for anchor nodes
		void move_rel_pos(double x, double y) //need to update to include z
		{
			pos.update(render_entity::X, pos.at(render_entity::X)+x);
			pos.update(render_entity::Y, pos.at(render_entity::Y)+y);
		}
		
		void move_rel_pos(vector input) //need to update to include z
		{
			pos = pos + input;
		}

		void fix_node()
		{
			fixed = true;
		}

		void add_accel(double x, double y)
		{
			acc.update(render_entity::X, acc.at(render_entity::X)+x);
			acc.update(render_entity::Y, acc.at(render_entity::Y)+y);
		}

		void add_accel(vector input)
		{
			acc = acc+input;
		}

		void set_accel(double x, double y)
		{
			acc = vector(x,y);
		}
		
		//updates velocity based on acceleration, then updates position based on velocity
		void move(double dt)
		{
			// vel[0] = vel[0]  + acc[0]*dt;
			// vel[1] = vel[1]  + acc[1]*dt;
			// pos[0] = pos[0]  + vel[0]*dt;
			// pos[1] = pos[1]  + vel[1]*dt;

			vel = vel + (acc*dt);
			pos = pos + (vel*dt);
		}

		void reset()
		{
			vel = vector(0,0);
			acc = vector(0,0);
			update_angle();
		}

		// add_constraint : directly modifies acceleration based on input constraint
		void add_constraint(vector constraint)
		{
			acc.remove_component(constraint);
			vel.remove_component(constraint);
		}

		double dist_sq(node* node)
		{
			
			double joint_x = this->pos.at(render_entity::X);
			
			double joint_y = this->pos.at(render_entity::Y);
			
			return ((node->get_pos(render_entity::X)-joint_x)*(node->get_pos(render_entity::X)-joint_x)) + ((node->get_pos(render_entity::Y)-joint_y)*(node->get_pos(render_entity::Y)-joint_y));
		}

		double dist(node* node)
		{
			return sqrt(this->dist_sq(node));
		}
		
		bool is_terminal()
		{
			if(this->connected_nodes.empty())
			{
				std::cout << "WARNING: Orphan node!" << std::endl;
			}
			return this->connected_nodes.size()<=1;
		}


		// assumes that each node is connected to up to two other nodes
		node* get_other(node* input)
		{
			if(!this->is_terminal() && this->connected_to(input))
			{
				if(connected_nodes[0]== input)
				{
					return connected_nodes[1];
				}
				else
				{
					return connected_nodes[0];
				}
			}
			std::cout << "ERROR: node not connected to this joint." << std::endl;
			if(this->is_terminal())
			{
				std::cout << "ERROR-CTD: Actually, this is a terminal node. This error should not have been triggered" << std::endl;
			}
			return nullptr;
		}


		std::string to_string()
		{
			// std::string str =  "pos: (" + std::to_string(this->get_pos(this->X)) + ", " + std::to_string(this->get_pos(this->Y)) + ") \tvel: (" + std::to_string(vel.at(this->X)) + ", " + std::to_string(vel.at(this->Y))  + ") \tacc: (" + std::to_string(acc.at(this->X)) + ", " + std::to_string(acc.at(this->Y)) +")" ;
			// std::string str =  "(" + std::to_string(this->current_pos.at(this->X)) + ", " + std::to_string(this->current_pos.at(this->Y)) + ") \tneutral: " + std::to_string(neutral_angle) + ", angle:" +std::to_string(current_angle) ;

			std:: string str = "pos: " + pos.to_string() + "\t vel: " + vel.to_string() + "\t acc: " + acc.to_string();
			return str;
		}

		// should attempt to resolve within one timestep. input is the node that was most recently moved.
		//moves other_node
		void enforce_dist_constraint(node* last_node, double dt)
		{

			// no orphan nodes
			if(connected_nodes.size() !=0)
			{
				node* other_node;
				// used on proximal end
				if(last_node == nullptr)
				{
					other_node = this->connected_nodes[0];
				}
				// middle nodes
				else
				{
					other_node = this->get_other(last_node);
				}
				// std::cout << "Other Node " << other_node->to_string() << std::endl;
				double dist_sq = this->dist_sq(other_node);
				double target_dist_sq = joint_distance*joint_distance;

				// if the distance is greater than the target distance
				if(abs(dist_sq-target_dist_sq) > dist_tol)
				{
					// direction of motion away from last_node toward current node
					vector move_dir = (other_node->get_pos()- this->get_pos()).normalize();
					double move_dist = joint_distance-this->dist(other_node);
					vector target_motion = move_dir*(move_dist);
					vector accel = target_motion/(dt*dt);
					other_node->add_accel(accel);
					// std::cout << "Added acceleration: " << accel_curr_node.to_string() << std::endl;

				}

			}
			else
			{
				std::cout << "WARNING: Orphan node detected: " << this->to_string() << std::endl;
			}

		}
	
		// accelerates node that is not last_node based on angle formed by three nodes
		void apply_bending_force(node* last_node)
		{
			if(!this->is_terminal())
			{
				// std::cout << "\tcurrent angle: " << current_angle << ", target angle: " << neutral_angle << std::endl;
			
				// cmath uses radians.
				if(abs(current_angle-neutral_angle) > angle_tol) //current_angle should have been updated in the reset function
				{

					node* move_node = this->get_other(last_node);
					// std::cout << "\tenforcing angle. current difference is " << std::to_string(current_angle-neutral_angle);
					vector A = this->pos-last_node->get_pos();
					vector B = move_node->get_pos() -this->pos;
					vector force_dir = B.get_perpen_toward(A); //normalized direction of force
					// std::cout << "\tforce direction " << force_dir.to_string();
					// std::cout << "\tapplied acceleration " << (force_dir*(spring_constant*abs(current_angle-neutral_angle))).to_string() << std::endl;

					// force = k*theta
					move_node->add_accel(force_dir*(spring_constant*abs(current_angle-neutral_angle)));
					// std::cout << "bending acceleration: " << move_node->get_acc().to_string() << std::endl;
					
					//              move_node
					//                 /
					//                /   vector B
					//               /
					//            this node 
					//              |
					//              |    vector A
					//              |
					//          last_node

				}
			}
		}

		void apply_constraint(vector obstacle)
		{
			acc.remove_component(obstacle); //propegates to velocity
		}

		~node()
		{
			for(int i = 0; i < connected_nodes.size(); i++)
			{
				delete connected_nodes[i];
			}
		}
};

class line_obstacle: public render_entity
{
	public:
		double pos[4];


		line_obstacle(double x1, double y1, double x2, double y2)
		{
			pos[0] = x1;
			pos[1] = y1; 
			pos[2] = x2;
			pos[3] = y2;
		}

		 line_obstacle(const line_obstacle &input)
		{
			copy(input);
		}

		// // @TODO: get siddedness.
		void get_normal(double norm_vec[])
		{
			norm_vec[0] = -(pos[3]-pos[1]);
			norm_vec[1] =  (pos[2]-pos[0]);

		}

		// given an intended 2D acceleration or velocity, removes the component that would cause violation of constraints
		void confine_motion(double new_motion[], double old_motion[])
		{
			// new_motion 
		}

		void draw()
		{
			int x1, y1, x2, y2;
		    glColor3ub(0,255, 0);	

			render_entity::PhysicalCoordToScreenCoord(x1, y1, pos[0], pos[1]);
			render_entity::PhysicalCoordToScreenCoord(x2, y2, pos[2], pos[3]);
			render_entity::DrawLine(x1, y1, x2, y2);
		}

		double get_x_left()
		{
			return std::min(pos[0], pos[2]);
		}
		double get_x_right()
		{
			return std::max(pos[0], pos[2]);
		}
		double get_y_bot()
		{
			return std::min(pos[1], pos[3]);
		}
		double get_y_top()
		{
			return std::max(pos[1], pos[3]);
		}

		vector get_vec()
		{
			vector ret_vec = vector(pos[0]-pos[3], pos[1]-pos[4]);
			return ret_vec;
		}

		void get_p1(double &x, double &y)
		{
			x = pos[0];
			y = pos[1];
		}

		void get_p2(double &x, double &y)
		{
			x = pos[2];
			y = pos[3];
		}

		void operator=(const line_obstacle &input)
		{
			copy(input);
		}
		
		private:
			void copy(const line_obstacle &input)
			{
				this->pos[0] = input.pos[0];
				this->pos[1] = input.pos[1];
				this->pos[2] = input.pos[2];
				this->pos[3] = input.pos[3];
			}


};


class sdf2D
{
	public:
		static int id;

		int this_id;
		double* sdf = nullptr;
		
		// width of the actual window
		int window_width, window_height; 

		// size of the sdf when considering resolution
		int sdf_width, sdf_height;

		// add pix width and height
		double resolution;

		sdf2D()
		{
		
		}

		sdf2D(const sdf2D &incoming)
		{
			this->copy(incoming);
		}

		void copy(const sdf2D &incoming)
		{
			this->sdf_height = incoming.sdf_height;
			this->sdf_width = incoming.sdf_width;
			this->window_height = incoming.window_height;
			this->window_width = incoming.window_width;
			this->resolution = incoming.resolution;
			this->reset();

			if(incoming.sdf != nullptr)
			{
				// std::cout << "size " << this->sdf_width << " " << this->sdf_height << std::endl;
				// std::cout << "Allocating size " << this->sdf_width*this->sdf_height << std::endl;
				this->sdf = new double[this->sdf_width*this->sdf_height]; // <- problem line!
				// std::cout << (long long int)(this->sdf) << std::endl;

				std::memcpy(sdf, incoming.sdf, sizeof(double) * sdf_width * sdf_height);

			}
		}

		sdf2D(std::vector<line_obstacle> obstacles,int window_width,int window_height, double resolution)
		{
			this_id = id;
			id++;

			this->resolution = resolution;
			this->window_height = window_height;
			this->window_width = window_width;
			this->sdf_width = int(window_width/resolution);
			this->sdf_height = int(window_height/resolution);

			// building 2d array
			// sdf = new double*[sdf_width];
			// for(int i = 0; i < sdf_width; i++)
			// {
			// 	sdf[i] = new double[sdf_height];
			// 	for(int c = 0; c < sdf_height; c++)
			// 	{
			// 		sdf[i][c] = -1;
			// 	}
			// }

			sdf = new double[sdf_width*sdf_height];
			// std::fill(sdf, sdf + (this->sdf_width*this->sdf_height), -1);

			// for(int i = 0; i < sdf_width*sdf_height; i++)
			// {
			// 	sdf[i] = -1;
			// }


			fill_sdf(obstacles);

			// // print all values in sdf
			// for(int i = 0; i < sdf_width; i++)
			// {
			// 	for(int j = 0; j < sdf_height; j++)
			// 	{
			// 		if(get_val(i,j) != -1)
			// 		{
			// 			std::cout << get_val(i, j) << " " ;
			// 		}
			// 	}
			// 	// std:: cout << "" << std::endl;
			// }
			
			std::cout << "SDF complete." << std::endl;
		}

		void fill_sdf(std::vector<line_obstacle> obstacles)
		{
			// staging locations to be evaluated
			std::vector<std::tuple<int, int>> queue;

			// adding obstacles to sdf
			for(line_obstacle obs : obstacles)
			{
				std::vector<std::tuple<int, int>> temp_pixels = line_to_pixels(obs);

				// iterate through line pixels
				for(std::tuple<int, int> pixel: temp_pixels)
				{
					// update sdf as 0 distance to obstacle
					set_val(pixel, 0);
					// add to queue
					queue.push_back(pixel);
				} 
			}

			// wildfire
			while(!queue.empty())
			{
				// gets the last item from the queue
				std::tuple<int, int> current = (queue.back());
				queue.pop_back();

			}
		}

		std::vector<std::tuple<int, int>>  line_to_pixels(line_obstacle obs)
		{

			// @TODO: Add check to see if the line is within the window
			std::vector<std::tuple<int, int>> points;
			double x1, y1, x2, y2, dx, dy;
			obs.get_p1(x1, y1);
			obs.get_p2(x2, y2);

			dx = x2-x1;
			dy = y2-y1;

			// horizontal. vertical motion less than one block
			if(abs(dy) < resolution)
			{
				std::cout << "Adding horizontal to sdf" << std::endl;
				double temp_x = x1;
				int steps = int(abs(dx/resolution));
				for(int i = 0; i < steps; i++)
				{
					std::tuple<int,int> temp_pos = pos_to_ind(x1+(i*dx/steps),y1);
					std::cout << "Point " << i << ": (" << std::get<0>(temp_pos) << ", " << std::get<1>(temp_pos) << ")" << std::endl;
					points.push_back(temp_pos);
				}
				// add last point
				points.push_back(pos_to_ind(x2,y2));
				
			}

			return points;

		}


		// converts real-world coordinates to indices on the sdf;
		std::tuple <int, int> pos_to_ind(double x, double y)
		{
			return std::make_tuple(int(std::floor(x/resolution)), int(std::floor(y/resolution))); //floor and int casting is probably redundant
		}

		// converts sdf pixel coordinates to real world coordinates.  Assumes center of pixel
		void ind_to_pos(double& x_real, double& y_real, std::tuple <int, int> ind)
		{
			x_real = (std::get<0>(ind)*resolution) + (resolution/2);
			y_real = (std::get<1>(ind)*resolution) + (resolution/2);
		}
		

		double get_val(std::tuple <int, int> index)
		{
			return sdf[(std::get<0>(index)*sdf_width)+ std::get<1>(index)];
		}

		double get_val(int x_ind, int y_ind)
		{
			return sdf[(x_ind*sdf_width)+ y_ind];
		}

		void set_val(std::tuple <int, int> index, double val)
		{
			sdf[(std::get<0>(index)*sdf_width)+ std::get<1>(index)] = val;

		}		

		void render()
		{
			// need to scale down
		}

		~sdf2D()
		{
			// std::cout << "calling from destructor" << std::endl;
			reset();
		}

		void reset()
		{
			if(nullptr != sdf)
			{
				// std::cout << "deallocating size " << this->sdf_width*this->sdf_height << std::endl;
				// std::cout << (long long int)(this->sdf) << std::endl;
				delete[] this->sdf;
				this->sdf = nullptr;
			}
		}

		void operator=(const sdf2D& input)
		{
			// std::cout << "assigning from " << this_id << " to " << id++ <<std::endl;
			// this->this_id = id;
			this->copy(input);
		

		}

		


};

class collision_detector
{
	public: 
		std::vector<line_obstacle> obstacles;
		sdf2D dist_field;

		collision_detector()
		{
			
		}

		// resolution is the number of units per pixel. Assumed to be meters
		collision_detector( std::vector<line_obstacle> obstacles,int window_width,int window_height, double resolution)
		{
			this->obstacles = obstacles;
			dist_field = sdf2D(obstacles, window_height, window_height, resolution);
			
		}

		void build_sdf()
		{

		}

		
		// returns index of obstacle connecting with node. If no collision, returns -1 
		int check_collision(node* nd)
		{
			int ind = 0;
			for(auto iter = obstacles.begin(); iter != obstacles.end(); iter++)
			{
				if( coarse_overlap(nd, (*iter)))
				{
					std::cout << "overlap detected!: (" << nd->get_pos(0) << ", " << nd->get_pos(1) << ") and line (" << (*iter).pos[0] << ", " <<  (*iter).pos[1] << ") -> (" << (*iter).pos[2] << ", " <<  (*iter).pos[3] << ")" << std::endl; 
					return ind;
				};
				ind++;
			}
			return -1;

		}

		// returns a vector re
		vector get_constraint_vec(int index)
		{
			return obstacles[index].get_vec();	
		}

		void render_sdf()
		{
			dist_field.render();
		}

		// ~collision_detector()
		// {
		// 	for(int i = 0; i < obstacles.size(); i++)
		// 	{
		// 		delete obstacles[i];
		// 	}
		// }


	private:
		bool coarse_overlap(node* nd, line_obstacle obstacle)
		{
			double nd_x_l = nd->get_pos().at(0) - nd->get_rad()/2;
			double nd_x_r = nd->get_pos().at(0) + nd->get_rad()/2;
			double nd_y_b = nd->get_pos().at(1) - nd->get_rad()/2;
			double nd_y_t = nd->get_pos().at(1) + nd->get_rad()/2;

			// AABB bounding box
			if(nd_x_l<obstacle.get_x_right() &&
				nd_x_r>obstacle.get_x_left() &&
				nd_y_b<obstacle.get_y_top() &&
				nd_y_t>obstacle.get_y_bot())
			{
				return true;
			}
			return false;

		}
};


class catheter : public render_entity
{
	public:
		std::vector<node*> nodes;
		int num_nodes;
		collision_detector det;

		node* base_node;

		catheter(double x_origin, double y_origin, double x_dir, double y_dir, double joint_distance, int num_segments, double radius, double spring_const, collision_detector detector)
		{

			this->det = detector;
			double unit_x = x_dir/hypot(x_dir, y_dir);
			double unit_y = y_dir/hypot(x_dir, y_dir);

			double step_x = unit_x*joint_distance;
			double step_y = unit_y*joint_distance;

			num_nodes = num_segments+1;
			// double spring_const = 700;
			//setting nodes
			for(int i = 0; i < num_nodes; i++)
			{
				node* temp_node = new node(x_origin + (step_x*i), y_origin + (step_y*i),radius, PI, spring_const, joint_distance);
				if(0 == i)
				{
					base_node = temp_node;
					base_node->fix_node();
				}
				nodes.push_back(temp_node);
				
			}

			//connecting nodes
			for(int i = 1; i < num_nodes; i++)
			{
				nodes[i]->connect_node(nodes[i-1]);
				
			}

			//reading nodes

			int count = 0;
			for(auto iter = nodes.begin(); iter != nodes.end(); iter++)
			{
				std::cout << "node " << count << ": (" << (*iter)->get_pos((*iter)->X) << ", " << (*iter)->get_pos((*iter)->Y) << ")" << std::endl;
				count++;
			}

			std::cout << "cath build complete " << std::endl;



		}


		void move_input(double x, double y)
		{
				double x_constrained = x;
				double y_constrained = y;
				
				int col_ind = det.check_collision(base_node);
				if(-1 != col_ind)
				{
					// this is the direction of the wall, not the normal
					vector constraint = det.get_constraint_vec(col_ind);
					// constraint = constraint.get_perpen_toward()
					vector motion = vector(x_constrained,y_constrained);

					vector new_motion = constraint.normalize()*(constraint.dot(motion));



					std::cout << "Input Motion: " << motion.to_string();
					// motion = motion.remove_component(constraint);
					std::cout << "new motion: " << new_motion.to_string() <<std::endl;
					base_node->move_rel_pos(new_motion);
					// std::cout << "\t old acc: " << nodes[i+1]->acc.to_string() << " new acc:  ";
					// base_node->apply_constraint(det.get_constraint_vec(col_ind));
					// std::cout << nodes[i+1]->acc.to_string()  << std::endl;
				}
				else
				{
					base_node->move_rel_pos(x_constrained,y_constrained);
				}
		}

		void update(double dt)
		{
			int col_ind = -1;

			nodes[0]->reset();
			nodes[0]->enforce_dist_constraint(nullptr,dt);
			nodes[1]->move(dt);

			// iterates through all nonterminal nodes
			for(int i = 1; i < num_nodes-1; i ++)
			{
				nodes[i]->reset();

				
				// std::cout << "enforcing distance constraint for node " <<  std::to_string(i) << "....";
				nodes[i]->enforce_dist_constraint(nodes[i-1],dt);
				// std::cout << "done" << std::endl;
				// std::cout << "distance constraint enforced. " <<  std::to_string(i) << std::endl;
				
				// std::cout << "bending force for joint " <<  std::to_string(i) << "...." <<std::endl;
				nodes[i]->apply_bending_force(nodes[i-1]);
				// std::cout << "\tdone" << std::endl;

				// std::cout << "node " << std::to_string(i)<< " pushing node  " << std::to_string(i+1) << ": " << nodes[i+1]->to_string() << std::endl;
				// std::cout << "Acceleration after adding bending and constraints: " << nodes[i]->get_acc().to_string() << std::endl;
				col_ind = det.check_collision(nodes[i+1]);
				if(-1 != col_ind)
				{
					// std::cout << "\t old acc: " << nodes[i+1]->acc.to_string() << " new acc:  ";
					nodes[i+1]->apply_constraint(det.get_constraint_vec(col_ind));
					std::cout << nodes[i+1]->acc.to_string()  << std::endl;
				}

				nodes[i+1]->move(dt);					
			}
			nodes[num_nodes-1]->reset();
			// std::cout << "---------------------------------" << std::endl;

		}

		~catheter()
		{
			// delete base_node;
			base_node = nullptr;
			for(int i = 0; i < nodes.size(); i++)
			{
				delete nodes[i];
			}
		}

		void operator=(const catheter& input)
		{
			this->nodes = input.nodes;
			this->num_nodes = input.num_nodes;
			this->base_node = input.base_node;
		}


		void draw()
		{
			// draw node connections
			int n1x, n1y, n2x, n2y;
			glColor3ub(0,0, 255);	
			for(int i = 0; i < num_nodes-1; i++)
			{
				render_entity::PhysicalCoordToScreenCoord(n1x, n1y, nodes[i]->get_pos(render_entity::X),   nodes[i]->get_pos(render_entity::Y));
				render_entity::PhysicalCoordToScreenCoord(n2x, n2y, nodes[i+1]->get_pos(render_entity::X), nodes[i+1]->get_pos(render_entity::Y));
				render_entity::DrawLine(n1x, n1y, n2x, n2y);
			}

			// draw nodes
			for(int i = 0; i < num_nodes; i++)
			{
				// std::cout << "Drawing node " << i << std::endl;
				nodes[i]->draw();
			}

			// std::cout << "All nodes  drawn" << std::endl;

		}

};

int sdf2D::id = 0;

int main()
{
	// vector wall_vec (0.5,0.5);
	// vector motion_up(0,1);
	// vector motion_down(0,-1);
	// vector motion_left(-1,0);
	// vector motion_right(1,0);

	// std::cout <<"Should not be able to move up or right." << std::endl;
	// std::cout << "attempted up. Post constrint: " << motion_up.remove_component(wall_vec).to_string() << std::endl;
	// std::cout << "attempted down. Post constrint: " << motion_down.remove_component(wall_vec).to_string() << std::endl;
	// std::cout << "attempted left. Post constrint: " << motion_left.remove_component(wall_vec).to_string() << std::endl;
	// std::cout << "attempted right. Post constrint: " << motion_right.remove_component(wall_vec).to_string() << std::endl;
	

	int window_width = 800;
	int window_height = 600;

	// add obstacles
	std::vector<line_obstacle> obs;
	line_obstacle line = line_obstacle(5,7,10,7);
	obs.push_back(line);

	collision_detector cd(obs, window_width,window_height, 0.25);

	// std::cout << "1 " << (long long int )(cd.dist_field.sdf) << std::endl;

	catheter cath(1,1,0,1,1.5,2,0.25, 50, cd);


	std::cout << "opening window..." << std::endl;
	FsOpenWindow(16,16,window_width,window_height,1, "Catheter Simulation");
	int key;

	

	double mv_vel = 0.5;
	double dt = 0.05;

	bool show_sdf = false;
	bool show_obstacles = true;
    while(FSKEY_ESC!=(key=FsInkey()))
    {
		FsPollDevice();
		
		switch(key)
        {
		case FSKEY_UP:
			cath.move_input(0, mv_vel);
			break;
		case FSKEY_DOWN:
			cath.move_input(0, -mv_vel);
			break;
        case FSKEY_LEFT:
			cath.move_input(-mv_vel, 0);
            break;
        case FSKEY_RIGHT:
			cath.move_input(mv_vel, 0);
            break;
		case FSKEY_Q:
			// show SDF
			show_sdf = !show_sdf;
			break;
		case FSKEY_R:
			// show obstacles
			show_obstacles = !show_obstacles;
			break;
			
        }


		cath.update(dt);
		
		
		glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
		cath.draw();

		if(show_obstacles)
		{
			// draw obstacles
			for (auto iter = obs.begin(); iter != obs.end(); iter++)
			{
				(*iter).draw();
			}
		}
		if(show_sdf)
		{
			cd.render_sdf();
		}
		
        FsSwapBuffers();
        FsSleep(25);
	}


	return 0;
}