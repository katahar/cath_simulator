#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#include <tuple>
#include <queue>
#include <chrono>
#include <ctime>
#include <fstream>
#include <sstream>

#include <iostream>
#include "fssimplewindow.h"
#include <Eigen/Dense>


#define PI 3.14159265

class node;
class joint;
class collision_detector;
class catheter;
class environment;

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

		// assumes radians
		vector rotate(double angle)
		{
			vector ret_vec = vector( (this->at(0)*cos(angle)) - this->at(1)*sin(angle), (this->at(0)*sin(angle)) + this->at(1)*cos(angle));
			return ret_vec;
		}

		//returns a vector where input component (eg. wall NOT-normal) is removed from this vector (eg velocity)
		vector remove_component(vector input_vec)
		{
			// normalize input
			vector input_norm = input_vec.normalize();

		    // Compute the projection of velocity onto the wall normal
			vector projection = input_norm*(this->dot(input_vec));

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

		double scale = 6; // pixels/sim unit
		

		// @TODO: Add x rendering criteria too
		// prevents crashing when moving vertically offscreen.
		bool on_screen(double px, double py)
		{
			int sx, sy;
			PhysicalCoordToScreenCoord(sx,sy,px,py);
			if(sy<=600 && sy>=0)
			{
				return true;
			}
			return false;
		}

		bool on_screen(int sx, int sy)
		{
			if(sy<=600 && sy>=0)
			{
				return true;
			}
			return false;
		}



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
			if(on_screen(cx,cy))
			{
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
					if(on_screen(int(x),int(y)))
					{
						glVertex2d(x,y);
					}
				}

				glEnd();
			}
			else
			{
				// std::cout << "CENTER OFF SCREEN" <<std::endl;
			}
		}

		// void DrawRect(int x1,int y1,int x2,int y2,int fill)
		// {
		// 	if(0!=fill)
		// 	{
		// 		glBegin(GL_QUADS);
		// 	}
		// 	else
		// 	{
		// 		glBegin(GL_LINE_LOOP);
		// 	}

		// 	glVertex2i(x1,y1);
		// 	glVertex2i(x2,y1);
		// 	glVertex2i(x2,y2);
		// 	glVertex2i(x1,y2);

		// 	glEnd();
		// }

		void  DrawLine(int x1,int y1,int x2,int y2)
		{
			if(on_screen(x1,y1) && on_screen(x2,y2))
			{
				glBegin(GL_LINES);

				glVertex2i(x1,y1);
				glVertex2i(x2,y2);

				glEnd();
				
			}
		}

		// void DrawLineThick(int x1,int y1,int x2,int y2, int thickness)
		// {
		// 	int x_bl = x1-(thickness/2);
		// 	int x_br = x1+(thickness/2);
		// 	int x_tl = x2-(thickness/2);
		// 	int x_tr = x2+(thickness/2);
		// 	int y_t = y2;
		// 	int y_b = y1;
		// 	glBegin(GL_QUADS);
		// 	glVertex2i(x_tl,y_t);
		// 	glVertex2i(x_tr,y_t);
		// 	glVertex2i(x_br,y_b);
		// 	glVertex2i(x_bl,y_b);

		// 	glEnd();

		// }
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
		double angle_tol = (PI/180)*1; //degrees

		vector pos;
		vector vel = vector(0,0);
		vector acc = vector(0,0);

		double mass;
		bool fixed;
		bool is_tip;

		double radius;
		std::vector<node*> connected_nodes;


		node(double x, double y, double radius, double neutral_angle, double spring_const, double joint_dist, bool is_tip = false)
		{
			pos = vector(x,y);
			// pos[0] = x;
			this->is_tip = is_tip;
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
				// https://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors/21486462#21486462
				vector A = connected_nodes[0]->get_pos() - this->get_pos();
				vector B = connected_nodes[1]->get_pos() - this->get_pos();
				current_angle = atan2(B.at(1), B.at(0))-atan2(A.at(1), A.at(0));
				// normalize it to the range [0, 2 π):
				if (current_angle < 0) {current_angle  += 2 *PI; }			
				// std::cout << "new method: " << current_angle ; 


				// // std::cout<< "Angle: " << current_angle<< std::endl;
			}
			else
			{
				current_angle = 0;
			}
			return current_angle;
		}

		double get_spring_const()
		{
			return this->spring_constant;
		}

		void draw()
		{
			if(this->is_tip)
			{
				glColor3ub(235,140,52);
			}
			else
			{
				glColor3ub(255,0,0);
			}

			int sRad, sx, sy;
			render_entity::PhysicalCoordToScreenDim(sRad, radius);
			render_entity::PhysicalCoordToScreenCoord(sx,sy,pos.at(render_entity::X), pos.at(render_entity::Y));
			render_entity::DrawCircle(sx,sy,render_entity::scale*0.5,true);
		}

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
		
		//updates velocity based on acceleration, then updates position based on velocity. Velocity always reset to 0
		void move(double dt)
		{
			double damping = 5;
			vel = vel + (acc*dt);
			acc = acc - (vel*damping*dt); 

			vel = (acc*dt);
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
				// if(this->is_tip)
				// {std::cout << "\ttolerance: " << angle_tol << "  current angle: " << current_angle << ", target angle: " << neutral_angle << std::endl;}
			
				// cmath uses radians.
				if(abs(current_angle-neutral_angle) > angle_tol) //current_angle should have been updated in the reset function
				{

					node* move_node = this->get_other(last_node);
					// std::cout << "\tenforcing angle. current angle: " << current_angle << " difference " << std::to_string(current_angle-neutral_angle);
					// std::cout << " \tdifference " << std::to_string(current_angle-neutral_angle);
					vector A = last_node->get_pos()-this->pos;
					
					// A_rot is vector A rotated by the target angle, which is the goal configuration of move_node
					vector A_rot = A.rotate(neutral_angle);
					
					vector B = move_node->get_pos() -this->pos;
					vector force_dir = B.get_perpen_toward(A_rot); //normalized direction of force
					// std::cout << "\tforce direction " << force_dir.to_string();
					// std::cout << "\tapplied acceleration " << (force_dir*(spring_constant*abs(current_angle-neutral_angle))).to_string() << std::endl;

					// std::cout << "\t Force magnitude: " << spring_constant*abs(current_angle-neutral_angle)*1 << std::endl;
					// force = k*theta
					move_node->add_accel(force_dir*(spring_constant*abs(current_angle-neutral_angle)*1));
					// std::cout << "\t\tbending acceleration: " << move_node->get_acc().to_string() << std::endl;
					
					//              move_node
					//         \        /
					//   A_rot  \     /   vector B
					//           \   /
					//            this node 
					//              |
					//              |    vector A
					//              |
					//          last_node

				}
			}
		}

		// returns the difference between the current angle and the neutral angle.
		double get_angle_dif()
		{
			return abs(this->current_angle-this->neutral_angle);
		}

		double get_neutral_angle()
		{
			return this->neutral_angle;
		}

		void apply_constraint(vector obstacle)
		{
			acc.remove_component(obstacle); //propegates to velocity
		}

		void update_neutral_angle(float new_angle)
		{
			this->neutral_angle = new_angle;
		}
		
		void update_joint_dist(double new_dist)
		{
			this->joint_distance = new_dist;
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
		
		std::string to_string()
		{
			std::string str = "(" + std::to_string(pos[0]) + ", " + std::to_string(pos[1]) + ") -> (" + std::to_string(pos[2]) + ", " + std::to_string(pos[3]) + ")";
			return str;
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

class closed_obstacle
{
	public: 
		// in global frame.
		std::vector<line_obstacle> lines;
		int num_sides;

		// in global frame. Never manually assigned. Always derived
		vector center; 

		closed_obstacle()
		{
			//nothing
		}

		closed_obstacle(const closed_obstacle &input)
		{
			copy(input);
		}

		closed_obstacle(std::vector<line_obstacle> lines)
		{
			this->lines = lines;
			num_sides = lines.size();
			this->calculate_center();
		}

		// populates from a vector of corner points
		closed_obstacle(std::vector<vector> points)
		{
			this->num_sides = points.size();
			for(int i = 0; i < num_sides-1; i++)
			{
				line_obstacle temp_line = line_obstacle(points[i].at(0),points[i].at(1), points[i+1].at(0),points[i+1].at(1) );
				this->lines.push_back(temp_line);
			}
			// side from the last point to the first
			line_obstacle temp_line = line_obstacle(points[num_sides-1].at(0),points[num_sides-1].at(1), points[0].at(0),points[0].at(1));
			this->lines.push_back(temp_line);

			this->calculate_center();

		}

		void operator=(const closed_obstacle &input)
		{
			copy(input);
		}

		vector get_center()
		{
			return center;
		}

		line_obstacle get_line(int index)
		{
			if(index<num_sides)
			{
				return lines[index];
			}
			else
			{
				std::cout << "ERROR: side index outside of bounds" << std::endl;
			}
		}

		// returns another closed_obstacle recentered at new location. Shape and orientation is maintained
		closed_obstacle transform(vector new_center)
		{
			vector offset = new_center-this->center;
			
			std::vector<line_obstacle> new_lines;
			for(int i = 0; i < this->num_sides; i++)
			{
				double x1, y1, x2, y2;
				lines[i].get_p1(x1,y1);
				lines[i].get_p2(x2,y2);
				line_obstacle temp_line = line_obstacle(x1 + offset.at(0), y1 + offset.at(1), x2+offset.at(0), y2+offset.at(1));
				new_lines.push_back(temp_line);
			}

			closed_obstacle ret_obs = closed_obstacle(new_lines);
			return ret_obs;
		}

		int get_num_sides()
		{
			return this->num_sides;
		}

		void draw()
		{
			for(line_obstacle ln:lines)
			{
				ln.draw();
			}
		}

		std::string to_string()
		{
			std::string str = "line obstacle: ";
			for(line_obstacle ln:lines)
			{
				str = str + "\n\t\t" + ln.to_string();
			}
			return str;

		}

	private:
		void copy(const closed_obstacle &input)
		{
			this->lines = input.lines;
			this->num_sides = lines.size();

			this->calculate_center();
		}

		void calculate_center()
		{
			double x_cent = 0, y_cent = 0, x_temp, y_temp;
			for(int i = 0; i < num_sides; i++)
			{
				// only need the first one because
				lines[i].get_p1(x_temp, y_temp);
				x_cent +=x_temp;
				y_cent += y_temp;
			}
			center = vector(x_cent/num_sides, y_cent/num_sides);
		}


};

class sdf2D: public render_entity
{
	public:
		double* sdf = nullptr;
		int* sdf_render = nullptr;
		
		// width of the actual window, render units/pixels
		int window_width, window_height; 

		// size of the sdf when considering resolution: sdf units
		int sdf_width, sdf_height;

		// pixels and render units are interchangable 
		// render unit/sdf unit
		double rend_resolution;

		// meter/sdf unit. calculated by  dividing rend_resolution (render unit/sdf unit) by scale (render unit/m)
		double global_resolution;

		int directs[8][2] = {	{ 0, 1},
								{ 1, 0},
								{ 0,-1},
								{-1, 0},
								{-1,-1},
								{ 1,-1},
								{ 1, 1},
								{-1, 1}	};

		int extended_directs[16][2]= {  {0,2},
										{1,2},
										{2,2},
										{2,1},
										{2,0},
										{2,-1},
										{2,-2},
										{1,-2},
										{0,-2},
										{-1,-2},
										{-2,-2},
										{-2,-1},
										{-2,0},
										{-2,1},
										{-2,2},
										{-1,2} 	};


		// to be scaled in meters
		double dist [8] = { 1,1,1,1,sqrt(2), sqrt(2), sqrt(2), sqrt(2)};
		std::chrono::time_point<std::chrono::system_clock> startTime;
		

		sdf2D()
		{
		
		}

		sdf2D(const sdf2D &incoming)
		{
			this->copy(incoming);
		}

		void copy(const sdf2D &incoming)
		{
			// std::cout << __LINE__ << std::endl;
			this->sdf_height = incoming.sdf_height;
			this->sdf_width = incoming.sdf_width;
			this->window_height = incoming.window_height;
			this->window_width = incoming.window_width;
			this->rend_resolution = incoming.rend_resolution;
			this->global_resolution = incoming.global_resolution;
			this->reset();
			// std::cout << __LINE__ << std::endl;

			// scaling distances by resolutoin
			for(int i = 0; i < 8; i++)
			{
				this->dist[i] = this->dist[i]*global_resolution;
			}
			// std::cout << __LINE__ << std::endl;

			// copying the sdf
			if(incoming.sdf != nullptr)
			{
				// std::cout << "size " << this->sdf_width << " " << this->sdf_height << std::endl;
				// std::cout << "Allocating size " << this->sdf_width*this->sdf_height << std::endl;

				this->sdf = new double[this->sdf_width*this->sdf_height]; 
				// std::cout << (long long int)(this->sdf) << std::endl;

				std::memcpy(sdf, incoming.sdf, sizeof(double) * sdf_width * sdf_height);

			}

			// std::cout << __LINE__ << std::endl;

			// copying the sdf_render
			if(incoming.sdf_render != nullptr)
			{
				// std::cout << "Allocating size " <<  this->sdf_width << " * " << this->sdf_height << "= " << this->sdf_width*this->sdf_height << std::endl;
				this->sdf_render = new int[this->window_width*this->window_height]; // <- problem line!
				// std::cout << (long long int)(this->sdf) << std::endl;
				std::memcpy(sdf_render, incoming.sdf_render, sizeof(int) * window_width * window_height);


			}

			

		}

		sdf2D(std::vector<line_obstacle> obstacles,int window_width,int window_height, double rend_resolution)
		{
			this->rend_resolution = rend_resolution;
			this->global_resolution = rend_resolution/render_entity::scale;
			this->window_height = window_height;
			this->window_width = window_width;
			this->sdf_width = int(window_width/rend_resolution);
			this->sdf_height = int(window_height/rend_resolution);

			
			// std::cout << "Sdf Width:  " << sdf_width << ", sdf height " << sdf_height <<std::endl;

			for(int i = 0; i < 8; i++)
			{
				this->dist[i] = this->dist[i]*global_resolution;
			}


			sdf = new double[sdf_width*sdf_height];
			std::fill(sdf, sdf + (this->sdf_width*this->sdf_height), -1);

			sdf_render = new int[window_width*window_height];
			std::fill(sdf_render, sdf_render + (this->window_width*this->window_height), -1);


			fill_sdf(obstacles);

			// print all values in sdf
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

		sdf2D(std::vector<closed_obstacle> obstacles,int window_width,int window_height, double rend_resolution)
		{
			this->rend_resolution = rend_resolution;
			this->global_resolution = rend_resolution/render_entity::scale;
			this->window_height = window_height;
			this->window_width = window_width;
			this->sdf_width = int(window_width/rend_resolution);
			this->sdf_height = int(window_height/rend_resolution);

			
			std::cout << "\n\n\nSdf Width:  " << sdf_width << ", sdf height " << sdf_height <<std::endl;
			for(int i = 0; i < 8; i++)
			{
				this->dist[i] = this->dist[i]*global_resolution;
			}

			std::cout << " ======= Scale Definitions =======" << std::endl;
			std::cout << "window size: " << window_width/render_entity::scale << " x " << window_height/render_entity::scale << " sim units" <<  std::endl;
			std::cout << "scale: " << render_entity::scale << " pixels / sim unit" << std::endl;
			std::cout << "rendering resolution: " << rend_resolution << " pixels / sdf unit" << std::endl;
			std::cout << "global resolution: " << 1/global_resolution << " sdf unit / sim units" << std::endl;
			std::cout << "Each sim unit is 1 real mm. \n\n\n" << std::endl;


			sdf = new double[sdf_width*sdf_height];
			std::fill(sdf, sdf + (this->sdf_width*this->sdf_height), -1);

			sdf_render = new int[window_width*window_height];
			std::fill(sdf_render, sdf_render + (this->window_width*this->window_height), -1);


			fill_sdf(obstacles);

			// // h range 4-6
			// // v range 11 - 13
			// int x_low, x_high, y_low, y_high;
			// real_to_sdf(x_low, y_low, 4.8,6.5);
			// real_to_sdf(x_high, y_high, 5.5,7.25);
			// std::cout << "x window: " << x_low << ", " << x_high << "\t y window" << y_low << ", " << y_high << std::endl;
			// // print all values in sdf
			// for(int j = y_high; j > y_low; j--)
			// {
			// 	for(int i = x_low; i < x_high; i++)
			// 	{
			// 		if(get_sdf_val(i, j) != 0)
			// 		{
			// 			std::cout <<     printf("%.1f", get_sdf_val(i, j) )<< " " ;
			// 		}
			// 		else
			// 		{
			// 			std::cout <<     printf("*.*") << " " ;
			// 		}
			// 	}
			// 	std:: cout << "" << std::endl;
			// }
			
			std::cout << "SDF complete." << std::endl;
		}

        void start_timer()
        {
            startTime = std::chrono::system_clock::now();
        }

        int cumulative_time()
        {
            std::chrono::time_point<std::chrono::system_clock> curTime = std::chrono::system_clock::now();
            return ceil(std::chrono::duration_cast<std::chrono::milliseconds>(curTime - startTime).count()/1000);
        }

		void fill_sdf(std::vector<line_obstacle> obstacles)
		{
			// staging locations to be evaluated
			// assumes locations in queue are indexes of the sdf. No real-world coordinates should be used.
			std::queue<std::tuple<int, int>> q;

			// adding obstacles to sdf
			for(line_obstacle obs : obstacles)
			{
				std::vector<std::tuple<int, int>> temp_pixels = line_to_pixels(obs);

				// iterate through line pixels
				for(std::tuple<int, int> pixel: temp_pixels)
				{
					// update sdf as 0 distance to obstacle
					set_sdf_val(pixel, 0);
					// add to queue
					q.push(pixel);
				} 
			}

			run_wavefront(q);

			fill_sdf_render();

			std::cout << "render reference complete"<< std::endl;

		}

		void fill_sdf(std::vector<closed_obstacle> obstacles)
		{
			// staging locations to be evaluated
			// assumes locations in queue are indexes of the sdf. No real-world coordinates should be used.
			std::queue<std::tuple<int, int>> q;

			// adding obstacles to sdf
			for(closed_obstacle obs : obstacles)
			{
				for(int i = 0; i < obs.get_num_sides(); i++)
				{

					std::vector<std::tuple<int, int>> temp_pixels = line_to_pixels(obs.get_line(i));

					// iterate through line pixels
					for(std::tuple<int, int> pixel: temp_pixels)
					{
						// update sdf as 0 distance to obstacle
						set_sdf_val(pixel, 0);
						// add to queue
						q.push(pixel);
					} 
				}

			}

			run_wavefront(q);

			negate_interior(obstacles);

			fill_sdf_render();

			std::cout << "render reference complete"<< std::endl;

		}

		//gives interior of obstacles negative sdf values
		void negate_interior(std::vector<closed_obstacle> obstacles)
		{
			std::queue<std::tuple<int, int>> q;

			// adds all obstacle centers to the queue
			for(closed_obstacle obs:obstacles)
			{
				std::tuple<int,int> temp_pos = real_to_sdf(obs.get_center().at(0), obs.get_center().at(1));
				set_sdf_val(temp_pos, -1*get_sdf_val(temp_pos));
				q.push(temp_pos);
			}

			run_neg_wavefront(q);
		}

		void run_neg_wavefront(std::queue<std::tuple<int, int>> q)
		{
			std::cout << "Starting neg wavefront." << std::endl;

			// int interior_x, interior_y, ext_x, ext_y;
			// real_to_sdf(interior_x, interior_y, 11,9);
			// real_to_sdf(ext_x, ext_y, 11,9);


			// filling values from obstacle locations. wildfire
			while(!q.empty())
			{
				//pulls from front of queue

				std::tuple<int, int> current = (q.front());
				q.pop();
				// std::cout << "evaluating "<<  std::get<0>(current) << ", " << std::get<1>(current) << std::endl;


				// iterate over directions
				for(int i = 0; i < 4; i++)
				{
					
					if(in_bounds((std::get<0>(current)+directs[i][0]),(std::get<1>(current)+directs[i][1]) ))
					{

						// std::cout << "location " <<  std::get<0>(current) << ", " << std::get<1>(current) << " rel val " << directs[i][0] << ", " << directs[i][1] << std::endl;
						// in_bounds((std::get<0>(current)+directs[i][0]),(std::get<1>(current)+directs[i][1]), true);
						
						// if not an obstacle boundary
						if(get_sdf_val((std::get<0>(current)+directs[i][0]),(std::get<1>(current)+directs[i][1]) ) > 0 )
						{

							std::tuple<int, int> temp_pos = std::make_tuple((std::get<0>(current)+directs[i][0]),(std::get<1>(current)+directs[i][1]) );
							// std::cout << "Sdf Width:  " << sdf_width << ", sdf height " << sdf_height <<std::endl;
							// std::cout << "setting (" << std::get<0>(temp_pos) << ", "  << std::get<1>(temp_pos) << ") = " <<std::endl;
							// std::cout << __LINE__ << std::endl;
							set_sdf_val(temp_pos, -1*get_sdf_val(current, directs[i][0], directs[i][1])); 
							// std::cout << __LINE__ << std::endl;

							q.push(temp_pos);
						}
						// else if (get_sdf_val((std::get<0>(current)+directs[i][0]),(std::get<1>(current)+directs[i][1]) ) == 0)
						// {
						// 	std::cout << "hit wall" << std::endl;
						// }

					}
					else
					{
						std::cout << "\toutside of bounds. location " <<  std::get<0>(current)+directs[i][0] << ", " << std::get<1>(current) +directs[i][1] << std::endl;

					}
					

				}
				

			}
			std::cout << "neg wavefront complete." << std::endl;

		}

		void run_wavefront(std::queue<std::tuple<int, int>> q)
		{
			std::cout << "Starting wavefront." << std::endl;
			start_timer();


			// filling values from obstacle locations. wildfire
			while(!q.empty())
			{
				//pulls from front of queue

				std::tuple<int, int> current = (q.front());
				q.pop();


				// iterate over directions
				for(int i = 0; i < 8; i++)
				{
					
					if(in_bounds((std::get<0>(current)+directs[i][0]),(std::get<1>(current)+directs[i][1]) ))
					{

						// std::cout << "location " <<  std::get<0>(current) << ", " << std::get<1>(current) << " rel val " << directs[i][0] << ", " << directs[i][1] << std::endl;
						// in_bounds((std::get<0>(current)+directs[i][0]),(std::get<1>(current)+directs[i][1]), true);
						
						// if unexplored or farther than current+distance
						if(get_sdf_val((std::get<0>(current)+directs[i][0]),(std::get<1>(current)+directs[i][1]) ) == -1 ||
							get_sdf_val((std::get<0>(current)+directs[i][0]),(std::get<1>(current)+directs[i][1]) ) > get_sdf_val(current)+ dist[i]) 
						{

							std::tuple<int, int> temp_pos = std::make_tuple((std::get<0>(current)+directs[i][0]),(std::get<1>(current)+directs[i][1]) );
							// std::cout << "Sdf Width:  " << sdf_width << ", sdf height " << sdf_height <<std::endl;
							// std::cout << "setting (" << std::get<0>(temp_pos) << ", "  << std::get<1>(temp_pos) << ") = " << get_val(current)+ dist[i] <<std::endl;
							// std::cout << __LINE__ << std::endl;
							set_sdf_val(temp_pos, get_sdf_val(current)+ dist[i]); 
							// std::cout << __LINE__ << std::endl;

							q.push(temp_pos);
						}

					}
					

				}
				

			}
			std::cout << "wavefront complete. " << cumulative_time() << " seconds" << std::endl;

		}

		void fill_sdf_render()
		{
			// used to scale color based on window size
			int color_scale = 3;//std::min(window_height, window_width);

			// double x_real_world, y_real_world;
			int x_render, y_render;
			for(int x = 0; x < sdf_width; x++)
			{
				for(int y = 0; y < sdf_height; y++)
				{
					// converts from sdf to real world coordinates
					sdf_to_rend(x_render, y_render, std::make_tuple(x,y));
					if(get_block_min(x,y) > color_scale)
					{
						sdf_render[(y_render*window_width)+x_render] = 255;
					}
					else
					{
						sdf_render[(y_render*window_width)+x_render] = int(255*(get_block_min(x,y)/color_scale));
					}

				}
			}

			std::cout << "fill_sdf_render complete" << std::endl;

		}

		// input is in sdf coordinates @TODO: actually implement
		double get_block_min(int x_ind, int y_ind)
		{

			double min_dist = get_sdf_val(x_ind,y_ind);
			// @TODO: Remove this return to actually get block min
			return min_dist;

			for(int i = 0; i < 8; i++)
			{
				if(in_bounds(x_ind + directs[i][0], y_ind + directs[i][1] ))
				{
					min_dist = std::min(min_dist, get_sdf_val(x_ind + directs[i][0], y_ind + directs[i][1]) );
				}
			}
			return min_dist;
		}

		// returns direction toward the nearest obstacle (increments of pi/4). Input is in sdf coordinates
		vector get_gradient(int x_sdf, int y_sdf)
		{
			// std::cout << "sdf value at current point (" <<   x_sdf << ", " <<  y_sdf << "): " << get_sdf_val(x_sdf, y_sdf) << std::endl;
			double min_grad = 1000;
			int min_ind = -1;
			
			
			for(int i = 7; i >=0; i--)
			{
				if(in_bounds(x_sdf + directs[i][0], y_sdf + directs[i][1] ))
				{

					// std::cout << "sdf at (" <<   x_sdf + directs[i][0] << ", " <<  y_sdf + directs[i][1] << "): " << get_sdf_val(x_sdf + directs[i][0], y_sdf + directs[i][1] ) << std::endl;
					if(get_sdf_val(x_sdf + directs[i][0], y_sdf + directs[i][1] ) < get_sdf_val(x_sdf,y_sdf) && get_sdf_val(x_sdf + directs[i][0], y_sdf + directs[i][1] ) < min_grad )
					{
						min_ind = i;
						min_grad = get_sdf_val(x_sdf + directs[i][0], y_sdf + directs[i][1] );
					}
				}
			}
			return vector(directs[min_ind][0], directs[min_ind][1] );

			// for(int i = 0; i <16; i++)
			// {
			// 	if(in_bounds(x_sdf + extended_directs[i][0], y_sdf + extended_directs[i][1] ))
			// 	{
			// 		std::cout << "sdf at (" <<   x_sdf + extended_directs[i][0] << ", " <<  y_sdf + extended_directs[i][1] << "): " << get_sdf_val(x_sdf + extended_directs[i][0], y_sdf + extended_directs[i][1] ) << std::endl;
			// 		if(get_sdf_val(x_sdf + extended_directs[i][0], y_sdf + extended_directs[i][1] ) < get_sdf_val(x_sdf,y_sdf)  )
			// 		{
			// 			return vector(extended_directs[i][0], extended_directs[i][1] ).normalize();
			// 		}
			// 	}
			// }
		
		}

		std::vector<std::tuple<int, int>>  line_to_pixels(line_obstacle obs)
		{

			// @TODO: Add check to see if the line is within the window
			std::vector<std::tuple<int, int>> points;
			int interp_scale = 100;
			double x1, y1, x2, y2, dx, dy;
			obs.get_p1(x1, y1);
			obs.get_p2(x2, y2);
		
			dx = x2-x1;
			dy = y2-y1;

			// horizontal. vertical motion less than one block
			if(abs(dy) < global_resolution)
			{
				// std::cout << "Adding horizontal to sdf" << std::endl;
				int steps = int(abs(dx/global_resolution))*interp_scale;
				for(int i = 0; i < steps; i++)
				{
					std::tuple<int,int> temp_pos = real_to_sdf(x1+(i*dx/steps),y1);
					// std::cout << "Point " << i << ": (" << std::get<0>(temp_pos) << ", " << std::get<1>(temp_pos) << ")" << std::endl;
					points.push_back(temp_pos);
				}
				// add last point
				points.push_back(real_to_sdf(x2,y2));
				
			}
			// vertical
			else if (abs(dx) < global_resolution)
			{
				// std::cout << "Adding vertical  to sdf" << std::endl;
				int steps = int(abs(dy/global_resolution))*interp_scale;
				for(int i = 0; i < steps; i++)
				{
					std::tuple<int,int> temp_pos = real_to_sdf(x1,y1+(i*dy/steps));
					// std::cout << "Point " << i << ": (" << std::get<0>(temp_pos) << ", " << std::get<1>(temp_pos) << ")" << std::endl;
					points.push_back(temp_pos);
				}
				// add last point
				points.push_back(real_to_sdf(x2,y2));
				
			}
			// other lines
			else
			{
				// std::cout << "Adding other to sdf" << std::endl;
				int x_steps = int(abs(dy/global_resolution));
				int y_steps = int(abs(dx/global_resolution));

				int steps = std::max(x_steps,y_steps)*20; // @todo: probably a better way to scale this. Originally added because of gaps in lines.
				for(int i = 0; i < steps; i++)
				{
					std::tuple<int,int> temp_pos = real_to_sdf(x1+(i*dx/steps),y1+(i*dy/steps));
					// std::cout << "Point " << i << ": (" << std::get<0>(temp_pos) << ", " << std::get<1>(temp_pos) << ")" << std::endl;
					points.push_back(temp_pos);
				}
				// add last point
				points.push_back(real_to_sdf(x2,y2));
				
			}


			return points;

		}


		// converts real-world coordinates to indices on the sdf;
		std::tuple <int, int> real_to_sdf(double x, double y)
		{
			return std::make_tuple(int(std::floor(x/global_resolution)), int(std::floor(y/global_resolution))); //floor and int casting is probably redundant
		}

		// converts real-world coordinates to indices on the sdf;
		void real_to_sdf(int& x_sdf, int& y_sdf, double x, double y)
		{
			x_sdf = int(std::floor(x/global_resolution));
			y_sdf = int(std::floor(y/global_resolution)); //floor and int casting is probably redundant
			// std::cout << "real: " << x << ", " << y << ", sdf: " << x_sdf << ", " << y_sdf << std::endl;
		}
	
		// converts sdf pixel coordinates to rendering coordinates.  
		void sdf_to_rend(int& x_rend, int& y_rend, std::tuple <int, int> ind_sdf)
		{
			x_rend = int((std::get<0>(ind_sdf)*rend_resolution));
			y_rend = int((std::get<1>(ind_sdf)*rend_resolution));
		}

		bool in_bounds(int x, int y, bool debug = false)
		{
			if(debug)
			{
				std::cout << "x: " << x << " y: " << y << " xub: " << (x<sdf_width) << " yub: " << (y<sdf_height) << " lb " << (y>=0 && x>=0) << std::endl;
			}
			return x<sdf_width && x>=0 && y<sdf_height && y>=0;
		}

		// input is sdf indices
		double get_sdf_val(std::tuple <int, int> index)
		{
			return sdf[(std::get<1>(index)*sdf_width)+ std::get<0>(index)];
		}

		// input is sdf indices and a relative direction
		double get_sdf_val(std::tuple <int, int> index, int rel_x, int rel_y)
		{
			return sdf[((std::get<1>(index)+rel_y)*sdf_width)+ (std::get<0>(index)+rel_x)];
		}

		double get_sdf_val(int x_ind, int y_ind)
		{
			return sdf[(y_ind*sdf_width)+ x_ind];
		}

		void set_sdf_val(std::tuple <int, int> index, double val)
		{
			sdf[(std::get<1>(index)*sdf_width)+ std::get<0>(index)] = val;
		}		

		void render()
		{
			glBegin(GL_POINTS);
		
			for(int x = 0; x < window_width; x++)
			{
				for(int y = 0; y < window_height; y++)
				{
					if(sdf_render[(y*window_width)+x] > 0)
					{
						glColor3ub(255, 255,sdf_render[(y*window_width)+x]);
					}
					else
					{
						glColor3ub( -sdf_render[(y*window_width)+x],255, 255);
					}
					glVertex2i(x,window_height-y);
				}
			}

			glEnd();
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

			if(nullptr != sdf_render)
			{
				// std::cout << "deallocating size " << this->sdf_width*this->sdf_height << std::endl;
				// std::cout << (long long int)(this->sdf) << std::endl;
				delete[] this->sdf_render;
				this->sdf_render = nullptr;
			}
		}

		void operator=(const sdf2D& input)
		{
			// std::cout << "assigning from " << this_id << " to " << id++ <<std::endl;
			this->copy(input);
		}

		


};

class environment
{

	public: 
		std::vector<closed_obstacle> obs;

		environment(std::string obs_file)
		{
			obs = import_obstacles(obs_file);
		}

		environment(const environment &incoming)
		{
			this->copy(incoming);
		}

		std::vector<closed_obstacle> get_obs()
		{
			return obs;
		}


		std::vector<closed_obstacle> import_obstacles(std::string file)
		{
			std::cout << "\n\nImporting environment" << std::endl;

			FsChangeToProgramDir();

			std::fstream obs_file;
			obs_file.open(file, std::ios::in);
			
			if(obs_file.is_open())
			{
				std::string line;
				
				// ignoring the first line
				std::getline(obs_file, line);
				
				// getting the number of polygons (obstacles)
				std::getline(obs_file, line);
				int num_obstacles= std::stoi(line);
				std::cout << "Number of obstacles (triangles) file: " << num_obstacles << std::endl;

				// blank line
				std::getline(obs_file, line);

				for(int i = 0; i < num_obstacles; i++)
				{
					std::vector<vector> obs_corners;
					
					// gets number of vertices for a given obstacle
					std::getline(obs_file, line);
					int num_vertices= std::stoi(line);
					// each corner
					for(int j = 0; j < num_vertices; j++)
					{
						// converts line to std::vector, represents the coordinate of a corner
						std::getline(obs_file, line);
						std::vector<double> temp_coord = str_to_vec(line);
						obs_corners.emplace_back(temp_coord[0], temp_coord[1]); 
					}
					// skip empty line
					std::getline(obs_file, line);

					// corners to a closed obstacle
					closed_obstacle temp_obs = closed_obstacle(obs_corners);
					obs.push_back(temp_obs);
					
				}

				obs_file.close();

				std::cout << "Imported obstacles: " << std::endl;
				for(int i = 0; i < num_obstacles; i++)
				{
					std::cout << "\t" << (obs[i].to_string()) << std::endl;
				}
			}
			else
			{
				std::cout << "ERROR: Failed to open " << file << std::endl;
			}

			return obs;
		}

		std::vector<double> str_to_vec(std::string input)
		{
			std::vector<double> vect;

			std::stringstream ss(input);

			float i;

			while (ss >> i)
			{
				vect.push_back(i);

				if (ss.peek() == ',')
				ss.ignore();
			}
			return vect;
		}

	private:
		void copy(const environment &incoming)
		{
			this->obs = incoming.obs;
		}

		void operator=(const environment &incoming)
		{
			this->copy(incoming);
		}

};


class collision_detector
{
	public: 
		// std::vector<line_obstacle> line_obs;
		// std::vector<closed_obstacle> closed_obs;
		sdf2D dist_field;
		double node_rad;


		collision_detector()
		{
		}

		collision_detector(const collision_detector &incoming)
		{
			copy(incoming);
		}

		// resolution is the number of units per pixel. Assumed to be meters
		collision_detector( std::vector<line_obstacle> obstacles,int window_width,int window_height, double resolution, double node_rad)
		{
			// this->line_obs = obstacles;
			dist_field = sdf2D(obstacles, window_width, window_height, resolution);
			this->node_rad = node_rad;
		}
		
		// resolution is the number of units per pixel. Assumed to be meters
		collision_detector( std::vector<closed_obstacle> obstacles,int window_width,int window_height, double resolution, double node_rad)
		{
			// this->closed_obs = obstacles;
			dist_field = sdf2D(obstacles, window_width, window_height, resolution);
			this->node_rad = node_rad;
		}

		// @TODO: convert to bool. No longer returns index of obstacle
		// returns index of obstacle connecting with node. If no collision, returns -1 
		int check_collision(node* nd)
		{

			if(fine_overlap(nd))
			{
				// std::cout << "fine overlap detected!" << std::endl;
				return 1;
			}
			return -1;

		}

		vector get_obs_norm(double x_pos, double y_pos)
		{
			int x_sdf, y_sdf; 
			dist_field.real_to_sdf(x_sdf, y_sdf, x_pos, y_pos);
			return dist_field.get_gradient(x_sdf,y_sdf);
		}

		void render_sdf()
		{
			dist_field.render();
		}

		double get_penetration_dist(node* nd)
		{
			int x_sdf, y_sdf; 
			dist_field.real_to_sdf(x_sdf, y_sdf, nd->get_pos(0), nd->get_pos(1));
			// std::cout << "sdf at current location is" << dist_field.get_sdf_val(x_sdf, y_sdf) << std::endl;
			if(dist_field.get_sdf_val(x_sdf, y_sdf)  > nd->get_rad() )
			{
				return 0;
			}
			return nd->get_rad() - dist_field.get_sdf_val(x_sdf, y_sdf) ;
		}

		double get_split_penetration_dist(node* nd1,node* nd2)
		{
			int x_sdf, y_sdf; 
			dist_field.real_to_sdf(x_sdf, y_sdf, (nd1->get_pos(0)+nd2->get_pos(0))/2, (nd1->get_pos(1)+nd2->get_pos(1))/2);
			// std::cout << "sdf at current location is" << dist_field.get_sdf_val(x_sdf, y_sdf) << std::endl;
			if(dist_field.get_sdf_val(x_sdf, y_sdf)  > nd1->get_rad() )
			{
				return 0;
			}
			return nd1->get_rad() - dist_field.get_sdf_val(x_sdf, y_sdf) ;
		}

		void copy(const collision_detector &incoming)
		{
			this->dist_field = incoming.dist_field;
			// this->line_obs = incoming.line_obs;
			this->node_rad = incoming.node_rad;

		}

		void operator=(const collision_detector &incoming)
		{
			copy(incoming);

		}

		

	private:

		bool fine_overlap(node* nd)
		{
			int x_sdf, y_sdf; 
			dist_field.real_to_sdf(x_sdf, y_sdf, nd->get_pos(0), nd->get_pos(1));
			// std::cout << "sdf at current location is" << dist_field.get_sdf_val(x_sdf, y_sdf) << std::endl;
			return dist_field.get_sdf_val(x_sdf, y_sdf) <= this->node_rad;
		}
};


class catheter : public render_entity
{
	public:
		static const int CW = 3;
		static const int CCW  = 4;

		std::vector<node*> nodes; //include tip nodes
		std::vector<node*> tip_nodes;
		int num_nodes;
		int num_tip_nodes; //assumes that one is shared with the base catheter
		collision_detector det;

		node* base_node;

		struct tip_config{ 
			int rot_angle; //in degrees
			std::vector<double> joint_distances;
			std::vector<double> joint_angles;
		};

		int num_tip_configs;
		std::vector<tip_config> tip_configs;

		int tip_config_ind;

		catheter(double x_origin, double y_origin, double x_dir, double y_dir, double joint_distance, int num_segments, double radius, double spring_const, collision_detector detector)
		{

			this->det = detector;
			double unit_x = x_dir/hypot(x_dir, y_dir);
			double unit_y = y_dir/hypot(x_dir, y_dir);

			double step_x = unit_x*joint_distance;
			double step_y = unit_y*joint_distance;

			num_nodes = num_segments+1;

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
			
			
			
			build_tip(nodes[num_nodes-1]);

			std::cout << "cath build complete " << std::endl;



		}

		// end_base is the node where the tip will be attached. Assumes rotation angle is 0
		void build_tip(node* end_base)
		{
			load_tip_configs("endpoints/endpoint_config_sparse.txt");

			tip_config_ind = 0;

			// adjusts the last base node's neutral angle
			end_base->update_neutral_angle(tip_configs[tip_config_ind].joint_angles[0]);
			end_base->is_tip = true;
			tip_nodes.push_back(end_base);

			for(int i = 1; i < num_tip_nodes-1; i++)
			{
				node* temp_node = new node(tip_nodes[i-1]->get_pos(0) + (tip_configs[tip_config_ind].joint_distances[i-1]*cos(tip_configs[tip_config_ind].joint_angles[i-1])), 
											tip_nodes[i-1]->get_pos(1) + (tip_configs[tip_config_ind].joint_distances[i-1]*sin(tip_configs[tip_config_ind].joint_angles[i-1])), 
											tip_nodes[i-1]->get_rad(), 
											tip_configs[tip_config_ind].joint_angles[i], 
											tip_nodes[i-1]->get_spring_const(), 
											tip_configs[tip_config_ind].joint_distances[i], 
											true);
				nodes.push_back(temp_node);
				tip_nodes.push_back(temp_node);
			}

			// adding last node
			node* temp_node = new node(tip_nodes[num_tip_nodes-2]->get_pos(0) + (tip_configs[tip_config_ind].joint_distances[num_tip_nodes-2]*cos(tip_configs[tip_config_ind].joint_angles[num_tip_nodes-2])), 
											tip_nodes[num_tip_nodes-2]->get_pos(1) + (tip_configs[tip_config_ind].joint_distances[num_tip_nodes-2]*sin(tip_configs[tip_config_ind].joint_angles[num_tip_nodes-2])), 
											tip_nodes[num_tip_nodes-2]->get_rad(), 
											PI, 
											tip_nodes[num_tip_nodes-2]->get_spring_const(), 
											3, 
											true);
			nodes.push_back(temp_node);
			tip_nodes.push_back(temp_node);

			//connecting nodes
			for(int i = 1; i < num_tip_nodes; i++)
			{
				tip_nodes[i]->connect_node(tip_nodes[i-1]);
			}
			
			num_nodes+=num_tip_nodes-1;


			//reading nodes
			int count = 0;
			for(auto iter = tip_nodes.begin(); iter != tip_nodes.end(); iter++)
			{
				std::cout << "tip node " << count << ": (" << (*iter)->get_pos((*iter)->X) << ", " << (*iter)->get_pos((*iter)->Y) << ")" << std::endl;
				count++;
			}
			

		}

		// updates tip_config_ind index and returns the associated configuration
		tip_config get_config(int direction )
		{
			// assumes we always have configurations 0-180
			if(CCW == direction)
			{
				tip_config_ind++;
				if(tip_config_ind > num_tip_configs-1)
				{
					tip_config_ind = 0;
				}
			}
			else if(CW == direction)
			{
				tip_config_ind--;
				if(tip_config_ind < 0)
				{
					tip_config_ind = num_tip_configs-1;
				}
			}
			else
			{
				std::cout << "ERROR: Invalid applied direction" << std::endl;
			}

			return tip_configs[tip_config_ind];

		}


		// Use catheter::CW or catheter::CCW as input
		void rotate_tip(int direction)
		{
			// getting the updated configuration after rotating
			tip_config temp_config = get_config(direction);

			// std::cout << "Tip nodes: " << num_tip_nodes << std::endl;
			// print_config(temp_config);

			std::cout << "Rotating to " << temp_config.rot_angle << std::endl;
			for(int i = 0; i < num_tip_nodes-1; i++)
			{
				tip_nodes[i]->update_neutral_angle(temp_config.joint_angles[i]);
				tip_nodes[i]->update_joint_dist(temp_config.joint_distances[i]);
			}

		}

		void move_input(double x, double y)
		{
				double x_constrained = x;
				double y_constrained = y;
				
				int col_ind = det.check_collision(base_node);
				// collision detected
				// std::cout << __LINE__ << std::endl;

				// @todo: fix this if. Keeping for reference for now.
				if(false)
				// if(-1 != col_ind)
				{
					
					
					
					std::cout << __LINE__ << std::endl;
					vector input_motion = vector(x,y);
					std::cout << __LINE__ << std::endl;
					vector obstacle_norm = det.get_obs_norm(base_node->get_pos(0), base_node->get_pos(1));
					std::cout << __LINE__ << std::endl;
					std::cout << "input motion" << input_motion.to_string() << std::endl;
					std::cout << "obstacle norm being removed" << obstacle_norm.to_string() << std::endl;
					vector constrained_motion = input_motion.remove_component(obstacle_norm);
					std::cout << __LINE__ << std::endl;

					// // this is the direction of the wall, not the normal
					// vector constraint = det.get_constraint_vec(col_ind);
					// // constraint = constraint.get_perpen_toward()
					// vector motion = vector(x_constrained,y_constrained);

					// vector new_motion = constraint.normalize()*(constraint.dot(motion));



					std::cout << "Input Motion: " << input_motion.to_string();
					// motion = motion.remove_component(constraint);
					std::cout << "new motion: " << constrained_motion.to_string() <<std::endl;
					base_node->move_rel_pos(constrained_motion);


					// std::cout << "\t old acc: " << nodes[i+1]->acc.to_string() << " new acc:  ";
					// base_node->apply_constraint(det.get_constraint_vec(col_ind));
					// std::cout << nodes[i+1]->acc.to_string()  << std::endl;
				}
				else
				{
					base_node->move_rel_pos(x_constrained,y_constrained);
				}
		}

		// applies motion to all nodes
		void translate(double x, double y)
		{
			for(auto nd:nodes)
			{
				nd->move_rel_pos(x,y);
			}

		}

		// returns the direction from the base node tothe next node
		double get_dir(int coord)
		{
			return nodes[1]->get_pos(coord)-base_node->get_pos(coord); 
		}

		// resolves surface penetration as defined by the SDF by applying spring force on the node
		void resolve_penetration(node* node, double dt, double spring_const)
		{

			double pen_lim = 0.2;

			node->reset();
			double penetration_dist = det.get_penetration_dist(node);
			if(penetration_dist>pen_lim)
			{
				std::cout << "reset penetration dist from " << penetration_dist << "to " << pen_lim << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<  std::endl;
				penetration_dist = pen_lim;
			}
			else
			{
				std::cout << "penetration distance " << penetration_dist <<std::endl;
			}
			vector obstacle_norm = det.get_obs_norm(node->get_pos(0), node->get_pos(1));
			node->add_accel(obstacle_norm*-1*spring_const*penetration_dist);
			std::cout << "applied accel" << (obstacle_norm*-1*spring_const*penetration_dist).to_string() << std::endl;
			node->move(dt);
		}

		void resolve_nd_split(node* node1, node* node2, double dt, double spring_const)
		{
			node1->reset();
			node2->reset();
			double split_penetration_dist = det.get_split_penetration_dist(node1,node2);
			
			std::cout << "split penetration distance " << split_penetration_dist << std::endl;
			
			vector obstacle_norm = det.get_obs_norm((node1->get_pos(0)+node2->get_pos(0))/2, (node1->get_pos(1)+node2->get_pos(1))/2);
			node1->add_accel(obstacle_norm*-1*spring_const*split_penetration_dist*2);
			node2->add_accel(obstacle_norm*-1*spring_const*split_penetration_dist*2);
			std::cout << "applied accel" << (obstacle_norm*-1*spring_const*split_penetration_dist*2).to_string() << std::endl;
			node1->move(dt);
			node2->move(dt);
		}

		void update(double dt)
		{
			double obstacle_spring_const = 50;
			int col_ind = -1;
			
			if(-1 != det.check_collision(nodes[0]))
			{
				// std::cout << "Detected collision on input node" << std::endl;
				resolve_penetration(nodes[0],dt, obstacle_spring_const);
			}
			nodes[0]->reset();
			nodes[0]->enforce_dist_constraint(nullptr,dt);
			nodes[1]->move(dt);

			// iterates through all nonterminal nodes
			for(int i = 1; i < num_nodes-1; i ++)
			{
				nodes[i]->reset();

				if(-1 != det.check_collision(nodes[i+1]))
				{
					// std::cout << "\t old acc: " << nodes[i+1]->acc.to_string() << " new acc:  ";
					// vector obstacle_norm = det.get_obs_norm(nodes[i+1]->get_pos(0), nodes[i+1]->get_pos(1));
					// nodes[i+1]->apply_constraint(obstacle_norm);
					// std::cout << nodes[i+1]->acc.to_string()  << std::endl;
					resolve_penetration(nodes[i+1], dt, obstacle_spring_const);
				}

				// checks segment collision
				if(seg_collision(nodes[i], nodes[i+1]))
				{
					resolve_nd_split(nodes[i], nodes[i+1], dt, obstacle_spring_const);
				}

				// std::cout << "enforcing distance constraint for node " <<  std::to_string(i) << "....";
				nodes[i]->enforce_dist_constraint(nodes[i-1],dt);
				// std::cout << "done" << std::endl;
				// std::cout << "distance constraint enforced. " <<  std::to_string(i) << std::endl;
				
				// std::cout << "bending force for joint " <<  std::to_string(i) << "...." <<std::endl;
				
				// angle is too sudden.
				if(i>1 && nodes[i]->get_angle_dif()>PI/5)
				{
					double center_diff = nodes[i]->get_angle_dif();
					double neutral_old = nodes[i-1]->get_neutral_angle();
					nodes[i-1]->update_neutral_angle(neutral_old-(center_diff)/2);
					nodes[i-1]->apply_bending_force(nodes[i-2]); //applies force to current node
					nodes[i-1]->enforce_dist_constraint(nodes[i-2],dt);

					nodes[i]->move(dt);	//moves current node
					nodes[i-1]->update_neutral_angle(neutral_old); //resets old node

					nodes[i]->apply_bending_force(nodes[i-1]); //moves next node


				}
				// respond normally
				else
				{
					nodes[i]->apply_bending_force(nodes[i-1]);
				}
				// std::cout << "\tdone" << std::endl;

				// std::cout << "node " << std::to_string(i)<< " pushing node  " << std::to_string(i+1) << ": " << nodes[i+1]->to_string() << std::endl;
				// std::cout << "Acceleration after adding bending and constraints: " << nodes[i]->get_acc().to_string() << std::endl;
				
				// moved to before the collision detection 
				nodes[i+1]->move(dt);
				
				


									
			}
			nodes[num_nodes-1]->reset();
			// std::cout << "---------------------------------" << std::endl;

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

		void load_tip_configs(std::string file)
		{
			std::cout << "\n\nImporting tip configurations" << std::endl;
			
			FsChangeToProgramDir();

			std::fstream tip_file;
			tip_file.open(file, std::ios::in);
			if(tip_file.is_open())
			{
				std::string line;
				
				// ignoring the first line
				std::getline(tip_file, line);
				
				// getting the number of nodes in tip 
				std::getline(tip_file, line);
				num_tip_nodes= std::stoi(line);
				std::cout << "Number of nodes in tip: " << num_tip_nodes << std::endl;

				// Getting the number of configurations. Assumes that 180 degrees is covered.
				std::getline(tip_file, line);
				num_tip_configs= std::stoi(line);
				std::cout << "Number of nodes in tip: " << num_tip_configs << std::endl;

				for(int i = 0; i < num_tip_configs; i++)
				{

					// getting the angle
					std::getline(tip_file, line);
					int temp_angle = std::stoi(line);

					// getting the distances
					std::getline(tip_file, line);
					std::vector<double> temp_distances = str_to_vec(line);
					
					// getting the angles
					std::getline(tip_file, line);
					std::vector<double> temp_angles= str_to_vec(line);
					
					// ignoring the blank line
					std::getline(tip_file, line);

					tip_config temp_config = {temp_angle, temp_distances, temp_angles};
					tip_configs.push_back(temp_config);
				}

				tip_file.close();

				// std::cout << "Imported tip  configs: " << std::endl;
				// for(int i = 0; i < num_tip_configs; i++)
				// {
				// 	print_config(tip_configs[i]);
				// }
			}
			else
			{
				std::cout << "ERROR: Failed to open " << file << std::endl;
			}

		}

		std::vector<double> str_to_vec(std::string input)
		{
			std::vector<double> vect;

			std::stringstream ss(input);

			float i;

			while (ss >> i)
			{
				vect.push_back(i);

				if (ss.peek() == ',')
				ss.ignore();
			}
			return vect;
		}

		void print_config(tip_config input)
		{
			std::cout << "Rotation Angle: " << input.rot_angle << std::endl;
			
			std::cout << "\t Node Distances: ";
			for(int i = 0; i < input.joint_distances.size(); i++)
			{
				std::cout << input.joint_distances[i] << ", ";
			}
			std::cout << ""<< std::endl;

			std::cout << "\t joint angles: ";
			for(int i = 0; i < input.joint_angles.size(); i++)
			{
				std::cout << input.joint_angles[i] << ", ";
			}
			std::cout << ""<< std::endl;
		}


		// checks collision between nodes 
		bool seg_collision(node* nd1, node* nd2)
		{
			// could be split by an obstacle.
			if(1 == det.check_collision(nd1) && 1 == det.check_collision(nd2))
			{
				double split_penetration_dist = det.get_split_penetration_dist(nd1,nd2);
				double penetration_dist1 = det.get_penetration_dist(nd1);
				double penetration_dist2 = det.get_penetration_dist(nd2);
				if(split_penetration_dist>penetration_dist1 || split_penetration_dist>penetration_dist2)
				{
					// std::cout << "split penetration distance " << split_penetration_dist << std::endl;
					// std::cout << "penetration distance 1" << penetration_dist1 << std::endl;
					// std::cout << "penetration distance 2 " << penetration_dist2 << std::endl;
					return true;
				}

			}
			return false;
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


		

};


int main()
{
	// in pixels
	int window_width = 800;
	int window_height = 600;

	// assume that scale is at 1000 sim units = 1 "real" meter. 1 sim = 1 mm

	// catheter params
	double x_start = 10;
	double y_start = 40;
	double x_dir = 1;
	double y_dir = 0;
	double joint_dist = 3;
	int num_segs = 12;
	double node_rad = 0.445/2;
	double spring_const = 70;

	// collisoion detection params
	double cd_res = 0.25;

	// // add closed obstacles
	// std::vector<vector> obs_corners1;
	// obs_corners1.emplace_back(60,60); 
	// obs_corners1.emplace_back(70,60); 
	// obs_corners1.emplace_back(70,45); 


	// std::vector<vector> obs_corners2;
	// obs_corners2.emplace_back(20,40); 
	// obs_corners2.emplace_back(100,40); 
	// obs_corners2.emplace_back(100,20); 
	// obs_corners2.emplace_back(20,20); 

	// std::vector<vector> obs_corners3;
	// obs_corners3.emplace_back(80,40); 
	// obs_corners3.emplace_back(90,40); 
	// obs_corners3.emplace_back(90,50); 

	// std::vector<vector> obs_corners4;
	// obs_corners4.emplace_back(20,60); 
	// obs_corners4.emplace_back(100,60); 
	// obs_corners4.emplace_back(100,90); 
	// obs_corners4.emplace_back(20,90); 

	// closed_obstacle temp_obs = closed_obstacle(obs_corners1);
	// obs.push_back(temp_obs);
	// temp_obs = closed_obstacle(obs_corners2);
	// obs.push_back(temp_obs);
	// temp_obs = closed_obstacle(obs_corners3);
	// obs.push_back(temp_obs);
	// temp_obs = closed_obstacle(obs_corners4);
	// obs.push_back(temp_obs);
	// obs.push_back(temp_obs.transform(vector(20,20)));

	environment env = environment("environments/simple_split.txt");
	std::vector<closed_obstacle> obs;
	obs = env.get_obs();



	collision_detector cd(obs, window_width,window_height, cd_res, node_rad);

	// std::cout << "1 " << (long long int )(cd.dist_field.sdf) << std::endl;

	std::cout << "CD build complete. " << std::endl;

	catheter cath(x_start, y_start, x_dir, y_dir, joint_dist, num_segs, node_rad, spring_const, cd);
	std::cout << "Cath build complete. " << std::endl;

	std::cout << "opening window..." << std::endl;
	FsOpenWindow(16,16,window_width,window_height,1, "Catheter Simulation");
	int key;

	

	double mv_vel = 0.75;
	double dt = 0.1;

	bool show_sdf = false;
	bool show_obstacles = true;
    while(FSKEY_ESC!=(key=FsInkey()))
    {
		FsPollDevice();
		
		switch(key)
        {
		case FSKEY_UP:
			// formerly move_input
			cath.translate(0, mv_vel*dt);
			break;
		case FSKEY_DOWN:
			cath.translate(0,-mv_vel*dt);
			break;
        case FSKEY_LEFT:
			cath.translate(-mv_vel*dt, 0);
            break;
        case FSKEY_RIGHT:
			cath.translate( mv_vel*dt, 0);
            break;
		case FSKEY_A:
			cath.rotate_tip( catheter::CCW);
            break;
		case FSKEY_D:
			cath.rotate_tip( catheter::CW);
            break;
		case FSKEY_Q:
			// show SDF
			show_sdf = !show_sdf;
			std::cout << "sdf: " << show_sdf << "\t obstacles: " << show_obstacles << std::endl;
			break;
		case FSKEY_E:
			// show obstacles
			show_obstacles = !show_obstacles;
			std::cout << "sdf: " << show_sdf << "\t obstacles: " << show_obstacles << std::endl;
			break;
        }

		

		cath.update(dt);
		
		
		glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);

		if(show_sdf)
		{
			cd.render_sdf();
		}
		if(show_obstacles)
		{
			// draw obstacles
			for (auto iter = obs.begin(); iter != obs.end(); iter++)
			{
				(*iter).draw();
			}
		}

		

		cath.draw();

        FsSwapBuffers();
        FsSleep(25);
	}

	std::cout <<"Escape key pressed!" << std::endl;
	return 0;
}