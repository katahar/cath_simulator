#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cmath>
#include <vector>
#include <string>

#include <iostream>
#include "fssimplewindow.h"
// #include "Eigen/Core"
// #include <Eigen3/Dense>


class node;
class joint;

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

		//returns a vector where input component (eg. wall normal) is removed from this vector (eg velocity)
		vector remove_component(vector input_vec)
		{
			// normalize input
			vector input_norm = input_vec.normalize();

		    // Compute the projection of velocity onto the wall normal
			vector projection = input_norm*(this->dot(input_norm));

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

		// moved to parent class
		// const int X = 0;
		// const int Y = 1;

		// joint* reg_joint;
		// double pos[2];
		// double vel[2] = {0,0};
		// double acc[2] = {0,0};

		vector pos;
		vector vel = vector(0,0);
		vector acc = vector(0,0);

		double mass;
		bool fixed;

		double x, y, radius;
		std::vector<joint*> connected_joints;

		node(double x, double y, double radius)
		{
			pos = vector(x,y);
			// pos[0] = x;
			// pos[1] = y;
			this->radius = radius;
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

		double get_rad()
		{
			return this->radius;
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

		double dist(node* other)
		{
			return (other->pos-this->pos).get_length();
		}

		void reset()
		{
			vel = vector(0,0);
			acc = vector(0,0);
		}

		// add_constraint : directly modifies acceleration based on input constraint
		void add_constraint(vector constraint)
		{
			acc.remove_component(constraint);
			vel.remove_component(constraint);
		}

		std::string to_string()
		{
			// std::string str =  "pos: (" + std::to_string(this->get_pos(this->X)) + ", " + std::to_string(this->get_pos(this->Y)) + ") \tvel: (" + std::to_string(vel.at(this->X)) + ", " + std::to_string(vel.at(this->Y))  + ") \tacc: (" + std::to_string(acc.at(this->X)) + ", " + std::to_string(acc.at(this->Y)) +")" ;
			std:: string str = "pos: " + pos.to_string() + "\t vel: " + vel.to_string() + "\t acc: " + acc.to_string();
			return str;
		}

};

class joint:public render_entity
{
	public:
		// moved to parent class
		// const int X = 0;
		// const int Y = 1;

		node* node1;
		node* node2;
		double neutral_angle = 0;
		double current_angle;
		double spring_constant = 5; 
		double joint_distance = 0; //distance between two bodies. Body to joint "center" is joint_distance/2
		double dist_tol = 0.05; //meters

		joint(node* node1, node* node2, double neutral_angle, double spring_const)
		{
			this->node1 = node1;
			this->node2 = node2;
			this->neutral_angle = neutral_angle;
			this->spring_constant = spring_const;
			this->joint_distance = hypot(node1->get_pos(node1->X)- node2->get_pos(node2->X), node1->get_pos(node1->Y)- node2->get_pos(node2->Y));
		}

		double get_pos(int index)
		{
			if(index < 2)
			{
				return (node1->get_pos(index) + node2->get_pos(index))/2;
			}
			else
			{
				std::cout << "ERROR: Joint indexing out of bounds for : " << index << std::endl;
				return -1;
			}

		}

		vector get_pos()
		{
			return (node1->get_pos()+node2->get_pos())/2;
		}

		// @TODO: Add angle update function. Maybe at the same time as force prop?
		std::string to_string()
		{
			std::string str =  "(" + std::to_string(this->get_pos(this->X)) + ", " + std::to_string(this->get_pos(this->Y)) + ") \tneutral: " + std::to_string(neutral_angle) + ", angle:" +std::to_string(current_angle) ;
		}

		void draw()
		{
			int sxn1, syn1, sxn2, syn2, sxj, syj;
		    glColor3ub(0,0, 255);	

			render_entity::PhysicalCoordToScreenCoord( sxj,  syj,  this->get_pos(render_entity::X),  this->get_pos(render_entity::Y));
			render_entity::PhysicalCoordToScreenCoord(sxn1, syn1, node1->get_pos(render_entity::X), node1->get_pos(render_entity::Y));
			render_entity::PhysicalCoordToScreenCoord(sxn2, syn2, node2->get_pos(render_entity::X), node2->get_pos(render_entity::Y));
			render_entity::DrawLine(sxj, syj, sxn1, syn1);
			render_entity::DrawLine(sxj, syj, sxn2, syn2);
		}

		double joint_dist_sq(node* node)
		{
			double joint_x = this->get_pos(render_entity::X);
			double joint_y = this->get_pos(render_entity::Y);
			return ((node->get_pos(render_entity::X)-joint_x)*(node->get_pos(render_entity::X)-joint_x)) + ((node->get_pos(render_entity::Y)-joint_y)*(node->get_pos(render_entity::Y)-joint_y));
		}

		double joint_dist(node* node)
		{
			return sqrt(this->joint_dist_sq(node));
		}
		
		node* get_other(node* input)
		{
			if(input==node1 || input == node2)
			{
				if(input == node1)
				{
					return node2;
				}
				else
				{
					return node1;
				}
			}
			std::cout << "ERROR: node not connected to this joint." << std::endl;
			return nullptr;
		}

		double get_angle()
		{
			vector joint_pos = this->get_pos();
			double numer = node1->get_pos().dot(node2->get_pos());
			double denom = node1->get_pos().get_length() * node2->get_pos().get_length();
			current_angle = acos(numer/denom);
			return current_angle;
		}

		// should attempt to resolve within one timestep. input is the node that was most recently moved.
		void enforce_dist_constraint(node* last_node, double dt)
		{
			//target distance between two nodes in a joint from law of cosines.
			node* move_node = this->get_other(last_node);
			double cur_angle = this->get_angle();
			double half_joint_dist = (joint_distance/2);
			double goal_len = sqrt( 2*(half_joint_dist*half_joint_dist)  + (2*half_joint_dist*half_joint_dist*cos(cur_angle)));
			double cur_len  = sqrt(joint_dist_sq(node1) + joint_dist_sq(node2) + (2*joint_dist(node1)*joint_dist(node2)*cos(cur_angle)));

			if(abs(cur_len-goal_len) > dist_tol)
			{
				std::cout << "enforcing distance!" << cur_len-goal_len;
				vector norm_dist = (move_node->get_pos()-last_node->get_pos()).normalize();
				vector target_motion = norm_dist*(goal_len-cur_len);
				std::cout << "\ttarget motion " << target_motion.to_string();
				std::cout << "\tapplied acc " << (target_motion/(dt*dt)).to_string() << std::endl;
				// assuming that target_dist/dt^2 = acceleration
				move_node->add_accel(target_motion/(dt*dt));
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

};

class catheter
{
	public:
		std::vector<joint*> joints;
		std::vector<node*> nodes;
		int num_nodes, num_joints; //num_nodes = num_joints+1

		node* base_node;

		void build_cath(double x_origin, double y_origin, double x_dir, double y_dir, double joint_distance, int num_segments, double radius)
		{
			double unit_x = x_dir/hypot(x_dir, y_dir);
			double unit_y = y_dir/hypot(x_dir, y_dir);

			double step_x = unit_x*joint_distance;
			double step_y = unit_y*joint_distance;

			num_nodes = num_segments+1;
			num_joints = num_segments;

			//setting nodes
			for(int i = 0; i < num_nodes; i++)
			{
				node* temp_node = new node(x_origin + (step_x*i), y_origin + (step_y*i), radius);
				if(0 == i)
				{
					base_node = temp_node;
					base_node->fix_node();
				}
				nodes.push_back(temp_node);
			}


			//reading nodes
			int count = 0;
			for(auto iter = nodes.begin(); iter != nodes.end(); iter++)
			{
				std::cout << "node " << count << ": (" << (*iter)->get_pos((*iter)->X) << ", " << (*iter)->get_pos((*iter)->Y) << ")" << std::endl;
				count++;
			}

			//setting joints
			for(int i = 0; i < num_nodes-1 ; i++)
			{
				joint* temp_joint = new joint(nodes[i], nodes[i+1], 0, 5) ;
				joints.push_back(temp_joint);
			}

			// reading joints
			count = 0;
			for(auto iter = joints.begin(); iter !=joints.end(); iter++)
			{
				std::cout << "joint " << count << ": (" << (*iter)->get_pos((*iter)->X) << ", " << (*iter)->get_pos((*iter)->Y) << ")" << std::endl;
				count++;
			}

			// std::cout << "cath build complete " << std::endl;

		}


		void move_input(double x, double y)
		{
			base_node->move_rel_pos(x,y);
		}

		void update(double dt)
		{
			// need to update by node and joint simultaneously?
			// for(int i = 1; i < num_nodes; i++)
			// {
			// 	//causes indexing error
			// 	nodes[i]->move(dt);
			// }

			for(int i = 0; i < num_joints; i ++)
			{
				nodes[i]->reset();
				joints[i]->enforce_dist_constraint(nodes[i],dt);
				std::cout << "node " + std::to_string(i) + ": " + nodes[i+1]->to_string() << std::endl;
				nodes[i+1]->move(dt);
			}
			std::cout << "---------------------------------" << std::endl;

		}

		void draw()
		{

			//drawing joints
			for(int i = 0; i < num_joints; i++)
			{
				// std::cout << "Drawing joint " << i << std::endl;
				joints[i]->draw();
			}
			
			// std::cout << "All joints drawn" << std::endl;

			for(int i = 0; i < num_nodes; i++)
			{
				// std::cout << "Drawing node " << i << std::endl;
				nodes[i]->draw();
			}

			// std::cout << "All nodes  drawn" << std::endl;

		}

};


int main()
{
	// std::cout <<" hello" << std::endl;
	// vector test_vec = vector(1,1);
	// std::cout << "division: " << (test_vec/2).to_string() << std::endl;
	// std::cout << "multiplication: " << (test_vec*2).to_string() << std::endl;
	// std::cout << "addition: " << (test_vec+test_vec).to_string() << std::endl;
	// std::cout << "subtraction: " << (test_vec-test_vec).to_string() << std::endl;

	catheter cath; 
	cath.build_cath(0,0,1,1,1.5,3,0.25);



	std::cout << "attempting to open window..." ;
	FsOpenWindow(16,16,800,600,1);
	std::cout << "complete" << std::endl;


	int key;

	line_obstacle line = line_obstacle(5,7,10,5);
	double norm_vec[2];
	line.get_normal(norm_vec);
	std::cout << "the normal of (" << line.pos[0] << ", " << line.pos[1] << "), (" << line.pos[2] << ", " << line.pos[3] << ")";
	std::cout << "is (" << norm_vec[0] << ", " << norm_vec[1] << ")" << std::endl;


	double mv_vel = 0.5;
	double dt = 0.05;
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
        }

		cath.update(dt);

		glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
		cath.draw();
		line.draw();
        FsSwapBuffers();
        FsSleep(25);
	}

	return 0;
}