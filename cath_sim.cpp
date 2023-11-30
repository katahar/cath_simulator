#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>

#include <iostream>
#include "fssimplewindow.h"
// #include "Eigen/Core"
// #include <Eigen3/Dense>


#define PI 3.14159265
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

		// @TODO: update for 3d
		//finds the perpendicular of the current vector facing toward the target vector, normalized
		vector get_perpen_toward(vector target)
		{
			vector perpendicular = vector(-vec[1], vec[0]);
			if(target.dot(perpendicular) < 0) //obtuse angle, facing toward target
			{
				return perpendicular;
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

		// node* node1;
		// node* node2;
		double neutral_angle = PI; //angle between adjacent nodes
		double current_angle;
		double spring_constant = 20; 
		double joint_distance = 0; //distance between two bodies. Body to joint "center" is joint_distance/2
		double dist_tol = 0.05; //meters
		double angle_tol = 10*PI/180; //meters

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
		//attempts to move self and other node
		void enforce_dist_constraint(node* last_node, double dt)
		{

			
			// compare distance from last_node to self, and move self.
			double last_self_dist_sq = this->dist_sq(last_node);
			
			double target_dist_sq = joint_distance*joint_distance;
			
			// applied acceleration of current node. used to fix acceration of other node.
			vector accel_curr_node = vector(0,0);
			
			// distance from last_node and current node is outside of tolerance
			if(abs(last_self_dist_sq-target_dist_sq) > dist_tol)
			{
				
				// direction of motion away from last_node toward current node
				vector move_dir = (this->get_pos()-last_node->get_pos()).normalize();
				
				double move_dist = joint_distance-this->dist(last_node);
				
				vector target_motion = move_dir*(move_dist);
				
				accel_curr_node = target_motion/(dt*dt);
				
				this->add_accel(accel_curr_node);
				
			}

			// this is probably redundant, so removed for now.
			// if(!this->is_terminal())
			// {
			// 	node* other_node = this->get_other(last_node);
				
			// 	double self_other_dist_sq = this->dist_sq(other_node);
				
			// 	// distance from current node to other_node is outside of tolerance
			// 	if(abs(self_other_dist_sq-target_dist_sq) > dist_tol)
			// 	{
			// 		// direction of motion away from current node toward other_node
					
			// 		vector move_dir = (other_node->get_pos()-this->get_pos()).normalize();
					
			// 		double move_dist = this->dist(other_node)-joint_distance;
					
			// 		vector target_motion = move_dir*(move_dist);
					
			// 		vector accel_other = target_motion/(dt*dt);
					
			// 		this->add_accel(accel_curr_node+accel_other);
					
				
			// 	}
			// }
			

		}

		void apply_bending_force(node* last_node)
		{
			if(!this->is_terminal())
			{
				// std::cout << "\tcurrent angle: " << current_angle << ", target angle: " << neutral_angle << std::endl;
			
				// cmath uses radians.
				if(abs(current_angle-neutral_angle) > angle_tol)
				{

					node* move_node = this->get_other(last_node);
					std::cout << "\tenforcing angle. current difference is " << std::to_string(current_angle-neutral_angle);
					vector A = this->pos-last_node->get_pos();
					vector B = move_node->get_pos() -this->pos;
					vector force_dir = B.get_perpen_toward(A); //normalized direction of force
					std::cout << "\tforce direction " << force_dir.to_string();
					std::cout << "\tapplied acceleration " << (force_dir*(spring_constant*abs(current_angle-neutral_angle))).to_string() << std::endl;

					// force = k*theta
					move_node->add_accel(force_dir*(spring_constant*abs(current_angle-neutral_angle)));
					
					
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

class catheter : public render_entity
{
	public:
		std::vector<node*> nodes;
		int num_nodes;

		node* base_node;

		void build_cath(double x_origin, double y_origin, double x_dir, double y_dir, double joint_distance, int num_segments, double radius)
		{
			double unit_x = x_dir/hypot(x_dir, y_dir);
			double unit_y = y_dir/hypot(x_dir, y_dir);

			double step_x = unit_x*joint_distance;
			double step_y = unit_y*joint_distance;

			num_nodes = num_segments+1;

			//setting nodes
			for(int i = 0; i < num_nodes; i++)
			{
				node* temp_node = new node(x_origin + (step_x*i), y_origin + (step_y*i),radius, PI, 5, joint_distance);
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
			base_node->move_rel_pos(x,y);
		}

		void update(double dt)
		{
			
			nodes[0]->reset();
			for(int i = 1; i < num_nodes; i ++)
			{
				nodes[i]->reset();

				
				std::cout << "enforcing distance constraint for joint " <<  std::to_string(i) << "....";
				nodes[i]->enforce_dist_constraint(nodes[i-1],dt);
				std::cout << "done" << std::endl;
				// std::cout << "distance constraint enforced. " <<  std::to_string(i) << std::endl;
				
				// std::cout << "bending force for joint " <<  std::to_string(i) << "...." <<std::endl;
				// nodes[i]->apply_bending_force(nodes[i-1]);
				// std::cout << "\tdone" << std::endl;

				std::cout << "node " + std::to_string(i)<< std::endl;

				
				nodes[i]->move(dt);				
			}
			nodes[num_nodes-1]->reset();
			std::cout << "---------------------------------" << std::endl;

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


int main()
{
	// std::cout <<" hello" << std::endl;
	// vector test_vec = vector(1,1);
	// std::cout << "division: " << (test_vec/2).to_string() << std::endl;
	// std::cout << "multiplication: " << (test_vec*2).to_string() << std::endl;
	// std::cout << "addition: " << (test_vec+test_vec).to_string() << std::endl;
	// std::cout << "subtraction: " << (test_vec-test_vec).to_string() << std::endl;


		vector A = vector(0,1);
		vector B = vector(0.7,0.7);
		double numer = A.dot(B);
		double denom = A.get_length() * B.get_length();
		double ang_r = acos(numer/denom);
		std::cout << "Angle on right: " << std::to_string(A.dot(B));
		
		vector C = vector(0.7,-0.7);
		 numer = A.dot(C);
		 denom = A.get_length() * B.get_length();
		double ang_l = acos(numer/denom);
		std::cout << "Angle on Left" << std::to_string(A.dot(C)) <<std::endl;

	catheter cath; 
	cath.build_cath(1,1,0,1,1.5,3,0.25);



	std::cout << "opening window..." << std::endl;
	FsOpenWindow(16,16,800,600,1);


	int key;

	// line_obstacle line = line_obstacle(5,7,10,5);
	// double norm_vec[2];
	// line.get_normal(norm_vec);
	// std::cout << "the normal of (" << line.pos[0] << ", " << line.pos[1] << "), (" << line.pos[2] << ", " << line.pos[3] << ")";
	// std::cout << "is (" << norm_vec[0] << ", " << norm_vec[1] << ")" << std::endl;


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
        FsSwapBuffers();
        FsSleep(25);
	}

	return 0;
}