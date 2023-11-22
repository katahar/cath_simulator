#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cmath>
#include <vector>

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

		joint* reg_joint;
		double pos[2];
		double vel[2];
		double acc[2];
		double mass;

		double x, y, radius;
		std::vector<joint*> connected_joints;

		node(double x, double y, double radius)
		{
			pos[0] = x;
			pos[1] = y;
			this->radius = radius;
		}

		double get_pos(int index)
		{
			// @TODO: Add out of bounds handling
			return pos[index];
		}

		double get_vel(int index)
		{
			// @TODO: Add out of bounds handling
			return vel[index];
		}

		double get_acc(int index)
		{
			// @TODO: Add out of bounds handling
			return acc[index];
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
			render_entity::PhysicalCoordToScreenCoord(sx,sy,pos[render_entity::X], pos[render_entity::Y]);
			render_entity::DrawCircle(sx,sy,sRad,true);
		}
		// @TODO: String
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
		double curent_angle;
		double spring_constant = 5; 
		double joint_distance = 0; //distance between two bodies. Body to joint "center" is joint_distance/2

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
			}

		}

		void draw()
		{
			int sxn1, syn1, sxn2, syn2, sxj, syj;
		    glColor3ub(0,0, 255);	

			render_entity::PhysicalCoordToScreenCoord(sxj, syj, get_pos(render_entity::X), get_pos(render_entity::Y));
			render_entity::PhysicalCoordToScreenCoord(sxn1, syn1, node1->get_pos(render_entity::X), node1->get_pos(render_entity::Y));
			render_entity::PhysicalCoordToScreenCoord(sxn2, syn2, node2->get_pos(render_entity::X), node2->get_pos(render_entity::Y));
			render_entity::DrawLine(sxj, syj, sxn1, syn1);
			render_entity::DrawLine(sxj, syj, sxn2, syn2);
		}

		// void enforce_constraint()

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


};

class catheter
{
	public:
		std::vector<joint*> joints;
		std::vector<node*> nodes;
		int num_nodes, num_joints; //num_nodes = num_joints+1

		

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
				nodes.push_back(temp_node);
			}


			//reading nodes
			// int count = 0;
			// for(auto iter = nodes.begin(); iter != nodes.end(); iter++)
			// {
			// 	std::cout << "node " << count << ": (" << (*iter)->get_pos((*iter)->X) << ", " << (*iter)->get_pos((*iter)->Y) << ")" << std::endl;
			// 	count++;
			// }

			//setting joints
			for(int i = 0; i < num_nodes-1 ; i++)
			{
				joint* temp_joint = new joint(nodes[i], nodes[i+1], 0, 5) ;
				joints.push_back(temp_joint);
			}

			// reading joints
			// count = 0;
			// for(auto iter = joints.begin(); iter !=joints.end(); iter++)
			// {
			// 	std::cout << "joint " << count << ": (" << (*iter)->get_pos((*iter)->X) << ", " << (*iter)->get_pos((*iter)->Y) << ")" << std::endl;
			// 	count++;
			// }

			// std::cout << "cath build complete " << std::endl;

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
	std::cout <<" hello" << std::endl;
	catheter cath; 
	cath.build_cath(0,0,1,1,1.5,5,0.25);



	std::cout << "attempting to open window..." ;
	FsOpenWindow(16,16,800,600,1);
	std::cout << "complete" << std::endl;


	int key;

	line_obstacle line = line_obstacle(5,5,10,10);
	double norm_vec[2];
	line.get_normal(norm_vec);
	std::cout << "the normal of (" << line.pos[0] << ", " << line.pos[1] << "), (" << line.pos[2] << ", " << line.pos[3] << ")";
	std::cout << "is (" << norm_vec[0] << ", " << norm_vec[1] << ")" << std::endl;

    while(FSKEY_ESC!=(key=FsInkey()))
    {
		FsPollDevice();
		

		glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
		cath.draw();
        FsSwapBuffers();
        FsSleep(25);
	}

	return 0;
}