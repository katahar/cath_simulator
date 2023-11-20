#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cmath>
#include <vector>

#include <iostream>
#include "fssimplewindow.h"

class node;
class joint;

class node
{
	public:

		const int X = 0;
		const int Y = 1;

		joint* reg_joint;
		double pos[2];
		double vel[2];
		double acc[2];

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

		// @TODO: String
};

class joint
{
	public:
		const int X = 0;
		const int Y = 1;

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

		// double[] get_joint_ctr()
		// {
		// 	double joint_ctr[2];
		// 	joint_ctr[this->X] = (node1->get_dim(node1->X) + node2->get_dim(node2->X))/2;
		// 	joint_ctr[this->Y] = (node1->get_dim(node1->Y) + node2->get_dim(node2->Y))/2;
		// 	return joint_ctr;
		// }

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

};

class catheter
{
	public:
		std::vector<joint*> joints;
		std::vector<node*> nodes;
		int num_nodes;

	
		void build_cath(double x_origin, double y_origin, double x_dir, double y_dir, double joint_distance, int num_segments, double radius)
		{
			double unit_x = x_dir/hypot(x_dir, y_dir);
			double unit_y = y_dir/hypot(x_dir, y_dir);

			double step_x = unit_x*joint_distance;
			double step_y = unit_y*joint_distance;

			//setting nodes
			for(int i = 0; i < num_segments; i++)
			{
				node* temp_node = new node(x_origin + (step_x*i), y_origin + (step_y*i), radius);
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
			for(int i = 0; i < num_segments-1; i++)
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

		}

};


int main()
{
	std::cout <<" hello" << std::endl;
	catheter cath; 
	cath.build_cath(0,0,1,1,1.5,5,0.25);




	FsOpenWindow(16,16,800,600,1);


	int key;

    while(FSKEY_ESC!=(key=FsInkey()))
    {
		FsPollDevice();
		// FsSwapBuffers();

		glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
		// drawing here
        FsSwapBuffers();
        FsSleep(25);
	}

	return 0;
}