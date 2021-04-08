// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// Insert a new point into the tree
		// create a new node and place correctly with in the root 
		insertHelper(&root,0,point,id);
	}

	void insertHelper (Node** node, uint depth, std::vector<float> point, int id)
	{
		// Tree is empty at this point
		if(*node==NULL)
		{
			*node = new Node(point,id);
		}
		else
		{
			// Calculate current dim
			uint cd = depth % 2;

			// if depth is even, look at x value (cd=0), else look at y value (cd=1)
			// if point dim is less than the current nodes dim, then branch left
			if(point[cd] < ((*node)->point[cd])) //
			{
				insertHelper(&((*node)->left), depth+1, point, id);
			}
			// if point dim is greater or equal than the current nodes dim, then branch left
			else
			{
				insertHelper(&((*node)->right), depth+1, point, id);
			}
			
		}
	}



	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};






