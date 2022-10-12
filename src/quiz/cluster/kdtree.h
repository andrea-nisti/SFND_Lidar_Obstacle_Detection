/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}

	~Node()
	{
		delete left;
		delete right;
	}
};

Node *makeNode(std::vector<float> point, int id)
{
	auto node{new Node{point, id}};
	return node;
}

enum class NodeSide
{
	LEFT,
	RIGHT
};

bool setChild(Node *node, Node *const child, NodeSide side)
{
	if (node == nullptr or child == nullptr)
	{
		return false;
	}

	switch (side)
	{
	case NodeSide::LEFT:
		if (node->left != nullptr)
		{
			return false;
		}
		node->left = child;
		break;

	case NodeSide::RIGHT:
		if (node->right != nullptr)
		{
			return false;
		}
		node->right = child;
		break;

	default:
		return false;
		break;
	}
}

void insertHelper(Node *&target, Node *const new_node, int depth = 0)
{
	if (target == nullptr)
	{
		target = new_node;
	}
	else
	{
		auto dim{target->point.size()};
		auto index{depth % dim};
		if ( new_node->point.at(index) < target->point.at(index))
		{
			target = target->left;
			insertHelper(target, new_node, ++depth);
		}
		else
		{
			target = target->right;
			insertHelper(target, new_node, ++depth);
		}
	}
}
struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	~KdTree()
	{
		delete root;
	}

	void insert(std::vector<float> point, int id)
	{
		auto new_node{makeNode(point, id)};\
		auto target{root};
		if (root == nullptr)
		{
			root = new_node;
		}

		insertHelper(target, new_node);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
};
