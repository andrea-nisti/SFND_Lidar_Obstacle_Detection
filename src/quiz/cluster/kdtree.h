/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cmath>

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
	return true;
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

	// Helpers
	static void insertHelper(Node *&target, Node *const new_node, int depth = 0)
	{
		if (target == nullptr)
		{
			target = new_node;
		}
		else
		{
			auto dim{target->point.size()};
			auto index{depth % dim};
			if (new_node->point.at(index) < target->point.at(index))
			{
				insertHelper(target->left, new_node, ++depth);
			}
			else
			{
				insertHelper(target->right, new_node, ++depth);
			}
		}
	}

	static void searchHelper(std::vector<float> target, Node const *const node, float distanceTol, std::vector<int> &ids, int depth = 0)
	{
		if (node != nullptr)
		{
			auto di{std::vector<float>(target.size(), 0)};
			size_t idx{0};
			for (auto &component : di)
			{
				component = fabs(node->point.at(idx) - target.at(idx));
				++idx;
			}

			auto is_in_box{std::all_of(di.cbegin(), di.cend(), [distanceTol](float comp)
									   { return comp <= distanceTol; })};

			if (is_in_box)
			{
				// compute distance : e.g. sqrt(x*x + y*y + z*z)
				float d{sqrtf(std::accumulate(di.begin(), di.end(), 0, [](float accumulator, float comp)
											  { return accumulator + comp * comp; }))};
				if (d <= distanceTol)
				{
					ids.push_back((node->id));
				}
			}

			if ((target.at(depth % target.size()) - distanceTol) < node->point.at(depth % target.size()))
			{
				searchHelper(target, node->left, distanceTol, ids, ++depth);
			}

			if ((target.at(depth % target.size()) + distanceTol) > node->point.at(depth % target.size()))
			{
				searchHelper(target, node->right, distanceTol, ids, ++depth);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		auto new_node{makeNode(point, id)};
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
		searchHelper(target, root, distanceTol, ids);
		return ids;
	}
};
