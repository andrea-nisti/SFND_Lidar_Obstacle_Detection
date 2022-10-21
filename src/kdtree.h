#ifndef KD_TREE_CLUSTER
#define KD_TREE_CLUSTER

#include "render/render.h"
#include <cmath>

namespace euclidean_clustering
{
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

		static Node *makeNode(std::vector<float> point, int id)
		{
			auto node{new Node{point, id}};
			return node;
		}
	};

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
				auto index{depth % target->point.size()};
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
				auto diff_vector{std::vector<float>(target.size(), 0)};
				size_t idx{0};
				for (auto &component : diff_vector)
				{
					component = fabs(node->point.at(idx) - target.at(idx));
					++idx;
				}

				auto is_in_box{std::all_of(diff_vector.cbegin(), diff_vector.cend(), [distanceTol](float comp)
										   { return comp <= distanceTol; })};

				if (is_in_box)
				{
					// compute distance : e.g. sqrt(x*x + y*y + z*z)
					float distance{sqrtf(std::accumulate(diff_vector.begin(), diff_vector.end(), 0, [](float accumulator, float comp)
												  { return accumulator + comp * comp; }))};
					if (distance <= distanceTol)
					{
						ids.push_back((node->id));
					}
				}

				auto index{depth % target.size()};

				if ((target.at(index) - distanceTol) < node->point.at(index))
				{
					searchHelper(target, node->left, distanceTol, ids, ++depth);
				}

				if ((target.at(index) + distanceTol) > node->point.at(index))
				{
					searchHelper(target, node->right, distanceTol, ids, ++depth);
				}
			}
		}

		void insert(std::vector<float> point, int id)
		{
			auto new_node{Node::makeNode(point, id)};
			auto target{root};
			if (root == nullptr)
			{
				root = new_node;
				return;
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

} // namespace

#endif
