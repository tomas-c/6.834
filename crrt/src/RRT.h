#include <cstddef>
#include <cfloat>
#include <vector>
#include <algorithm>
#include <iostream>

class State {

};

class Input {

};

template <class StateClass, class InputClass>
class Problem {
public:
	StateClass init, goal;

	Problem(StateClass ninit, StateClass ngoal) {
		init = ninit;
		goal = ngoal;
	}

	virtual StateClass random_state() = 0;
	virtual bool advance(StateClass &nx, InputClass &nu, const StateClass &x1, const StateClass &x2, bool reverse=false) = 0;
	virtual bool equal(const StateClass &x1, const StateClass &x2) = 0;
	virtual float metric(const StateClass &x1, const StateClass &x2) = 0;
};

template <class StateClass, class InputClass>
class Node {
public:
	typedef Node<StateClass, InputClass> NodeClass;

public:
	StateClass x;
	InputClass u;

	Node *parent;

	Node(StateClass nx, InputClass nu, NodeClass *nparent) {
		x = nx; u = nu; parent = nparent;
	}
};

template <class StateClass, class InputClass>
class Tree {
public:
	typedef Node<StateClass, InputClass> NodeClass;
	typedef typename std::vector<NodeClass*>::iterator iterator;
	typedef typename std::vector<NodeClass*>::const_iterator const_iterator;

public:
	NodeClass* root;
	std::vector<NodeClass*> nodes;

	Tree() {
		root = NULL;
	}

	~Tree() {
		for(iterator it = nodes.begin(); it != nodes.end(); it++) {
			delete *it;
		}
	}

	void add_root(StateClass x, InputClass u) {
		root = new NodeClass(x, u, NULL);

		nodes.clear();
		nodes.push_back(root);
	}

	NodeClass* add_child(NodeClass *parent, const StateClass &x, const InputClass &u) {
		NodeClass *child = new NodeClass(x, u, parent);
		nodes.push_back(child);
		return child;
	}
};

template <class StateClass, class InputClass, class ProblemClass>
class RRTBase {
public:
	typedef Node<StateClass, InputClass> NodeClass;
	typedef Tree<StateClass, InputClass> TreeClass;

public:
	ProblemClass *p;

	RRTBase(ProblemClass *problem) {
		p = problem;
	}

protected:
	NodeClass* extend(TreeClass *tree, StateClass x, bool reverse=false) {
		NodeClass* nearest_node = nearest_neighbor(tree, x);
		if(nearest_node != NULL) {
			StateClass nx; InputClass nu;
			bool success = p->advance(nx, nu, nearest_node->x, x, reverse);
			if(success) {
				return tree->add_child(nearest_node, nx, nu);
			}
		}
		return NULL;
	}

	NodeClass* nearest_neighbor(const TreeClass *t, const StateClass &x) {
		float min_dist = FLT_MAX;
		NodeClass *min_node = NULL;
		for(typename TreeClass::const_iterator it = t->nodes.begin(); it != t->nodes.end(); it++) {
			NodeClass *node = *it;

			float dist = p->metric(x, node->x);
			if(dist < min_dist) {
				min_dist = dist;
				min_node = node;
			}
		}

		return min_node;
	}


};

template <class StateClass, class InputClass, class ProblemClass>
class BIRRT: public RRTBase<StateClass, InputClass, ProblemClass> {
	using RRTBase<StateClass, InputClass, ProblemClass>::p;
	using RRTBase<StateClass, InputClass, ProblemClass>::extend;

public:
	typedef Node<StateClass, InputClass> NodeClass;
	typedef Tree<StateClass, InputClass> TreeClass;

public:
	TreeClass t_init, t_goal;

	BIRRT(ProblemClass *problem): RRTBase<StateClass, InputClass, ProblemClass>(problem) {
		p = problem;
	}

	std::pair<NodeClass*, NodeClass*> run(int max_interations) {
		t_init = TreeClass(); t_init.add_root(p->init, InputClass());
		t_goal = TreeClass(); t_goal.add_root(p->goal, InputClass());

		bool reverse = false;
		TreeClass* t1 = &t_init; TreeClass* t2 = &t_goal;

		int iteration = 0;
		while(iteration < max_interations) {
			iteration++;

			if(iteration % 100 == 0) {
				std::cout << "Iteration = \t" << iteration;
				std::cout << "\tt_init size = \t" << t_init.nodes.size();
				std::cout << "\tt_goal size = \t" << t_goal.nodes.size();
				std::cout << std::endl;
			}

			StateClass x_rand = p->random_state();

			// Extend Tree 1 towards the random state
			NodeClass* new1 = extend(t1, x_rand, reverse);
			if(new1 != NULL) {
				// Extend Tree 2 towards the new state
				NodeClass* new2 = extend(t2, new1->x, !reverse);
				// Check if goal was reached
				if(new2 != NULL and p->equal(new1->x, new2->x)) {
					std::cout << "Solution found in " << iteration << " iterations" << std::endl;
					if(reverse)
						return std::make_pair(new2, new1);
					else
						return std::make_pair(new1, new2);
				}
			}

			// swap trees
			std::swap(t1, t2);
			reverse = !reverse;
		}

		std::cout << "No solution found" << std::endl;
		return std::pair<NodeClass*, NodeClass*>(NULL, NULL);
	}

	void get_sulution(std::vector<StateClass> &states, std::vector<InputClass> &inputs, NodeClass *n_init, NodeClass *n_goal) {
		NodeClass *n;

		n = n_init;
		while(n != NULL) {
			states.push_back(n->x); inputs.push_back(n->u);
			n = n->parent;
		}
		std::reverse(states.begin(),states.end());
		std::reverse(inputs.begin(),inputs.end());

		n = n_goal;
		while(n != NULL) {
			states.push_back(n->x); inputs.push_back(n->u);
			n = n->parent;
		}
	}
};