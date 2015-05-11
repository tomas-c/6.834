#include <iostream>
#include <random>
#include <RRT.h>
#include <Eigen/Dense>

#include <mgl2/mgl.h>

using namespace Eigen;
using namespace std;

class TestState: public State, public Vector2f {
	using Vector2f::Vector2f;
public:
	
};

class TestInput: public Input, public Vector2f {
	using Vector2f::Vector2f;
public:
	
};

class TestProblem: public Problem<TestState, TestInput> {
	const float STEP = 0.05;
	const float STEP_SQR = STEP*STEP;

public:
	TestProblem(TestState ninit, TestState ngoal): Problem(ninit, ngoal) {

	}

	virtual TestState random_state() {
		static std::default_random_engine generator;
  		static std::uniform_real_distribution<double> distribution(-1.0,1.0);
		return TestState((float)distribution(generator), (float)distribution(generator));
	}

	bool advance(TestState &nx, TestInput &nu, const TestState &x1, const TestState &x2, bool reverse=false) {
		nu = x2-x1;
		if(nu.squaredNorm() > STEP_SQR) {
			nu = nu.normalized()*STEP;
		}
		nx = x1+nu;

		// Simulate obstacle
		if((nx - Vector2f(0.5,0.5)).squaredNorm() < 0.2*0.2) {
			return false;
		}

		return true;
	}

	virtual bool equal(const TestState &x1, const TestState &x2) {
		return (x2-x1).squaredNorm() < 0.0000001;
	}

	virtual float metric(const TestState &x1, const TestState &x2) {
		return (x2-x1).squaredNorm();
	}
};

void visualize_tree(mglGraph &gr, Tree<TestState, TestInput> &tree, const char color) {
	string line_style = string("B");
	line_style.append(1, color);

	for(int i = 0; i < tree.nodes.size(); i++) {
		Node<TestState, TestInput>* n = tree.nodes[i];
		mglPoint p1(n->x[0], n->x[1]);
		if(n->parent != NULL) {
			mglPoint p2(n->parent->x[0], n->parent->x[1]);
			gr.Line(p1, p2, line_style.c_str());
			gr.Ball(p2, color);
		}
		gr.Ball(p1, color);
	}
	gr.Ball(mglPoint(1, 1), color);
}

int main() {
	TestProblem problem(TestState(0., 0.), TestState(1., 1.));
	BIRRT<TestState, TestInput, TestProblem> solver(&problem);

	solver.run(1000);

	mglGraph gr(0, 1000, 1000);
	gr.SetRange('x', -1., 1.);
	gr.SetRange('y', -1., 1.);
  	visualize_tree(gr, solver.t_init, 'r');
  	visualize_tree(gr, solver.t_goal, 'b');
  	gr.WriteFrame("test.png");
}