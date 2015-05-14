#include <iostream>
#include <random>
#include <cmath>
#include <chrono>
#include <RRT.h>
#include <Eigen/Dense>

#include "CollisionChecker.cpp"

using namespace Eigen;
using namespace std;

class TestState: public State {
public:
	Vector3f p;
	Quaternion<float> q;
};

class TestInput: public Input {
public:
	Vector3f t;
	Quaternion<float> r;
};

class TestProblem: public Problem<TestState, TestInput> {
public:
	const float MAX_TRANSLATION = 0.025;
	const float MAX_ROTATION = M_PI/180*1;

	const float MAX_TRANSLATION_SQR = MAX_TRANSLATION*MAX_TRANSLATION;

	CollisionChecker* collision_checker;


	TestProblem(TestState ninit, TestState ngoal, const char* mesh_file1, const char* mesh_file2): Problem(ninit, ngoal) {
		 collision_checker = new CollisionChecker(mesh_file1, mesh_file2);
	}

	~TestProblem() {
		delete collision_checker;
	}

	virtual TestState random_state() {
		static std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());

		TestState x;

		// Random position
		static std::uniform_real_distribution<double> p_distribution(-3.0,3.0);
		x.p[0] = p_distribution(generator);
		x.p[1] = p_distribution(generator);
		x.p[2] = p_distribution(generator);

		// Random quaternion
		static std::uniform_real_distribution<double> q_distribution(0.0,1.0);
		double rand[] = {q_distribution(generator), q_distribution(generator), q_distribution(generator)};
	    float r1 = sqrt(1.0 - rand[0]);
	    float r2 = sqrt(rand[0]);
	    float pi2 = M_PI * 2.0;
	    float t1 = pi2 * rand[1];
	    float t2 = pi2 * rand[2];
	    x.q = Quaternion<float>(sin(t2)*r2, cos(t2)*r2, sin(t1)*r1, cos(t1)*r1);
		
		return x;
	}

	bool advance(TestState &nx, TestInput &nu, const TestState &x1, const TestState &x2, bool reverse=false) {
		// Translation
		nu.t = x2.p - x1.p;
		if(nu.t.squaredNorm() > MAX_TRANSLATION_SQR) {
			nu.t.normalize();
			nu.t *= MAX_TRANSLATION;
		}
		nx.p = x1.p + nu.t;

		// Rotation
		float angle = 2*acos(x1.q.dot(x2.q));
		if(angle > M_PI)
			angle -= 2*M_PI;
		if(fabs(angle) < MAX_ROTATION) {
			nx.q = x2.q;
		} else {
			nx.q = x1.q.slerp(fabs(MAX_ROTATION/angle), x2.q);
			nx.q.normalize();
		}
		nu.r = nx.q * x1.q.inverse();

		if(reverse) {
			nu.r = nu.r.inverse();
			nu.t = -nu.t;
		}

		return !collides(nx);
	}

	virtual bool equal(const TestState &x1, const TestState &x2) {
		float dp = (x2.p-x1.p).squaredNorm();
		float dq = x1.q.dot(x2.q); dq = 1 - dq*dq;
		return dp < 0.001 and dq < 0.001;
	}

	virtual float metric(const TestState &x1, const TestState &x2) {
		float dp = (x2.p-x1.p).squaredNorm();
		float dq = x1.q.dot(x2.q); dq = 1 - dq*dq;
		return dp + dq;
	}

	bool collides(const TestState &x) {
    	collision_checker->setPosition2(x.p[0], x.p[1], x.p[2]);
        collision_checker->setRotation2(x.q.w(), x.q.x(), x.q.y(), x.q.z());
		return collision_checker->collides();
	}

	bool is_near(const TestState &x1, const TestState &x2) {
		if((x2.p-x1.p).squaredNorm() < MAX_TRANSLATION_SQR) {
			float angle = 2*acos(x1.q.dot(x2.q));
			if(angle > M_PI)
				angle -= 2*M_PI;
			return fabs(angle) < MAX_ROTATION;
		}
		return false;
	}
};

void test_solution(vector<TestState> &states, vector<TestInput> &inputs){
	for(int i = 1; i < states.size(); i++) {
		TestState &x1 = states[i-1];
		TestState &x2 = states[i];

		// Check translation
		if((x2.p-x1.p).squaredNorm() > 0.06*0.06) {
			cout << "States " << i-1 << " and " << i << " have large translation of " << (x2.p-x1.p).norm() << endl;
		}

		// Check rotation
		float angle = 2*acos(x2.q.dot(x1.q));
		angle *= 180/M_PI;
		if(angle > 180)
			angle -= 360;
		if(fabs(angle) > 6) {
			cout << "States " << i-1 << " and " << i << " have large rotation of " << angle << endl;
		}
	}
}

void write_solution(vector<TestState> &states, vector<TestInput> &inputs, const char* file){
	FILE* fw = fopen(file, "w");
	for(int i = 0; i < states.size(); i++) {
		TestState &s = states[i];
		fprintf(fw, "%e %e %e", s.p[0], s.p[1], s.p[2]);
		fprintf(fw, " %e %e %e %e", s.q.w(), s.q.x(), s.q.y(), s.q.z());
		fprintf(fw, "\n");
	}
	fclose(fw);
}

void write_vector(vector<double> &data, const char* file) {
	FILE* fw = fopen(file, "a");
	for(int i = 0; i < data.size(); i++) {
		fprintf(fw, "%E\n", data[i]);
	}
	fclose(fw);
}

int main() {
	TestState init, goal;

	init.p = Vector3f(0.0, 0.0, 0.0);
	init.q = Quaternion<float>(1.0, 0.0, 0.0, 0.0);

	goal.p = Vector3f(2.0, 0.0, -1.9);
	goal.q = Quaternion<float>(sqrt(2)/2, 0, sqrt(2)/2, 0);

	TestProblem problem(init, goal, "hole.obj", "pipe.obj");

	// Run many iterations to get some statistics
	/*vector<double> iterations , tree1_size, tree2_size, times;
	for(int i = 0; i < 200; i++) {
		auto start = chrono::steady_clock::now();
		BIRRT<TestState, TestInput, TestProblem> solver(&problem);
		pair<Node<TestState, TestInput>*, Node<TestState, TestInput>*> solution_nodes = solver.run(100000);
		auto end = chrono::steady_clock::now();

		iterations.push_back(solver.iterations_completed);
		tree1_size.push_back(solver.t_init.nodes.size());
		tree2_size.push_back(solver.t_goal.nodes.size());

		auto diff = end - start;
		times.push_back(chrono::duration <double, milli> (diff).count());
	}
	write_vector(iterations, "iterations.txt");
	write_vector(tree1_size, "tree1.txt");
	write_vector(tree2_size, "tree2.txt");
	write_vector(times, "times.txt");*/

	
	BIRRT<TestState, TestInput, TestProblem> solver(&problem);

	pair<Node<TestState, TestInput>*, Node<TestState, TestInput>*> solution_nodes = solver.run(100000);

	if (solution_nodes.first == NULL or solution_nodes.second == NULL) {
		exit(0);
	}

	vector<TestState> states;
	vector<TestInput> inputs;
	solver.get_sulution(states, inputs, solution_nodes.first, solution_nodes.second);

	test_solution(states, inputs);

	write_solution(states, inputs, "solution.txt");
}