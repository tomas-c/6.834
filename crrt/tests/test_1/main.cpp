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
	const float MAX_TRANSLATION = 0.05;
	const float MAX_ROTATION = M_PI/180*5;

	const float MAX_TRANSLATION_SQR = MAX_TRANSLATION*MAX_TRANSLATION;

	CollisionChecker* collision_checker;


	TestProblem(TestState ninit, TestState ngoal, const char* mesh_file, Vector3f box): Problem(ninit, ngoal) {
		 collision_checker = new CollisionChecker(mesh_file);
		 collision_checker->setBox(0.05, 0.3, 0.05);
	}

	~TestProblem() {
		delete collision_checker;
	}

	virtual TestState random_state() {
		static std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());

		TestState x;

		// Random position
		static std::uniform_real_distribution<double> p_distribution(-1.0,1.0);
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

	//Quaternion<float> quaternion_product(const Quaternion<float> &q1, const Quaternion<float> &q2) {

		//return Quaternion<float>(w, x, y, z);
	//}

	bool advance(TestState &nx, TestInput &nu, const TestState &x1, const TestState &x2, bool reverse=false) {
		// Translation
		nu.t = x2.p - x1.p;
		if(nu.t.squaredNorm() > MAX_TRANSLATION_SQR) {
			nu.t.normalize();
			nu.t *= MAX_TRANSLATION;
		}
		nx.p = x1.p + nu.t;

		// Rotation
		/*nu.r = x1.q.slerp(1, x2.q);
		float angle = 2*acos(nu.r.dot(x1.q));
		if(angle > M_PI)
			angle -= 2*M_PI;
		if(fabs(angle) > MAX_ROTATION) {
			nu.r = x1.q.slerp(fabs(MAX_ROTATION/angle), x2.q);
		}*/

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

		return !collides(nx);
	}

	virtual bool equal(const TestState &x1, const TestState &x2) {
		float dp = (x2.p-x1.p).squaredNorm();
		float dq = x1.q.dot(x2.q); dq = 1 - dq*dq;
		return dp < 0.01 and dq < 0.01;
	}

	virtual float metric(const TestState &x1, const TestState &x2) {
		float dp = (x2.p-x1.p).squaredNorm();
		float dq = x1.q.dot(x2.q); dq = 1 - dq*dq;
		return dp + dq;
	}

	bool collides(const TestState &x) {
    	collision_checker->setPosition(x.p[0], x.p[1], x.p[2]);
        collision_checker->setRotation(x.q.w(), x.q.x(), x.q.y(), x.q.z());
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
		fprintf(fw, "%e,%e,%e", s.p[0], s.p[1], s.p[2]);
		fprintf(fw, ",%e,%e,%e,%e", s.q.w(), s.q.x(), s.q.y(), s.q.z());
		fprintf(fw, "\n");
	}
	fclose(fw);
}

int main() {
	TestState init, goal;

	init.p = Vector3f(0.5, 0.0, 0.0);
	init.q = Quaternion<float>(0.0, 0.0, 0.0, 1.0);

	goal.p = Vector3f(0.5, 0.0, 0.8);
	goal.q = Quaternion<float>(0.0, 0.0, 0.0, 1.0);

	TestProblem problem(init, goal, "/home/tomas/Documents/6.834/mousetrap4.obj", Vector3f(0.05, 0.3, 0.05));

	BIRRTStar<TestState, TestInput, TestProblem> solver(&problem);

	pair<Node<TestState, TestInput>*, Node<TestState, TestInput>*> solution_nodes = solver.run(100000);

	if (solution_nodes.first == NULL or solution_nodes.second == NULL) {
		exit(0);
	}

	vector<TestState> states;
	vector<TestInput> inputs;
	solver.get_sulution(states, inputs, solution_nodes.first, solution_nodes.second);

	test_solution(states, inputs);

	write_solution(states, inputs, "/home/tomas/Documents/6.834/repo/solution2.txt");
}