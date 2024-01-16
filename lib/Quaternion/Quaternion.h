#ifndef QUATERNION
#define QUATERNION

#include <math.h>
#include "Vector3D.h"

class Quat {
public:
	Quat(double q0_=1, double q1_=0, double q2_=0, double q3_=0);
	~Quat();
	double q0 = 1;
	double q1 = 0;
	double q2 = 0;
	double q3 = 0;
};

class Quaternion
{
public:
	Quaternion();
	~Quaternion();
	double get_scalar(Quat q1);
	Vect3D get_vector(Quat q1);
	Quat unit();
	Quat zero();
	Quat add(Quat q1, Quat q2);
	Quat sub(Quat q1, Quat q2);
	Quat mul(Quat q1, Quat q2);
	Quat mul_scalar(Quat q1, double s);
	Quat div_scalar(Quat q1, double s);
	Quat conj(Quat q1);
	double norm(Quat q1);
	Quat normalize(Quat q1);
	Quat inv(Quat q1);
	Quat pow_scalar(Quat q, double r);
	Quat exp_quat(Quat q);
	Quat log_e(Quat q);
};

class Diff {
public:
	Diff();
	~Diff();
	Quat diff(Quat q_t, double dt);
	void reset();
private:
	Quaternion quat;
	Quat q_t_1 = quat.zero();

};

class Integrate {
public:
	Integrate();
	~Integrate();
	Quat integrate(Quat q_t, double dt);
	void reset();
private:
	Quaternion quat;
	Quat q_t_p_1 = quat.zero();

};

#endif