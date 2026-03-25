#include "kinematics.h"


int Microbot::InverseKinematics(Taskspace t, Jointspace &j){

	// write your inverse kinematics here, here is a sample:

	int i = 0; // declaration for the return variable

	printf("inside InverseKinematics\n");  // to be removed when the function is complete
	fflush(stdout);

//	j.t[1] = t.x + PI; // use passed-through data (t) to calculate return data (j)

    // Extract taskspace values
	int Microbot::InverseKinematics(Taskspace t, Jointspace &j){

	    double px = t.x;
	    double py = t.y;
	    double pz = t.z;

	    double theta1 = std::atan2(py, px);
	    double theta234 = t.p + (PI / 2);
	    double theta5 = t.r;

	    double c1 = cos(theta1);
	    double s1 = sin(theta1);
	    double c234 = cos(theta234);
	    double s234 = sin(theta234);

	    // Wrist position
	    double Wx = px - d * c1 * s234;
	    double Wy = py - d * s1 * s234;
	    double Wz = pz + d * c234;

	    double r = sqrt(Wx*Wx + Wy*Wy);

	    // Theta3
	    double c3 = (Wx*Wx + Wy*Wy + (Wz - h)*(Wz - h)) / (2 * a * a);
	    c3 = std::clamp(c3, -1.0, 1.0);

	    double s3 = -sqrt(1 - c3*c3);
	    double theta3 = std::atan2(s3, c3);

	    // Theta2
	    double c2 = ((Wz - h) * s3 + r * (1 + c3)) / (2 * a * (1 + c3));
	    double s2 = ((Wz - h) * (1 + c3) - s3 * r) / (2 * a * (1 + c3));
	    double theta2 = std::atan2(s2, c2);

	    double theta4 = theta234 - theta2 - theta3;

	    // Assign
	    j.t[0] = theta1;
	    j.t[1] = theta2;
	    j.t[2] = theta3;
	    j.t[3] = theta4;
	    j.t[4] = theta5;




	i = 1; // assign a return value, usually error code
	return(i);
}

int Microbot::ForwardKinematics(Jointspace j, Taskspace &t){
	// write your forward kinematics here
	return(0);
}

int Microbot::MoveTo(Taskspace &t){
	// write your move-to function here
	return(0);
}
