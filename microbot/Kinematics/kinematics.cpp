#include "kinematics.h"

//taskspace is variables of the taskspace: x y z p r g
//jointspace is joint angles theta1 to theta6
//registerspace is used in the step command
int Microbot::InverseKinematics(Taskspace ts, Jointspace &js){
	// Constants from your robot and homework solution
	    const double h = 195.072; // d1 [cite: 4]
	    const double a = 177.8;   // a2 = a3 [cite: 4]
	    const double d = 96.52;   // d5 [cite: 4]

	    double s2, c2;
	    double s3,c3;

	    double t1 = atan2(ts.y,ts.x);
	    double t2 = atan2(s2,c2);
	    double t234 = ts.p+(PI/2);
	    return 1;
}

int Microbot::ForwardKinematics(Jointspace j, Taskspace &t){
	// write your forward kinematics here
	//theta values from jointspace
	double t1 = j.t[0];
	double t2 = j.t[1];
	double t3 = j.t[2];
	double t4 = j.t[3];
	double t5 = j.t[4];

//	double d1 = 195.072;//mm, 7.68 inches;  BASE SEGMENT LENGTH
//	double a2 = 177.8;//mm, 7.0 inches;		LINK 1 LENGTH
//	double a3 = 177.8; //mm, 7.0 inches; 	LINK 2 LENGTH
//	double d5 = 96.52; //mm, 3.80 inches;	LINK 3 LENGTH
    const double d1 = 252; // d1 [cite: 4]
    const double a2 = 178;   // a2 = a3 [cite: 4]
    const double a3 = 178;   // a2 = a3 [cite: 4]
    const double d5 = 80;   // d5 [cite: 4]

	double c1 = cos(t1), s1 = sin(t1);
	double c2 = cos(t2), s2 = sin(t2);
	double c23 = cos(t2+t3), s23 = sin(t2+t3);
	double c234 = cos(t2+t3+t4), s234 = sin(t2+t3+t4);
	double t234 = t2+t3+t4;
	//taskspace variables
	double x	=	c1*(a2*c2 + a3*c23 + d5*s234);
	double y	=	s1*(a2*c2 + a3*c23 + d5*s234);
	double z	=	d1 + a2*s2 + a3*s23 - d5*c234;
	double p	= 	(t234 - (PI/2))* (180.0 / PI);
	double r	=	t5* (180.0 / PI);

	t.x = x;
	t.y = y;
	t.z = z;
	t.p = p;
	t.r = r;

	return 1;
}


int Microbot::MoveTo(Jointspace nextJ, Jointspace &currentJ, Registerspace &delta) {
	// Page 189 Step Ratios
	    const double k[5] = {
	        7072.0 / (2.0 * PI), // Base
	        7072.0 / (2.0 * PI), // Shoulder
	        4158.0 / (2.0 * PI), // Elbow
	        1536.0 / (2.0 * PI), // Right Wrist
	        1536.0 / (2.0 * PI)  // Left Wrist
	    };

	    int nextSteps[5], currentSteps[5];

	    for(int i = 0; i < 5; i++) {
	        nextSteps[i] = (int)(nextJ.t[i] * k[i]);
	        currentSteps[i] = (int)(currentJ.t[i] * k[i]);

	        // Calculate relative steps for the motor (Registerspace is 1-indexed)
	        delta.r[i+1] = nextSteps[i] - currentSteps[i];

	        // TRUNCATION: Update nextJ with the actual angle achieved by integer steps
	        nextJ.t[i] = (double)nextSteps[i] / k[i];
	    }

	    // Indices 6 and 7 are usually for the gripper or unused
	    delta.r[6] = 0;
	    delta.r[7] = 0;

	    return 1;
}

