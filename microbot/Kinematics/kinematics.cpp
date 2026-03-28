#include "kinematics.h"

//taskspace is variables of the taskspace: x y z p r g
//jointspace is joint angles theta1 to theta6
//registerspace is used in the step command
int Microbot::InverseKinematics(Taskspace ts, Jointspace &js){
	// Constants from your robot and homework solution
//	    const double h = 195.072; // d1 [cite: 4]
//	    const double a = 177.8;   // a2 = a3 [cite: 4]
//	    const double d = 96.52;   // d5 [cite: 4]
	    const double h = 252; // d1 [cite: 4]
	    const double a = 178;   // a2 = a3 [cite: 4]
	    const double d = 80;   // d5 [cite: 4]

	    double theta1 = atan2(ts.y,ts.x);
	    double p = ts.p * (PI/180.0);
	    double theta234 = p + (PI/2.0);
	    double theta5 = ts.r * (PI / 180.0);
	    double theta6 = ts.g;
	    double c1 = cos(theta1), s1 = sin(theta1);
	    double c234 = cos(theta234), s234 = sin(theta234);
	    //wrist
	    double wx = ts.x - d * c1 * s234;
	    double wy = ts.y - d * s1 * s234;
	    double wz = ts.z + d * c234;
	    //wx + wy squared
	    double r_sq = wx * wx + wy * wy;
	    double c3 = ((r_sq + pow(wz - h, 2)) / (2.0 * a * a)) - 1.0;
	    //solve for theta3
	    //check correctness in lab
	    if (c3 > 1.0) c3 = 1.0;
	    else if (c3 < -1.0) c3 = -1.0;
	    double s3 = -sqrt(1.0 - c3 * c3);

	    double theta3 = atan2(s3,c3);

	    //solve for theta2
	    double common_denom = 2.0 * a * (1.0 + c3);
	    double sqrt_rxy = sqrt(r_sq);
	    double c2, s2;
	    if (wx >= 0) {
	            c2 = ((wz - h) * s3 + sqrt_rxy * (1.0 + c3)) / common_denom;
	            s2 = ((wz - h) * (1.0 + c3) - s3 * sqrt_rxy) / common_denom;
	        } if (wx <= 0){
	            c2 = ((wz - h) * s3 - sqrt_rxy * (1.0 + c3)) / common_denom;
	            s2 = ((wz - h) * (1.0 + c3) + s3 * sqrt_rxy) / common_denom;
	        }
	    double theta2 = atan2(s2,c2);
	    //solve for theta4
	    double theta4 = theta234-theta2-theta3;


	    printf("Inside Inverse Kinematics\n");
	    printf("T1:%f degrees: %f\n",theta1,theta1* (PI / 180.0));
	    printf("T2:%f degrees: %f\n",theta2,theta2* (PI / 180.0));
	    printf("T3:%f degrees: %f\n",theta3,theta3* (PI / 180.0));
	    printf("T4:%f degrees: %f\n",theta4,theta4* (PI / 180.0));
	    printf("T5:%f degrees: %f\n",theta5,theta5* (PI / 180.0));
		js.t[0] = theta1;
	    js.t[1] = theta2;
	    js.t[2] = theta3;
	    js.t[3] = theta4;
	    js.t[4] = theta5;
	    js.t[5] = theta6;
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
	t.g = j.t[5];
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

