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
	    if (ts.z < 10.0) { // 10mm safety buffer above the table
	        printf("Safety Error: Target Z is too low! (Collision Risk): %.2f \n",ts.z);
	        return 0;
	    }
	    // 1. Calculate horizontal distance from center
	    double dist_2d = sqrt(ts.x * ts.x + ts.y * ts.y);

	    // 2. Minimum Reach Safety Check
	    // If the target is too close (inside the robot's own body)
	    if (dist_2d < 100.0 && ts.z < (h + 50)) {
	        printf("Safety Error: Target is too close to the base (Self-Collision Risk)!\n");
	        return 0;
	    }
	    double dist_to_target = sqrt(pow(ts.x, 2) + pow(ts.y, 2) + pow(ts.z - 195.072, 2));

	        // 2. Max theoretical reach is a2 + a3 + d5
	        if (dist_to_target > (h+a+d)) {
	            printf("Safety Error: Target is physically too far away!\n");
	            return 0; // Return failure to main.cpp
	        }

	    double theta1 = atan2(ts.y,ts.x);

	    if (theta1 < (-1.5708) || theta1 > (1.5708)){
	    	printf("Theta1 out of bounds: %.2f\n",theta1);
	    	return 0;
	    }

	    double p = ts.p * (PI/180.0);
	    double theta234 = p + (PI/2.0);

	    double theta5 = ts.r * (PI / 180.0);

	    if (theta5 < -(1.5708) || theta5 > (1.5708)){
	     	printf("Theta5 out of bounds: %.2f\n",theta5);
	    	    	return 0;
	    }

	    double theta6 = ts.g * 0; //fuck
	    double c1 = cos(theta1), s1 = sin(theta1);
	    double c234 = cos(theta234), s234 = sin(theta234);
	    //wrist
	    double wx = ts.x - d * c1 * s234;
	    double wy = ts.y - d * s1 * s234;
	    double wz = ts.z + d * c234;
	    //wx + wy squared
	    double r_sq = wx * wx + wy * wy;
	    double horizontal_dist = sqrt(ts.x * ts.x + ts.y * ts.y);
	    if (horizontal_dist < 50.0 && ts.z < 100.0) {
	        printf("Safety Error: Target too close to base (Collision Risk): %.2f\n",horizontal_dist);
	        return 0;
	    }
	    double dist_shoulder_to_wrist_sq = r_sq + pow(wz - h, 2);
	    if (dist_shoulder_to_wrist_sq > pow(a + a, 2)) {
	        printf("Safety Error: Target wrist position is physically unreachable!\n");
	        return 0;
	    }
	    double c3 = ((r_sq + pow(wz - h, 2)) / (2.0 * a * a)) - 1.0;
	    //solve for theta3
//	    //check correctness in lab

	    if (c3 > 1.00001 || c3 < -1.00001) {
	            printf("Math Error: Target point results in impossible joint geometry. C3: %f\n",c3);
	            return 0;
	        }
	    if (c3 > 1.0) c3 = 1.0;
	    if (c3 < -1.0) c3 = -1.0;
	    double s3 = -sqrt(1.0 - c3 * c3);

	    double theta3 = atan2(s3,c3);

	    if (theta3 > 0 || theta3 <-2.6006){ //0, -144
	   	     	printf("Theta3 out of bounds: %.2f\n",theta3);
	   	    	    	return 0;
	   	    }

	    //solve for theta2
	    double common_denom = 2.0 * a * (1.0 + c3);
	    if (fabs(common_denom) < 0.000001) {
	        printf("Singularity Error: The arm is either fully extended or folded!\n");
	        return 0;
	    }
	    double sqrt_rxy = sqrt(r_sq);
	    double c2, s2;
	    if (wx >= 0) {
	            c2 = ((wz - h) * s3 + sqrt_rxy * (1.0 + c3)) / common_denom;
	            s2 = ((wz - h) * (1.0 + c3) - s3 * sqrt_rxy) / common_denom;
	        } else{
	            c2 = ((wz - h) * s3 - sqrt_rxy * (1.0 + c3)) / common_denom;
	            s2 = ((wz - h) * (1.0 + c3) + s3 * sqrt_rxy) / common_denom;
	        }


	    double theta2 = atan2(s2,c2);

	    if (theta2 > 2.5133 || theta2 < -0.6109){ //144 , -35   .. -.34 used to be  -0.6109
		   	     	printf("Theta2 out of bounds: %.2f\n",theta2);
		   	    	    	return 0;
		   	    }


	    //solve for theta4
	    double theta4 = theta234-theta2-theta3;

	    if (theta4 > 4.7124 || theta4 < -4.7124){ // 270 deg
		   	     	printf("Theta4 out of bounds: %.2f\n",theta4);
		   	    	    	return 0;
		   	    }

	    printf("Inside Inverse Kinematics\n");
	    printf("T1: in degrees: %.2f in rads: %.2f\n",theta1* (180.0/ PI),theta1);
	    printf("T2: in degrees: %.2f in rads: %.2f\n",theta2* (180.0/ PI),theta2);
	    printf("T3: in degrees: %.2f in rads: %.2f\n",theta3* (180.0/ PI),theta3);
	    printf("T4: in degrees: %.2f in rads: %.2f\n",theta4* (180.0/ PI),theta4);
	    printf("T5: in degrees: %.2f in rads: %.2f\n",theta5* (180.0/ PI ),theta5);
	    printf("T6: in degrees: %.2f in rads: %.2f\n",theta6* (180.0/ PI ),theta6);
		js.t[0] = theta1 ;
	    js.t[1] = theta2;
	    js.t[2] = theta3;
	    js.t[3] = theta4;
	    js.t[4] = theta5;
	    js.t[5] = theta6;
	    return 1;
}
//must accept radians
int Microbot::ForwardKinematics(Jointspace j, Taskspace &t){
	// write your forward kinematics here
	//theta values from jointspace
	//should already be radians from IK function
	double t1 = j.t[0];
	double t2 = j.t[1];
	double t3 = j.t[2];
	double t4 = j.t[3];
	double t5 = j.t[4] ;

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
		printf("Inside Forward Kinematics:\n");
		printf("X: %.2f \n", t.x);
	    printf("Y: %.2f \n", t.y);
	    printf("Z: %.2f \n", t.z);
	    printf("P: in degrees: %.2f in rads: %.2f\n", t.p, t.p* (PI/180.0));
	    printf("R: in degrees: %.2f in rads: %.2f \n", t.r, t.r* (PI/180.0));
	    printf("G: in degrees: %.2f in rads: %.2f \n", t.g, t.g* (PI/180.0));
	return 1;
}


int Microbot::MoveTo(Jointspace nextJ,Jointspace &currentJ, Registerspace &delta) {

	memset(delta.r, 0, sizeof(delta.r));
	//1. we start at a position in task space
	//2. we get a new task space to move to
	//3. check if  valid joint POS using IK
	//4. test if in valid task space range using FK
	//5. determine num of steps to move per joint
	//6. send steps


	//take difference of old jointspace and new jointspace
	//use that to convert to steps


	// Page 189 Step Ratios

	    const double k[5] = {
	    	1125, // Base
	        1125, // Shoulder
	        672, // Elbow
	        244.4, // Right Wrist
	        244.4// Left Wrist
	    };


	    double d_theta[5];
	        for(int i = 0; i < 5; i++) {
	            d_theta[i] = nextJ.t[i] - currentJ.t[i];
	        }

	        // APPLY SIGN FLIPS HERE based on your hardware testing:
	            delta.r[1] = (int)(d_theta[0] * k[0]);  // Base (Check if left/right is correct)
	            delta.r[2] = (int)(d_theta[1] * -k[1]); // Shoulder (Flipped: + angle = Up)
	            delta.r[3] = (int)(d_theta[2] * -k[2]); // Elbow (Flipped: + angle = Out/Up)

	            // Differential Wrist Logic
	            double pitch_steps = d_theta[3] * k[3];
	            double roll_steps  = d_theta[4] * k[4];

	            // If Pitch is also inverted, flip the sign of pitch_steps here:
	            delta.r[4] = (int)(pitch_steps + roll_steps);
	            delta.r[4] = (int)(pitch_steps);
	            delta.r[5] = (int)(-pitch_steps - roll_steps);
	            delta.r[5] = (int)(-pitch_steps );


	    return 1;
}

