#include "kinematics.h"


int Microbot::InverseKinematics(Taskspace t, Jointspace &j){
	// Constants from your robot and homework solution
	    const double h = 195.072; // d1 [cite: 4]
	    const double a = 177.8;   // a2 = a3 [cite: 4]
	    const double d = 96.52;   // d5 [cite: 4]
//		    const double h = 252; // d1 [cite: 4]
//		    const double a = 178;   // a2 = a3 [cite: 4]
//		    const double d = 80;   // d5 [cite: 4]
	    // 1. Orientation and Base Angle
	    // Convert Taskspace degrees to Radians [cite: 25, 26, 37]
	    double p_rad = t.p * (PI / 180.0);
	    double r_rad = t.r * (PI / 180.0);

	    j.t[4] = r_rad; // theta_5 = r [cite: 25]
	    double theta_234 = p_rad + (PI / 2.0); // theta_234 = p + 90 deg [cite: 26]

	    // Solve for theta_1 (Base) [cite: 26]
	    j.t[0] = atan2(t.y, t.x);

	    // 2. Solve for Wrist Center (wx, wy, wz)
	    // This removes the hand (d5) to simplify the arm geometry
	    double c1 = cos(j.t[0]);
	    double s1 = sin(j.t[0]);
	    double c234 = cos(theta_234);
	    double s234 = sin(theta_234);

	    double wx = t.x - d * c1 * s234; // Px - d*c1*s234
	    double wy = t.y - d * s1 * s234; // Py - d*s1*s234
	    double wz = t.z + d * c234;      // Pz + d*c234

	    // 3. Solve for theta_3 (Elbow)
	    // Law of cosines based on wrist position
	    double r_xy_sq = (wx * wx) + (wy * wy);
	    double c3 = (r_xy_sq + pow(wz - h, 2)) / (2 * a * a) - 1; //

	    // Use the "Elbow Down" solution (s3 < 0) as per solution
	    double s3 = -sqrt(1.0 - (c3 * c3));
	    j.t[2] = atan2(s3, c3); // theta_3

	    // 4. Solve for theta_2 (Shoulder)
	    double r_xy = sqrt(r_xy_sq);
	    // theta_2 = atan2((wz-h)(1+c3) - s3*r_xy, (wz-h)s3 + r_xy(1+c3))
	        double num_s2 = (wz - h) * (1.0 + c3) - s3 * r_xy;
	        double num_c2 = (wz - h) * s3 + r_xy * (1.0 + c3);

	        j.t[1] = atan2(num_s2, num_c2);

	    // 5. Solve for theta_4 (Wrist Pitch)
	    // theta_4 = theta_234 - theta_2 - theta_3 [cite: 33]
	    j.t[3] = theta_234 - j.t[1] - j.t[2];

	    // --- BOUNDS CHECKING (Page 189) ---
	        // Convert current radians to degrees for easy checking
	        double j0 = j.t[0] * (180.0 / PI); //base
	        double j1 = j.t[1] * (180.0 / PI); //shlder
	        double j2 = j.t[2] * (180.0 / PI); //elbow
	        double j3 = j.t[3] * (180.0 / PI); //wrist pitch J4
	        double j4 = j.t[4] * (180.0 / PI); //wrist roll J5
	        // 1. Base: +/- 90
	        if (j0 < -90 || j0 > 90){
	        	printf("fail j0\n");
	        	return 0;}

	        // 2. Shoulder: +144 to -35
	        if (j1 < -35 || j1 > 144){
	        	printf("fail j1\n");
	        	return 0;}

	        // 3. Elbow: 0 to -149 (Note: Your elbow moves in negative degrees)
	        if (j2 < -149 || j2 > 0){
	        	printf("fail j2\n");
	        	return 0;}

	        // 4. Wrist Pitch: +/- 90 (This was j4 in your code, should be j3)
	        if (j3 < -90.5 || j3 > 90.5) {
	            printf("fail j3: Actual calculated angle was %.4f\n", j3);
	            return 0;
	        }

	        // 5. Wrist Roll: +/- 270 (This was j3 in your code, should be j4)
	        if (j4 < -270 || j4 > 270){
	        	printf("fail j4\n");
	        	return 0;}

	        // Check for "Imaginary" math (Reach too far)
	        if (std::isnan(j.t[1]) || std::isnan(j.t[2])) return 0;

	        return 1; // Success

}

int Microbot::ForwardKinematics(Jointspace j, Taskspace &t){
	// write your forward kinematics here
	//theta values from jointspace
	double t1 = j.t[0];
	double t2 = j.t[1];
	double t3 = j.t[2];
	double t4 = j.t[3];
	double t5 = j.t[4];

	double d1 = 195.072;//mm, 7.68 inches;  BASE SEGMENT LENGTH
	double a2 = 177.8;//mm, 7.0 inches;		LINK 1 LENGTH
	double a3 = 177.8; //mm, 7.0 inches; 	LINK 2 LENGTH
	double d5 = 96.52; //mm, 3.80 inches;	LINK 3 LENGTH
//    const double d1 = 252; // d1 [cite: 4]
//    const double a2 = 178;   // a2 = a3 [cite: 4]
//    const double a3 = 178;   // a2 = a3 [cite: 4]
//    const double d5 = 80;   // d5 [cite: 4]

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

