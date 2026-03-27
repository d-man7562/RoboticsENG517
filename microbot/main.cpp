#include "kinematics.h"


void testKinematics(Microbot &robot) {
    Jointspace inputJoints, outputJoints;
    Taskspace targetTask;

    // 1. Setup Test Case (Angles in Radians for the functions)
    inputJoints.t[0] = 15.0 * (PI / 180.0);
    inputJoints.t[1] = 20.0 * (PI / 180.0);
    inputJoints.t[2] = -30.0 * (PI / 180.0);
    inputJoints.t[3] = 10.0 * (PI / 180.0);
    inputJoints.t[4] = 90.0 * (PI / 180.0);

    // 2. Run Forward Kinematics
    robot.ForwardKinematics(inputJoints, targetTask);

    printf("FK Results:\n");
    printf("X: %.3f (Expected: 330.89)\n", targetTask.x);
    printf("Y: %.3f (Expected: 88.66)\n", targetTask.y);
    printf("Z: %.3f (Expected: 201.97)\n", targetTask.z);

    // 3. Run Inverse Kinematics using the FK output
    robot.InverseKinematics(targetTask, outputJoints);

    printf("\nIK Results (Back to Degrees):\n");
    for(int i=0; i<5; i++) {
        printf("Joint %d: %.2f deg (Expected matches input)\n",
                i+1, outputJoints.t[i] * (180.0 / PI));
    }
}


int main() {
    Microbot robot;
    Jointspace currentJoints, nextJoints;
    Taskspace currentTask, nextTask;
    Registerspace delta;
    int speed = 235; // Recommended speed from your manual

    // STEP 1: Initialize at Home (Manual Page 181)
    currentTask.x = 125.0; currentTask.y = 0.0; currentTask.z = 20.0;
    currentTask.p = -90.0; currentTask.r = 0.0;

    // Convert home position to joints to establish our starting "zero"
    robot.InverseKinematics(currentTask, currentJoints);

    while(1) {
    	// --- STEP 2: Display Current State (Both mm and Degrees) ---
    	        printf("\n==================================================");
    	        printf("\nCURRENT POSITION (mm):  X:%.1f Y:%.1f Z:%.1f P:%.1f",
    	                currentTask.x, currentTask.y, currentTask.z, currentTask.p);

    	        printf("\nCURRENT JOINTS (deg): J1:%.2f J2:%.2f J3:%.2f J4:%.2f",
    	                currentJoints.t[0] * (180.0/PI), currentJoints.t[1] * (180.0/PI),
    	                currentJoints.t[2] * (180.0/PI), currentJoints.t[3] * (180.0/PI));
    	        printf("\n==================================================\n");

    	        printf("Enter target X Y Z P R: ");
    	        fflush(stdout);

    	        if (scanf("%lf %lf %lf %lf %lf", &nextTask.x, &nextTask.y, &nextTask.z, &nextTask.p, &nextTask.r) != 5) break;
    	        if (nextTask.x == 0) break;

    	        // --- STEP 3: IK ---
    	        if (robot.InverseKinematics(nextTask, nextJoints) == 0) {
    	            // Note: If this fails, currentJoints and currentTask remain UNCHANGED.
    	            // This prevents the "desync" where the code thinks it moved but didn't.
    	            printf(">> ERROR: Target unreachable. Staying at current position.\n");
    	            continue;
    	        }

    	        // --- STEP 4 & 5: Move and Update ---
    	        robot.MoveTo(nextJoints, currentJoints, delta); // nextJoints gets truncated here
    	        robot.SendStep(speed, delta);

    	        currentJoints = nextJoints; // Now accurately updated with truncated angles
    	        robot.ForwardKinematics(currentJoints, currentTask);

    	        printf(">> Move Complete.\n");
    	    }}


