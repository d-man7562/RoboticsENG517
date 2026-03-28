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
    printf("=====TEST KINEMNATICS=============\n");
    for (int i=0; i<5;i++){
    	printf("Input angles for joint %d in degrees: %.2f rads: %.2f\n",i+1,inputJoints.t[i]*(180/PI),inputJoints.t[i]);
    }
    // 2. Run Forward Kinematics
    robot.ForwardKinematics(inputJoints, targetTask);
    printf("FK Results:\n");
    printf("X: %.3f (Expected: 330.89)\n", targetTask.x);
    printf("Y: %.3f (Expected: 88.66)\n", targetTask.y);
    printf("Z: %.3f (Expected: 201.97)\n", targetTask.z);
    printf("P: degrees: %.3f rads: %.2f (Expected: -90)\n", targetTask.p, targetTask.p* (PI/180.0));
    printf("R: degrees: %.3f rads: %.2f (Expected: 90)\n", targetTask.r, targetTask.r* (PI/180.0));
    printf("G: degrees: %.3f rads: %.2f (Expected: g/no change)\n", targetTask.g, targetTask.g* (PI/180.0));
    // 3. Run Inverse Kinematics using the FK output
    robot.InverseKinematics(targetTask, outputJoints);

    printf("\nIK Results (Back to Degrees):\n");
    for(int i=0; i<4; i++) {
        printf("Joint %d: %.2f deg (Expected matches input %.2f)\n",i+1, outputJoints.t[i] * (180.0 / PI), inputJoints.t[i] *( 180.0/PI));

    }
    printf("Joint 5: %.2f deg (Expected matches input %.2f)\n",outputJoints.t[4],inputJoints.t[4]*(180/PI));
    printf("=====END TEST KINEMNATICS=============\n");
}


int main() {
    Microbot robot;
    Jointspace currentJoints, nextJoints;
    Taskspace currentTask, nextTask;
    Registerspace delta;
    int speed = 235; // Recommended speed from your manual
    testKinematics(robot);
    // STEP 1: Initialize at Home (Manual Page 181)
    currentTask.x = 125.0; currentTask.y = 0.0; currentTask.z = 20.0;
    currentTask.p = -90.0; currentTask.r = 0.0;

    // Convert home position to joints to establish our starting "zero"
    robot.InverseKinematics(currentTask, currentJoints);

    while(1) {
        // STEP 2: Prompt User (Use %lf for doubles!)
        printf("\nCurrent Pos: X:%.1f Y:%.1f Z:%.1f P:%.1f R:%.1f\n", currentTask.x, currentTask.y, currentTask.z,currentTask.p,currentTask.r);
        printf("Enter target X Y Z P R (Enter 0 0 0 0 0 to quit): ");
        fflush(stdout);
        if (scanf("%lf %lf %lf %lf %lf", &nextTask.x, &nextTask.y, &nextTask.z, &nextTask.p, &nextTask.r) != 5) break;
        if (nextTask.x == 0) break;

        // STEP 3: Run IK for the target configuration
        if (robot.InverseKinematics(nextTask, nextJoints) == 0) {
        	printf("Coordinates out of bounds\n");
        	continue;
        }

        // STEP 4: Calculate Steps and Send to Robot
        robot.MoveTo(nextJoints, currentJoints, delta);

        // Physically move the motors using the calculated delta
        robot.SendStep(speed, delta);

        // STEP 5: Update current state
        // Use the truncated joint angles (already updated inside MoveTo)
        // to find the actual taskspace position
        currentJoints = nextJoints;
        robot.ForwardKinematics(currentJoints, currentTask);

        printf("Move Complete. Actual X:%.2f Y:%.2f Z:%.2f\n", currentTask.x, currentTask.y, currentTask.z);
    }

    return 0;
}


