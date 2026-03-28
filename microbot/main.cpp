#include "kinematics.h"


void testKinematics(Microbot &robot) {
    Jointspace in, o;
    Taskspace t;

    // 1. Setup Test Case (Angles in Radians for the functions)
    //NEED TO INPUT RADIANS INTO FK
    in.t[0] = 15.0;
    in.t[1] = 20.0;
    in.t[2] = -30.0;
    in.t[3] = 10.0;
    in.t[4] = 90.0;
    printf("=====TEST KINEMNATICS=============\n");
    for (int i=0; i<6;i++){
    	printf("Input angles for joint %d in degrees: %.2f rads: %.2f\n",i+1,in.t[i],in.t[i]*(PI/180.0));
    }
    // 2. Run Forward Kinematics
    robot.ForwardKinematics(in, t);
    printf("FK Results:\n");
    printf("X: (Expected: 330.89)\n");
    printf("Y: (Expected: 88.66)\n");
    printf("Z:  (Expected: 201.97)\n");
    printf("P: (Expected: -90)\n");
    printf("R: (Expected: 90)\n");
    printf("G: (Expected: g/no change)\n");
    // 3. Run Inverse Kinematics using the FK output
    robot.InverseKinematics(t, o);

    printf("\nIK Results (Back to Degrees):\n");
    for(int i=0; i<5; i++) {
        printf("Joint %d: %.2f rads %.2f (Expected matches input %f )\n",i+1, o.t[i],o.t[i] * (PI/180.0), in.t[i]);

    }
    printf("Joint 6: %.2f rads %.2f (Expected matches input %f)\n",o.t[5],o.t[5]* (PI/180.0),in.t[5]);
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


