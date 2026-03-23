#include "kinematics.h"

void assignParameters(int r[7]) { // r[1..7]
    char line[256];
    int j = 1;

    printf("Enter value for delta 1-7, delimited by commas, 0 for no joint movement. ENTER NOTHING to exit:\n");
    fflush(stdout);

    if (!fgets(line, sizeof(line), stdin)) {
        printf("No input detected, quitting.\n");
        exit(1);
    }

    // check for empty line
    if (line[0] == '\n') {
    	printf("Good bye.\n");
        exit(1);
    }

    char *token = strtok(line, ",");
    while (token != NULL && j <= 7) {
        // skip leading/trailing whitespace
        while (*token == ' ' || *token == '\t') token++;

        int value;
        char *endptr;

        if (token[0] == '\0' || token[0] == ',') {
            // empty token -> assign 0
            value = 0;

        }
        else {
            value = strtol(token, &endptr, 10);

            // check for invalid input
            if (*endptr != '\0' && *endptr != '\n' && *endptr != ' ' && *endptr != '\t') {
                printf("Non-integer input detected, quitting.\n");
                exit(1);
            }
            if (*endptr == token[0]) value = 0;
        }

        r[j++] = value;
        token = strtok(NULL, ",");
    }

    // Fill remaining indices with 0 if fewer than 7 tokens
    while (j <= 7) r[j++] = 0;

    // Print array for verification
    printf("Values entered:\n");
    for (int i = 1; i <= 7; i++) {
        printf("%d ", r[i]);
    }
    printf("\n");
}
int main()
{
	Microbot robot;				// Local variable of the microbot class
	Registerspace delta;		// Local variable for input of motor steps
	Jointspace j;				// Local variable for kinematic calculations
	Taskspace t;				// Local variable for kinematic calculations


// Example; replace it with your own program
while (1){
	int spe = 235;				// Motor speed; should not be higher than 240
	int i = 1;
	int out = 0;
	assignParameters(delta.r);


	while(i<=7) {
		out = robot.SendStep(spe, delta);	// Send instruction to the microbot
		printf("i= %d, out= %d\n", i, out);
		fflush(stdout);
		i++;
	};

	printf("Enter 1000 for reset\nEnter any other integer to continue: ");fflush(stdout);
	int temp;
	scanf("%d", &temp);
	printf("temp = %d\n",temp);
	if (temp == 1000){
    for (int k=1;k<8;k++){
    	delta.r[k] = -delta.r[k];
    };
	i=1;
	while(i<=7) {
//-10,-10,-10,-10,-10,-10,-10
    		out = robot.SendStep(spe, delta);	// Send instruction to the microbot
    		printf("i= %d, out= %d\n", i, out);
    		fflush(stdout);
    		i++;
    	};
	i=1;
	}else
		fflush(stdin);


	t.x = 20; // Taskspace data to be passed in by value

	printf("Input to InverseKinematics: t.x= %f\n", t.x); // Shows how to display a structured variable
	fflush(stdout);

	out = robot.InverseKinematics(t,j); // Jointspace data retrieved by reference

	printf("Output from InverseKinematics: j.t[1]= %f, return= %d\n", j.t[1], out);
	fflush(stdout);

	delta.r[1] = BASE_STEPS * j.t[1];

//	printf("Done, hit Enter to exit: ");fflush(stdout);
//	getchar();

}}
