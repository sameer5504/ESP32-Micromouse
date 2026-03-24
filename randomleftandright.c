// Maen Foqaha - 1220847
// Mohammad Ewais - 1220053
// Samir Ali - 1222350
// Ibrahim Al Ardah - 1220874
#include <stdio.h>
#include <stdlib.h>
#include "API.h"
#include <time.h>

void log_text(char* text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

int main(int argc, char* argv[]) {
    log_text("Running Using Random Left/Right Hand Rule...");
    int x=0;  // Current X
    int y=0;  // Current Y
    int x1= API_mazeWidth()/2;  // Goal X (Center)
    int x2 = x1-1;  // Goal X (Center)
    int y1 = API_mazeHeight()/2;  // Goal Y (Center)
    int y2 = y1-1;  // Goal Y (Center)
    int direction=0; // 0: +Y, 1: -X, 2: -Y, 3: +X
    API_setColor(x1, y1, 'R');
    API_setColor(x1, y2, 'R');
    API_setColor(x2, y1, 'R');
    API_setColor(x2, y2, 'R');
    API_setText(0, 0, "START");
    while (!((x==x1 && y==y1)||(x==x1 && y==y2)||(x==x2 && y==y1)||(x==x2 && y==y2))) { // Keep going until reaching one of the goal coordinates
        srand(time(NULL));  // Seed
		int random_choice = rand()%2;
		if (random_choice==0){ // Use Left Hand Rule
			if (!API_wallLeft()) {  // When there is no wall on the left then turn left
				API_turnLeft();
				direction++;
			}
			while (API_wallFront()) {  // When there is no left, and there is a wall infront keep turning right until there is no wall infront
				API_turnRight();
				direction--;
				if (direction == -1){
				direction=3;
				}
			}
		}else{  // Use Right Hand Rule
			if (!API_wallRight()) {  // When there is no wall on the right then turn right
				API_turnRight();
				direction--;
				if (direction == -1){
				direction=3;
				}
			}
			while (API_wallFront()) {  // When there is no right, and there is a wall infront keep turning left until there is no wall infront
				API_turnLeft();
				direction++;
			}
		}
        API_moveForward();
        API_setColor(x, y, 'G');
        switch (direction %4)  // Update Current (X,Y) depending on the direction of movement
        {
        case 0:
            y++;
            break;
        case 1:
            x--;
            break;
        case 2:
            y--;
            break;
        case 3:
            x++;
            break;     
        default:
            break;
        }
        char str[50];
        sprintf(str, "Moved to (%d,%d)", x, y);
        log_text(str);
    }
	    log_text("Goal Reached !");
}