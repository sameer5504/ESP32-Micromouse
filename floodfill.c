// Maen Foqaha - 1220847
// Mohammad Ewais - 1220053
// Samir Ali - 1222350
// Ibrahim Al Ardah - 1220874
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "API.h"

// Print to Log
static void log_text(const char* text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}
//----------DIRECTIONS-----------
// 0 = North (+Y), 1 = East (+X), 2 = South (-Y), 3 = West (-X)
static const int dx[4]     = { 0,  1,  0, -1 };
static const int dy[4]     = { 1,  0, -1,  0 };
static const int revdir[4] = { 2,  3,  0,  1 };
static const char* dirChar[4] = {"North","East","South","West"};

//------------STRUCTS------------
//Stack
typedef struct stack_node *ptrS;
struct stack_node{
    int wantedDir;
    ptrS next;
};
typedef ptrS Stack;
//Queue
typedef struct queue_node *ptrQN;
struct queue_node{
    int x, y;
    ptrQN next;
};
typedef ptrQN QNode;
typedef struct empty_queue *Queue;
struct empty_queue{
    ptrQN front;
    ptrQN rear;
};
//Stack Functions
static Stack stackPush(Stack s, int wantedDir){
    ptrS n = (ptrS)malloc(sizeof(*n));
    n->wantedDir = wantedDir;
    n->next = s;
    return n;
}
static int stackIsEmpty(Stack s){ return s==NULL; }
static int stackPop(Stack* ps){
    if(!*ps) return -1;
    ptrS t = *ps;
    int d = t->wantedDir;
    *ps = t->next;
    free(t);
    return d;
}
// Queue Functions
static Queue makeQueue(){
    Queue q = (Queue)malloc(sizeof(*q));
    q->front = q->rear = NULL;
    return q;
}
static int queueEmpty(Queue q){ return q->front==NULL; }
static void enqueue(Queue q, int x, int y){
    QNode n = (QNode)malloc(sizeof(*n));
    n->x = x; n->y = y; n->next = NULL;
    if(q->rear) q->rear->next = n; else q->front = n;
    q->rear = n;
}
static int dequeue(Queue q, int* x, int* y){
    if(!q->front) return 0;
    QNode n = q->front;
    *x = n->x; *y = n->y;
    q->front = n->next;
    if(!q->front) q->rear = NULL;
    free(n);
    return 1;
}
//-----------Rotation------------
static void rotateRight(int* h){
	API_turnRight();
	*h = (*h + 1) & 3;
	}
static void rotateLeft(int* h){
	API_turnLeft();
	*h = (*h + 3) & 3;
	}
static void rotateTo(int target, int* h){
    // rotate right until match
    while((*h & 3) != (target & 3)) rotateRight(h);
}

//----------Heuristics-----------
void manhattanToNearestCenter(int W,int H,int x,int y){
    int x1 = W/2;
	int	x2 = x1-1; 
	int y1 = H/2;
	int	y2 = y1-1;
    int c1 = abs(x - x1) + abs(y - y1);
    int c2 = abs(x - x1) + abs(y - y2);
    int c3 = abs(x - x2) + abs(y - y1);
    int c4 = abs(x - x2) + abs(y - y2);
    int min = c1;
	if(c2 < min) min = c2;
	if(c3 < min) min = c3;
	if(c4 < min) min = c4;
    char str[16];
    sprintf(str, "%d", min);
    API_setText(x, y, str);
}

//------ Sensing the walls ------
// Sensor mapping: front -> heading
// left  -> (heading + 3) & 3 (counter-clockwise)
// right -> (heading + 1) & 3 (clockwise)

static void senseAndUpdateWalls(int W,int H,int x,int y,int heading, unsigned char walls[16][16][4]){
    int front = API_wallFront() ? 1 : 0;
    int left  = API_wallLeft()  ? 1 : 0;
    int right = API_wallRight() ? 1 : 0;

    int dirFront = heading & 3;
    int dirLeft  = (heading + 3) & 3; // left is ccw
    int dirRight = (heading + 1) & 3; // right is cw

    walls[y][x][dirFront] = front;
    walls[y][x][dirLeft]  = left;
    walls[y][x][dirRight] = right;

    // Boundaries are walls
    for(int d=0; d<4; ++d){
        int nx = x + dx[d], ny = y + dy[d];
        if(nx < 0 || ny < 0 || nx >= W || ny >= H) walls[y][x][d] = 1;
    }

    // Mirror walls into neighbors
    for(int d=0; d<4; ++d){
        int nx = x + dx[d], ny = y + dy[d];
        if(nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
        walls[ny][nx][revdir[d]] = walls[y][x][d];
    }
}

//----GLOBAL FLOOD-FILL (BFS)----
// Recompute exact shortest distances from all center goal cells using known walls.
static void recomputeFlood(int W,int H,int cost[16][16], unsigned char walls[16][16][4]){
    const int INF = 1<<28;
    // init
    for(int yy=0; yy<H; ++yy) for(int xx=0; xx<W; ++xx) cost[yy][xx] = INF;

    // center cells (goal) - set to 0 and enqueue
    int gx1 = W/2, gx2 = gx1 - 1;
    int gy1 = H/2, gy2 = gy1 - 1;
    int qx[16*16], qy[16*16];
    int head = 0, tail = 0;

    if(gx1>=0 && gx1<W && gy1>=0 && gy1<H){
		cost[gy1][gx1] = 0;
		qx[tail]=gx1;
		qy[tail]=gy1;
		tail++;
		}
    if(gx1>=0 && gx1<W && gy2>=0 && gy2<H){
		if(cost[gy2][gx1] > 0){
			cost[gy2][gx1] = 0;
			qx[tail]=gx1;
			qy[tail]=gy2;
			tail++;
			}
		}
    if(gx2>=0 && gx2<W && gy1>=0 && gy1<H){
		if(cost[gy1][gx2] > 0){ 
		cost[gy1][gx2] = 0;
		qx[tail]=gx2;
		qy[tail]=gy1;
		tail++;
			}
		}
    if(gx2>=0 && gx2<W && gy2>=0 && gy2<H){
		if(cost[gy2][gx2] > 0){
			cost[gy2][gx2] = 0;
			qx[tail]=gx2;
			qy[tail]=gy2;
			tail++;
			}
		}

    while(head < tail){
        int cx = qx[head], cy = qy[head];
        head++;
        int base = cost[cy][cx];
        for(int d=0; d<4; ++d){
            if(walls[cy][cx][d]) continue;
            int nx = cx + dx[d], ny = cy + dy[d];
            if(nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            if(cost[ny][nx] > base + 1){
                cost[ny][nx] = base + 1;
                qx[tail] = nx; qy[tail] = ny; tail++;
            }
        }
    }

    // update text display
    for(int yy=0; yy<H; ++yy){
        for(int xx=0; xx<W; ++xx){
            char buf[8];
            if(cost[yy][xx] < INF) sprintf(buf, "%d", cost[yy][xx]);
            else sprintf(buf, "X");
            API_setText(xx, yy, buf);
        }
    }
}
//------------- Main ------------
int main(int argc, char* argv[]) {
	
    log_text("Running Using Flood Fill Algorithm...");
	// Calculate Goal Coordinates
    const int W = API_mazeWidth();
    const int H = API_mazeHeight();
    int gx1 = W/2;
	int	gx2 = gx1 - 1;
	int gy1 = H/2;
	int	gy2 = gy1 - 1;
    API_setText(0,0,"START");
    static int cost[16][16];
    static unsigned char walls[16][16][4];  // 1=blocked, 0=open
    static unsigned char visited[16][16];
	
    for(int yy=0; yy<H; ++yy){  // Calculate Manhattan Distance for all cells
        for(int xx=0; xx<W; ++xx){
            visited[yy][xx] = 0;
            for(int d=0; d<4; ++d) {
				walls[yy][xx][d] = 0;
				}
            manhattanToNearestCenter(W,H,xx,yy);
        }
    }

    // Set Maze borders as walls
    for(int xx=0; xx<W; ++xx){
		walls[0][xx][2] = 1;
		walls[H-1][xx][0] = 1;
		}
    for(int yy=0; yy<H; ++yy){
		walls[yy][0][3] = 1;
		walls[yy][W-1][1] = 1;
		}
		
    int x = 0; // Start at 0,0 and heading in direction 0 (North)
	int y = 0;
	int heading = 0;
    Stack path = NULL;
	
	// For detecting the final path
	int path_signature_x = 0;
	int path_signature_y = 0;
	int previous_path_signature_x=0;
	int previous_path_signature_y=0;
	int final_path_found = 0;
    recomputeFlood(W,H,cost,walls);
	int i=1;
	int i2=1;
	
	while(1){ // Keeps running, so minimum run time can be used as the official run time
		previous_path_signature_x = path_signature_x;
		previous_path_signature_y = path_signature_y;
		path_signature_x = 0;
		path_signature_y = 0;
		char strLog[50];
		if (final_path_found==0){
			sprintf(strLog, "=== RUN #%d USING SENSORS ===", i);
			i++;
		}else {
			sprintf(strLog, "=== RUN #%d WITHOUT SENSORS ===", i2);
			i2++;
		}
        log_text(strLog);
		
		API_setColor(gx1, gy1, 'R');
		API_setColor(gx1, gy2, 'R');
		API_setColor(gx2, gy1, 'R');
		API_setColor(gx2, gy2, 'R');
		
		while(!( (x==gx1&&y==gy1) || (x==gx1&&y==gy2) || (x==gx2&&y==gy1) || (x==gx2&&y==gy2) )){ // Until reaching the goal
		path_signature_x = path_signature_x + x;
		path_signature_y = path_signature_y + y;
        visited[y][x] = 1;
		
		if (final_path_found==0){  // Use Sensors and Flood Fill Algorithm
			// sense and update walls based on sensors relative to heading
			senseAndUpdateWalls(W,H,x,y,heading,walls);
			// recompute full flood after any new wall info
			recomputeFlood(W,H,cost,walls);
			// choose neighbor with lowest cost
		}
        int bestDir = -1;
        int here = cost[y][x];
        int bestCost = here;
        for(int d=0; d<4; ++d){
            if(walls[y][x][d]) continue;
            int nx = x + dx[d], ny = y + dy[d];
            if(nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            if(cost[ny][nx] < bestCost){
                bestCost = cost[ny][nx];
                bestDir = d;
            }
        }
		
		int direction_if_backtracking = stackPop(&path);
		if (bestDir==direction_if_backtracking){
			rotateTo(direction_if_backtracking, &heading);
			x += dx[direction_if_backtracking]; y += dy[direction_if_backtracking];
            char buf[128]; sprintf(buf, "Backtracked to (%d,%d) heading %s", x, y, dirChar[heading]); log_text(buf);
			API_moveForward();
		}else{
            path = stackPush(path, direction_if_backtracking);
			// go to bestDir
            rotateTo(bestDir, &heading);
            if(!API_moveForward()){
                // unexpected blocked: mark and recompute
                walls[y][x][bestDir] = 1;
                int nx = x + dx[bestDir], ny = y + dy[bestDir];
                if(nx >= 0 && ny >= 0 && nx < W && ny < H) walls[ny][nx][revdir[bestDir]] = 1;
                recomputeFlood(W,H,cost,walls);
            } else {
                // push reverse direction for backtrack
				API_setColor(x, y, 'G');
                path = stackPush(path, revdir[bestDir]);
                x += dx[bestDir]; y += dy[bestDir];
                char buf[128]; sprintf(buf, "Moved to (%d,%d) heading %s", x, y, dirChar[heading]); log_text(buf);
		}
        }
    }
        if(( (x==gx1&&y==gy1) || (x==gx1&&y==gy2) || (x==gx2&&y==gy1) || (x==gx2&&y==gy2) )) {
        log_text("Goal Reached ! Returning to START using stack...");
        // Pop directions and walk back to the start
        while(!stackIsEmpty(path)) {
            int want = stackPop(&path);       // this is an absolute dir (0..3) to move next
            rotateTo(want, &heading);         // face that direction from current heading
            if(!API_moveForward()){
                break;
            }
			API_setColor(x,y,'B');
            x += dx[want];
            y += dy[want];

            char buf[128];
            sprintf(buf, "Returning: moved to (%d,%d) heading %s", x, y, dirChar[heading]);
            log_text(buf);
        }

        if(x==0 && y==0) {
            log_text("Back at START. Run complete.");
		if ((path_signature_x == previous_path_signature_x) && (previous_path_signature_y==path_signature_y && final_path_found == 0)){
			log_text("Final Path Found");
			final_path_found = 1;
		}
            API_setText(0,0,"START");
			API_clearAllColor();
        } else {
            log_text("Error - Did not reach START.");
        }
	}
	}
    return 0;
}