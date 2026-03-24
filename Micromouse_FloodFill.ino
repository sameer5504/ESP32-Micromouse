// === Libraries and Definitions ===
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Arduino.h>

// Pins
// Motor Pins
#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14
// Encoder Pins
#define L_ENC 32
#define R_ENC 4
// LiDAR Pins and IR Pins
#define SHT_LOX1 17
#define SHT_LOX2 18
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define IR_FRONT 19
#define MAX_MAZE 8 // max maze

// Tuning Constants
const float wheel_d = 46.0;  // Wheel Diameter (mm)
const float wheel_c = PI * wheel_d;  // Wheel Circumference (mm)
const int counts_per_rev = 203;  // Encoder ticks per rev
const int cell_size = 190;  // Cell size (mm)
const int BASE_SPEED = 50;  // Forward Movement Speed (0-255)
const int MIN_DRIVE_PWM = 40;  // Minimum Forward Movement Speed
const int TURN_BASE_SPEED = 50;  // Turn Speed (0-255)
const int MIN_TURN_PWM = 24;  // Minimum Turn Speed
const float KP_YAW = 2.0;
const float TURN_KP = 1.5;
const float TURN_TOLERANCE = 0.5;  // Tolerance for angle when using MPU
const long CELL_TICKS = (long)round((cell_size / wheel_c) * counts_per_rev);  // Ticks Per Cell

// Global Variables
volatile long leftTicks = 0;
volatile long rightTicks = 0;
Adafruit_MPU6050 mpu;
float currentAngle = 0;
float gyroZ_bias = 0;
unsigned long lastTime;

// Structs for Flood Fill
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
static int stackPeek(Stack s){
    if(!s) return -1;
    return s->wantedDir;
}
// Motor and Encoders
void IRAM_ATTR leftISR(){
    leftTicks++;
}

void IRAM_ATTR rightISR(){
    rightTicks++;
}

void writePWMtoPin(int pin, int value) {
    analogWrite(pin, value);
}

void setMotor(int left, int right) {
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);
    // Left
    if(left > 0){
        writePWMtoPin(IN1, 0);
        writePWMtoPin(IN2, left);
    } else {
        writePWMtoPin(IN1, -left);
        writePWMtoPin(IN2, 0);
    }
    // Right
    if(right > 0){
        writePWMtoPin(IN3, right);
        writePWMtoPin(IN4, 0);
    } else {
        writePWMtoPin(IN3, 0);
        writePWMtoPin(IN4, -right);
    }
}

void stopMotors() {
    writePWMtoPin(IN1,0);
    writePWMtoPin(IN2,0);
    writePWMtoPin(IN3,0);
    writePWMtoPin(IN4,0);
}

// MPU
void calibrateGyro() {
    delay(1000);
    float sum = 0;
    const int samples = 600;
    for(int i=0;i<samples;i++){
        sensors_event_t a,g,t;
        mpu.getEvent(&a,&g,&t);
        sum += g.gyro.z;
        delay(2);
    }
    gyroZ_bias = sum / (float)samples;
}

void updateAngle() {
    sensors_event_t a,g,t;
    mpu.getEvent(&a,&g,&t);
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    if (dt > 0.05f) dt = 0.05f;  // clamp large jumps
    lastTime = now;
    float gyroZ = (g.gyro.z - gyroZ_bias) * 57.29577951308232f; // rad/s -> deg/s
    currentAngle += gyroZ * dt;
}

float angleError(float target, float current) {
    return target - current;
}
// Sensors (Lidar + IR)
Adafruit_VL53L0X loxLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measureLeft;
VL53L0X_RangingMeasurementData_t measureRight;
void setupLIDAR() {
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    delay(10);
    digitalWrite(SHT_LOX1, HIGH);
    loxRight.begin(LOX1_ADDRESS);
    digitalWrite(SHT_LOX2, HIGH);
    loxLeft.begin(LOX2_ADDRESS);
}

// === API Functions ===
bool API_wallFront() {
    int v = digitalRead(IR_FRONT);
    return (v == LOW);
}

bool API_wallLeft() {
    loxLeft.rangingTest(&measureLeft, false);
    return (measureLeft.RangeMilliMeter - 43) <= 150;
}

bool API_wallRight() {
    loxRight.rangingTest(&measureRight, false);
    return (measureRight.RangeMilliMeter - 43) <= 150;
}

bool API_moveForward() {
    leftTicks = 0; rightTicks = 0;
    float targetYaw = currentAngle;
    lastTime = micros();
    while( (leftTicks + rightTicks) / 2 < CELL_TICKS ) {
        updateAngle();
        float error = currentAngle - targetYaw;
        float corr = KP_YAW * error;
        int leftSpeed = (int)round(BASE_SPEED - corr);
        int rightSpeed = (int)round(BASE_SPEED + corr);
        if(abs(leftSpeed) < MIN_DRIVE_PWM){
            if(leftSpeed >= 0){
                leftSpeed = MIN_DRIVE_PWM;
            }else{
                leftSpeed = -MIN_DRIVE_PWM;
            }
        }
        if (abs(rightSpeed) < MIN_DRIVE_PWM) {
            if (rightSpeed >= 0) {
                rightSpeed = MIN_DRIVE_PWM;
            } else {
                rightSpeed = -MIN_DRIVE_PWM;
            }
        }
        setMotor(leftSpeed, rightSpeed);
    }
    stopMotors();
    delay(120);
    return true;
}

void API_turnRight() {
    float start = currentAngle;
    float target = start + 81.5f;
    lastTime = micros();
    while (fabs(angleError(target, currentAngle)) > TURN_TOLERANCE) {
        updateAngle();
        float err = angleError(target, currentAngle);
        int pwm = (int)round(fabs(err) * TURN_KP);
        pwm = constrain(pwm, MIN_TURN_PWM, TURN_BASE_SPEED);
        if (err > 0) {
            setMotor(pwm, -pwm); // Turn Right
        } else {
            setMotor(-pwm, pwm); // Reverse to target
        }
    }
    stopMotors();
    delay(40);
    int brakePwm = MIN_TURN_PWM;
    setMotor(-brakePwm, brakePwm);
    delay(60);
    stopMotors();
    delay(80);
}

void API_turnLeft() {
    float start = currentAngle;
    float target = start - 82.0f;
    lastTime = micros();
    while (fabs(angleError(target, currentAngle)) > TURN_TOLERANCE) {
        updateAngle();
        float err = angleError(target, currentAngle);
        int pwm = (int)round(fabs(err) * TURN_KP);
        pwm = constrain(pwm, MIN_TURN_PWM, TURN_BASE_SPEED);
        if (err < 0) {
            setMotor(-pwm, pwm); // Turn Left
        } else {
            setMotor(pwm, -pwm); // Reverse to correct overshoot
        }
    }
    stopMotors();
    delay(40);
    int brakePwm = MIN_TURN_PWM;
    setMotor(brakePwm, -brakePwm);
    delay(60);
    stopMotors();
    delay(80);
}
// === Flood Fill ===
static void log_text(const char* text) {
    Serial.print(text);
}
//----------DIRECTIONS-----------
// 0 = North (+Y), 1 = East (+X), 2 = South (-Y), 3 = West (-X)
static const int dx[4]     = { 0,  1,  0, -1 };
static const int dy[4]     = { 1,  0, -1,  0 };
static const int revdir[4] = { 2,  3,  0,  1 };
static const char* dirChar[4] = {"North","East","South","West"};

//-----------Rotation------------
static void rotateRight(int* h){
    API_turnRight();
    *h = (*h + 1) & 3;
}
static void rotateLeft(int* h){
    API_turnLeft();
    *h = (*h + 3) & 3;
}
static void rotateTo(int target, int* h) {
    if (target < 0 || target > 3) return;
    int diff = (target - *h + 4) & 3;
    if (diff == 0) return;
    else if (diff == 1) rotateRight(h);
    else if (diff == 2) { rotateRight(h); rotateRight(h); }
    else if (diff == 3) rotateLeft(h);
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
}
//------ Sensing the walls ------
// Sensor mapping: front -> heading
// left  -> (heading + 3) & 3 (counter-clockwise)
// right -> (heading + 1) & 3 (clockwise)

static void senseAndUpdateWalls(int W,int H,int x,int y,int heading, unsigned char walls[MAX_MAZE][MAX_MAZE][4]){
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
static void recomputeFlood(int W,int H,int cost[MAX_MAZE][MAX_MAZE], unsigned char walls[MAX_MAZE][MAX_MAZE][4]){
    const int INF = 1<<28;
    for(int yy=0; yy<H; ++yy) for(int xx=0; xx<W; ++xx) cost[yy][xx] = INF;

    // center cells (goal) - set to 0 and enqueue
    int gx1 = W/2, gx2 = gx1 - 1;
    int gy1 = H/2, gy2 = gy1 - 1;
    int qx[MAX_MAZE * MAX_MAZE], qy[MAX_MAZE * MAX_MAZE];
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
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(IN1,OUTPUT);
    pinMode(IN2,OUTPUT);
    pinMode(IN3,OUTPUT);
    pinMode(IN4,OUTPUT);
    pinMode(L_ENC, INPUT_PULLUP);
    pinMode(R_ENC, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(L_ENC), leftISR, RISING);
    attachInterrupt(digitalPinToInterrupt(R_ENC), rightISR, RISING);
    setupLIDAR();
    Wire.begin(21,22);
    if(!mpu.begin()){
        Serial.println("MPU6050 not found!");
        while(1);
    }
    calibrateGyro();
    lastTime = micros();
    Serial.println("READY");
}

void loop() {
    log_text("Running Using Flood Fill Algorithm...");
    delay(1000);
    Serial.print("1.");
    delay(1000);
    Serial.print("2.");
    delay(1000);
    Serial.print("3.");
    // Calculate Goal Coordinates
    const int W = 8;
    const int H = 8;
    int gx1 = W/2;
    int	gx2 = gx1 - 1;
    int gy1 = H/2;
    int	gy2 = gy1 - 1;
    static int cost[MAX_MAZE][MAX_MAZE];
    static unsigned char walls[MAX_MAZE][MAX_MAZE][4]; // 1=blocked, 0=open
    static unsigned char visited[MAX_MAZE][MAX_MAZE];
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
            Serial.println(strLog);			i++;
        }else {
            sprintf(strLog, "=== RUN #%d WITHOUT SENSORS ===", i2);
            Serial.println(strLog);			i2++;
        }
        log_text(strLog);

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
                int nx = x + dx[d];
                int ny = y + dy[d];
                if(nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                if(cost[ny][nx] < bestCost){
                    bestCost = cost[ny][nx];
                    bestDir = d;
                }
            }

            int direction_if_backtracking = stackPeek(path);
            if (direction_if_backtracking >= 0 && bestDir == direction_if_backtracking) {
                stackPop(&path);
                rotateTo(direction_if_backtracking, &heading);
                x += dx[direction_if_backtracking];
                y += dy[direction_if_backtracking];
                char buf[128];
                sprintf(buf, "Backtracked to (%d,%d) heading %s", x, y, dirChar[heading]);
                log_text(buf);
                API_moveForward();
            }
            else {
                // go to bestDir
                rotateTo(bestDir, &heading);
                if(!API_moveForward()){
                    // unexpected blocked
                    walls[y][x][bestDir] = 1;
                    int nx = x + dx[bestDir], ny = y + dy[bestDir];
                    if(nx >= 0 && ny >= 0 && nx < W && ny < H) walls[ny][nx][revdir[bestDir]] = 1;
                    recomputeFlood(W,H,cost,walls);
                } else {
                    // push reverse direction for backtrack
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
            } else {
                log_text("Error - Did not reach START.");
            }
        }
    }
}