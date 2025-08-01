

#define pwmA 3
#define inA1 5
#define inA2 4
#define std 6
#define inB1 7
#define inB2 8
#define pwmB 9


float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;
float integral_error = 0;
float previous_error = 0;
float PID = 0;

const int sensor_count = 8;
double sensor_reading[sensor_count];

double cal_min[sensor_count]={100,100,100,100,100,100,100,100};
double cal_max[sensor_count]={800,800,800,800,800,800,800,800};

float weights[sensor_count/2] = {40.0,30.0,20.0,10.0};


//function signatures
void sensor_read();
void caliberate();
void pid_follower();
void line_follow();
void motor_speed();
void update_display();









void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
