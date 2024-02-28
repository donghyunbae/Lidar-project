#include <Arduino.h>
#include <FastAccelStepper.h>

// 핀 선언
#define IR_1 13
#define IR_2 12
#define PHOTO 14
#define STEPPER_STEP  25
#define STEPPER_DIR   26
#define STEPPER_ENABLE 27

#define STEP_ANGLE 0.06
#define MICRO_STEPPING 32

// Flag 선언(For External Interrupt)
volatile bool IR_1_flag = false;
volatile bool IR_2_flag = false;
volatile bool Photo_flag = false;
volatile bool zero_POS_set_yet = false;         //zero_setting 완료 아직 안되었는지
volatile bool zero_POS_set_change = false;           //zero_setting_change
volatile bool moved_to_zero_finished = false; //zero 도착 완료

// Sampling time 선언
#define DESIRED_SAMPLING_TIME  10 // 10 micro second
unsigned long long g_current_sampling_time = 0;
unsigned long long g_start_sampling_time = 0;
unsigned long long g_base_time_tick = 0;
bool g_flag_sampling_time = false; // check sampling time to run program

// 시작 위치 0 선언
long zero_POS;
long CURRENT_POSITION ;
long tilting_POS;
long target_position = 0;   //틸팅 위치변수
long init_to_zero = 45;

// Motor 값 설정 (초기 운전)
long speed = 234; // microsec per step
int Acc = 1000000; // acceleration
int MSR = 32; // microstep from motordriver(TMC2225)
long pulse = 192000;

FastAccelStepperEngine engine = FastAccelStepperEngine(); // fastaccelstepper클래스의 engine 인스턴스 생성
FastAccelStepper *stepper = NULL; // 기본세팅

//====================================
long angle_to_pulse(float angle)
{
    return (long)(MICRO_STEPPING * angle / STEP_ANGLE);                                                                                                              
}

//====================================
float pulse_to_angle(long pulse)
{
    return (float)(STEP_ANGLE * (float)pulse / MICRO_STEPPING);
}

//====================================
void reset_system()
{
//넘으면 다시 setup 시작

}


//====================================
void check_sampling_time()
{
    g_current_sampling_time = micros();
    if (g_current_sampling_time - g_start_sampling_time > DESIRED_SAMPLING_TIME){
        g_flag_sampling_time = true;
        g_base_time_tick++;
        g_start_sampling_time = g_current_sampling_time;
    }
}

//====================================
void call_current_position()
{
    CURRENT_POSITION = stepper->getCurrentPosition();
    static unsigned long time_tick = 0;                                          //time_tick 선언
    if (g_base_time_tick == 0){ //딱 처음 0일때 time_tick도 0
        time_tick = 0;
    }
    if (time_tick % 1000 == 0){                                                 //10ms.
        Serial.print("CURRENT_STEP = ");
        Serial.print(CURRENT_POSITION);
        Serial.print("     /     ");
        Serial.print("CURRENT_ANGLE = ");
        Serial.print(pulse_to_angle(CURRENT_POSITION));
        Serial.print("     /     ");
        Serial.print("CURRENT SPEED = ");
        Serial.println(stepper->getCurrentSpeedInUs());

    }
    time_tick++;
}

//====================================
void stepper_start()
{
    if (stepper){
        CURRENT_POSITION = stepper->getCurrentPosition();

        stepper->setSpeedInUs(speed);
        stepper->setAcceleration(Acc);
        stepper->runForward();
        stepper->moveTo(pulse, false);
        Serial.print("START_POSITION : ");
        Serial.println(CURRENT_POSITION);
    }else{
        Serial.println("Stepper Connection is failed.");
    }
}
//====================================
void move_to_zero()                                                             //현재 zero_point
{   
    if (Photo_flag){
        long LOWER_POS = stepper->getCurrentPosition();
        Serial.print("LOWER_POS : ");
        Serial.println(LOWER_POS);
        stepper->forceStop();

        if (!zero_POS_set_yet){                                                 //딱  한번만 zero_set  해줌
            zero_POS = LOWER_POS - angle_to_pulse(init_to_zero);
            zero_POS_set_yet = true;
            Serial.print("will go To zero. zero_target_pos : ");
            Serial.println(zero_POS);
        }

        stepper->runForward();
        stepper->moveTo(zero_POS, true);
        delay(100);
        if(stepper->getCurrentPosition() == zero_POS && !stepper->isRunning()){
            moved_to_zero_finished = true;
            target_position = zero_POS;                                         //목표위치 초기화
            delay(1000);            
            Serial.println("Arrived ZERO position!");
            stepper->setCurrentPosition(0);
            zero_POS = 0;
            Serial.print("ZERO POS : ");
            Serial.println(stepper->getCurrentPosition());
        }
        Photo_flag = false;
    }
}

//====================================

void start_tilting_mode_1(float frequency, float tilt_range)                    //mode_1 is moving to zero position
{   
    static unsigned long time_tick = 0;
    if (g_base_time_tick == 0){
        time_tick = 0;
    }

    long tilt_pulse = angle_to_pulse(tilt_range);
    unsigned long period = 100000 / frequency;                                  // 주기 = 1/주파수 (micro)
    //long speed = 1000000 / frequency / tilt_pulse;
    
    if(moved_to_zero_finished){
        long speed = 1000000 / frequency / tilt_pulse ;                             // ..  
        stepper->setSpeedInUs(speed);
        long Upper_target = zero_POS + tilt_pulse;
        long Lower_target = zero_POS - tilt_pulse;
        if (time_tick >= period){
            CURRENT_POSITION = stepper->getCurrentPosition();
            if (CURRENT_POSITION >= Upper_target){
                target_position = Lower_target;
            }else if (CURRENT_POSITION <= Lower_target){
                target_position = Upper_target;
            }
            stepper->moveTo(target_position);
            time_tick = 0;
        }
    time_tick++;    
    }
}

void start_tilting_mode_2(float frequency, float tilt_range, float set_zero)                       
{    
    static long zero_POS ; 
    long tilt_pulse = angle_to_pulse(tilt_range);
    unsigned long period = 10000 / frequency ;                                                 //f= 2, t = 1/2 = 1000000/f 
    
    static unsigned long time_tick = 0;
    if (g_base_time_tick == 0){
        time_tick = 0;
    }

    if (!zero_POS_set_change && zero_POS_set_yet){                                                 //zero_set_change
        zero_POS = zero_POS + set_zero; // zero_POS 값을 업데이트
        Serial.print("will go To zero. New_zero_target_pos : ");
        Serial.println(zero_POS);
        stepper->moveTo(zero_POS, true);

        if(stepper->getCurrentPosition() == 0 && !stepper->isRunning()){
            moved_to_zero_finished = true;
            delay(200);
            Serial.println("Arrived ZERO position!");
            stepper->setCurrentPosition(0);
            Serial.print("ZERO POS : ");
            Serial.println(stepper->getCurrentPosition());
            zero_POS = 0;
            zero_POS_set_change = true;
        }
        
    } 

    if(moved_to_zero_finished){
        long speed = 1000000 / frequency / 2 / tilt_pulse;                             
        stepper->setSpeedInUs(speed);
        long Upper_target = zero_POS + tilt_pulse;
        long Lower_target = zero_POS - tilt_pulse;
        if (time_tick >= period){                                                                       //1tick = 10micros. period :10000 , f = 2 -> 50000 micros (50ms)
            CURRENT_POSITION = stepper->getCurrentPosition();
            if (CURRENT_POSITION >= Upper_target){
                target_position = Lower_target;
            }else if (CURRENT_POSITION <= Lower_target){
                target_position = Upper_target;
            }
            stepper->moveTo(target_position);
            time_tick = 0;
        }
    time_tick++;    
    }
}


//====================================
void call_back_IR_1()
{
    IR_1_flag = true;
}

//====================================
void call_back_IR_2()
{
    IR_2_flag = true;
}

//====================================
void call_back_Photo()
{
    Photo_flag = true;
}

//====================================

void setup()
{
    Serial.begin(115200);
    pinMode(IR_1, INPUT_PULLUP);
    pinMode(IR_2, INPUT_PULLUP);
    pinMode(PHOTO, INPUT);

    attachInterrupt(digitalPinToInterrupt(IR_1), call_back_IR_1, RISING);
    attachInterrupt(digitalPinToInterrupt(IR_2), call_back_IR_2, RISING);
    attachInterrupt(digitalPinToInterrupt(PHOTO), call_back_Photo, FALLING);

    engine.init();
    stepper = engine.stepperConnectToPin(STEPPER_STEP);
    stepper->setDirectionPin(STEPPER_DIR, false);
    stepper->setEnablePin(STEPPER_ENABLE);
    stepper->setAutoEnable(true);
    stepper->setSpeedInUs(speed);
    stepper_start();
}

//====================================
void loop()
{   
    check_sampling_time();
    
    if(g_flag_sampling_time){
        call_current_position();
    }
    move_to_zero();
    //start_tilting_mode_1(2, 5);
    start_tilting_mode_2(2, 2, 0);                                //test 필요함.
}

/*
틸트 범위를 넘어갔을때 reset_system 함수 구현
적외선 센서 감지 구현

이 코드를 더 최적화시킬 방법은 무엇일까 ?

1. 싱크 안맞추고 동작   초기 zero포지션에서 동작
2. 싱크 맞추고 동작     그냥 tilt
*/



//asdfasfadsdf