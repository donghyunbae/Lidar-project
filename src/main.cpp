#include <Arduino.h>
#include <FastAccelStepper.h>
#include <HardwareSerial.h>
#include <math.h>

// 핀 선언
#define IR_1 13
#define IR_2 12
#define PHOTO 14
#define STEPPER_STEP  25
#define STEPPER_DIR   26
#define STEPPER_ENABLE 27
#define MASTER_RX 33
#define MASTER_TX 32

//harmonic stepper motor (nema17)
#define STEP_ANGLE 0.06
#define MICRO_STEPPING 32

//operation mode this is following to current_state of lidar sensor
#define INIT_MODE 0
#define TRAN_MODE 1
#define TILT_MODE 2

//ref value for moving Lidarsensor
float g_freq_ref ;
float g_tilt_range_ref ;
float g_pivot_angle_ref ;
float g_freq = 0 ;
float g_tilt_range = 0 ;
float g_pivot_angle = 0 ;
int mode ;
int new_mode ;

//float g_speed_ref;
//float g_lower_angle_ref
//float g_upper_angle_ref

//flag 선언(For External Interrupt)
volatile bool ir_1_flag = false;
volatile bool ir_2_flag = false;
volatile bool photo_flag = false;
volatile bool zero_pos_set_yet = false;                           //zero_setting 완료 아직 안되었는지
volatile bool zero_pos_set_change = false;                        //zero_setting_change
volatile bool init_fun_finished = false;                          //zero 도착 완료
volatile bool tran_fun_finished = false;                          //zero 도착 완료

//Sampling time 선언
#define DESIRED_SAMPLING_TIME  10 // 10 micro second
unsigned long long g_current_sampling_time = 0;
unsigned long long g_start_sampling_time = 0;
unsigned long long g_base_time_tick = 0;
bool g_flag_sampling_time = false; // check sampling time to run program

// 시작 위치 0 선언
long g_zero_pos ;
long g_current_position ;

long target_position = 0;   //틸팅 위치변수
long Upper_target ;
long Lower_target ;
long tilt_pulse ;
long init_to_zero = 45;

// Motor 값 설정 (초기 운전)
long speed = 234; // microsec per step
int acc = 1000000; // acceleration
int msr = 32; // microstep from motordriver(TMC2225)
long pulse = 192000;

FastAccelStepperEngine engine = FastAccelStepperEngine(); // fastaccelstepper클래스의 engine 인스턴스 생성
FastAccelStepper *stepper = NULL; // 기본세팅
HardwareSerial master_serial(2);

volatile unsigned long g_timer_count = 0;
volatile bool g_is_sampling_time = false;

hw_timer_t * g_hw_timer = NULL;
portMUX_TYPE g_hw_timer_mux = portMUX_INITIALIZER_UNLOCKED;

// function prototypes
long angle_to_pulse(float angle);
float pulse_to_angle(long pulse);
void reset_system();
void check_sampling_time();
void call_current_position();
void stepper_start();
void move_to_zero();
void call_back_IR_1();
void call_back_IR_2();
void call_back_Photo();
void master_uart_rx_isr_callback();
void init_fun();
void trans_fun();
void tilt_fun();
void loop_fun();
void select_mode();

void IRAM_ATTR on_hw_timer_callback()                                           //hardware timer
{
    portENTER_CRITICAL_ISR(&g_hw_timer_mux);

    g_timer_count++;

    if (g_timer_count % DESIRED_SAMPLING_TIME == 0){
        g_is_sampling_time = true;
    }
    
    portEXIT_CRITICAL_ISR(&g_hw_timer_mux);
}

//====================================
long angle_to_pulse(float angle)
{
    return (long) round(MICRO_STEPPING * angle / STEP_ANGLE);                                                                                                              
}

//====================================
float pulse_to_angle(long pulse)
{
    return (float)((STEP_ANGLE * (float)pulse / MICRO_STEPPING));
}

//====================================
void reset_system()
{
//넘으면 다시 setup 시작s
}

//====================================
void call_current_position()
{
    g_current_position = stepper->getCurrentPosition();
    Serial.print(" Time: ");
    Serial.print(g_timer_count);
    Serial.print("  /  ");
    Serial.print("CURRENT_STEP = ");
    Serial.print(g_current_position);
    Serial.print("  /  ");
    Serial.print("CURRENT_ANGLE = ");
    Serial.print(pulse_to_angle(g_current_position));
    Serial.print("  /  ");
    Serial.print("CURRENT SPEED = ");
    Serial.print(stepper->getCurrentSpeedInUs());
    Serial.print("  /  ");
    Serial.print("MODE = ");
    Serial.print(mode);
    Serial.print("  /  ");
    Serial.print("pivot angle : ");
    Serial.println(g_zero_pos);

    master_serial.print("Time: ");
    master_serial.println(g_timer_count);

    /*
    g_current_position = stepper->getCurrentPosition();
    master_serial.print("Time: ");
    master_serial.print(g_timer_count);s
    master_serial.print(", CURRENT_STEP = ");
    master_serial.print(g_current_position);
    master_serial.print("     /     ");
    master_serial.print("CURRENT_ANGLE = ");
    master_serial.print(pulse_to_angle(g_current_position));
    master_serial.print("     /     ");
    master_serial.print("CURRENT SPEED = ");
    master_serial.println(stepper->getCurrentSpeedInUs());
    */
}

//====================================
void stepper_start()
{
    if (stepper){
        g_current_position = stepper->getCurrentPosition();

        stepper->setSpeedInUs(speed);
        stepper->setAcceleration(acc);
        stepper->runForward();
        stepper->moveTo(pulse, false);
        Serial.print("START_POSITION : ");
        Serial.print(g_current_position);
        Serial.print(" mode : ");
        Serial.println(mode);
    }else{
        Serial.println("Stepper Connection is failed.");
    }
}
//====================================
void move_to_zero()                                                             //현재 zero_point
{   
    int init_zero = 0;

    if (photo_flag){
        long LOWER_POS = stepper->getCurrentPosition();
        Serial.print("LOWER_POS : ");
        Serial.println(LOWER_POS);
        stepper->forceStop();

        if (!zero_pos_set_yet){                                                 //딱  한번만 zero_set  해줌
            init_zero = LOWER_POS - angle_to_pulse(init_to_zero);
            zero_pos_set_yet = true;
            Serial.print("will go To zero. zero_target_pos : ");
            Serial.println(init_zero);
        }

        stepper->runForward();
        stepper->moveTo(init_zero, true);
        g_zero_pos = init_zero;
        delay(100);
        
        if(stepper->getCurrentPosition() == g_zero_pos && !stepper->isRunning()){
            init_fun_finished = true;
            target_position = g_zero_pos;                                         //목표위치 초기화
            delay(1000);            
            Serial.println("Arrived ZERO position!");
            stepper->setCurrentPosition(0);
            g_zero_pos = 0;
            Serial.print("ZERO POS : ");
            Serial.println(stepper->getCurrentPosition());
        }
        photo_flag = false;
    }
}

//====================================
void IRAM_ATTR call_back_IR_1()
{
    ir_1_flag = true;
}

//====================================
void IRAM_ATTR call_back_IR_2()
{
    ir_2_flag = true;
}

//====================================
void IRAM_ATTR call_back_Photo()
{
    photo_flag = true;
}

//====================================

void IRAM_ATTR master_uart_rx_isr_callback()
{
    while(master_serial.available()) {
		Serial.println(master_serial.read());
	}
    /*
    uint16_t size = master_serial.available();
	master_serial.printf("Got %d bytes on Serial to read\r\n", size);
	while(master_serial.available()) {
		master_serial.write(master_serial.read());
	}
	master_serial.printf("\r\nSerial data processed!\r\n");
    */
}

void init_fun()
{   
    move_to_zero();
}

void trans_fun()
{   
    //g_pivot_angle = g_pivot_angle_ref;
    
    //stepper->moveTo(g_zero_pos, true);
    stepper->setSpeedInUs(234);
    stepper->moveTo(Lower_target, true);
    
    //delay(1000);
    if(stepper->getCurrentPosition() == Lower_target && !stepper->isRunning()){
        //g_zero_pos = g_zero_pos + angle_to_pulse(g_pivot_angle); // zero_pos 값을 업데이트
        Serial.println("moved to lower target");
        //Serial.println(g_zero_pos);
        //stepper->moveTo(Lower_target, true);
        //tran_fun_finished = true;
        mode = TILT_MODE;
    }
}

void set_speed()
{   
    g_freq = g_freq_ref; 
    g_tilt_range = g_tilt_range_ref
; 
    g_pivot_angle = g_pivot_angle_ref;
    

    tilt_pulse = angle_to_pulse(g_tilt_range); 
    Upper_target = g_zero_pos + angle_to_pulse(g_pivot_angle) + tilt_pulse/2;
    Lower_target = g_zero_pos + angle_to_pulse(g_pivot_angle) - tilt_pulse/2;
    
    unsigned long period = 10000 / g_freq;                                                       //f= 2, t = 1/2 = 1000000/f 
    float w = 2.0f * g_freq * g_tilt_range;                                                        // w = 2*pi*f(1Hz->360도), 따라서 w = f * tilt_range인데, 2번 반복이므로 --> , w = 2 * f * tilt_range;
    speed = (long)round(1e6/(float)angle_to_pulse(w));                               
}

void tilt_fun()
{     
    stepper->setSpeedInUs(speed);
    
    g_current_position = stepper->getCurrentPosition();
    if (g_current_position >= Upper_target){
        target_position = Lower_target;
    }else if (g_current_position <= Lower_target){
        target_position = Upper_target;
    }
    stepper->moveTo(target_position);  
    
}

void setup()
{
    // initialize interrupts 
    pinMode(IR_1, INPUT_PULLUP);
    pinMode(IR_2, INPUT_PULLUP);
    pinMode(PHOTO, INPUT);

    attachInterrupt(digitalPinToInterrupt(IR_1), call_back_IR_1, RISING);
    attachInterrupt(digitalPinToInterrupt(IR_2), call_back_IR_2, RISING);
    attachInterrupt(digitalPinToInterrupt(PHOTO), call_back_Photo, FALLING);

    // initialize serial ports: Serial for debugging and master_serial for communication with the master   
    Serial.begin(115200);
    master_serial.begin(115200, SERIAL_8N1, MASTER_RX, MASTER_TX);
    master_serial.onReceive(master_uart_rx_isr_callback);
    
    // initialize stepper motor driver 
    engine.init();
    stepper = engine.stepperConnectToPin(STEPPER_STEP);
    stepper->setDirectionPin(STEPPER_DIR, false);
    stepper->setEnablePin(STEPPER_ENABLE);
    stepper->setAutoEnable(true);
    stepper->setSpeedInUs(speed);
    stepper_start();

    int base_sampling_time = 1; // base sampling time = 1ms
    
    // hardware timer 
    // arguments for timerBegin() --> timer id: {0,1,2,3}, prescale:80 (using 80MHz crystal), {rising_edge(true), falling_edge(false)}
    // (80,000,000 / prescaler) tics / sec 
    g_hw_timer = timerBegin(0, 80, true);  

    timerAttachInterrupt(g_hw_timer, &on_hw_timer_callback, true);
    timerAlarmWrite(g_hw_timer, 1000, true);  // 1000 ticks : 1ms 
    timerAlarmEnable(g_hw_timer);
}
void generate_control_command(unsigned long cnt, float& g_freq_ref, float& g_tilt_range_ref, float& g_pivot_angle_ref)
{
    if(cnt == 1000){
        g_freq_ref = 1; g_tilt_range_ref
     = 2; g_pivot_angle_ref = 0; 

        //Serial.print("mode : ");Serial.print(mode);Serial.print(" g_freq : ");Serial.print(g_freq);Serial.print(" g_tilt_range : ");Serial.print(g_tilt_range);Serial.print(" g_current_position : "); Serial.print(g_current_position); Serial.print(" Lower_target : ");Serial.print(Lower_target); Serial.print(" g_pivot_angle : "); Serial.println(g_pivot_angle);
    }else if(cnt == 2000){
        g_freq_ref = 0.5; g_tilt_range_ref
     = 1; g_pivot_angle_ref = 10;

        //Serial.print("mode : ");Serial.print(mode);Serial.print(" g_freq : ");Serial.print(g_freq);Serial.print(" g_tilt_range : ");Serial.print(g_tilt_range);Serial.print(" g_current_position : "); Serial.print(g_current_position); Serial.print(" Lower_target : ");Serial.print(Lower_target); Serial.print(" g_pivot_angle : "); Serial.println(g_pivot_angle);
    }else if(cnt == 3000){
        g_freq_ref = 1; g_tilt_range_ref
     = 4; g_pivot_angle_ref = -30; 

        //Serial.print("mode : ");Serial.print(mode);Serial.print(" g_freq : ");Serial.print(g_freq);Serial.print(" g_tilt_range : ");Serial.print(g_tilt_range);Serial.print(" g_current_position : "); Serial.print(g_current_position); Serial.print(" Lower_target : ");Serial.print(Lower_target); Serial.print(" g_pivot_angle : "); Serial.println(g_pivot_angle);
    }else if(cnt == 4000){
        g_freq_ref = 2; g_tilt_range_ref
     = 2; g_pivot_angle_ref = 20; 

        //Serial.print("mode : ");Serial.print(mode);Serial.print(" g_freq : ");Serial.print(g_freq);Serial.print(" g_tilt_range : ");Serial.print(g_tilt_range);Serial.print(" g_current_position : "); Serial.print(g_current_position); Serial.print(" Lower_target : ");Serial.print(Lower_target); Serial.print(" g_pivot_angle : "); Serial.println(g_pivot_angle);
    }else if(cnt == 5000){
        g_freq_ref = 2; g_tilt_range_ref
     = 4; g_pivot_angle_ref = 15; 

        //Serial.print("mode : ");Serial.print(mode);Serial.print(" g_freq : ");Serial.print(g_freq);Serial.print(" g_tilt_range : ");Serial.print(g_tilt_range);Serial.print(" g_current_position : "); Serial.print(g_current_position); Serial.print(" Lower_target : ");Serial.print(Lower_target); Serial.print(" g_pivot_angle : "); Serial.println(g_pivot_angle);
    }else if(cnt == 6000){
        g_freq_ref = 1; g_tilt_range_ref
     = 6; g_pivot_angle_ref = 5; 

        //Serial.print("mode : ");Serial.print(mode);Serial.print(" g_freq : ");Serial.print(g_freq);Serial.print(" g_tilt_range : ");Serial.print(g_tilt_range);Serial.print(" g_current_position : "); Serial.print(g_current_position); Serial.print(" Lower_target : ");Serial.print(Lower_target); Serial.print(" g_pivot_angle : "); Serial.println(g_pivot_angle);
    }else if(cnt == 7000){
        g_freq_ref = 0.5; g_tilt_range_ref
     = 4; g_pivot_angle_ref = 0; 

        //Serial.print("mode : ");Serial.print(mode);Serial.print(" g_freq : ");Serial.print(g_freq);Serial.print(" g_tilt_range : ");Serial.print(g_tilt_range);Serial.print(" g_current_position : "); Serial.print(g_current_position); Serial.print(" Lower_target : ");Serial.print(Lower_target); Serial.print(" g_pivot_angle : "); Serial.println(g_pivot_angle);
    }else if(cnt == 8000){
        g_freq_ref = 2; g_tilt_range_ref
     = 1; g_pivot_angle_ref = 15; 

        //Serial.print("mode : ");Serial.print(mode);Serial.print(" g_freq : ");Serial.print(g_freq);Serial.print(" g_tilt_range : ");Serial.print(g_tilt_range);Serial.print(" g_current_position : "); Serial.print(g_current_position); Serial.print(" Lower_target : ");Serial.print(Lower_target); Serial.print(" g_pivot_angle : "); Serial.println(g_pivot_angle);
    }else if(cnt == 9000){
        g_freq_ref = 2; g_tilt_range_ref
     = 2; g_pivot_angle_ref = 10; 

        //Serial.print("mode : ");Serial.print(mode);Serial.print(" g_freq : ");Serial.print(g_freq);Serial.print(" g_tilt_range : ");Serial.print(g_tilt_range);Serial.print(" g_current_position : "); Serial.print(g_current_position); Serial.print(" Lower_target : ");Serial.print(Lower_target); Serial.print(" g_pivot_angle : "); Serial.println(g_pivot_angle);
    }
}

/*
void generate_control_command(unsigned long cnt, int *mode, float *freq, float *tilt_range)
{
    if(cnt == 1000){
        *mode = 2; *g_freq = 1; *g_tilt_range = 2; 
        Serial.print(*mode); Serial.print(", ");Serial.print(*freq);Serial.print(", ");Serial.println(*tilt_range);
    }else if(cnt == 2000){
        *mode = 2; *g_freq = 2; *g_tilt_range = 2; 
        Serial.print(*mode); Serial.print(", ");Serial.print(*freq);Serial.print(", ");Serial.println(*tilt_range);
    }else if(cnt == 3000){
        *mode = 2; *g_freq = 0.5; *g_tilt_range = 5; 
        Serial.print(*mode); Serial.print(", ");Serial.print(*freq);Serial.print(", ");Serial.println(*tilt_range);
    }
}           그치만 이 방법은 loop에서 호출 시 &(reference)기호를 붙혀주어야함
*/

//====================================
void loop()
{
    static unsigned long count = 0;

    if (g_is_sampling_time){
        //call_current_position();
        g_is_sampling_time = false;
        generate_control_command(count, g_freq_ref, g_tilt_range_ref
    , g_pivot_angle_ref);
        Serial.print("count : ");Serial.print(count);Serial.print(" / mode : ");Serial.print(mode);Serial.print(" / g_freq : ");Serial.print(g_freq);Serial.print(" / g_tilt_range : ");Serial.print(g_tilt_range);Serial.print(" / current_position : ");Serial.print(g_current_position); Serial.print(" / Lower_target : ");Serial.print(Lower_target);Serial.print(" / g_pivot_angle : "); Serial.print(g_pivot_angle);Serial.print(" / speed : "); Serial.print(speed); Serial.print(" / g_zero_pos : "); Serial.println(g_zero_pos);
        count++;
    }  
    select_mode();
    loop_fun();
}   

void select_mode()
{
    if(!init_fun_finished){
        mode = INIT_MODE;
    }else{
        if((g_freq != g_freq_ref) || (g_tilt_range != g_tilt_range_ref
    ) || (g_pivot_angle != g_pivot_angle_ref)){
            mode = TRAN_MODE;
            set_speed();
        
            //Serial.print("mode : ");Serial.print(mode);Serial.print(" / g_freq : ");Serial.print(g_freq);Serial.print(" / g_tilt_range : ");Serial.print(g_tilt_range);Serial.print(" / current_position : ");Serial.print(g_current_position); Serial.print(" / Lower_target : ");Serial.print(Lower_target);Serial.print(" / g_pivot_angle : "); Serial.print(g_pivot_angle);Serial.print(" / speed : "); Serial.print(speed); Serial.print(" / g_zero_pos : "); Serial.println(g_zero_pos);
        
            //mode = TRAN_MODE;
            //if (tran_fun_finished) {
            //    mode = TILT_MODE;
            //}
            //tran_fun_finished = false;
        }
        
        
    }

}

void loop_fun()
{
    switch (mode){
        case INIT_MODE:
            init_fun();
            break;
    
        case TRAN_MODE:
            trans_fun();
            break;

        case TILT_MODE:
            tilt_fun();
            break;
        }
}

/*          이런 방법도 있지만 포인터를 사용하면 더 쉽게 더 간결하게 코드 작성이 가능하다. !
    if(g_timer_count < 15000){
        mode = 0;
    }else if (g_timer_count >= 15000 && g_timer_count < 20000){
        mode = 2;
        g_freq = 1;
        g_tilt_range = 2;
    }else if (g_timer_count >= 20000 && g_timer_count < 25000){
        mode = 1;
        g_pivot_angle = 5;
        g_tilt_range = 2;
    }else if (g_timer_count >= 25000 && g_timer_count < 30000){
        mode = 2;
        g_freq = 2;
        g_tilt_range = 1;
    }else if (g_timer_count >= 35000 && g_timer_count < 40000){
        mode = 2;
        g_freq = 1;
        g_tilt_range = 1;
    }else if (g_timer_count >= 40000 && g_timer_count < 45000){
        mode = 1;
        g_pivot_angle = 10;
        g_tilt_range = 2;
    }else if (g_timer_count >= 45000 && g_timer_count < 50000){
        mode = 2;
        g_freq = 1;
        g_tilt_range = 2;
    }else if (g_timer_count >= 50000 && g_timer_count < 55000){
        mode = 0;
    }else if (g_timer_count >= 550000 && g_timer_count < 600000){
        stepper->moveTo(g_zero_pos);
    }
*/
