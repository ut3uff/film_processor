
#include <LovyanGFX.hpp>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EasyBuzzer.h>
//#include <ezBuzzer.h>
//#include <CuteBuzzerSounds.h>

#undef max //for "vector" include
#include <vector>
#include <math.h>

#include "Sk_Modernist_Bold245pt7b.h"
#include "Sk_Modernist_Regular36pt7b.h"
#include "Sk_Modernist_Regular11pt7b.h"
#include "Sk_Modernist_Regular9pt7b.h"
#include "Sk_Modernist_Regular5pt7b.h"
#include "Sk_Modernist_Regular4pt7b.h"
#include "Sk_Modernist_Regular3pt7b.h"

#include "spritesheet_icons.h"

//#include "soc/rtc_io_reg.h"
#include <XT_DAC_Audio.h>
#include "SoundData.h"

//if harware RC filter is used - can be implemented the encoder too
#define EB_FAST 10     
#define EB_DEB 50      
#define EB_CLICK 400   
#include <EncButton.h>

#define PROGRAM_VERSION                     4.1

// time correction
#define _SEC_per_MINUTE_                      60L        // Only redefine minute for debug/tests
#define _mS_per_SEC_                          1000L
#define _mS_per_MINUTE_                       _mS_per_SEC_ * _SEC_per_MINUTE_
#define _uS_per_mS_                           1000L

#define log_1(x) (Serial.print(millis()), Serial.print("\t"), Serial.println(x))
#define log_2(x, y) (Serial.print(millis()), Serial.print("\t"), Serial.print(x), Serial.println(y))
#define log_3(x, y, z) (Serial.print(millis()), Serial.print("\t"), Serial.print(x), Serial.print(y), Serial.println(z))
#define log_4(x, y, z, w) (Serial.print(millis()), Serial.print("\t"), Serial.print(x), Serial.print(y), Serial.print(z), Serial.println(w))
#define log_6(x, y, z, w, a, b) (Serial.print(millis()), Serial.print("\t"), Serial.print(x), Serial.print(y), Serial.print(z), Serial.print(w), Serial.print(a), Serial.println(b))



typedef int32_t (*t_ptr_Algorithm)(int32_t, float);

const float exp_e = -0.1;                      // oluline constant :)
const float exp_std_temp = 24; 
#define P1DEV_STD_TEMPERATURE  	20.0                // default expected temperature
#define P2DEV_STD_TEMPERATURE  	38.0   
const	float P2DEV_COEFF1[] = { -0.5419, 6.6523, -26.77, 90.114 };        
const	float P2DEV_COEFF2[] = { -0.0301, 0.2719, -0.8618 };             
#define P2BLIX_STD_TEMPERATURE  38.0                // default expected temperature
#define P2BLIX_CONSTANT					4
#define P3DEV_STD_TEMPERATURE  	38.0                // default expected temperature
const int32_t correctionApplyTime_1_ms = 20 * _mS_per_SEC_;   // correct exp time @ 20 seconds since start
int32_t 			correctionApplyTime_2_ms = 2 * correctionApplyTime_1_ms; 	// second correct exp time since start
#define SECOND_CORECTION_TIMESTAMP_PERCENT 33
bool tempCorrectionApplied_1 = false;
bool tempCorrectionApplied_2 = false;
int32_t timeAdjust;


// ================================================================================================================
// Pins, ESP-32
// ================================================================================================================
#define PIN_SWITCH_1_UP          	13
#define PIN_SWITCH_2_RIGHT        12
#define PIN_SWITCH_3_DOWN         14
#define PIN_SWITCH_4_BACK       	27
#define PIN_SWITCH_5_START       	26
#define PIN_SWITCH_6_POWER        35

#define PIN_ENCODER_SWITCH_ENTER  22
#define PIN_ENCODER_CLK           19
#define PIN_ENCODER_DT            21
#define PIN_POWER_DCDC            16
#define PIN_VOLTAGE_SENSE         34
#define PIN_BUZZER                25
#define PIN_ONE_WIRE_DS18B20      15  
#define PIN_MOTOR_EN              33
#define PIN_MOTOR_PHS             32


RTC_DATA_ATTR int bootCount = 0;
#define BUTTON_PIN_BITMASK 0x800000000 // 2^35 in hex
// #define BUTTON_EXT1_PIN GPIO_NUM_35
#define BUTTON_EXT1_PIN GPIO_NUM_26
bool GO_TO_SLEEP = false;
uint32_t inactiveTime_s = 0;
uint32_t lastPressTime_ms = 0;
#define INACTIVITY_THRESHHOLD_s 	 10*_SEC_per_MINUTE_
// ================================================================================================================
// UI
// ================================================================================================================
enum Menu {
	MENU_SPLASH,           
	MENU_STORED_PROGRAMS,
	MENU_PROGRAM_OVERVIEW,	
	MENU_DEVELOP       
};

Menu current_menu = MENU_SPLASH;

// BUTTONS
enum Buttons {
  BUTTON_UP,
  BUTTON_RIGHT,
  BUTTON_DOWN,
  BUTTON_BACK,
  BUTTON_START,
  BUTTON_POWER,
	BUTTON_ENC_CW,
	BUTTON_ENC_CCW
};

//Colors 24bit RGB888
// #define COLOR24_BLACK 						0x000000U
// #define COLOR24_WHITE 						0xFFFFFFU
// #define COLOR24_YELLOW 						0xFFAA00U
// #define COLOR24_LIGHT_GREY 				0xCCCCCCU 
// #define COLOR24_DARK_GREY 				0x666666U 
// #define COLOR24_RED 							0xCC3333U 
// #define COLOR24_TEST 							0x3333CCU 


//Colors 16bit RGB565
#define COLOR16_BLACK 						(uint16_t)0x0000
#define COLOR16_WHITE 						(uint16_t)0xFFFF
#define COLOR16_YELLOW 						(uint16_t)0xFE40 
#define COLOR16_LIGHT_GREY 				(uint16_t)0xCE79 
#define COLOR16_DARK_GREY 				(uint16_t)0x632C 
#define COLOR16_RED 							(uint16_t)0xC986 
#define COLOR16_TEST							(uint16_t)0b0011100011101111


// Sizes
#define MENU_HEADER_HEIGHT               60
#define MENU_HEADER_TAB		               80
#define MENU_SPACE_HEIGHT                5
#define MENU_ENTRY_HEIGHT                30
#define MENU_ENTRY_DEFAULT_WIDTH         35
#define MENU_PROCESS_TIMES_TOP_PADDING   25
#define MENU_LEFT_PADDING                4 
#define MENU_LEFT_PADDING_TIME           45
#define MENU_LEFT_PADDING_AGITATION      89

#define MENU_DISPLAYED_PROGRAMS_COUNT		 5

// ================================================================================================================
// Temperature sensor
// ================================================================================================================
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(PIN_ONE_WIRE_DS18B20);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;

#define TEMPERATURE_RESET_VALUE   999.0f
float temperature_current_c = TEMPERATURE_RESET_VALUE;
float temperature_old_c = TEMPERATURE_RESET_VALUE;
bool temperature_acquired_at_least_once = false;

// ================================================================================================================
// Develop programs defs
// ================================================================================================================

enum Agitation
{
	AGITATION_OFF,
	AGITATION_STICK,
	AGITATION_ROTATIONAL
};

enum Compensation
{
	COMPENSATION_OFF,
	COMPENSATION_BW_DEV,
	COMPENSATION_TETENAL_DEV,
	COMPENSATION_TETENAL_BLIX,
	COMPENSATION_CINESTILL_DEV
};

enum IconColor
{
	BACKGROUND_RED,
	BACKGROUND_RED_YELLOW_ICON,
	BACKGROUND_BLACK,
	BACKGROUND_GREY
};


enum StepParameter
{
	NONE,
	DURATION,
	AGITATION,
	COMPENSATION
};

StepParameter selectedColumn = NONE;

typedef struct{
   uint32_t number;        
   char name[20];
   int32_t duration_s;        
   Agitation agitation;
	 Compensation compensation;
   bool isSelected;             
}t_STEP;

//////////////////
#define PROGRAMS_COUNT          				3  
#define DEFAULT_SELECTED_PROGRAM				0
#define DEFAULT_SELECTED_STEP						0

uint32_t idSelectedProgram = DEFAULT_SELECTED_PROGRAM;
uint32_t idDisplayTopProgram = 0;
uint32_t idSelectedStep = DEFAULT_SELECTED_STEP;
uint32_t idDisplayTopStep = 0;
uint32_t idActiveStep = 0;
bool 		 developPaused = false;


typedef struct{
   uint32_t number;        
   char name_index[6];  // for menu entries
   char name_short[10];  // for menu headers
   char name_long[20];   // for menu entries
	 int32_t step_count;
	 std::vector<t_STEP> step;
   bool isSelected;             
}t_PROGRAM;

t_PROGRAM program[PROGRAMS_COUNT];
t_PROGRAM defaultProgram[PROGRAMS_COUNT];
t_PROGRAM activeProgram;

#define MAX_DURATION_S  99*_SEC_per_MINUTE_+55
#define TIMESTEP_S			5
// ================================================================================================================
// CONST process step defs
// ================================================================================================================

const char PROGRAM_NAMES_SHORT [12][10] =  { "B&W", "C-41", "C-41", "C-43", "C-44", "C-45", "C-46", "C-47", "C-48", "C-49", "C-50", "C-51" };
const char PROGRAM_NAMES_ADD [12][20] =  { " -Ilford", " -Tetenal", " -Cinestill" };

#define  P1_STEPS		4
#define  P2_STEPS		5
#define  P3_STEPS		5

const char P1_STEP_NAMES [P1_STEPS][20] =  { "DEV", "STOP", "FIX", "RINSE" };
const char P2_STEP_NAMES [P2_STEPS][20] =  { "HEAT", "DEV", "BLIX", "RINSE", "STAB" };
const char P3_STEP_NAMES [P3_STEPS][20] =  { "HEAT", "DEV", "BLIX", "RINSE", "STAB" };

const int32_t P1_STEP_DURATION [P1_STEPS] =  {  10*_SEC_per_MINUTE_, 1*_SEC_per_MINUTE_, 		7*_SEC_per_MINUTE_, 15*_SEC_per_MINUTE_ };
const int32_t P3_STEP_DURATION [P3_STEPS] =  {	1*_SEC_per_MINUTE_, 3*_SEC_per_MINUTE_+30,	8*_SEC_per_MINUTE_,  3*_SEC_per_MINUTE_, 1*_SEC_per_MINUTE_ };
const int32_t P2_STEP_DURATION [P2_STEPS] =  { 	5*_SEC_per_MINUTE_, 3*_SEC_per_MINUTE_+15, 	4*_SEC_per_MINUTE_,  6*_SEC_per_MINUTE_, 1*_SEC_per_MINUTE_ };

const Agitation P1_AGITATION[P1_STEPS] = { AGITATION_ROTATIONAL, 	AGITATION_STICK, 				AGITATION_ROTATIONAL, 	AGITATION_OFF };
const Agitation P2_AGITATION[P2_STEPS] = { AGITATION_OFF, 				AGITATION_ROTATIONAL, 	AGITATION_ROTATIONAL, 	AGITATION_OFF, 	AGITATION_OFF };
const Agitation P3_AGITATION[P3_STEPS] = { AGITATION_OFF, 				AGITATION_ROTATIONAL, 	AGITATION_ROTATIONAL, 	AGITATION_OFF, 	AGITATION_OFF };

const Compensation P1_COMPENSATION[P1_STEPS] = { COMPENSATION_BW_DEV, 	COMPENSATION_OFF, 					COMPENSATION_OFF, 					COMPENSATION_OFF };
const Compensation P2_COMPENSATION[P2_STEPS] = { COMPENSATION_OFF, 		COMPENSATION_TETENAL_DEV, 	COMPENSATION_TETENAL_BLIX, 	COMPENSATION_OFF, 	COMPENSATION_OFF };
const Compensation P3_COMPENSATION[P3_STEPS] = { COMPENSATION_OFF, 		COMPENSATION_CINESTILL_DEV, COMPENSATION_OFF, 					COMPENSATION_OFF, 	COMPENSATION_OFF };
// ================================================================================================================
// Develop process defs
// ================================================================================================================

#define TIMER_INPUT_ISR_STEP_uS                  20*_uS_per_mS_

volatile int32_t current_step_time_left_ms = 0;

//BATTTERY
#define GET_SOC_TIMEOUT 20*_mS_per_SEC_
uint32_t lastProcessBatterySoc = 0;
int32_t  batteryVoltage_mv = 0;
int32_t batterySoc = 50; //percentage
bool isCharging = false;

//Buzzer
#define BUZZER_ON_DURATION 300
#define BUZZER_OFF_DURATION 300
#define BUZZER_FREQ 2500
int32_t beepCount = 1;
bool beepFirstDone  = false,
		 beepSecondDone = false;

XT_Wav_Class ForceWithYou(notification);    // create an object of type XT_Wav_Class that is used by 
																						// the dac audio class (below), passing wav data as parameter.                                    
XT_DAC_Audio_Class DacAudio(PIN_BUZZER,1);  // Create the main player class object. 
																						// Use GPIO 25, one of the 2 DAC pins and timer 1
XT_Sequence_Class Sequence;                // The sequence object, you add your sounds above to this object (see setup below)


// Motor program
#define MOTOR_PROGRAM_ROTATIONAL_STEPS_COUNT        5
#define MOTOR_PROGRAM_STICK_STEPS_COUNT             3

enum MotorDirection {
	M_STP,
	M_FWD,
	M_REV
};

int32_t motorProgramRotationalPeriod_ms;
int32_t motorProgramStickPeriod_ms;

typedef struct{
   int32_t duration_ms;        
   MotorDirection motor_state;
}MOTOR_PROGRAM;
MOTOR_PROGRAM motorProgramStick[MOTOR_PROGRAM_STICK_STEPS_COUNT];
MOTOR_PROGRAM motorProgramRotational[MOTOR_PROGRAM_ROTATIONAL_STEPS_COUNT];

uint32_t motorTimeFromStart_ms = 0;

// ================================================================================================================
// Hardware timer for switches (b1...b2: worked a bit better than attaching all keys to interrupts)
// ================================================================================================================
hw_timer_t * timer_input = NULL;
portMUX_TYPE timer_Mux = portMUX_INITIALIZER_UNLOCKED;


// =========================================    }else if (current_menu == MENU_STORED_PROGRAMS){=======================================================================
// Main setup
// (A lot of it graphics)
// ================================================================================================================
//EncButton encoder library object
//EncButton<EB_TICK, PIN_ENCODER_CLK, PIN_ENCODER_DT, PIN_ENCODER_SWITCH_ENTER> enc;  // энкодер с кнопкой <A, B, KEY>
EncButton<EB_CALLBACK, PIN_SWITCH_6_POWER> buttonPower;       // 


// ESP32 SPI
struct LGFX_Config{
  static constexpr spi_host_device_t spi_host = VSPI_HOST;
  static constexpr int dma_channel = 1;
  static constexpr int spi_sclk = 18;
  static constexpr int spi_mosi = 23;
  static constexpr int spi_miso = -1;
  static constexpr int spi_dlen = 8;
	static constexpr bool spi_3wire  = false; 
};

static lgfx::LGFX_SPI<LGFX_Config> lcd;
static lgfx::Panel_ILI9341 panel;

LGFX_Sprite framebuffer_sprite(&lcd);
LGFX_Sprite icon30_sprite(&framebuffer_sprite);

void IRAM_ATTR on_timer_input_isr(){
  portENTER_CRITICAL_ISR(&timer_Mux);
  process_physical_switches();          // Crude hack for quick responsiveness
  process_encoder();
	if (!developPaused){
		if (current_step_time_left_ms >= TIMER_INPUT_ISR_STEP_uS/_uS_per_mS_){
			current_step_time_left_ms -= TIMER_INPUT_ISR_STEP_uS/_uS_per_mS_;
			motorTimeFromStart_ms += TIMER_INPUT_ISR_STEP_uS/_uS_per_mS_;
		}else{
			current_step_time_left_ms = 0;
		}
	}
  portEXIT_CRITICAL_ISR(&timer_Mux);  
}

void setup(void){
  Serial.begin(115200);
	 ++bootCount;
	 GO_TO_SLEEP = false;

	 print_wakeup_reason();
	 if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) init_pins();
	 
  log_1("AGO film processor starting up..");

  // Assign various setting values to the panel class.
  // Set the SPI clock for normal write operation.
  
  //                b1: 10000000
  // 2021-05-25/ev: b2: 30000000 in an attempt to equalize the frequencies,
  //                             this fixes some annoying glitches when menu is being updated now faster
  panel.freq_write = 50000000; 
  // Set the SPI clock for fill write operation.
  // It may work even if you set the clock higher than freq_write.
  panel.freq_fill  = 50000000;  // b1 & b2: 30000000
  // Set the SPI clock for read operation.
  panel.freq_read  = 16000000;
  // Set the SPI mode. (0~3)
  panel.spi_mode = 0;
  // Set the SPI mode when read operation. (0~3)
  panel.spi_mode_read = 0;
  // Sets the number of dummy bits for pixel readout.
  panel.len_dummy_read_pixel = 8;
  // Set the readability of the data. If reading of the data is not possible, set false.
  panel.spi_read = false;
  // Set to "true" for a panel that uses MOSI pins to read data.
  panel.spi_3wire = false;
  // Set the pin number for connecting the CS pins of the LCD.
  panel.spi_cs = 5;
  // Set the pin number for connecting the D/C pins of the LCD.
  panel.spi_dc = 2;  // "AO" 
  // Set the pin number for connecting the RST pins of the LCD.
  panel.gpio_rst = 4;
  // Set the backlight pin number.
  panel.gpio_bl  = 34;  // N/A
  // Set the backlight control PWM channel number.
  panel.pwm_ch_bl = 7;  // N/A
  // Set the backlight level (rue=turns on HIGH / false=turns on LOW)
  panel.backlight_level = true;
  // Set the panel color inversion.
  panel.reverse_invert = false;  // Set the RGB/BGR color order.
  panel.rgb_order = false;  // Set the internal memory size of the LCD driver.
  panel.memory_width  = 240;
  panel.memory_height = 320;  // Set the size of the pixels that can be displayed on the LCD panel.
  panel.panel_width  = 240;
  panel.panel_height = 320;
	
	  // Set the number of offset pixels.
  panel.offset_x = 0;
  panel.offset_y = 0;  // Set the default rotation number.
  panel.rotation = 3;  // Set the number of rotation offset number.
  panel.offset_rotation = 0;  // After setting up, you can pass the panel pointer to the lcd.setPanel function.
  lcd.setPanel(&panel);  // Initializing the SPI bus and panel will make it available.
  lcd.init();  
  lcd.startWrite();
  lcd.setColorDepth(16);
  lcd.setTextFont(2);
  
  //framebuffer_sprite.setBuffer(const_cast<std::uint8_t*>(info), 32, 32, 16);
  framebuffer_sprite.setColorDepth(16);
  framebuffer_sprite.setTextFont(2);
  framebuffer_sprite.createSprite(320, 120);
	
	icon30_sprite.setColorDepth(16);
  icon30_sprite.setTextFont(1);
  //icon30_sprite.createSprite(30, 30);
	//log_1("Sprite created");
	
	
  init_process_program_default_values();
  init_motor_program_default_values();
	init_pins();
  
  // Start the DS18B20 sensor
  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, 12);
  process_temperature();
    
  
  timer_input = timerBegin(0, 80, true);
  timerAttachInterrupt(timer_input, &on_timer_input_isr, true);
  timerAlarmWrite(timer_input, TIMER_INPUT_ISR_STEP_uS, true);
  timerAlarmEnable(timer_input);

	if (idSelectedProgram >= MENU_DISPLAYED_PROGRAMS_COUNT) 
		idDisplayTopProgram = idSelectedProgram;
	if (PROGRAMS_COUNT - idDisplayTopProgram < MENU_DISPLAYED_PROGRAMS_COUNT) 
		idDisplayTopProgram = PROGRAMS_COUNT - MENU_DISPLAYED_PROGRAMS_COUNT;

batterySoc = getBatterySoc();
//EasyBuzzer.singleBeep(BUZZER_FREQ, 5000);

//enc.setEncType(EB_HALFSTEP);
//sb.init();
//cute.init(PIN_BUZZER);
//cute.play(S_MODE1);
	log_1("Setup ended.");

}

// ================================================================================================================
// Main loop
//    Updates temperature (if needed to display)
//    Updates UI graphics
// Keys processed using the timer and encoder interrupts
// ================================================================================================================

long menu_update_timer_ms = 0;
#define MENU_UPDATE_TIMEOUT_mS  30
volatile uint32_t loopCount = 0;
volatile uint32_t loopCounter = 0;
uint32_t everySecondTime = 0;

void loop(void){
  uint32_t now_ms = millis();
	loopCount++;
  if (now_ms - everySecondTime > 1000){
    everySecondTime = now_ms;
		loopCounter = loopCount;
		loopCount = 0;
		//log_1("Updating UI");
  }	

  //EasyBuzzer.update();

	buttonPower.tick();

	processBatterySoc();

	 DacAudio.FillBuffer();                // Fill the sound buffer with data
  // if(ForceWithYou.Playing==false)       // if not playing,
    // DacAudio.Play(&ForceWithYou);       // play it, this will cause it to repeat and repeat...
		
  //2021-05-25/ev b2: Acquire temperature only when a develop program is running
   //                 1-Wire acquistion default methods are slow
  if (current_menu == MENU_DEVELOP){
		update_develop_process();
    process_temperature();   
  }

  if (now_ms - menu_update_timer_ms > MENU_UPDATE_TIMEOUT_mS){
    menu_update_timer_ms = now_ms;
		processInactivityTimer();
    update_ui();
		draw_service();
		//log_1("Updating UI");
  }
	
	if (GO_TO_SLEEP) {
		goToSleep();
	}
}

void update_ui(){
    
		switch (current_menu)
		{
		case MENU_SPLASH:
      motor_stop();
      update_menu_splash();
			break;
		case MENU_STORED_PROGRAMS:
      motor_stop();
      update_menu_stored_programs();
				// if(ForceWithYou.Playing==false)       // if not playing,
				// DacAudio.Play(&ForceWithYou); 			
			break;
		case MENU_PROGRAM_OVERVIEW:
      motor_stop();
      update_menu_program_overview();
			break;
		case MENU_DEVELOP:
      update_menu_develop();
      update_develop_process();
			break;
		default:
      motor_stop();
			break;
		}		
}

long temperature_request_time_ms = 0;
bool temp_requested = false;
#define TEMPERATURE_REQUEST_TIME_PERIOD_mS    1000

void process_temperature(){
    // 2021-05-26/ev: Async operation
    if (temp_requested == true){
      // Wait until enough time since the sensors.requestTemperatures() call has passed!
      if (temperature_request_time_ms - millis() >= TEMPERATURE_REQUEST_TIME_PERIOD_mS){
        float temperature_interim_c = 0;
        temperature_interim_c = sensors.getTempCByIndex(0);
          if (temperature_interim_c < TEMPERATURE_RESET_VALUE){
            temperature_current_c = temperature_interim_c;
            temperature_acquired_at_least_once = true;
          }
        temp_requested = false;
				//log_3("Temp = ", temperature_current_c, " degC");
      }
    }else{
      temperature_request_time_ms = millis();
      sensors.setWaitForConversion(false); 
      sensors.requestTemperatures();
      sensors.setWaitForConversion(true);
      temp_requested = true;
    }
}

void init_motor_program_default_values(){
  // STICK program 
  motorProgramStick[0].duration_ms = 60000;
  motorProgramStick[0].motor_state = M_FWD;
  motorProgramStick[1].duration_ms = 50000;
  motorProgramStick[1].motor_state = M_STP;
  motorProgramStick[2].duration_ms = 10000;
  motorProgramStick[2].motor_state = M_FWD;

	
  // Sum the defined lengths for internal purposes
  motorProgramStickPeriod_ms = 0;
  for(int32_t i = 1; i < MOTOR_PROGRAM_STICK_STEPS_COUNT; i++){
    motorProgramStickPeriod_ms += motorProgramStick[i].duration_ms;
  }
	
	
	//ROTATIONAL program
	motorProgramRotational[0].duration_ms = 10000;
  motorProgramRotational[0].motor_state = M_FWD;
  motorProgramRotational[1].duration_ms = 500;
  motorProgramRotational[1].motor_state = M_STP;
  motorProgramRotational[2].duration_ms = 11000;
  motorProgramRotational[2].motor_state = M_REV;
  motorProgramRotational[3].duration_ms = 500;
  motorProgramRotational[3].motor_state = M_STP;
  motorProgramRotational[4].duration_ms = 11000;
  motorProgramRotational[4].motor_state = M_FWD;
	
  // Sum the defined lengths for internal purposes
  motorProgramRotationalPeriod_ms = 0;
  for(int32_t i = 1; i < MOTOR_PROGRAM_ROTATIONAL_STEPS_COUNT; i++){
    motorProgramRotationalPeriod_ms += motorProgramRotational[i].duration_ms;
  }
}

void init_process_program_default_values(){
		
	char temp_str[6];
	for (int32_t i=0; i<PROGRAMS_COUNT; i++){
		program[i].number = i;
		(idSelectedProgram == i) ? program[i].isSelected = true : program[i].isSelected = false;
		strcpy(program[i].name_index, "P");
		sprintf(temp_str, "%d", i+1);
		strcat(program[i].name_index, temp_str);
		strcpy(program[i].name_short, PROGRAM_NAMES_SHORT[i]);
		strcpy(program[i].name_long, PROGRAM_NAMES_SHORT[i]);
		//strcat(program[i].name_long, " ");
		strcat(program[i].name_long, PROGRAM_NAMES_ADD[i]);
	}
	
	program[0].step_count = P1_STEPS;
	program[1].step_count = P2_STEPS;
	program[2].step_count = P3_STEPS;
	
	
	t_STEP temp_step;
	strcpy(temp_step.name, "NEW_STEP");

		for (int32_t i=0; i<program[0].step_count; i++){
			temp_step.number = i;
			strcpy(temp_step.name, P1_STEP_NAMES[i]);
			temp_step.duration_s = P1_STEP_DURATION[i];
			temp_step.agitation = P1_AGITATION[i];
			temp_step.compensation =  P1_COMPENSATION[i];
			temp_step.isSelected = false;
			
			program[0].step.push_back(temp_step);
		}
	
		for (int32_t i=0; i<program[1].step_count; i++){
			temp_step.number = i;
			strcpy(temp_step.name, P2_STEP_NAMES[i]);
			temp_step.duration_s = P2_STEP_DURATION[i];
			temp_step.agitation = P2_AGITATION[i];
			temp_step.compensation =  P2_COMPENSATION[i];
			temp_step.isSelected = false;
			
			program[1].step.push_back(temp_step);
		}
	
		for (int32_t i=0; i<program[2].step_count; i++){
			temp_step.number = i;
			strcpy(temp_step.name, P3_STEP_NAMES[i]);
			temp_step.duration_s = P3_STEP_DURATION[i];
			temp_step.agitation = P3_AGITATION[i];
			temp_step.compensation =  P3_COMPENSATION[i];
			temp_step.isSelected = false;
			
			program[2].step.push_back(temp_step);
		}
	
	defaultProgram[0] = program[0];
	defaultProgram[1] = program[1];
	defaultProgram[2] = program[2];
	
}
void init_pins(){
	//BUTTONS
	pinMode(PIN_SWITCH_1_UP,INPUT);
	pinMode(PIN_SWITCH_2_RIGHT,INPUT);
	pinMode(PIN_SWITCH_3_DOWN,INPUT);
	pinMode(PIN_SWITCH_4_BACK,INPUT);
	pinMode(PIN_SWITCH_5_START,INPUT);
	pinMode(PIN_SWITCH_6_POWER,INPUT);
		gpio_pullup_dis(BUTTON_EXT1_PIN);	
		gpio_pulldown_en(BUTTON_EXT1_PIN);  
	buttonPower.setButtonLevel(HIGH);
	buttonPower.attach(RELEASE_HANDLER, processPowerButton_isr);
   
	//ENCODER
		pinMode(PIN_ENCODER_SWITCH_ENTER,INPUT_PULLUP);
		pinMode(PIN_ENCODER_CLK,INPUT_PULLUP);
		pinMode(PIN_ENCODER_DT,INPUT_PULLUP);

		attachInterrupt(PIN_ENCODER_CLK, process_encoder_isr, CHANGE);
		attachInterrupt(PIN_ENCODER_DT, process_encoder_isr, CHANGE);
	
	//DCDC ON/OFF PIN
	digitalWrite(PIN_POWER_DCDC, LOW);
	pinMode(PIN_POWER_DCDC, OUTPUT);
	digitalWrite(PIN_POWER_DCDC, LOW);

	//Buzzer
	// pinMode(PIN_BUZZER, OUTPUT);
	// digitalWrite(PIN_BUZZER, LOW);
	//EasyBuzzer.setPin(PIN_BUZZER); 
	//digitalWrite(PIN_BUZZER, LOW); // ToDo: PWM for this type of buzzer?
	
	//Motor
	pinMode(PIN_MOTOR_EN, OUTPUT);
	digitalWrite(PIN_MOTOR_EN, LOW);
	pinMode(PIN_MOTOR_PHS, OUTPUT);
	digitalWrite(PIN_MOTOR_PHS, LOW);
	motor_stop();
   
}

#define SWITCH_DEBOUNCE_READY         0
#define SWITCH_DEBOUNCE_IGNORE        1
uint8_t switch_debounce = SWITCH_DEBOUNCE_READY;

void process_physical_switches(){
	//log_2("UP  =",digitalRead(PIN_SWITCH_1_UP));
	//log_2("RIGHT=",digitalRead(PIN_SWITCH_2_RIGHT));
	//log_2("DOWN=",digitalRead(PIN_SWITCH_3_DOWN));
	//log_2("BACK=",digitalRead(PIN_SWITCH_4_BACK));
	//log_2("STRT=",digitalRead(PIN_SWITCH_5_START));
	//process_any_switch();

  if (current_menu == MENU_SPLASH){
    if (process_any_switch() == true){
      switch_to_menu(MENU_STORED_PROGRAMS);
    }else{
      switch_debounce = SWITCH_DEBOUNCE_READY;
    }
  }else if (current_menu == MENU_STORED_PROGRAMS){
    if (digitalRead(PIN_SWITCH_4_BACK)){
      switch_to_menu(MENU_SPLASH);
    }else if (digitalRead(PIN_SWITCH_2_RIGHT)){
			switch_to_menu(MENU_PROGRAM_OVERVIEW);
    }else if (digitalRead(PIN_SWITCH_5_START)){
			switch_to_menu(MENU_PROGRAM_OVERVIEW);
    }else if (digitalRead(PIN_SWITCH_1_UP)){
			processStoredProgramSelection(BUTTON_UP);
    }else if (digitalRead(PIN_SWITCH_3_DOWN)){
			processStoredProgramSelection(BUTTON_DOWN);
    }else if (digitalRead(PIN_ENCODER_SWITCH_ENTER) == LOW){
			switch_to_menu(MENU_PROGRAM_OVERVIEW);
    }else{
      switch_debounce = SWITCH_DEBOUNCE_READY;
    }
  }else if (current_menu == MENU_PROGRAM_OVERVIEW){
    if (digitalRead(PIN_SWITCH_1_UP)){
      processProgramOverviewSelection(BUTTON_UP);
    }else if (digitalRead(PIN_SWITCH_2_RIGHT)){
      processProgramOverviewSelection(BUTTON_RIGHT);
    }else if (digitalRead(PIN_SWITCH_3_DOWN)){
      processProgramOverviewSelection(BUTTON_DOWN);
    }else if (digitalRead(PIN_SWITCH_4_BACK)){
      if (selectedColumn == NONE) {
				switch_to_menu(MENU_STORED_PROGRAMS);
			} else {
					if (switch_debounce == SWITCH_DEBOUNCE_READY){
						selectedColumn = NONE;	
						switch_debounce = SWITCH_DEBOUNCE_IGNORE;
					}
			}
    }else if (digitalRead(PIN_SWITCH_5_START)){
			//int32_t idStartStep = (developPaused ? idActiveStep : idSelectedStep);
			initDevelopProcess(idSelectedStep);
			switch_to_menu(MENU_DEVELOP);
    // }else if (digitalRead(PIN_SWITCH_6_POWER)){
      // switch_to_menu(MENU_SPLASH);
    }else if (digitalRead(PIN_ENCODER_SWITCH_ENTER) == LOW){
			processProgramOverviewSelection(BUTTON_RIGHT);
    }else{
      switch_debounce = SWITCH_DEBOUNCE_READY;
    }
  }else if (current_menu == MENU_DEVELOP){
    if (digitalRead(PIN_SWITCH_5_START)){
			if (developPaused) initDevelopProcess(idActiveStep);
      //switch_to_menu(MENU_PROGRAM_OVERVIEW);    // ToDo: Cancel notification or something
    }else if (digitalRead(PIN_SWITCH_4_BACK)){
			finishDevelop();
      switch_to_menu(MENU_PROGRAM_OVERVIEW);
    }else{
      switch_debounce = SWITCH_DEBOUNCE_READY;
    }
  }
}

void switch_to_menu(Menu menu_idx){
  if (switch_debounce == SWITCH_DEBOUNCE_READY){
    current_menu = menu_idx;
    switch_debounce = SWITCH_DEBOUNCE_IGNORE;
  }
}

bool process_any_switch(){
  if (digitalRead(PIN_SWITCH_1_UP) ||
      digitalRead(PIN_SWITCH_2_RIGHT) ||
      digitalRead(PIN_SWITCH_3_DOWN) ||
      digitalRead(PIN_SWITCH_4_BACK) ||
      digitalRead(PIN_SWITCH_5_START) ||
      //digitalRead(PIN_SWITCH_6_POWER) ||
      (digitalRead(PIN_ENCODER_SWITCH_ENTER) == LOW) // ToDo: Enter operation: pull-ups?
    ){
			lastPressTime_ms = millis();
      return true;
    }else{
      return false;
    }
}

void processProgramOverviewSelection(Buttons button){
  if (switch_debounce == SWITCH_DEBOUNCE_READY){
    if (button == BUTTON_RIGHT){				// Switch to another parameter within this row
			switch (selectedColumn)
			{
			case NONE:
				selectedColumn = DURATION;
				break;
			case DURATION:
				selectedColumn = AGITATION;
				break;
			case AGITATION:
				selectedColumn = COMPENSATION;
				break;
			default:
				selectedColumn = NONE;
				break;
			}		
		} else if (button == BUTTON_UP) {		
				switch (selectedColumn)
				{
				case NONE:
					(idSelectedStep > 0) ? idSelectedStep-- : idSelectedStep = program[idSelectedProgram].step_count-1;
					break;
				case DURATION:
					if (program[idSelectedProgram].step[idSelectedStep].duration_s < MAX_DURATION_S){
						program[idSelectedProgram].step[idSelectedStep].duration_s+=TIMESTEP_S;
					} else {
							program[idSelectedProgram].step[idSelectedStep].duration_s = 0;
					} 
					break;
				case AGITATION:
						//
						switch (program[idSelectedProgram].step[idSelectedStep].agitation)
						{
						case AGITATION_OFF:
							program[idSelectedProgram].step[idSelectedStep].agitation = AGITATION_ROTATIONAL;
							break;
						case AGITATION_ROTATIONAL:
							program[idSelectedProgram].step[idSelectedStep].agitation = AGITATION_STICK;
							break;
						case AGITATION_STICK:
							program[idSelectedProgram].step[idSelectedStep].agitation = AGITATION_OFF;
							break;
						default:
							break;
						}
						//
					break;
				case COMPENSATION:
						//
						switch (program[idSelectedProgram].step[idSelectedStep].compensation)
						{
						case COMPENSATION_OFF:
							program[idSelectedProgram].step[idSelectedStep].compensation = 
								defaultProgram[idSelectedProgram].step[idSelectedStep].compensation;
							break;
						case COMPENSATION_CINESTILL_DEV:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;
						case COMPENSATION_TETENAL_BLIX:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;
						case COMPENSATION_TETENAL_DEV:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;	
						case COMPENSATION_BW_DEV:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;	
						default:
							break;
						}
						//
					break;					
				default:
					break;
				}			
		} else if (button == BUTTON_DOWN) {	
				switch (selectedColumn)
				{
				case NONE:
					(idSelectedStep < program[idSelectedProgram].step_count-1) ? idSelectedStep++ : idSelectedStep = 0;		
					break;
				case DURATION:
					if (program[idSelectedProgram].step[idSelectedStep].duration_s > 0){
						program[idSelectedProgram].step[idSelectedStep].duration_s-=TIMESTEP_S;
					} else {
							program[idSelectedProgram].step[idSelectedStep].duration_s = MAX_DURATION_S;
					} 	
					break;
				case AGITATION:
						//
						switch (program[idSelectedProgram].step[idSelectedStep].agitation)
						{
						case AGITATION_OFF:
							program[idSelectedProgram].step[idSelectedStep].agitation = AGITATION_STICK;
							break;
						case AGITATION_STICK:
							program[idSelectedProgram].step[idSelectedStep].agitation = AGITATION_ROTATIONAL;
							break;
						case AGITATION_ROTATIONAL:
							program[idSelectedProgram].step[idSelectedStep].agitation = AGITATION_OFF;
							break;
						default:
							break;
						}
						//
					break;
				case COMPENSATION:
						//
						switch (program[idSelectedProgram].step[idSelectedStep].compensation) 
						{
						case COMPENSATION_OFF:
							program[idSelectedProgram].step[idSelectedStep].compensation = 
								defaultProgram[idSelectedProgram].step[idSelectedStep].compensation;
							break;
						case COMPENSATION_BW_DEV:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;
						case COMPENSATION_TETENAL_DEV:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;
						case COMPENSATION_TETENAL_BLIX:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;
						case COMPENSATION_CINESTILL_DEV:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;								
						default:
							break;
						}
						//
					break;
				default:
					break;
				}		
		//} else if (button == BUTTON_UP) {		
		//			(idSelectedStep > 0) ? idSelectedStep-- : idSelectedStep = program[idSelectedProgram].step_count-1;	
		//} else if (button == BUTTON_DOWN) {	
		//			(idSelectedStep < program[idSelectedProgram].step_count-1) ? idSelectedStep++ : idSelectedStep = 0;
		} else if (button == BUTTON_ENC_CW) {		
				switch (selectedColumn)
				{
				// case NONE:
					// (idSelectedStep > 0) ? idSelectedStep-- : idSelectedStep = program[idSelectedProgram].step_count-1;
					// break;
				case DURATION:
					if (program[idSelectedProgram].step[idSelectedStep].duration_s < MAX_DURATION_S){
						program[idSelectedProgram].step[idSelectedStep].duration_s+=TIMESTEP_S;
					} else {
							program[idSelectedProgram].step[idSelectedStep].duration_s = 0;
					} 
					break;
				case AGITATION:
						//
						switch (program[idSelectedProgram].step[idSelectedStep].agitation)
						{
						case AGITATION_OFF:
							program[idSelectedProgram].step[idSelectedStep].agitation = AGITATION_ROTATIONAL;
							break;
						case AGITATION_ROTATIONAL:
							program[idSelectedProgram].step[idSelectedStep].agitation = AGITATION_STICK;
							break;
						case AGITATION_STICK:
							program[idSelectedProgram].step[idSelectedStep].agitation = AGITATION_OFF;
							break;
						default:
							break;
						}
						//
					break;
				case COMPENSATION:
						//
						switch (program[idSelectedProgram].step[idSelectedStep].compensation)
						{
						case COMPENSATION_OFF:
							program[idSelectedProgram].step[idSelectedStep].compensation = 
								defaultProgram[idSelectedProgram].step[idSelectedStep].compensation;
							break;
						case COMPENSATION_CINESTILL_DEV:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;
						case COMPENSATION_TETENAL_BLIX:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;
						case COMPENSATION_TETENAL_DEV:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;	
						case COMPENSATION_BW_DEV:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;	
						default:
							break;
						}
						//
					break;					
				default:
					break;
				}			
		} else if (button == BUTTON_ENC_CCW) {	
				switch (selectedColumn)
				{
				// case NONE:
					// (idSelectedStep < program[idSelectedProgram].step_count-1) ? idSelectedStep++ : idSelectedStep = 0;		
					// break;
				case DURATION:
					if (program[idSelectedProgram].step[idSelectedStep].duration_s > 0){
						program[idSelectedProgram].step[idSelectedStep].duration_s-=TIMESTEP_S;
					} else {
							program[idSelectedProgram].step[idSelectedStep].duration_s = MAX_DURATION_S;
					} 	
					break;
				case AGITATION:
						//
						switch (program[idSelectedProgram].step[idSelectedStep].agitation)
						{
						case AGITATION_OFF:
							program[idSelectedProgram].step[idSelectedStep].agitation = AGITATION_STICK;
							break;
						case AGITATION_STICK:
							program[idSelectedProgram].step[idSelectedStep].agitation = AGITATION_ROTATIONAL;
							break;
						case AGITATION_ROTATIONAL:
							program[idSelectedProgram].step[idSelectedStep].agitation = AGITATION_OFF;
							break;
						default:
							break;
						}
						//
					break;
				case COMPENSATION:
						//
						switch (program[idSelectedProgram].step[idSelectedStep].compensation) 
						{
						case COMPENSATION_OFF:
							program[idSelectedProgram].step[idSelectedStep].compensation = 
								defaultProgram[idSelectedProgram].step[idSelectedStep].compensation;
							break;
						case COMPENSATION_BW_DEV:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;
						case COMPENSATION_TETENAL_DEV:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;
						case COMPENSATION_TETENAL_BLIX:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;
						case COMPENSATION_CINESTILL_DEV:
							program[idSelectedProgram].step[idSelectedStep].compensation = COMPENSATION_OFF;
							break;								
						default:
							break;
						}
						//
					break;
				default:
					break;
				}		
		} 
    switch_debounce = SWITCH_DEBOUNCE_IGNORE;
	}
 }

void processStoredProgramSelection(Buttons button){
  if (switch_debounce == SWITCH_DEBOUNCE_READY){
    // Select another program
    if (button == BUTTON_UP){
      (idSelectedProgram > 0) ? idSelectedProgram-- : idSelectedProgram = PROGRAMS_COUNT-1;
    }else if (button == BUTTON_DOWN){
			(idSelectedProgram < PROGRAMS_COUNT-1) ? idSelectedProgram++ : idSelectedProgram = 0;
    }
    switch_debounce = SWITCH_DEBOUNCE_IGNORE;
  }
}

// ===============================================================================================
// Smoothest KY-040 low-level implementation tested (2021-05-20)
// Encoder 2 x ISR implementation by bennygodlin May '16 #29
// https://forum.arduino.cc/t/wiring-of-ky-040-rotary-encoder-plus-demo-code/235847/29
// ===============================================================================================
#define MAXLONG (2147483647L)
volatile bool rotEncIntRuns = false;
volatile long lastInt_ms = 0L;
uint32_t turnsCount = 0;
const long usWaitBeforePinCheck = 500L; // 100L; // 500L; // 10L;        //500,60 works
const long msBetweenIntRuns = 40L; // 200L; // 600L; // 20L; 

void process_encoder_isr(){
  // don't reenter the interrupt handler:
  if (rotEncIntRuns) return;
  rotEncIntRuns = true;
  
  // check that no less then msBetweenIntRuns
  //   passes between swapLedIntHand() runs
  long now_ms = micros();
  long diff_ms = now_ms - lastInt_ms;
  if (diff_ms < 0) diff_ms += MAXLONG;
  if (diff_ms < msBetweenIntRuns) {
    rotEncIntRuns = false;
    return;
  }

  interrupts(); // release interrupts for timers etc.
  
  checkDirChange();
  
  lastInt_ms = micros();
  rotEncIntRuns = false;
}

// rotEnc vars:
byte rotEncTicksPerCycle = 30;
volatile byte rotEncLastState;
volatile int rotDir = 0;

volatile int rotEncTicks = 0;
int rotEncTicksLast = 0;

void checkDirChange(){
  delayMicroseconds(usWaitBeforePinCheck);

  byte bVal = digitalRead(PIN_ENCODER_DT);
  byte aVal = digitalRead(PIN_ENCODER_CLK);
  if (aVal != bVal) { // knob is moving
    if (aVal != rotEncLastState) { // pinA changed first
       rotDir = 1;
    } else {// Otherwise B changed first and we're moving CCW
       rotDir = -1;
    }
  }
  else { // both pins are at same state:
    if (aVal != rotEncLastState) {
      rotEncTicks += rotDir;
			 turnsCount++;
			 lastPressTime_ms = millis();
    }
    rotEncLastState = aVal;  // mark new state
    //rotDir = 0;
  }
  if (rotEncTicks < 0)
    rotEncTicks = rotEncTicks + rotEncTicksPerCycle;
  else
    rotEncTicks = rotEncTicks % rotEncTicksPerCycle;
}

// ===============================================================================================
// Encoder processing (post-ISR routine)
// ===============================================================================================
#define TIME_RANGE_GENERAL_MAX_mS    5999 * _mS_per_SEC_
#define TIME_STEP_MIN_mS             10 * _mS_per_SEC_
#define TIME_STEP_MAX_mS             10 * _mS_per_SEC_
uint8_t encoder_ballistic_mode = 0;   // ToDo: Implement ballistic mode 

void process_encoder(){
	//EncButton part

	if (abs(rotEncTicks) > 0 ){  // rotEncTicks and rotDir accounted for in encoder connection interrupts
		Buttons direction;
		direction = (rotDir > 0) ? BUTTON_ENC_CW : BUTTON_ENC_CCW;
		
		switch (current_menu)
		{
		// case MENU_STORED_PROGRAMS:
			// processStoredProgramSelection(direction);
			// break;
		case MENU_PROGRAM_OVERVIEW:
			processProgramOverviewSelection(direction);
			break;
		default:
			break;
		}	

		rotEncTicks = 0;
		// rotEncTicks--;
	}
	
	
}

// ================================================================================================================
// "STORED PROGRAMS" menu
// ================================================================================================================
void update_menu_stored_programs(){
		
		//framebuffer_sprite.deleteSprite();
		framebuffer_sprite.createSprite(lcd.width(), MENU_HEADER_HEIGHT);
    framebuffer_sprite.fillScreen(COLOR16_DARK_GREY);
    framebuffer_sprite.setTextColor(COLOR16_YELLOW);
		framebuffer_sprite.setFont(&Sk_Modernist_Regular11pt7b);
		framebuffer_sprite.drawString("PROGRAMS", 90, 20);
			icon30_sprite.createSprite(30, 30);
			icon30_sprite.setSwapBytes(true);
			drawSpriteBattery(batterySoc, isCharging, &icon30_sprite);		
			icon30_sprite.pushSprite(lcd.width()-icon30_sprite.width()-5, 0);		
		framebuffer_sprite.pushSprite(0, 0);

		lcd.fillRect(0, MENU_HEADER_HEIGHT, lcd.width(), MENU_SPACE_HEIGHT, COLOR16_BLACK);
		
		if (idDisplayTopProgram > idSelectedProgram) 
			idDisplayTopProgram = idSelectedProgram;
		if ((idDisplayTopProgram + MENU_DISPLAYED_PROGRAMS_COUNT) <= idSelectedProgram) 
			idDisplayTopProgram = idSelectedProgram - MENU_DISPLAYED_PROGRAMS_COUNT + 1;
		
		uint32_t bottomProgram = PROGRAMS_COUNT-1;
		if ((idDisplayTopProgram + MENU_DISPLAYED_PROGRAMS_COUNT-1) < PROGRAMS_COUNT)	{
			bottomProgram = idDisplayTopProgram + MENU_DISPLAYED_PROGRAMS_COUNT-1;
		}
		
		uint32_t i;
		for(i = idDisplayTopProgram; i <= bottomProgram; i++){
		//log_2("i = ",i);	
			framebuffer_sprite.createSprite(lcd.width(), MENU_ENTRY_HEIGHT);
			if (program[i].number == idSelectedProgram) {
				framebuffer_sprite.fillScreen(COLOR16_RED);
				framebuffer_sprite.setTextColor(COLOR16_BLACK);
			} else {
				framebuffer_sprite.fillScreen(COLOR16_BLACK);
				framebuffer_sprite.setTextColor(COLOR16_YELLOW);
			}
			framebuffer_sprite.setFont(&Sk_Modernist_Regular9pt7b);
			framebuffer_sprite.drawString(program[i].name_index, 20, 8);
			//log_2("prog = ",program[i].name_index);	
			framebuffer_sprite.drawString(program[i].name_long, 50, 8);

			framebuffer_sprite.pushSprite(0, 
					MENU_HEADER_HEIGHT+MENU_SPACE_HEIGHT+(i-idDisplayTopProgram)*(MENU_ENTRY_HEIGHT+MENU_SPACE_HEIGHT));		
			lcd.fillRect(0, MENU_HEADER_HEIGHT+(i-idDisplayTopProgram+1)*(MENU_ENTRY_HEIGHT+MENU_SPACE_HEIGHT), 
					lcd.width(), MENU_SPACE_HEIGHT, COLOR16_BLACK);
		}
		
		for (uint32_t j = i; j < MENU_DISPLAYED_PROGRAMS_COUNT; j++){
			lcd.fillRect(0, MENU_HEADER_HEIGHT+MENU_SPACE_HEIGHT+(j-idDisplayTopProgram)*(MENU_ENTRY_HEIGHT+MENU_SPACE_HEIGHT), 
					lcd.width(), MENU_ENTRY_HEIGHT+MENU_SPACE_HEIGHT, COLOR16_BLACK);
		}
		//log_1("stored_prg_updated");
}

// ================================================================================================================
// "PROGRAM OVERVIEW" menu
// ================================================================================================================
void update_menu_program_overview(){
		
		draw_program_name_header();
		
		lcd.fillRect(0, MENU_HEADER_HEIGHT, lcd.width(), MENU_SPACE_HEIGHT, COLOR16_BLACK);
		
		if (idSelectedStep >= program[idSelectedProgram].step_count) idSelectedStep = 0;
		
		// log_2("Sstep = ",idSelectedStep);
		if (idDisplayTopStep > idSelectedStep) 
			idDisplayTopStep = idSelectedStep;
		if ((idDisplayTopStep + MENU_DISPLAYED_PROGRAMS_COUNT) <= idSelectedStep) 
			idDisplayTopStep = idSelectedStep - MENU_DISPLAYED_PROGRAMS_COUNT + 1;
		
		uint32_t bottomStep = program[idSelectedProgram].step_count-1;
		if ((idDisplayTopStep + MENU_DISPLAYED_PROGRAMS_COUNT-1) < program[idSelectedProgram].step_count )	{
			bottomStep = idDisplayTopStep + MENU_DISPLAYED_PROGRAMS_COUNT-1;
		}
		// log_2("Tstep = ",idDisplayTopStep);
		// log_2("Bstep = ",bottomStep);
		IconColor agitationImageColor, 	compensationImageColor;
		uint16_t rowBackgroundColor, 	durationBackgroundColor, 	compensationBackgroundColor;
		uint16_t rowTextColor, 				durationTextColor, 				compensationTextColor; 
		uint32_t i;
		for(i = idDisplayTopStep; i <= bottomStep; i++){
		//log_2("i = ",i);	
			rowBackgroundColor = durationBackgroundColor = compensationBackgroundColor = COLOR16_BLACK;
			rowTextColor = 	durationTextColor = compensationTextColor = COLOR16_YELLOW;
			agitationImageColor = compensationImageColor = BACKGROUND_BLACK;
		
			if (program[idSelectedProgram].step[i].number == idSelectedStep) {
				rowBackgroundColor = durationBackgroundColor = compensationBackgroundColor = COLOR16_RED;
				rowTextColor = 	durationTextColor = compensationTextColor = COLOR16_BLACK;
				agitationImageColor = compensationImageColor = BACKGROUND_RED;
					switch (selectedColumn)
					{
					case NONE:
						break;
					case DURATION:
						durationBackgroundColor = COLOR16_RED;
						durationTextColor = COLOR16_YELLOW;
						break;
					case AGITATION:
						agitationImageColor = BACKGROUND_RED_YELLOW_ICON;
						break;
					case COMPENSATION:
						compensationBackgroundColor = COLOR16_RED;
						compensationTextColor = COLOR16_YELLOW;
						compensationImageColor = BACKGROUND_RED_YELLOW_ICON;
						break;
					default:
						break;
					}	
			}
			
			framebuffer_sprite.createSprite(lcd.width(), MENU_ENTRY_HEIGHT);
			//STEP NAME
			framebuffer_sprite.fillScreen(rowBackgroundColor);
			framebuffer_sprite.setTextColor(rowTextColor);
			framebuffer_sprite.setFont(&Sk_Modernist_Regular9pt7b);
			framebuffer_sprite.drawString(program[idSelectedProgram].step[i].name, 30, 8);
					//DURATION
					framebuffer_sprite.fillRect(105, 0, 60, MENU_ENTRY_HEIGHT, durationBackgroundColor);
					char time_string[10];
					seconds2string(program[idSelectedProgram].step[i].duration_s, &time_string[0]);
					//framebuffer_sprite.setTextColor(durationTextColor, durationBackgroundColor);
					framebuffer_sprite.setTextColor(durationTextColor);
					framebuffer_sprite.drawString(time_string, 110, 8);
							//AGITATION
							//framebuffer_sprite.fillRect(110, 0, 30, MENU_ENTRY_HEIGHT, agitationBackgroundColor);
							icon30_sprite.createSprite(30, 30);
							icon30_sprite.setSwapBytes(true);
							drawSpriteAgitation(agitationImageColor, program[idSelectedProgram].step[i].agitation, &icon30_sprite);		
							icon30_sprite.pushSprite(205, 0);
									//COMPENSATION
									//icon30_sprite.createSprite(30, 30);
									//icon30_sprite.setSwapBytes(true);
									framebuffer_sprite.fillRect(240, 0, 76, MENU_ENTRY_HEIGHT, compensationBackgroundColor);
									drawSpriteCompensation(compensationImageColor, program[idSelectedProgram].step[i].compensation, &icon30_sprite);		
									icon30_sprite.pushSprite(284, 0);
									//framebuffer_sprite.setTextColor(compensationTextColor, compensationBackgroundColor);
									framebuffer_sprite.setTextColor(compensationTextColor);
									drawAddText(program[idSelectedProgram].step[i].compensation, &framebuffer_sprite);

			framebuffer_sprite.pushSprite(0, 
					MENU_HEADER_HEIGHT+MENU_SPACE_HEIGHT+(i-idDisplayTopStep)*(MENU_ENTRY_HEIGHT+MENU_SPACE_HEIGHT));		
			lcd.fillRect(0, MENU_HEADER_HEIGHT+(i-idDisplayTopStep+1)*(MENU_ENTRY_HEIGHT+MENU_SPACE_HEIGHT), 
					lcd.width(), MENU_SPACE_HEIGHT, COLOR16_BLACK);
		}
		
		for (uint32_t j = i; j < MENU_DISPLAYED_PROGRAMS_COUNT; j++){
			lcd.fillRect(0, MENU_HEADER_HEIGHT+MENU_SPACE_HEIGHT+(j-idDisplayTopStep)*(MENU_ENTRY_HEIGHT+MENU_SPACE_HEIGHT), 
					lcd.width(), MENU_ENTRY_HEIGHT+MENU_SPACE_HEIGHT, COLOR16_BLACK);
		}
		
		icon30_sprite.pushSprite(230, 10);

    lcd.startWrite();
		//log_1("progr_overview_updated");
}

// ================================================================================================================
// DEVELOP/STOP/FIX/RINSE combo menu
// ================================================================================================================
void draw_program_name_header(){
  // Draw static graphics

  // ToDo: find a method in gfx library to center text 
		framebuffer_sprite.createSprite(lcd.width(), MENU_HEADER_HEIGHT);
    framebuffer_sprite.fillScreen(COLOR16_DARK_GREY);
    framebuffer_sprite.fillRect(0, 0, MENU_HEADER_TAB, MENU_HEADER_HEIGHT, COLOR16_YELLOW);
		framebuffer_sprite.setTextSize(1);
		framebuffer_sprite.setFont(&Sk_Modernist_Regular11pt7b);
    framebuffer_sprite.setTextColor(COLOR16_BLACK);
		framebuffer_sprite.drawString(program[idSelectedProgram].name_index, 26, 20);
    framebuffer_sprite.setTextColor(COLOR16_YELLOW);
		framebuffer_sprite.drawString(program[idSelectedProgram].name_short, 180, 20);
			icon30_sprite.createSprite(30, 30);
			icon30_sprite.setSwapBytes(true);
			drawSpriteBattery(batterySoc, isCharging, &icon30_sprite);		
			icon30_sprite.pushSprite(lcd.width()-icon30_sprite.width()-5, 0);
		framebuffer_sprite.pushSprite(0, 0);

}

void draw_program_stage_row(){
  // Draw static graphics

  // ToDo: find a method in gfx library to center text 
		IconColor agitationImageColor, 	compensationImageColor;
		uint16_t rowBackgroundColor, 	durationBackgroundColor, 	compensationBackgroundColor;
		uint16_t rowTextColor, 				durationTextColor, 				compensationTextColor; 
		
		rowBackgroundColor = durationBackgroundColor = compensationBackgroundColor = COLOR16_LIGHT_GREY;
		rowTextColor = 	durationTextColor = compensationTextColor = COLOR16_BLACK;
		agitationImageColor = compensationImageColor = BACKGROUND_GREY;
		framebuffer_sprite.createSprite(lcd.width(), MENU_ENTRY_HEIGHT);
			//STEP NAME
			framebuffer_sprite.fillScreen(rowBackgroundColor);
			framebuffer_sprite.setTextColor(rowTextColor);
			framebuffer_sprite.setFont(&Sk_Modernist_Regular9pt7b);
			framebuffer_sprite.drawString(activeProgram.step[idActiveStep].name, 30, 8);
					//DURATION
					framebuffer_sprite.fillRect(105, 0, 60, MENU_ENTRY_HEIGHT, durationBackgroundColor);
					char time_string[10];
					seconds2string(activeProgram.step[idActiveStep].duration_s, &time_string[0]);
					//framebuffer_sprite.setTextColor(durationTextColor, durationBackgroundColor);
					framebuffer_sprite.setTextColor(durationTextColor);
					framebuffer_sprite.drawString(time_string, 110, 8);
							//AGITATION
							//framebuffer_sprite.fillRect(110, 0, 30, MENU_ENTRY_HEIGHT, agitationBackgroundColor);
							icon30_sprite.createSprite(30, 30);
							icon30_sprite.setSwapBytes(true);
							drawSpriteAgitation(agitationImageColor, activeProgram.step[idActiveStep].agitation, &icon30_sprite);		
							icon30_sprite.pushSprite(205, 0);
									//COMPENSATION
									//icon30_sprite.createSprite(30, 30);
									//icon30_sprite.setSwapBytes(true);
									framebuffer_sprite.fillRect(240, 0, 76, MENU_ENTRY_HEIGHT, compensationBackgroundColor);
									drawSpriteCompensation(compensationImageColor, activeProgram.step[idActiveStep].compensation, &icon30_sprite);		
									icon30_sprite.pushSprite(284, 0);
									//framebuffer_sprite.setTextColor(compensationTextColor, compensationBackgroundColor);
									framebuffer_sprite.setTextColor(compensationTextColor);
									drawAddText(activeProgram.step[idActiveStep].compensation, &framebuffer_sprite);
									
		framebuffer_sprite.pushSprite(0, MENU_HEADER_HEIGHT);
}

void draw_temperature_rect( LGFX_Sprite *ptr_sprite ){
	const tImage* image;
	uint16_t colorBackground, colorText;
	if (activeProgram.step[idActiveStep].compensation != COMPENSATION_OFF){
		colorBackground = COLOR16_RED;
		colorText = 			COLOR16_BLACK;
		image = &thermometer_red;
	} else {
		colorBackground = COLOR16_DARK_GREY;
		colorText = 			COLOR16_BLACK;
		image = &thermometer_grey;		
	}
	ptr_sprite->fillRect(ptr_sprite->width()-100, 0, 100, ptr_sprite->height(), colorBackground);

	ptr_sprite->pushImage(ptr_sprite->width()-(100-image->width)/2-image->width, 60, image->width, image->height, image->data);				

		if (temperature_acquired_at_least_once == true){
			ptr_sprite->setFont(&Sk_Modernist_Regular11pt7b);
			ptr_sprite->setTextSize(1);
			char temp_string[16];
			ptr_sprite->setTextColor(colorText);
			sprintf(temp_string,"%lu ", uint32_t(temperature_current_c + 0.5f));
			ptr_sprite->drawString(temp_string, ptr_sprite->width()-100+28, 30);  
				ptr_sprite->setFont(&Sk_Modernist_Regular5pt7b);
				ptr_sprite->drawString("o", ptr_sprite->width()-100+28+27, 30);  
					ptr_sprite->setFont(&Sk_Modernist_Regular11pt7b);
					ptr_sprite->drawString("C", ptr_sprite->width()-100+28+27+6, 30);
		}	
}


void update_menu_develop(){
				 
				draw_program_name_header();
				draw_program_stage_row();
				
				framebuffer_sprite.createSprite(lcd.width(), lcd.height()-MENU_HEADER_HEIGHT-MENU_ENTRY_HEIGHT);
				framebuffer_sprite.fillScreen(COLOR16_BLACK);
				// Temperature
				framebuffer_sprite.setSwapBytes(true);
				draw_temperature_rect(&framebuffer_sprite);


				// Time left
				framebuffer_sprite.setTextSize(1);
				framebuffer_sprite.setFont(&Sk_Modernist_Regular36pt7b);
				framebuffer_sprite.setTextColor(COLOR16_YELLOW);
				char time_string[10];
				seconds2string(current_step_time_left_ms / _mS_per_SEC_, &time_string[0]);
				framebuffer_sprite.drawString(time_string, 20, 40);  // ToDo: position #defines
				
				
				// time correction - time adjustment on screen
				if (developPaused != true){
					if (tempCorrectionApplied_1 == true) {
						framebuffer_sprite.setFont(&Sk_Modernist_Regular9pt7b);
						if (timeAdjust > 0) {
							framebuffer_sprite.drawString("+", 20, 13);
						} else {
							framebuffer_sprite.drawString("-", 20, 13);
						}
						seconds2string(abs(timeAdjust), &time_string[0]);
						framebuffer_sprite.drawString(time_string, 30, 15);
					}
				}
				
	framebuffer_sprite.pushSprite(0, MENU_HEADER_HEIGHT+MENU_ENTRY_HEIGHT);
  lcd.startWrite();
	//log_1("develop_updated");
}

// ================================================================================================================
// SPLASH (logo, start) menu
// ================================================================================================================
void update_menu_splash(){
  // Draw static graphics
	//framebuffer_sprite.deleteSprite();
	framebuffer_sprite.createSprite(320, 120);
	framebuffer_sprite.fillScreen(COLOR16_BLACK);
		framebuffer_sprite.setTextColor(COLOR16_YELLOW);  
		framebuffer_sprite.setTextSize(1);
		framebuffer_sprite.setCursor(64, 10);
		framebuffer_sprite.setFont(&Sk_Modernist_Bold245pt7b);
		framebuffer_sprite.println("AGO");  
			framebuffer_sprite.setCursor(60, 90);
			framebuffer_sprite.setFont(&Sk_Modernist_Regular11pt7b);
			framebuffer_sprite.println("FILM PROCESSOR");
	
	lcd.fillRect(0, 0, 320, 60, COLOR16_BLACK);
	framebuffer_sprite.pushSprite(0, 60);
	lcd.fillRect(0, 180, 320, 60, COLOR16_BLACK);
	
  lcd.startWrite();
	//log_1("splash_updated");
}

void draw_service(){
			char version_string[25];
			lcd.setTextColor(COLOR16_DARK_GREY);
			lcd.setTextSize(0.9);
			sprintf(version_string,"build %.1f", PROGRAM_VERSION);
			lcd.drawString(version_string, 3, 225);
			sprintf(version_string,"Boot count %d", bootCount);
			lcd.drawString(version_string, 60, 225);
			sprintf(version_string,"Batt, mv %d", batteryVoltage_mv);
			lcd.drawString(version_string, 150, 225);
			sprintf(version_string,"Turns %d", turnsCount);
			lcd.drawString(version_string, 240, 225);
			
			sprintf(version_string,"Inactive, s  %d", inactiveTime_s);
			lcd.drawString(version_string, 3, 210);			
			sprintf(version_string,"SoC %d", batterySoc);
			lcd.drawString(version_string, 150, 210);
			sprintf(version_string,"Dir %d", rotDir);
			lcd.drawString(version_string, 240, 210);			
			// sprintf(version_string,"Loops %d", loopCounter);
			// lcd.drawString(version_string, 240, 210);
}


// ================================================================================================================
// Main develop process handler
// ================================================================================================================
long develop_ms = 0;
uint32_t timestampDevelopStarted;
#define DEVELOP_PROCESS_TIMESCALE_mS          100 // 2021-05-26/ev: v3: Lowered from 1000 to 100. Fixed, change only for some obscure testing purposes

void initDevelopProcess(uint32_t idStep){
	if (switch_debounce == SWITCH_DEBOUNCE_READY){

      // Reset times
			activeProgram = program[idSelectedProgram];
			idActiveStep = idStep; 
      current_step_time_left_ms = activeProgram.step[idActiveStep].duration_s;
			current_step_time_left_ms *= _mS_per_SEC_;
			motorTimeFromStart_ms = 0;
			
      temperature_request_time_ms = millis();
			timestampDevelopStarted = millis();
      // temp correction
      tempCorrectionApplied_1 = false;
      tempCorrectionApplied_2 = false;
			developPaused = false;
					if(ForceWithYou.Playing==false)       // if not playing,
					DacAudio.Play(&ForceWithYou); 
			//EasyBuzzer.singleBeep(BUZZER_FREQ, 50, beepDone);

			
			
		//switch_debounce = SWITCH_DEBOUNCE_IGNORE;
	}    
}


void update_develop_process(){

  // 2021-05-25/ev: b2: go straight to the finish step
  if (current_step_time_left_ms == 0){
    //EasyBuzzer.singleBeep(BUZZER_FREQ, 1500, beepDone);
			Sequence.RemoveAllPlayItems(); 
			Sequence.AddPlayItem(&ForceWithYou);
			Sequence.AddPlayItem(&ForceWithYou);
			Sequence.AddPlayItem(&ForceWithYou);
			DacAudio.Play(&Sequence);
    motor_stop();
		idActiveStep++;
		developPaused = true;
		beepFirstDone = false;
		beepSecondDone = false;
    if (idActiveStep >= activeProgram.step_count) {
			switch_to_menu(MENU_PROGRAM_OVERVIEW);    // v2021-05-26/ev: Removed finish in favor of back to develop
			finishDevelop();
			return;
		}	else {
			current_step_time_left_ms = activeProgram.step[idActiveStep].duration_s*_mS_per_SEC_; //To show next step timer when paused
		}
  }
	
	if (developPaused) return;
	
  lastPressTime_ms = millis();
	
  // time correction - adjust exp time based on temperature difference
	t_ptr_Algorithm compensationAlgorithm_ptr;
	int32_t new_exp_time;
	uint32_t timeDiff;
	if (activeProgram.step[idActiveStep].compensation != COMPENSATION_OFF){	
		timeDiff = millis() - timestampDevelopStarted;
		if ((timeDiff >= correctionApplyTime_1_ms)  &&  (tempCorrectionApplied_1 == false)) 
		{
			switch (activeProgram.step[idActiveStep].compensation)
			{
			case COMPENSATION_BW_DEV:
				compensationAlgorithm_ptr = algorithmP1Dev;
				break;
			case COMPENSATION_TETENAL_DEV:
				compensationAlgorithm_ptr = algorithmP2Dev;
				break;
			case COMPENSATION_TETENAL_BLIX:
				compensationAlgorithm_ptr = algorithmP2Blix;
				break;
			case COMPENSATION_CINESTILL_DEV:
				compensationAlgorithm_ptr = algorithmP3Dev;
				break;
			default:
				compensationAlgorithm_ptr = algorithmP1Dev;
				break;
			}
			new_exp_time = compensationAlgorithm_ptr(activeProgram.step[idActiveStep].duration_s, temperature_current_c);
			current_step_time_left_ms = new_exp_time * _mS_per_SEC_ - timeDiff;
			tempCorrectionApplied_1 = true;
			correctionApplyTime_2_ms = SECOND_CORECTION_TIMESTAMP_PERCENT*new_exp_time*(_mS_per_SEC_/100);
			log_1("T1 corrected");
			log_2("T2 = ",correctionApplyTime_2_ms);			
		}
		
		if ((timeDiff >= correctionApplyTime_2_ms)  &&  (tempCorrectionApplied_2 == false)) 
		{	
			switch (activeProgram.step[idActiveStep].compensation)
			{
			case COMPENSATION_BW_DEV:
				compensationAlgorithm_ptr = algorithmP1Dev;
				break;
			case COMPENSATION_TETENAL_DEV:
				compensationAlgorithm_ptr = algorithmP2Dev;
				break;
			case COMPENSATION_TETENAL_BLIX:
				compensationAlgorithm_ptr = algorithmP2Blix;
				break;
			case COMPENSATION_CINESTILL_DEV:
				compensationAlgorithm_ptr = algorithmP3Dev;
				break;
			default:
				compensationAlgorithm_ptr = algorithmP1Dev;
				break;
			}
			new_exp_time = compensationAlgorithm_ptr(activeProgram.step[idActiveStep].duration_s, temperature_current_c);
			current_step_time_left_ms = new_exp_time * _mS_per_SEC_ - timeDiff;
			tempCorrectionApplied_2 = true;
			log_1("T2 corrected");
		}			
	}
  
  // Buzzer
  if (current_step_time_left_ms <= (10 * _mS_per_SEC_) && !beepFirstDone)
	{
    beepCount = 1;
		Sequence.RemoveAllPlayItems(); 
		Sequence.AddPlayItem(&ForceWithYou);
		DacAudio.Play(&Sequence);
    // EasyBuzzer.beep(
			// BUZZER_FREQ,		// Frequency in hertz(HZ).
			// 300, 		// On Duration in milliseconds(ms).
			// 0, 		// Off Duration in milliseconds(ms).
			// 1, 		// The number of beeps per cycle.
			// 100, 	// Pause duration.
			// 1, 		// The number of cycle.
			// beepDone
		// );
		beepFirstDone = true;
		log_2("beeps = ", beepCount);
	}
  if (current_step_time_left_ms <= (5 * _mS_per_SEC_) && !beepSecondDone)
	{
		beepCount = 2;
		Sequence.RemoveAllPlayItems(); 
		Sequence.AddPlayItem(&ForceWithYou);
		Sequence.AddPlayItem(&ForceWithYou);
		DacAudio.Play(&Sequence);
    // EasyBuzzer.beep(
			// BUZZER_FREQ,		// Frequency in hertz(HZ).
			// 300, 		// On Duration in milliseconds(ms).
			// 300, 		// Off Duration in milliseconds(ms).
			// 2, 		// The number of beeps per cycle.
			// 1000, 	// Pause duration.
			// 1, 		// The number of cycle.
			// beepDone
		// );
		beepSecondDone = true;
		log_2("beeps = ", beepCount);
	}
  

	
  // MOTOR
		MotorDirection set_motor_state = M_STP; 
    if (activeProgram.step[idActiveStep].agitation == AGITATION_ROTATIONAL){
			//log_2("t= ", motorTimeFromStart_ms);
			if (motorTimeFromStart_ms < motorProgramRotational[0].duration_ms){
				set_motor_state = motorProgramRotational[0].motor_state;
			} else {
				// 2021-05-26/ev: v3: Simple program, modulo takes care of the current time in the repeating program
				int32_t current_motor_program_time_ms = (motorTimeFromStart_ms - motorProgramRotational[0].duration_ms) % 
																									motorProgramRotationalPeriod_ms;
				// Somewhat 'complicated' search to have to define the motor program in duration lengths only
				int32_t current_program_position_ms = 0;
				for(int32_t i = 1; i < MOTOR_PROGRAM_ROTATIONAL_STEPS_COUNT; i++){
					if ((current_motor_program_time_ms >= current_program_position_ms) &&
							(current_motor_program_time_ms < (current_program_position_ms + motorProgramRotational[i].duration_ms)))
					{
							set_motor_state = motorProgramRotational[i].motor_state;
							break;
					}
					current_program_position_ms += motorProgramRotational[i].duration_ms;
				}	
			}
    } else if (activeProgram.step[idActiveStep].agitation == AGITATION_STICK) {
			if (motorTimeFromStart_ms < motorProgramStick[0].duration_ms){
				set_motor_state = motorProgramStick[0].motor_state;
			} else {
				int32_t current_motor_program_time_ms = (motorTimeFromStart_ms - motorProgramStick[0].duration_ms) % 
																									motorProgramStickPeriod_ms;
				// Somewhat 'complicated' search to have to define the motor program in duration lengths only
				int32_t current_program_position_ms = 0;
				for(int32_t i = 1; i < MOTOR_PROGRAM_STICK_STEPS_COUNT; i++){
					if ((current_motor_program_time_ms >= current_program_position_ms) &&
							(current_motor_program_time_ms < (current_program_position_ms + motorProgramStick[i].duration_ms)))
					{
							set_motor_state = motorProgramStick[i].motor_state;
							break;
					}
					current_program_position_ms += motorProgramStick[i].duration_ms;
				}	
			}			
		} else {
      set_motor_state = M_STP;
    }
		
		switch (set_motor_state)
		{
		case M_FWD:
			motor_run_forward();
			break;
		case M_REV:
			motor_run_reverse();
			break;
		case M_STP:
			motor_stop();
			break;
		default:
			motor_stop();
			break;
		}	
}

void finishDevelop()
{
				
			developPaused = false;
			tempCorrectionApplied_1 = false;
      tempCorrectionApplied_2 = false;
			
			correctionApplyTime_2_ms = 2*correctionApplyTime_1_ms;
}


int32_t algorithmP1Dev(int32_t timeOld_s, float temperature){
  log_1("calculating new exposure time");
  log_3("exp std time: ", timeOld_s, " seconds");
  log_3("std temp: ", P1DEV_STD_TEMPERATURE, " degrees");
  log_3("sol. temp: ", temperature, " degrees");
  int32_t timeNew_s = timeOld_s * exp(exp_e * (temperature - P1DEV_STD_TEMPERATURE));
	timeAdjust = timeNew_s - timeOld_s;
  log_2("new exp time: ", timeNew_s);
  return timeNew_s;
}

int32_t algorithmP2Dev(int32_t timeOld_s, float temperature){
  log_1("calculating new exposure time");
  log_3("exp std time: ", timeOld_s, " seconds");
  log_3("std temp: ", P2DEV_STD_TEMPERATURE, " degrees");
  log_3("sol. temp: ", temperature, " degrees");
	float timeOld_m = (float)timeOld_s/_SEC_per_MINUTE_;
	float f1 = (P2DEV_COEFF1[0]*pow(timeOld_m, 3.0) + P2DEV_COEFF1[1]*pow(timeOld_m, 2.0) + P2DEV_COEFF1[2]*timeOld_m + P2DEV_COEFF1[3]);
	log_2("f1 = ", f1);
	float f2 = (P2DEV_COEFF2[0]*pow(timeOld_m, 2.0) + P2DEV_COEFF2[1]*timeOld_m + P2DEV_COEFF2[2]);
	log_2("f2 = ", f2);
	f2 = (-1)/f2;
	log_2("f2 = ", f2);
  float result = pow((f1 / temperature), f2);
	log_2("res = ", result);
  int32_t timeNew_s = (int32_t)(result*_SEC_per_MINUTE_);
	timeAdjust = timeNew_s - timeOld_s;
  log_2("new exp time: ", timeNew_s);
  return timeNew_s;
}

int32_t algorithmP2Blix(int32_t timeOld_s, float temperature){
  log_1("calculating new exposure time");
  log_3("exp std time: ", timeOld_s, " seconds");
  log_3("std temp: ", P2BLIX_STD_TEMPERATURE, " degrees");
  log_3("sol. temp: ", temperature, " degrees");
  int32_t timeNew_s = (P2BLIX_STD_TEMPERATURE - temperature + (timeOld_s * P2BLIX_CONSTANT)) / P2BLIX_CONSTANT;
	timeAdjust = timeNew_s - timeOld_s;
  log_2("new exp time: ", timeNew_s);
  return timeNew_s;
}

int32_t algorithmP3Dev(int32_t timeOld_s, float temperature){
  log_1("calculating new exposure time");
  log_3("exp std time: ", timeOld_s, " seconds");
  log_3("std temp: ", P3DEV_STD_TEMPERATURE, " degrees");
  log_3("sol. temp: ", temperature, " degrees");
  int32_t timeNew_s = timeOld_s * exp(exp_e * (temperature - P3DEV_STD_TEMPERATURE));
	timeAdjust = timeNew_s - timeOld_s;
  log_2("new exp time: ", timeNew_s);
  return timeNew_s;
}

// ================================================================================================================
// DRV8838 Motor pin writes
// ================================================================================================================
void motor_run_forward(){
   digitalWrite(PIN_MOTOR_PHS, LOW);
   digitalWrite(PIN_MOTOR_EN, HIGH);
}
void motor_run_reverse(){
  digitalWrite(PIN_MOTOR_PHS, HIGH);
  digitalWrite(PIN_MOTOR_EN, HIGH);
}
void motor_stop(){
  digitalWrite(PIN_MOTOR_PHS, LOW);
  digitalWrite(PIN_MOTOR_EN, LOW);
}

// ================================================================================================================
// Auxiliary functions
// ================================================================================================================
void seconds2string(unsigned long seconds, char *ptr_buffer) {
    int i;
 
    const unsigned long s =  1;
    const unsigned long m = 60 * s;
 
    const unsigned long coeff[2] = { m, s };

    for ( i = 0; i < 2; i++ ){
        // ToDo: Reduced version, extend to hours etc if necessary
        unsigned long value;
        value   = seconds / coeff[i];
        seconds = seconds % coeff[i];
        // Print with zero padding
        if (value == 0){
          ptr_buffer += sprintf(ptr_buffer,"00");
        }else{
          if (value < 10){ptr_buffer += sprintf(ptr_buffer,"0");} 
          ptr_buffer += sprintf(ptr_buffer,"%lu",value);
        }
        // Separator
        if (i == 0){
          ptr_buffer += sprintf(ptr_buffer, ":");
        }
    }
}


void drawSpriteAgitation(IconColor color, Agitation agitation, LGFX_Sprite *ptr_sprite) {
	
	tImage image;
	switch (color)
	{
	case BACKGROUND_RED:
		image = spritesheet_red;
		break;
	case BACKGROUND_RED_YELLOW_ICON:
		image = spritesheet_redYellow;
		break;
	case BACKGROUND_BLACK:
		image = spritesheet_black;
		break;
	case BACKGROUND_GREY:
		image = spritesheet_grey;
		break;
	default:
		image = spritesheet_black;
		break;
	}
		
	int32_t x = 0, y;
	switch (agitation)
	{
	case AGITATION_OFF:
		y = -60;
		break;
	case AGITATION_STICK:
		y = 0;
		break;
	case AGITATION_ROTATIONAL:
		y = -30;
		break;
	default:
		x = 0;
		y = -90;
		break;
	}	
		
	ptr_sprite->pushImage(x, y, image.width, image.height, image.data);			
}

void drawSpriteCompensation(IconColor color, Compensation compensation, LGFX_Sprite *ptr_sprite) {
	
	tImage image;
	switch (color)
	{
	case BACKGROUND_RED:
		image = spritesheet_red;
		break;
	case BACKGROUND_RED_YELLOW_ICON:
		image = spritesheet_redYellow;
		break;
	case BACKGROUND_BLACK:
		image = spritesheet_black;
		break;
	case BACKGROUND_GREY:
		image = spritesheet_grey;
		break;
	default:
		image = spritesheet_black;
		break;
	}
		
	int32_t x = -30, y;
	switch (compensation)
	{
	case COMPENSATION_OFF:
		y = -90;
		break;
	case COMPENSATION_BW_DEV:
		y = 0;
		break;
	case COMPENSATION_TETENAL_DEV:
		y = -30;
		break;
	case COMPENSATION_TETENAL_BLIX:
		y = -30;
		break;
	case COMPENSATION_CINESTILL_DEV:
		y = -60;
		break;
	default:
		x = 0;
		y = -90;
		break;
	}	
		
	ptr_sprite->pushImage(x, y, image.width, image.height, image.data);			
}

void drawAddText(Compensation compensation, LGFX_Sprite *ptr_sprite) {
	
	ptr_sprite->setFont(&Sk_Modernist_Regular4pt7b);	
	switch (compensation)
	{
	case COMPENSATION_OFF:
		break;
	case COMPENSATION_BW_DEV:
		ptr_sprite->drawString("B&W", 269, 9);
		ptr_sprite->drawString("DEV", 269, 16);	
		break;
	case COMPENSATION_TETENAL_DEV:
		ptr_sprite->drawString("TETENAL", 250, 9);
		ptr_sprite->drawString("C-41 DEV", 249, 16);	
		break;
	case COMPENSATION_TETENAL_BLIX:
		ptr_sprite->drawString("TETENAL", 250, 9);
		ptr_sprite->drawString("C-41 BLIX", 248, 16);	
		break;
	case COMPENSATION_CINESTILL_DEV:
		ptr_sprite->drawString("CINESTILL", 242, 9);
		ptr_sprite->drawString("C-41 DEV", 249, 16);	
		break;
	default:
		break;
	}		
}

void drawSpriteBattery(int32_t soc, bool isCharging, LGFX_Sprite *ptr_sprite) {
	
	tImage image = spritesheet_batt_grey;
	int32_t y;	
	
	if (soc > 0) y = -90;
	if (soc > 25) y = -60;
	if (soc > 50) y = -30;
	if (soc > 75) y = 0;
	
	if (isCharging) y = -120;
		
	ptr_sprite->pushImage(0, y, image.width, image.height, image.data);			
}

void beepDone() {
	EasyBuzzer.stopBeep();
	//digitalWrite(PIN_BUZZER, LOW);
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\r\n",wakeup_reason); break;
  }
	
}

void goToSleep(){
	 gpio_pulldown_dis(BUTTON_EXT1_PIN);
	 gpio_pullup_en(BUTTON_EXT1_PIN);
		Serial.println("Going to sleep now");
		esp_sleep_enable_ext0_wakeup(BUTTON_EXT1_PIN,0); //1 = High, 0 = Low

		//If you were to use ext1, you would use it like
		// esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
		// esp_sleep_enable_ext1_wakeup(GPIO_NUM_35, ESP_EXT1_WAKEUP_ANY_HIGH);
		
		digitalWrite(PIN_POWER_DCDC, HIGH);
    GO_TO_SLEEP = false;
		//Go to sleep now
		esp_deep_sleep_start();
}

void processInactivityTimer(){
	
	inactiveTime_s = (millis() - lastPressTime_ms) / _mS_per_SEC_;

	if (inactiveTime_s > INACTIVITY_THRESHHOLD_s){ 
		GO_TO_SLEEP = true;
	}
}


void processPowerButton_isr(){
	//Serial.println("RELEASE_HANDLER");
	GO_TO_SLEEP = true;
}

void processBatterySoc(){
	if ((millis() - lastProcessBatterySoc) > GET_SOC_TIMEOUT){
		batterySoc = getBatterySoc();
		lastProcessBatterySoc = millis();
	}
}

float getBatterySoc(){
	#define MEASURE_NUM 20
    long sum = 0;                   // sum of samples taken
    float voltage_mv = 0.0;            // calculated voltage
    float output = 0.0;             //output value
    const float battery_max = 4200.0; //maximum voltage of battery
    const float battery_min = 3000.0;  //minimum voltage of battery before shutdown

    for (int i = 0; i < MEASURE_NUM; i++)
    {
        sum += analogRead(PIN_VOLTAGE_SENSE);
        //delayMicroseconds(1000);
    }
    // calculate the voltage
    voltage_mv = sum / (float)MEASURE_NUM;
    voltage_mv = (voltage_mv * 3300) / 4095.0; //for default reference voltage
		batteryVoltage_mv = voltage_mv*3.0;
    voltage_mv = batteryVoltage_mv/2.0;						//resistor divider attenuation
    // round value by two precision
    //voltage_mv = roundf(voltage_mv * 100) / 100;
    // Serial.print("voltage: ");
    // Serial.println(voltage_mv, 0);    
		int32_t soc = calculateSoc(voltage_mv);
    output = ((voltage_mv - battery_min) / (battery_max - battery_min)) * 100;
		return soc;
    // if (output < 100)
        // return output;
    // else
        // return 100.0f;	
}

int32_t calculateSoc(float voltage_mv){

	const float k[] = {1.8065268e-10, -2.7952603e-6, 1.6139918e-2, -4.1105357e1, 3.8916543e4};
	//const float k[] = {1.8065268e2, -2.7952603e3, 1.6139918e4, -4.1105357e4, 3.8916543e4};
	//y = 1,8065268E+02x4 - 2,7952603E+03x3 + 1,6139918E+04x2 - 4,1105357E+04x + 3,8916543E+04 //for V

	log_1(voltage_mv);
	float  soc = (k[0]*pow(voltage_mv, 4.0) + k[1]*pow(voltage_mv, 3.0) + k[2]*pow(voltage_mv, 2.0) + k[3]*voltage_mv + k[4]);
	log_1(soc);
	int32_t socInt = int32_t(soc + 0.5f);
	
	socInt = (socInt > 100) ? 100 : socInt;
	socInt = (socInt <   0) ?   0 : socInt;
	return socInt;
}



