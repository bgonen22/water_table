//#include "FastLED.h"
#include <Adafruit_NeoPixel.h>

//using namespace std;
#include <Wire.h>
#include <Adafruit_MCP23017.h>

// To use interrupts, you must include the AVR interrupt header:
//#include <avr/io.h>
//#include <avr/interrupt.h>

// Watch dog timer uses wdt_reset which resets the board if the loop is not called for
// a set amount of time - that is the device get "Stuck"
#include <avr/wdt.h>

#define NUM_OF_LEDS 525

#define LEDS_PIN 6

// delay between iterations
#define DELAY 2

#define NUM_OF_COLORS 25
// each iteration the color will jump in this value (0 for on color circle)
#define COLOR_JUMP 1

//the max radius of the circle
#define MAX_RADIUS 3

// Max power level
#define MAX_LEVEL 255

// the Teensy pin for interrupt
byte PinInt = 8;
byte PinInt2 = 9;
byte PinInt3 = 10;

#define NUM_OF_MCP 6
static const int button_map[4][2] = {{2,4},{4,2},{2,0},{0,2}}; // configuration of the buttons on the first block: 12,3,6,9
//static const int first_xy[5][2] = {{0,0},{0,5},{0,10},{0,15},{0,20}}; // the buttom left corner of the blocks
static const int mcp_block_map[][4] = {{0,1,2,3},{4,9,8,7},{6,5,10,11},{12,13,14,19},{18,17,16,15},{20,21,22,23},{24,-1,-1,-1}}; // map of the the control blocks of each mcp
// the blocks num:
// 4 9 14 19 24
// 3 8 13 18 23
// 2 7 12 17 22
// 1 6 11 16 21
// 0 5 10 15 20  

float increaseRadiusSize = 0.1;

// unconneted pin for randomize seed
#define UNCONNECTED_PIN 0

// max number of parallel circles
#define VECTOR_SIZE 50

#define MAP_SIZE 25
static const int ledsMap[MAP_SIZE*MAP_SIZE] PROGMEM  = {
 -1,1,2,3,-1,-1,208,209,210,-1,-1,211,212,213,-1,-1,418,419,420,-1,-1,421,422,423,-1,
 8,7,6,5,4,207,206,205,204,203,218,217,216,215,214,417,416,415,414,413,428,427,426,425,424,
 9,10,11,12,13,198,199,200,201,202,219,220,221,222,223,408,409,410,411,412,429,430,431,432,433,
 18,17,16,15,14,197,196,195,194,193,228,227,226,225,224,407,406,405,404,403,438,437,436,435,434,
 -1,19,20,21,-1,-1,190,191,192,-1,-1,229,230,231,-1,-1,400,401,402,-1,-1,439,440,441,-1,
 -1,24,23,22,-1,-1,189,188,187,-1,-1,234,233,232,-1,-1,399,398,397,-1,-1,444,443,442,-1,
 25,26,27,28,29,182,183,184,185,186,235,236,237,238,239,392,393,394,395,396,445,446,447,448,449,
 34,33,32,31,30,181,180,179,178,177,244,243,242,241,240,391,390,389,388,387,454,453,452,451,450,
 35,36,37,38,39,172,173,174,175,176,245,246,247,248,249,382,383,384,385,386,455,456,457,458,459,
 -1,42,41,40,-1,-1,171,170,169,-1,-1,252,251,250,-1,-1,381,380,379,-1,-1,462,461,460,-1,
 -1,43,44,45,-1,-1,166,167,168,-1,-1,253,254,255,-1,-1,376,377,378,-1,-1,463,464,465,-1,
 50,49,48,47,46,165,164,163,162,161,260,259,258,257,256,375,374,373,372,371,470,469,468,467,466,
 51,52,53,54,55,156,157,158,159,160,261,262,263,264,265,366,367,368,369,370,471,472,473,474,475,
 60,59,58,57,56,155,154,153,152,151,270,269,268,267,266,365,364,363,362,361,480,479,478,477,476,
 -1,61,62,63,-1,-1,148,149,150,-1,-1,271,272,273,-1,-1,358,359,360,-1,-1,481,482,483,-1,
 -1,66,65,64,-1,-1,147,146,145,-1,-1,276,275,274,-1,-1,357,356,355,-1,-1,486,485,484,-1,
 67,68,69,70,71,140,141,142,143,144,277,278,279,280,281,350,351,352,353,354,487,488,489,490,491,
 76,75,74,73,72,139,138,137,136,135,286,285,284,283,282,349,348,347,346,345,496,495,494,493,492,
 77,78,79,80,81,130,131,132,133,134,287,288,289,290,291,340,341,342,343,344,497,498,499,500,501,
 -1,84,83,82,-1,-1,129,128,127,-1,-1,294,293,292,-1,-1,339,338,337,-1,-1,504,503,502,-1,
 -1,85,86,87,-1,-1,124,125,126,-1,-1,295,296,297,-1,-1,334,335,336,-1,-1,505,506,507,-1,
 92,91,90,89,88,123,122,121,120,119,302,301,300,299,298,333,332,331,330,329,512,511,510,509,508,
 93,94,95,96,97,114,115,116,117,118,303,304,305,306,307,324,325,326,327,328,513,514,515,516,517,
 102,101,100,99,98,113,112,111,110,109,312,311,310,309,308,323,322,321,320,319,522,51,520,519,518,
 -1,103,104,105,-1,-1,106,107,108,-1,-1,313,314,315,-1,-1,316,317,318,-1,-1,523,524,525,-1
};

//CRGB leds[NUM_OF_LEDS];
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_OF_LEDS + 1, LEDS_PIN, NEO_GRB + NEO_KHZ800);
// prototypes
uint32_t getColor(byte color, byte user_power);
void clearAll();
int xy_to_pixel (int y,int x);
void OnInterupt();
void handleKeypress();
void FindButtonPressed (int x_start, int y_start, int reg,int &x, int &y);   
long get_rand_color();
void read_mcp_block(int status_reg,int first_x, int first_y);
void read_mcp(int mcp_num ) ;
void run_day_dream();
//

Adafruit_MCP23017 mcp[NUM_OF_MCP];

class circle 
{
  float radius;  
  int x;
  int y;
  int color;
  int level;  
  int size_dir; // if the cirle is geting bigger or smaller (1,-1)
  //int length_of_pixel_array = MAX_RADIUS*16;
  // --------------------------
  // set_pixel_level
  // --------------------------
  void set_pixel_level (int y,int x,int precent, int * pixel_precent) {
    int pixel = xy_to_pixel(y,x);
    //Serial.print("pixel_precent[pixel] "); Serial.print(pixel_precent[pixel]); Serial.print(" precent: "); Serial.println(precent);
    
    if (pixel_precent[pixel] > precent) {
      //Serial.print("pixel ");Serial.print(pixel);Serial.print(" pixel_precent ");Serial.print( pixel_precent[pixel] );Serial.print(" precent ");Serial.println(precent);
      return;// not overide pixel that got more intese form other calc
    } 
    
    pixel_precent[pixel] = precent;
     //if (pixel > length_of_pixel_array) { return;}
    int l = (level - (level*1.0*radius/MAX_RADIUS)) * precent/100;
    pixels.setPixelColor(pixel, getColor(color,l));
    //pixel_level[pixel] += precent;  
    //if (pixel_level[pixel] > 100) { pixel_level[pixel] = 100;}
    /*
     Serial.print("x y ");
    Serial.print(x);     
    Serial.print(" ");
    Serial.print(y);  
    Serial.print(" pixel ");                
    Serial.print(pixel);     
    Serial.print(" ");
    Serial.println(l);
    */
    
  }
  
  
  public:
  void advance_radius(float add) {
    //Serial.println(radius);
    radius+= add;
    //Serial.println(radius);
  }
  void set_x_y(int _x, int _y) {x = _x; y = _y;}
  float get_radius() {/*Serial.print("radius ");Serial.println(radius);*/return radius;}
  int get_x() {return x;}
  int get_y() {return y;}
  int get_color() {return color;}
  int get_level() {return level;}
  void advance_color (int jump) { color = (color +jump)%NUM_OF_COLORS;}
  int get_size_dir() {return size_dir;}
  void set_size_dir(int dir) {size_dir = dir;}
  

  void draw_shape() {
    int pixel_precent[NUM_OF_LEDS];
    memset(pixel_precent,0,sizeof(pixel_precent));
        
    for (int coord1 = 0 ; coord1 <= radius; coord1 = coord1+1) {      
      float coord2;      
      coord2 = sqrt(pow(radius,2)-pow(coord1,2));             
      int lower = (int)coord2;
      int mantisa = 100*( coord2 - lower);
      /*
      Serial.print("coord1 "); Serial.println(coord1);
      Serial.print("coord2 "); Serial.println(coord2);
      Serial.print("lower "); Serial.println(lower);
      Serial.print("mantisa "); Serial.println(mantisa);
      */
      set_pixel_level (y+coord1,x+lower,100- mantisa,pixel_precent);
      set_pixel_level (y+coord1,x-lower,100- mantisa,pixel_precent);
      set_pixel_level (y-coord1,x+lower,100- mantisa,pixel_precent);
      set_pixel_level (y-coord1,x-lower,100- mantisa,pixel_precent);
      set_pixel_level (y+lower,x+coord1,100- mantisa,pixel_precent);      
      set_pixel_level (y+lower,x-coord1,100- mantisa,pixel_precent);
      set_pixel_level (y-lower,x+coord1,100- mantisa,pixel_precent);
      set_pixel_level (y-lower,x-coord1,100- mantisa,pixel_precent);
      if (mantisa > 0) { 
        set_pixel_level (y+coord1,x+lower+1,mantisa,pixel_precent);
        set_pixel_level (y+coord1,x-lower-1,mantisa,pixel_precent);
        set_pixel_level (y-coord1,x+lower+1,mantisa,pixel_precent);
        set_pixel_level (y-coord1,x-lower-1,mantisa,pixel_precent);

        set_pixel_level (y+lower,x+coord1+1,mantisa,pixel_precent);
        set_pixel_level (y+lower,x-coord1-1,mantisa,pixel_precent);
        set_pixel_level (y-lower,x+coord1+1,mantisa,pixel_precent);
        set_pixel_level (y-lower,x-coord1-1,mantisa,pixel_precent);
      }   
    }    
     
    
  } //draw_shape
  circle () {};
  circle(int _x, int _y, float start_radius, int shape_color, int color_level) {
    x = _x;
    y = _y;
    radius = start_radius;    
    color = shape_color;
    level = color_level;
    size_dir = 1;
  }
  
};

// --------------- C_VECTOR ------------------
class c_vector
{
  int v_start;
  int v_end;
  circle c_vec[VECTOR_SIZE];
  int current_i;
  public:  
  void push_back(circle c) {
    int end_next = (v_end+1)%VECTOR_SIZE;
   // if (end_next == v_start) { return;}
    c_vec[v_end] = c;
    v_end = end_next;  
    Serial.print("v_end ");
    Serial.println(v_end);
      
  }

  void pop() {
    if (v_end == v_start) { return;}
    v_start = (v_start+1)%VECTOR_SIZE;    
    Serial.print("v_start ");
    Serial.println(v_start);
  }
  circle * start() {
    current_i = v_start;
    if (current_i == v_end) { return NULL;}
    return &c_vec[v_start];
  }
  circle * next() {
    if (current_i == v_end ) { return NULL;}        
    current_i = (current_i + 1) % VECTOR_SIZE;
    if (current_i == v_end) { return NULL;}
    return &c_vec[current_i];
  }
  void clear_vector() {
    v_start = 0;
    v_end = 0;    
  }
  c_vector () {
    v_start = 0;
    v_end = 0;        
  }
  
}; // end c_vetor
// ---------------------------------------------
volatile int interrupt_flag=0;

int easterEggMaskArray[4]; // This will be set for each easteregg corner

void init_watchdog() {
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  delayMicroseconds(1); // Need to wait a bit..
  WDOG_STCTRLH = 0x0001; // Enable WDG
  WDOG_TOVALL = 2000; // The next 2 lines sets the time-out value. This is the value that the watchdog timer compare itself to.
  WDOG_TOVALH = 0;
  WDOG_PRESC = 0; // This sets prescale clock so that the watchdog timer ticks at 1kHZ instead of the default 1kHZ/4 = 200 HZ
}
unsigned long  last_button_pressed_time;
unsigned long  ideal_time;
int its_day_dream;
int its_easter_egg;
//vector<circle> circle_vec;  
c_vector circle_vec;
void setup() {
//  Wire.begin ()
  //delay(1500); //TODO: should remove it
  Serial.begin(9600);
  init_watchdog();
  Serial.println("Starting up...");
  randomSeed(analogRead(UNCONNECTED_PIN));  
//  FastLED.addLeds<NEOPIXEL, LEDS_PIN>(leds, NUM_OF_LEDS);
  Serial.println("Init pixels...");
  pixels.begin();
  Serial.print("Initing ");
  Serial.print(NUM_OF_MCP);
  Serial.println(" MCP");
  
  for (int i = 0; i< NUM_OF_MCP; ++i) {     
    Serial.print("Init mcp ");
    Serial.println(i);
     mcp[i].begin(i);

    // We mirror INTA and INTB, so that only one line is required between MCP and Arduino for int reporting
    // The INTA/B will not be Floating 
    // INTs will be signaled with a LOW
    Serial.print("Setup interrupts for mcp");
    mcp[i].setupInterrupts(true,false,LOW);
    // Set GPI Pins 1-16 to Inputs Pulled High, change of state triggers Interrupt
    Serial.println("Init pins for mcp");
    for (int pin = 0; pin < 16; pin++)  {  
      mcp[i].pinMode(pin, INPUT);    
      mcp[i].pullUp(pin,HIGH); 
      mcp[i].setupInterruptPin(pin, CHANGE);
      Serial.print("Init pin");
      Serial.println(pin);
    }
    mcp[i].readGPIOAB();    // Resets MCP Interrupt    
  } 
  ideal_time = 0;
  last_button_pressed_time = 0;
  its_day_dream=0;
  its_easter_egg=0;
  clearAll(); 
  pixels.show();
  
  pinMode(PinInt, INPUT_PULLUP);
  pinMode(PinInt2, INPUT_PULLUP);
  pinMode(PinInt3, INPUT_PULLUP);

  //TODO test this new method 
  attachInterrupt(PinInt, OnInterupt, FALLING); 
  attachInterrupt(PinInt2, OnInterupt, FALLING); 
  attachInterrupt(PinInt3, OnInterupt, FALLING); 
  Serial.println("reset");

  for(int i=0;i<4;i++) {
    easterEggMaskArray[i] = 0;
  }

  // Create initialize circles
  circle c1(0,0,0,get_rand_color(),MAX_LEVEL);
  circle_vec.push_back(c1);
  circle c2(0,25,0,get_rand_color(),MAX_LEVEL);
  circle_vec.push_back(c2);
  circle c3(25,0,0,get_rand_color(),MAX_LEVEL);
  circle_vec.push_back(c3);
  circle c4(25,25,0,get_rand_color(),MAX_LEVEL);
  circle_vec.push_back(c4);

  circle c(15,10,0,get_rand_color(),MAX_LEVEL);
  circle_vec.push_back(c);
}
circle * it;


void update_watchdog() {
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts()
}

unsigned long TIME_TO_DO_DAY_DREAMING = 4000;

unsigned long TIME_TO_RESET_BUTTON = 1500;
unsigned long lastIntrruptTime = 0; //Starting from -TIME_TO_WAIT_ON_STUCK_MODE-1 so it wont get called on first call

//----------------------------
//  LOOP
//----------------------------
void loop() {
  clearAll();
  if (interrupt_flag) {
    handleKeypress();
  }

  // the program is alive...for now. 
  update_watchdog();

  //Day dreaming
  if(millis() - last_button_pressed_time > TIME_TO_DO_DAY_DREAMING) {            
      run_day_dream();               
      its_day_dream = 1;
  }
  
  run_vector();
    

    //Serial.println(millis() - ideal_time);
    if (millis() - ideal_time > TIME_TO_RESET_BUTTON) {
        // if there was no interrupt for 10 sec, maybe it got stuck so reseting
        //_reboot_Teensyduino_();
        for(int i=0;i<NUM_OF_MCP;i++) {
          mcp[i].readGPIOAB();
        }
        ideal_time = millis();
        Serial.println("Reset after idle time");
    }

    
    pixels.show();    
    delay(DELAY);     
    
}


int DAY_DREAM_DELAY = 5;
int MOVE_XY = 200;
int xy_counter = 0;
//----------------------------
//  run_vector
//----------------------------
void run_vector() {
   for (it = circle_vec.start(); it != NULL ; it = circle_vec.next()) {
        if (its_day_dream) {
          if (it->get_radius() >= MAX_RADIUS) {
            it->set_size_dir(-1);
            it->advance_radius(it->get_size_dir()*increaseRadiusSize);
          } else if (it->get_radius() <= 0) {
            it->set_size_dir(1);
            it->advance_radius(it->get_size_dir()*increaseRadiusSize);
          }
          if((xy_counter % MOVE_XY) == 0) {
             // in day dream the circke will move randomaly 
            int x = it -> get_x();
            int y = it -> get_y();
            int rand_x_min = -1; 
            int rand_x_max = 2;
            int rand_y_min = -1; 
            int rand_y_max = 2;
            if (x==0) {
              rand_x_min = 0;                        
            } else if (x==24) {
              rand_x_max = 1;            
            }
            if (y==0) {
              rand_y_min = 0;                        
            } else if (y==24) {
              rand_y_max = 0;            
            }
            //Serial.print("radius_min max: ");    
            //Serial.println(x);    
            //Serial.println(rand_x_min);    
            //Serial.println(rand_x_max);              
            //Serial.println(random (0,1));
            x += random (rand_x_min,rand_x_max);
            y += random (rand_y_min,rand_y_max);          
            it->set_x_y(x,y); 
          }
        } // end of if (its_day_dream)     
        if ( it ->get_radius() < MAX_RADIUS) {
          it->draw_shape();
          
          it->advance_radius(it->get_size_dir()*increaseRadiusSize);
          float r = it ->get_radius();
          int round_radius = (int)(100*r)%100;
          // Serial.print("r "); Serial.print(r);Serial.print(" (int)r "); Serial.print((int)r);Serial.print(" round radius ");Serial.println(round_radius);
        
          if (round_radius == 99 || round_radius == 1 || round_radius == 0 ) { 
            //Serial.println("color change");    
            it->advance_color(COLOR_JUMP); // when the radius advanced in int value, adbvance the color
          }          
          //Serial.print("radius: ");    
          //Serial.print(it->get_radius());    
          //Serial.print(" x: ");    
          //Serial.print(it -> get_x());    
          //Serial.print(" y: ");    
          //Serial.println(it -> get_y());    
  
       } else {   
         circle_vec.pop();              
       }   
      
    }  
}


//----------------------------
//  day_dream
//----------------------------
void run_day_dream() {  
  if (its_day_dream ) {return;}
  Serial.println("Day dreaming");
  int x = random(0,25);
  int y = random(0,25);
  circle c(x,y,0,get_rand_color(),MAX_LEVEL);
  circle_vec.push_back(c);           
  
}

//----------------------------
//  handleKeypress
//----------------------------
void handleKeypress() {
  Serial.println("handleKeypress!"); 
  detachInterrupt(PinInt);
  detachInterrupt(PinInt2);
  detachInterrupt(PinInt3);
  last_button_pressed_time = millis();
  if (its_day_dream) {
    // we are exiting day_dream, need to clear the vector
    circle_vec.clear_vector();    
    its_day_dream = 0;
  } 
  //delay(100); // for rebounce of interrupt - TODO - test this
  
 // int x = random(0,4);
//  int y = random(0,24);     
    
  // Get more information from the MCP from the INT
  //uint8_t pin=mcp.getLastInterruptPin();
  for (int i = 0; i < NUM_OF_MCP; ++i) {
    uint8_t val=mcp[i].getLastInterruptPinValue();
    if (val > 0) {continue;}
    Serial.print("Starting readmcp number "); 
    Serial.println(i); 
    read_mcp(i);
  }
  
   
  //while ( !(mcp.digitalRead(mcpPinA) && mcp.digitalRead(mcpPinB) && mcp.digitalRead(mcpPinC)));
  //PERHAPS MORE CLEAN INTRRUPT IS NEEDED
  for (int i = 0; i < NUM_OF_MCP; ++i) {
    mcp[i].readGPIOAB(); //RESET IT? Perhaps the code above
    //TODO: perhaps we just need to clear the mcp that was just interrupted?
  }  
  cli();
  interrupt_flag = 0;
  sei();

  attachInterrupt(PinInt, OnInterupt, FALLING);
  attachInterrupt(PinInt2, OnInterupt, FALLING);
  attachInterrupt(PinInt3, OnInterupt, FALLING);
  Serial.println("handleKeypress Handled");
  ideal_time = millis();
  last_button_pressed_time = millis();
}

//----------------------------
//  read_mcp_block
//  status_reg - the 4 buttons
//----------------------------
void read_mcp_block(int status_reg,int first_x, int first_y) {
  int x,y;
  Serial.print("status_reg "); 
  Serial.println(status_reg);   
  FindButtonPressed (first_x,first_y,status_reg,x,y);
  Serial.print("x y "); 
  Serial.print(x); 
  Serial.print(" "); 
  Serial.println(y);   
  if (x != -1) {
    circle c(x,y,0,get_rand_color(),MAX_LEVEL);
    circle_vec.push_back(c);
    CheckEasterEgg(x,y);
  }
}
//----------------------------
//  read_mcp
//----------------------------
void read_mcp(int mcp_num ) {
  Serial.print("read mcp "); 
  Serial.println(mcp_num); 
  uint8_t status_reg;
  int pin_status;
  int block_num,first_x,first_y;

  status_reg = ~(mcp[mcp_num].readGPIO(1));
  pin_status = status_reg & 15;  // Reading 4 bottom pins 00001111
  block_num = mcp_block_map[mcp_num][0];
  first_x = (block_num/5)*5;
  first_y = (block_num%5)*5;
  //Serial.println("about to read mcp block 1"); 
  read_mcp_block(pin_status,first_x,first_y);

  pin_status = status_reg >> 4; //Reading 4 top pins (MSB)
  block_num = mcp_block_map[mcp_num][1];
  first_x = (block_num/5)*5;
  first_y = (block_num%5)*5;
  //Serial.println("about to read mcp block 2"); 
  read_mcp_block(pin_status,first_x,first_y);

  status_reg = ~(mcp[mcp_num].readGPIO(0));      
  pin_status = status_reg & 15;  // Reading 4 bottom pins 00001111
  block_num = mcp_block_map[mcp_num][2];
  first_x = (block_num/5)*5;
  first_y = (block_num%5)*5;
  //Serial.println("about to read mcp block 3"); 
  read_mcp_block(pin_status,first_x,first_y);

  pin_status = status_reg >> 4;   //Reading 4 top pins (MSB)
  block_num = mcp_block_map[mcp_num][3];
  first_x = (block_num/5)*5;
  first_y = (block_num%5)*5;
  //Serial.println("about to read mcp block 4"); 

  read_mcp_block(pin_status,first_x,first_y);   
  
}


#define EASTER_EGG_TIMEOUT 1000 //The min amount of time to press all 4 corners
int easterEggPointer = 0;
//static const int easter_egg_button_xy[4][2] = {{0,0},{0,25},{25,0},{25,25}}; // All four corners x,y
static const int easter_egg_button_xy[4][2] = {{0,25},{2,19},{0,25},{0,25}}; // All four corners x,y FAKE FOR TESTING
unsigned long easter_egg_time;

void CheckEasterEgg(int x, int y) {
  Serial.println("Easter egg:");
  Serial.print(x);
  Serial.println(" ");
  Serial.print(y);

  unsigned long millies = millis();

  // Reset if timeout
  if(millies - easter_egg_time > EASTER_EGG_TIMEOUT){
    Serial.print("Clearing easteregg");
    for(int i=0;i<4;i++) {
      easterEggMaskArray[i] = 0; 
    }
  }
  
  //Check all 4 squares:
  for(int i=0;i<4;i++) {
    int easter_x = easter_egg_button_xy[i][0];
    int easter_y = easter_egg_button_xy[i][1];
    Serial.print("Easter x ");
    Serial.println(easter_x);
    Serial.print("Easter y ");
    Serial.println(easter_y);
    if (((x+5 >= easter_x) && (x-5 <= easter_x)) && 
      ((y+5 >= easter_y) && (y-5 <= easter_y))) {
        easterEggMaskArray[i] = 1;
        Serial.print("Correct easteregg square");
        easter_egg_time = millies;
      }
  }

  // Got to the last part - either reset or win
  bool is_all_lit = true;
  for(int i=0;i<4;i++) {
    if(easterEggMaskArray[i] != 1) {
      is_all_lit = false;
    }
  }
  
  
  if(is_all_lit) {
    easterEggPointer = 0;
    //its_easter_egg = 1;
    Serial.println("Start Easter Egg!!!");
    for(int i=0;i<random(5,10);i++) {
      circle c(i,i,0,get_rand_color(),MAX_LEVEL);
      circle_vec.push_back(c);
    }
  }
}


//----------------------------
//  get_rand_color
//----------------------------
long get_rand_color() {
  return random (0,NUM_OF_COLORS+1);
}
//----------------------------
//  FindButtonPressed
//----------------------------
void FindButtonPressed (int x_start, int y_start, int reg,int &x, int &y) {   
  switch (reg) {
    case 0:
     x=-1; y =-1;break; //No button was pressed
    case 1: //First button pressed
      x=x_start+button_map[0][0]; y=y_start+button_map[0][1]; break;
    case 2: //Second button pressed
      x=x_start+button_map[1][0]; y=y_start+button_map[1][1]; break;
    case 3: //2 buttons pressed - 1+2 - Corner is pressed
      x=x_start+(button_map[0][0]+button_map[1][0])/2; y=y_start+(button_map[0][1]+button_map[1][1])/2; break;
    case 4: //Third button pressed
      x=x_start+button_map[2][0]; y=y_start+button_map[2][1]; break;    
    case 6: //2 buttons pressed - Corner is pressed
      x=x_start+(button_map[1][0]+button_map[2][0])/2; y=y_start+(button_map[1][1]+button_map[2][1])/2; break;    
    case 8: // Forth button pressed
      x=x_start+button_map[3][0]; y=y_start+button_map[3][1]; break;
    case 9: //2 buttons pressed - Corner is pressed
      x=x_start+(button_map[0][0]+button_map[3][0])/2; y=y_start+(button_map[0][1]+button_map[3][1])/2; break;    
    case 12: //2 buttons pressed - 2 buttons from both side - "Center" is pressed
      x=x_start+(button_map[2][0]+button_map[3][0])/2; y=y_start+(button_map[2][1]+button_map[3][1])/2; break;         
    default: //Either 3 buttons or 4 buttons - "Center" is pressed
      x=x_start+(button_map[0][0]+button_map[2][0])/2; y=y_start+(button_map[0][1]+button_map[2][1])/2; break;
  }  
}


//----------------------------
//  clearAll
//----------------------------
void clearAll() {
  //Serial.println("clearAll");
  for (int i = 0; i< NUM_OF_LEDS; ++i) {
    pixels.setPixelColor(i,0);
  }
  //Serial.println("exit clearAll");

}

//----------------------------
//  getColor
//----------------------------
// Input a value 0 to NUM_OF_COLORS to get a color value.
// The colours are a transition r - g - b - back to r.
// the level is how brigt will be the light (0 to 255).
uint32_t getColor(byte color, byte user_power) {
  if (user_power==0) return pixels.Color(0, 0, 0); 
  float power;
  float level;
 // Serial.println(color);
  level = (1.0*user_power)/255;
  if(color < NUM_OF_COLORS/3) {
     power=1.0*color/(NUM_OF_COLORS/3)*255;
     return pixels.Color(level*(255 - power), 0, level*power); 
  } else if(color < 2*NUM_OF_COLORS/3) {
      color -= NUM_OF_COLORS/3;
      power=1.0*color/(NUM_OF_COLORS/3)*255;
  //    Serial.println(pixels.Color(0, level*power, level*(255 - power)));
      return pixels.Color(0, level*power, level*(255 - power));
  } else {
     color -= 2*NUM_OF_COLORS/3;
     power=1.0*color/(NUM_OF_COLORS/3)*255;
     return pixels.Color(level*power, level*(255 - power), 0);
  }
}

//----------------------------
//  xy_to_pixel
//----------------------------
int xy_to_pixel (int y,int x) {
  int inx = y*MAP_SIZE+x;
  //int led = pgm_read_word(&ledsMap[inx]) - 1;
  int led = ledsMap[inx] - 1;
 /* Serial.print("xy_to_pixel ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(x);
  Serial.print(" ");
  Serial.print("inx ");
  Serial.print(inx );
  Serial.print(" ");
  Serial.println(led);
  */
  if (x>=0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE && led > -1) {       
    return  led ;
  } else {
    return NUM_OF_LEDS;
  }
  
}

void OnInterupt() {
  cli();
  interrupt_flag = 1;
  sei();  
}
