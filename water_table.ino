//#include "FastLED.h"
#include <Adafruit_NeoPixel.h>

//#include <StandardCplusplus.h>
//#include <vector>
//#include <iterator>

//using namespace std;
#include <Wire.h>
#include <Adafruit_MCP23017.h>

// To use interrupts, you must include the AVR interrupt header:
//#include <avr/io.h>
//#include <avr/interrupt.h>


#define NUM_OF_LEDS 525

#define LEDS_PIN 6

// delay between iterations
#define DELAY 20 

#define NUM_OF_COLORS 25
// each iteration the color will jump in this value (0 for on color circle)
#define COLOR_JUMP 1

//the max radius of the circle
#define MAX_RADIUS 5

// Max power level
#define MAX_LEVEL 255

// the Teensy pin for interrupt
#define PinInt 8

// Two pins at the MCP (Ports A/B where some buttons have been setup.)
// Buttons connect the pin to grond, and pins are pulled up.
byte mcpPinA=1;
byte mcpPinB=28;

// unconneted pin for randomize seed
#define UNCONNECTED_PIN 0

// max number of parallel circles
#define VECTOR_SIZE 8

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

/*
// MCP23017 registers (everything except direction defaults to 0)

#define IODIRA   0x00   // IO direction  (0 = output, 1 = input (Default))
#define IODIRB   0x01
#define IOPOLA   0x02   // IO polarity   (0 = normal, 1 = inverse)
#define IOPOLB   0x03
#define GPINTENA 0x04   // Interrupt on change (0 = disable, 1 = enable)
#define GPINTENB 0x05
#define DEFVALA  0x06   // Default comparison for interrupt on change (interrupts on opposite)
#define DEFVALB  0x07
#define INTCONA  0x08   // Interrupt control (0 = interrupt on change from previous, 1 = interrupt on change from DEFVAL)
#define INTCONB  0x09
#define IOCON    0x0A   // IO Configuration: bank/mirror/seqop/disslw/haen/odr/intpol/notimp
//#define IOCON 0x0B  // same as 0x0A
#define GPPUA    0x0C   // Pull-up resistor (0 = disabled, 1 = enabled)
#define GPPUB    0x0D
#define INFTFA   0x0E   // Interrupt flag (read only) : (0 = no interrupt, 1 = pin caused interrupt)
#define INFTFB   0x0F
#define INTCAPA  0x10   // Interrupt capture (read only) : value of GPIO at time of last interrupt
#define INTCAPB  0x11
#define GPIOA    0x12   // Port value. Write to change, read to obtain value
#define GPIOB    0x13
#define OLLATA   0x14   // Output latch. Write to latch output.
#define OLLATB   0x15


#define PORT 0x20  // MCP23017 is on I2C port 0x20


// set register "reg" on expander to "data"
// for example, IO direction
void expanderWriteBoth (const byte reg, const byte data ) 
{
  Serial.print("start werite");
  Wire.beginTransmission (PORT);
  Wire.write (reg);
  Wire.write (data);  // port A
  Wire.write (data);  // port B
  Wire.endTransmission ();
  Serial.print("end werite");
} // end of expanderWrite

// read a byte from the expander
unsigned int expanderRead (const byte reg) 
{
  Wire.beginTransmission (PORT);
  Wire.write (reg);
  Wire.endTransmission ();
  Wire.requestFrom (PORT, 1);
  return Wire.read();
} // end of expanderRead

*/
//CRGB leds[NUM_OF_LEDS];
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_OF_LEDS + 1, LEDS_PIN, NEO_GRB + NEO_KHZ800);
// prototypes
uint32_t getColor(byte color, byte user_power);
void clearAll();
int xy_to_pixel (int y,int x);
void OnInterupt();
void handleKeypress();
//

Adafruit_MCP23017 mcp;

class circle 
{
  float radius;  
  int x;
  int y;
  int color;
  int level;  
  //int length_of_pixel_array = MAX_RADIUS*16;
  // --------------------------
  // set_pixel_level
  // --------------------------
  void set_pixel_level (int y,int x,int precent, int * pixel_precent) {
    int pixel = xy_to_pixel(y,x);
    if (pixel_precent[pixel] > precent) { return;} // not overide pixel that got more intese form other calc
    
    pixel_precent[pixel] = precent;
     //if (pixel > length_of_pixel_array) { return;}
    int l = (level - (level*1.0*radius/MAX_RADIUS)) * precent/100;
    pixels.setPixelColor(pixel, getColor(color,l));
    //pixel_level[pixel] += precent;  
    //if (pixel_level[pixel] > 100) { pixel_level[pixel] = 100;}
  /*   Serial.print("x y ");
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
  float get_radius() {/*Serial.print("radius ");Serial.println(radius);*/return radius;}
  //int get_x() {return center_x;}
  //int get_y() {return center_y;}
  //int get_color() {return color;}
  //int get_level() {return level;}
  void advance_color (int jump) { color = (color +jump)%NUM_OF_COLORS;}
  
  

  void draw_shape() {
    int pixel_precent[NUM_OF_LEDS];
    //int pixel_level[length_of_pixel_array];    
    //memset(pixel_level,0,sizeof(pixel_level));    
    for (int coord1 = 0 ; coord1 <= radius; coord1 = coord1+1) {
      //Serial.print("coord1 ");
      //Serial.println(coord1);
      float coord2;      
      coord2 = sqrt(pow(radius,2)-pow(coord1,2));             
      int lower = (int)coord2;
      int mantisa = 100*( coord2 - lower);
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
 
  c_vector () {
    v_start = 0;
    v_end = 0;        
  }
  
}; // end c_vetor
// ---------------------------------------------
volatile int interrupt_flag=0;

//vector<circle> circle_vec;  
c_vector circle_vec;
void setup() {
  Wire.begin ();  
  Serial.begin(9600);
  randomSeed(analogRead(UNCONNECTED_PIN));  
//  FastLED.addLeds<NEOPIXEL, LEDS_PIN>(leds, NUM_OF_LEDS);
  pixels.begin();
  mcp.begin();      // use default address 0
   
  // We mirror INTA and INTB, so that only one line is required between MCP and Arduino for int reporting
  // The INTA/B will not be Floating 
  // INTs will be signaled with a LOW
  mcp.setupInterrupts(true,false,LOW);

  // configuration for a button on port A
  // interrupt will triger when the pin is taken to ground by a pushbutton
  //mcp.pinMode(mcpPinA, INPUT);
  //mcp.pullUp(mcpPinA, HIGH);  // turn on a 100K pullup internally
  //mcp.setupInterruptPin(mcpPinA,FALLING); 

  // Set GPI Pins 1-16 to Inputs Pulled High, change of state triggers Interrupt
  for (int pin = 0; pin < 16; pin++)  {  
    mcp.pinMode(pin, INPUT);    
    mcp.pullUp(pin,HIGH); 
    mcp.setupInterruptPin(pin, CHANGE);
  }



  // similar, but on port B.
  mcp.pinMode(mcpPinB, INPUT);
  mcp.pullUp(mcpPinB, HIGH);  // turn on a 100K pullup internall
  mcp.setupInterruptPin(mcpPinB,FALLING);
  mcp.readGPIOAB();    // Resets MCP Interrupt

   clearAll(); 
  pixels.show();
  pinMode(PinInt, INPUT);   
  attachInterrupt(PinInt, OnInterupt, FALLING); 
  Serial.println("reset");

  /*
  // expander configuration register
  expanderWriteBoth (IOCON, 0b01100000); // mirror interrupts, disable sequential mode
 
  // enable pull-up on switches
  expanderWriteBoth (GPPUA, 0xFF);   // pull-up resistor for switch - both ports

  // invert polarity
  expanderWriteBoth (IOPOLA, 0xFF);  // invert polarity of signal - both ports
  
  // enable all interrupts
  expanderWriteBoth (GPINTENA, 0xFF); // enable interrupts - both ports

   // read from interrupt capture ports to clear them
  expanderRead (INTCAPA);
  expanderRead (INTCAPB);
  */
}
circle * it;
void loop() {
 // Serial.println("loop");    
  
 // Serial.println(getColor(1,100));
   clearAll();  
  if (interrupt_flag) {
    handleKeypress();    
  }
           
  
  //for (vector<circle>::iterator it = circle_vec.begin(); it != circle_vec.end(); ++ it) {
    for (it = circle_vec.start(); it != NULL ; it = circle_vec.next()) {
      if ( it ->get_radius() < MAX_RADIUS) {
    
        it->draw_shape();    
        it->advance_radius(1./50);
        float r = it ->get_radius();
        if (r - (int)r == 0 ) { 
          it->advance_color(COLOR_JUMP);
        }
        //  Serial.println(" ");
        Serial.println(it->get_radius());    
  
      } else {   
         circle_vec.pop();      
        //it = circle_vec.erase(it);          
      // it->advance_radius(-MAX_RADIUS);
      }
        
      pixels.show();
      delay(DELAY);     
      
    }
    
 
  
}
void handleKeypress() {
  interrupt_flag = 0;  
  int x = random(0,4);
  int y = random(0,24);    
  long rand_color = random (0,NUM_OF_COLORS+1);
  circle c(x,y,0,rand_color,MAX_LEVEL);      
  circle_vec.push_back(c);
    
  Serial.println("handleKeypress!");    
  
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

