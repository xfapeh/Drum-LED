#include <FastLED.h>
#include <MIDI.h>

#define DATA_PIN    4
#define NUM_LEDS    205
#define BRIGHTNESS  255

uint16_t reduceloop = 0;
//uint32_t  scale  = 255; // 255 ca. 1 s , 2550 ca. 7-10s??
//uint32_t  scalef = scale;
int minbright = 50;

//// TODO:
// scale[i] für 5 drums, für individuelles faden -- not necessary!

CRGB leds[NUM_LEDS];
CRGB starcolor[NUM_LEDS] = {};
CRGB lastrgb[NUM_LEDS];
    uint8_t huestar[128] = {};
    uint8_t brightstar[128] = {};

MIDI_CREATE_DEFAULT_INSTANCE();

void hitPad(int padsize, int padoffset, byte channelP, byte pitchP, int color, byte channel, byte pitch, byte velocity) {
  if ( channel == channelP && pitch == pitchP ) {
    velocity=min(2*velocity,127);    // scale up brightness a bit
    if ( pitch == 49 || pitch == 55 || pitch == 57 || pitch == 52 || pitch == 51 || pitch == 59 || pitch == 53 || 
         pitch == 46 || pitch == 26 || pitch == 42 || pitch == 22 || pitch == 44 || 
         pitch == 39 || pitch == 58 || pitch == 47 || pitch == 40 ) {
    for ( int i = padoffset ; i <= (padoffset+padsize) ; i++ ) {
      //leds[i] =       CHSV ( rgb2hsv_approximate( leds[i] ).h , 255 , 255*(uint8_t)velocity/127 ); // rainbow conversion -> FAIL!
      //hsv2rgb_spectrum( CHSV ( rgb2hsv_approximate( leds[i] ).h , 255 , 255*(uint8_t)velocity/127 ) , leds[i] );
      //scale = (uint32_t)velocity*scalef/127;
      hsv2rgb_spectrum( CHSV ( rgb2hsv_approximate( leds[i] ).h , 255 , 255*(uint8_t)velocity/127 ) , lastrgb[i] ); // save for color fade
      leds[i] = CRGB ( 255*(uint8_t)velocity/127 , 255*(uint8_t)velocity/127 , 255*(uint8_t)velocity/127 );         // flash white once
    }      
    FastLED.show();
    }
    if ( pitch == 36 ) {
    for ( int i = padoffset ; i <= (padoffset+padsize) ; i++ ) {
      hsv2rgb_spectrum( CHSV ( rgb2hsv_approximate( leds[i] ).h , 255 , 255*(uint8_t)velocity/127 ) , lastrgb[i] ); // save for color fade
      leds[i] = CRGB ( 0 , 0 , 0 );         // flash white once
    }      
    FastLED.show();
    }
    if ( pitch != 36 ) {
    for ( int i = padoffset ; i <= (padoffset+padsize) ; i++ ) {
      hsv2rgb_spectrum( CHSV ( rgb2hsv_approximate( lastrgb[i] ).h , 255 , 255*(uint8_t)velocity/127 ) , leds[i] ); // color fade
    }      
    FastLED.show();
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  delay( 3000); // 3 second delay for boot recovery, and a moment of silence
  FastLED.addLeds<WS2812B,DATA_PIN,GRB>(leds, NUM_LEDS)
        .setCorrection( TypicalLEDStrip );
        
  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  MIDI.setHandleNoteOn(handleNoteOn);  // Put only the name of the function
  MIDI.begin(0);   // faster?
}

void loop() {
  MIDI.read();   //  Alsa    0x90 Note on            009 038 080
  MIDI.read();   //  Alsa    0x80 Note off           009 038 064
  MIDI.read();   //  Serial  0x90 Note on            009 038 080  <- this is it!
  MIDI.read();   //  Serial  0x80 Note off           009 038 064
  //playsomedrums(1);
  
  colorchange();
  FastLED.show();

/*
// speed up / slow down fade out, adds some flicker to other drums...
  if ( scale > 0 ) {
      int powint=1;                  // speed up here 1: slow, 20: fast
      float scaler = pow(scale,powint)/pow(scalef,powint);
    for ( int i=0 ; i < NUM_LEDS ; i++ ) {
//      //leds[i] -= CHSV ( 0 , 0 , 10 ); 
      //leds[i] -= CRGB ( 1 , 1 , 1 ); 
      leds[i] = CRGB ( leds[i].r*scaler , leds[i].g*scaler , leds[i].b*scaler );
    }
    FastLED.show();
    if (millis()%1000 == 0) {           // slow down here: 1 = fast, 1000 = slower
      scale--;
    }
  }
  //if ( scale == 0 ) {
  //  //FastLED.clear();
  //}
  //delay(1000);
  //FastLED.show(); //position? -> slight flicker on low velocities
*/
}

void colorchange() {
  uint16_t wavenumber = 1000; // 2000 = 32 LEDs, 4000 = 16 LEDs
  uint16_t frequency  = 3;    // BPM, 2/60 = 1/30 Hz, 60/60 = 1 Hz

  //dynamic wavenumber and frequency
  wavenumber = beatsin16 ( 1 , 200 , 750 );  // works
  ////frequency = beatsin16 ( 2 , 10 , 20 );       // fail, phase glitch
  
  uint16_t hue16 = 0;
  //hue16 = beatsin16( frequency , 0 , 65535 , beatsin( 3 , 2 , 5 )*millis() , 64); // 16bit hue value, (BPM, min, max, timebase controls freq, phase)
  //hue16 = beatsin16( frequency , 0 , 65535 ); // 16bit hue value, sin (BPM, min, max)
  hue16 = 65535 - beat16( frequency );                // 16bit hue value, saw (BPM, min, max)

  for ( int i=0 ; i<NUM_LEDS ; i++ ) {
    hue16 += wavenumber;
    uint8_t hue8 = hue16 / 256;
    //CHSV lastled = rgb2hsv_approximate ( leds[i] );
    //leds[i] = CHSV ( hue8 , 255 , lastled.v );
    
    huestar[i*127/NUM_LEDS] = hue8;
    // problem: hue is undefined if brightness=0
    //brightstar[i*127/NUM_LEDS] = 255+0*lastled.v;      //brightstar bzw. lastled.v machen Ärger -> jetzt in brightset(); korrekt implementiert
    //brightstar[i*127/NUM_LEDS] = max(min((leds[i].r + leds[i].g + leds[i].b)/1,255),60);      //brightstar bzw. lastled.v machen Ärger
  }
  brightset();
  starset();  
  for ( int i=0 ; i < NUM_LEDS ; i++ ) {
    //nblend( leds[i], starcolor[i], 64);
    leds[i] =  starcolor[i];
  }
  
  //FastLED.show();
  //delay(5000);
}


void handleNoteOn(byte channel, byte pitch, byte velocity)
{
 // hitPad( padsize, padoffset, channelP, pitchP, color, channel, pitch, velocity)
    // Toms + Snare
    hitPad(44,   0, 10, 41, 0, channel, pitch, velocity);
    hitPad(44,  45, 10, 43, 0, channel, pitch, velocity);
    hitPad(34,  90, 10, 45, 0, channel, pitch, velocity);
    hitPad(34, 125, 10, 48, 0, channel, pitch, velocity);
    hitPad(44, 160, 10, 38, 0, channel, pitch, velocity);
    // Toms + Snare Rim
    hitPad(44,   0, 10, 39, 0, channel, pitch, velocity);
    hitPad(44,  45, 10, 58, 0, channel, pitch, velocity);
    hitPad(34,  90, 10, 47, 0, channel, pitch, velocity);
    hitPad(34, 125, 10, 50, 0, channel, pitch, velocity);
    hitPad(44, 160, 10, 40, 0, channel, pitch, velocity);
    // Cymbals Tip
    hitPad(10,  45, 10, 51, 0, channel, pitch, velocity);
    hitPad(10,  98, 10, 57, 0, channel, pitch, velocity);
    hitPad(10, 140, 10, 49, 0, channel, pitch, velocity);
    hitPad(10, 181, 10, 42, 0, channel, pitch, velocity);
    // Cymbals Rim
    hitPad(10,  45, 10, 59, 0, channel, pitch, velocity);
    hitPad(10,  98, 10, 52, 0, channel, pitch, velocity);
    hitPad(10, 140, 10, 55, 0, channel, pitch, velocity);
    hitPad(10, 181, 10, 22, 0, channel, pitch, velocity);
    // Ride + HH Rest
    hitPad(10,  45, 10, 53, 0, channel, pitch, velocity);
    hitPad(10, 181, 10, 46, 0, channel, pitch, velocity);
    hitPad(10, 181, 10, 26, 0, channel, pitch, velocity);
    hitPad(10, 181, 10, 44, 0, channel, pitch, velocity);
    // Bass
    hitPad(204, 0,  10, 36, 0, channel, pitch, velocity);
}

void playsomedrums(int i) {
  if ( i == 1 ) {
  EVERY_N_MILLISECONDS( 1000) {
    hitPad(44,   0, 10, 41, 0, 10, 41, 70);}
  EVERY_N_MILLISECONDS( 1437) {
    hitPad(44,  45, 10, 43, 0, 10, 43, 75);}
  EVERY_N_MILLISECONDS( 1960) {
    hitPad(34,  90, 10, 45, 0, 10, 45, 50);}
  EVERY_N_MILLISECONDS( 2610) {
    hitPad(34, 125, 10, 48, 0, 10, 48, 80);}
  EVERY_N_MILLISECONDS( 3088) {
    hitPad(44, 160, 10, 38, 0, 10, 38, 65);}
  }
}

void starset() 
{
  // Tom 4
       hsv2rgb_spectrum( CHSV( huestar[0] , 255, brightstar[0] ) , starcolor[19]  );
       hsv2rgb_spectrum( CHSV( huestar[0] , 255, brightstar[0] ) , starcolor[20]  );
       hsv2rgb_spectrum( CHSV( huestar[0] , 255, brightstar[0] ) , starcolor[21]  );
       hsv2rgb_spectrum( CHSV( huestar[0] , 255, brightstar[0] ) , starcolor[22]  );

       hsv2rgb_spectrum( CHSV( huestar[1] , 255, brightstar[1] ) , starcolor[16]  );
       hsv2rgb_spectrum( CHSV( huestar[1] , 255, brightstar[1] ) , starcolor[17]  );
       hsv2rgb_spectrum( CHSV( huestar[1] , 255, brightstar[1] ) , starcolor[18]  );
       hsv2rgb_spectrum( CHSV( huestar[1] , 255, brightstar[1] ) , starcolor[23]  );
      
       hsv2rgb_spectrum( CHSV( huestar[2] , 255, brightstar[2] ) , starcolor[14]  );
       hsv2rgb_spectrum( CHSV( huestar[2] , 255, brightstar[2] ) , starcolor[15]  );
       hsv2rgb_spectrum( CHSV( huestar[2] , 255, brightstar[2] ) , starcolor[24]  );
      
       hsv2rgb_spectrum( CHSV( huestar[3] , 255, brightstar[3] ) , starcolor[13]  );
       hsv2rgb_spectrum( CHSV( huestar[3] , 255, brightstar[3] ) , starcolor[25]  );
                                                  
       hsv2rgb_spectrum( CHSV( huestar[4] , 255, brightstar[4] ) , starcolor[12]  );
       hsv2rgb_spectrum( CHSV( huestar[4] , 255, brightstar[4] ) , starcolor[26]  );
      
       hsv2rgb_spectrum( CHSV( huestar[5] , 255, brightstar[5] ) , starcolor[11]  );
       hsv2rgb_spectrum( CHSV( huestar[5] , 255, brightstar[5] ) , starcolor[27]  );
      
       hsv2rgb_spectrum( CHSV( huestar[6] , 255, brightstar[6] ) , starcolor[9]   );
       hsv2rgb_spectrum( CHSV( huestar[6] , 255, brightstar[6] ) , starcolor[10]  );
      
       hsv2rgb_spectrum( CHSV( huestar[7] , 255, brightstar[7] ) , starcolor[8]   );
       hsv2rgb_spectrum( CHSV( huestar[7] , 255, brightstar[7] ) , starcolor[28]  );
      
       hsv2rgb_spectrum( CHSV( huestar[8] , 255, brightstar[8] ) , starcolor[6]   );
       hsv2rgb_spectrum( CHSV( huestar[8] , 255, brightstar[8] ) , starcolor[7]   );
       hsv2rgb_spectrum( CHSV( huestar[8] , 255, brightstar[8] ) , starcolor[29]  );
      
       hsv2rgb_spectrum( CHSV( huestar[9] , 255, brightstar[9] ) , starcolor[5]   );
      
       hsv2rgb_spectrum( CHSV( huestar[10], 255, brightstar[10]) , starcolor[3]   );
       hsv2rgb_spectrum( CHSV( huestar[10], 255, brightstar[10]) , starcolor[4]   );
       hsv2rgb_spectrum( CHSV( huestar[10], 255, brightstar[10]) , starcolor[30]  );
      
       hsv2rgb_spectrum( CHSV( huestar[11], 255, brightstar[11]) , starcolor[2]   );
      
       hsv2rgb_spectrum( CHSV( huestar[12], 255, brightstar[12]) , starcolor[1]   );
       hsv2rgb_spectrum( CHSV( huestar[12], 255, brightstar[12]) , starcolor[31]  );
      
       hsv2rgb_spectrum( CHSV( huestar[13], 255, brightstar[13]) , starcolor[0]   );
       hsv2rgb_spectrum( CHSV( huestar[13], 255, brightstar[13]) , starcolor[32]  );
      
       hsv2rgb_spectrum( CHSV( huestar[14], 255, brightstar[14]) , starcolor[33]  );
       hsv2rgb_spectrum( CHSV( huestar[14], 255, brightstar[14]) , starcolor[44]  );
      
       hsv2rgb_spectrum( CHSV( huestar[15], 255, brightstar[15]) , starcolor[34]  );
       hsv2rgb_spectrum( CHSV( huestar[15], 255, brightstar[15]) , starcolor[43]  );
      
       hsv2rgb_spectrum( CHSV( huestar[16], 255, brightstar[16]) , starcolor[35]  );
       hsv2rgb_spectrum( CHSV( huestar[16], 255, brightstar[16]) , starcolor[39]  );
       hsv2rgb_spectrum( CHSV( huestar[16], 255, brightstar[16]) , starcolor[40]  );
       hsv2rgb_spectrum( CHSV( huestar[16], 255, brightstar[16]) , starcolor[41]  );
       hsv2rgb_spectrum( CHSV( huestar[16], 255, brightstar[16]) , starcolor[42]  );
      
       hsv2rgb_spectrum( CHSV( huestar[17], 255, brightstar[17]) , starcolor[36]  );
       hsv2rgb_spectrum( CHSV( huestar[17], 255, brightstar[17]) , starcolor[37]  );
       hsv2rgb_spectrum( CHSV( huestar[17], 255, brightstar[17]) , starcolor[38]  );

   //  Tom 3                                          
       hsv2rgb_spectrum( CHSV( huestar[20], 255, brightstar[20]) , starcolor[58]  );
       hsv2rgb_spectrum( CHSV( huestar[20], 255, brightstar[20]) , starcolor[59]  );
      
       hsv2rgb_spectrum( CHSV( huestar[21], 255, brightstar[21]) , starcolor[55]  );
       hsv2rgb_spectrum( CHSV( huestar[21], 255, brightstar[21]) , starcolor[56]  );
       hsv2rgb_spectrum( CHSV( huestar[21], 255, brightstar[21]) , starcolor[57]  );
       hsv2rgb_spectrum( CHSV( huestar[21], 255, brightstar[21]) , starcolor[60]  );
       hsv2rgb_spectrum( CHSV( huestar[21], 255, brightstar[21]) , starcolor[61]  );
       hsv2rgb_spectrum( CHSV( huestar[21], 255, brightstar[21]) , starcolor[62]  );
      
       hsv2rgb_spectrum( CHSV( huestar[22], 255, brightstar[22]) , starcolor[53]  );      
       hsv2rgb_spectrum( CHSV( huestar[22], 255, brightstar[22]) , starcolor[54]  );
       hsv2rgb_spectrum( CHSV( huestar[22], 255, brightstar[22]) , starcolor[63]  );
      
       hsv2rgb_spectrum( CHSV( huestar[23], 255, brightstar[23]) , starcolor[52]  );
      
       hsv2rgb_spectrum( CHSV( huestar[24], 255, brightstar[24]) , starcolor[49]  );
       hsv2rgb_spectrum( CHSV( huestar[24], 255, brightstar[24]) , starcolor[50]  );
       hsv2rgb_spectrum( CHSV( huestar[24], 255, brightstar[24]) , starcolor[51]  );
       hsv2rgb_spectrum( CHSV( huestar[24], 255, brightstar[24]) , starcolor[64]  );
      
       hsv2rgb_spectrum( CHSV( huestar[25], 255, brightstar[25]) , starcolor[48]  );
       hsv2rgb_spectrum( CHSV( huestar[25], 255, brightstar[25]) , starcolor[65]  );
      
       hsv2rgb_spectrum( CHSV( huestar[26], 255, brightstar[26]) , starcolor[47]  );
      
       hsv2rgb_spectrum( CHSV( huestar[27], 255, brightstar[27]) , starcolor[46]  );
       hsv2rgb_spectrum( CHSV( huestar[27], 255, brightstar[27]) , starcolor[66]  );
      
       hsv2rgb_spectrum( CHSV( huestar[28], 255, brightstar[28]) , starcolor[45]  );
       hsv2rgb_spectrum( CHSV( huestar[28], 255, brightstar[28]) , starcolor[67]  );
       hsv2rgb_spectrum( CHSV( huestar[28], 255, brightstar[28]) , starcolor[89]  );
      
       hsv2rgb_spectrum( CHSV( huestar[29], 255, brightstar[29]) , starcolor[88]  );
      
       hsv2rgb_spectrum( CHSV( huestar[30], 255, brightstar[30]) , starcolor[68]  );
       hsv2rgb_spectrum( CHSV( huestar[30], 255, brightstar[30]) , starcolor[86]  );
       hsv2rgb_spectrum( CHSV( huestar[30], 255, brightstar[30]) , starcolor[87]  );
      
       hsv2rgb_spectrum( CHSV( huestar[31], 255, brightstar[31]) , starcolor[69]  );
       hsv2rgb_spectrum( CHSV( huestar[31], 255, brightstar[31]) , starcolor[85]  );
      
       hsv2rgb_spectrum( CHSV( huestar[32], 255, brightstar[32]) , starcolor[70]  );
       hsv2rgb_spectrum( CHSV( huestar[32], 255, brightstar[32]) , starcolor[84]  );
      
       hsv2rgb_spectrum( CHSV( huestar[33], 255, brightstar[33]) , starcolor[71]  );
       hsv2rgb_spectrum( CHSV( huestar[33], 255, brightstar[33]) , starcolor[83]  );
      
       hsv2rgb_spectrum( CHSV( huestar[34], 255, brightstar[34]) , starcolor[72]  );
       hsv2rgb_spectrum( CHSV( huestar[34], 255, brightstar[34]) , starcolor[73]  );
       hsv2rgb_spectrum( CHSV( huestar[34], 255, brightstar[34]) , starcolor[80]  );
       hsv2rgb_spectrum( CHSV( huestar[34], 255, brightstar[34]) , starcolor[81]  );
       hsv2rgb_spectrum( CHSV( huestar[34], 255, brightstar[34]) , starcolor[82]  );
      
       hsv2rgb_spectrum( CHSV( huestar[35], 255, brightstar[35]) , starcolor[74]  );
       hsv2rgb_spectrum( CHSV( huestar[35], 255, brightstar[35]) , starcolor[77]  );
       hsv2rgb_spectrum( CHSV( huestar[35], 255, brightstar[35]) , starcolor[78]  );
       hsv2rgb_spectrum( CHSV( huestar[35], 255, brightstar[35]) , starcolor[79]  );
      
       hsv2rgb_spectrum( CHSV( huestar[36], 255, brightstar[36]) , starcolor[75]  );
       hsv2rgb_spectrum( CHSV( huestar[36], 255, brightstar[36]) , starcolor[76]  );
      
   //  Tom 2                                          
       hsv2rgb_spectrum( CHSV( huestar[38], 255, brightstar[38]) , starcolor[98]  );
       hsv2rgb_spectrum( CHSV( huestar[38], 255, brightstar[38]) , starcolor[99]  );
       hsv2rgb_spectrum( CHSV( huestar[38], 255, brightstar[38]) , starcolor[100] );
      
       hsv2rgb_spectrum( CHSV( huestar[39], 255, brightstar[39]) , starcolor[95]  );
       hsv2rgb_spectrum( CHSV( huestar[39], 255, brightstar[39]) , starcolor[96]  );
       hsv2rgb_spectrum( CHSV( huestar[39], 255, brightstar[39]) , starcolor[97]  );
       hsv2rgb_spectrum( CHSV( huestar[39], 255, brightstar[39]) , starcolor[101] );
      
       hsv2rgb_spectrum( CHSV( huestar[40], 255, brightstar[40]) , starcolor[93]  );
       hsv2rgb_spectrum( CHSV( huestar[40], 255, brightstar[40]) , starcolor[94]  );
       hsv2rgb_spectrum( CHSV( huestar[40], 255, brightstar[40]) , starcolor[102] );
      
       hsv2rgb_spectrum( CHSV( huestar[41], 255, brightstar[41]) , starcolor[92]  );
       hsv2rgb_spectrum( CHSV( huestar[41], 255, brightstar[41]) , starcolor[103] );
      
       hsv2rgb_spectrum( CHSV( huestar[42], 255, brightstar[42]) , starcolor[90]  );
       hsv2rgb_spectrum( CHSV( huestar[42], 255, brightstar[42]) , starcolor[91]  );
       hsv2rgb_spectrum( CHSV( huestar[42], 255, brightstar[42]) , starcolor[104] );
      
       hsv2rgb_spectrum( CHSV( huestar[43], 255, brightstar[43]) , starcolor[124] );
      
       hsv2rgb_spectrum( CHSV( huestar[44], 255, brightstar[44]) , starcolor[105] );
       hsv2rgb_spectrum( CHSV( huestar[44], 255, brightstar[44]) , starcolor[122] );
       hsv2rgb_spectrum( CHSV( huestar[44], 255, brightstar[44]) , starcolor[123] );
      
       hsv2rgb_spectrum( CHSV( huestar[45], 255, brightstar[45]) , starcolor[106] );
       hsv2rgb_spectrum( CHSV( huestar[45], 255, brightstar[45]) , starcolor[121] );
      
       hsv2rgb_spectrum( CHSV( huestar[46], 255, brightstar[46]) , starcolor[107] );
       hsv2rgb_spectrum( CHSV( huestar[46], 255, brightstar[46]) , starcolor[120] );
      
       hsv2rgb_spectrum( CHSV( huestar[47], 255, brightstar[47]) , starcolor[108] );
       hsv2rgb_spectrum( CHSV( huestar[47], 255, brightstar[47]) , starcolor[118] );
       hsv2rgb_spectrum( CHSV( huestar[47], 255, brightstar[47]) , starcolor[119] );
      
       hsv2rgb_spectrum( CHSV( huestar[48], 255, brightstar[48]) , starcolor[109] );
       hsv2rgb_spectrum( CHSV( huestar[48], 255, brightstar[48]) , starcolor[110] );
       hsv2rgb_spectrum( CHSV( huestar[48], 255, brightstar[48]) , starcolor[116] );
       hsv2rgb_spectrum( CHSV( huestar[48], 255, brightstar[48]) , starcolor[117] );
      
       hsv2rgb_spectrum( CHSV( huestar[49], 255, brightstar[49]) , starcolor[111] );
       hsv2rgb_spectrum( CHSV( huestar[49], 255, brightstar[49]) , starcolor[112] );
       hsv2rgb_spectrum( CHSV( huestar[49], 255, brightstar[49]) , starcolor[113] );
       hsv2rgb_spectrum( CHSV( huestar[49], 255, brightstar[49]) , starcolor[114] );
       hsv2rgb_spectrum( CHSV( huestar[49], 255, brightstar[49]) , starcolor[115] );
      
   //  Tom 1                                          
       hsv2rgb_spectrum( CHSV( huestar[50], 255, brightstar[50]) , starcolor[127] );
       hsv2rgb_spectrum( CHSV( huestar[50], 255, brightstar[50]) , starcolor[128] );
      
       hsv2rgb_spectrum( CHSV( huestar[51], 255, brightstar[51]) , starcolor[125] );
       hsv2rgb_spectrum( CHSV( huestar[51], 255, brightstar[51]) , starcolor[126] );
       hsv2rgb_spectrum( CHSV( huestar[51], 255, brightstar[51]) , starcolor[129] );
       hsv2rgb_spectrum( CHSV( huestar[51], 255, brightstar[51]) , starcolor[130] );
      
       hsv2rgb_spectrum( CHSV( huestar[52], 255, brightstar[52]) , starcolor[131] );
       hsv2rgb_spectrum( CHSV( huestar[52], 255, brightstar[52]) , starcolor[157] );
       hsv2rgb_spectrum( CHSV( huestar[52], 255, brightstar[52]) , starcolor[158] );
       hsv2rgb_spectrum( CHSV( huestar[52], 255, brightstar[52]) , starcolor[159] );
      
       hsv2rgb_spectrum( CHSV( huestar[53], 255, brightstar[53]) , starcolor[132] );
       hsv2rgb_spectrum( CHSV( huestar[53], 255, brightstar[53]) , starcolor[133] );
       hsv2rgb_spectrum( CHSV( huestar[53], 255, brightstar[53]) , starcolor[156] );
      
       hsv2rgb_spectrum( CHSV( huestar[54], 255, brightstar[54]) , starcolor[134] );
       hsv2rgb_spectrum( CHSV( huestar[54], 255, brightstar[54]) , starcolor[154] );
       hsv2rgb_spectrum( CHSV( huestar[54], 255, brightstar[54]) , starcolor[155] );
      
       hsv2rgb_spectrum( CHSV( huestar[55], 255, brightstar[55]) , starcolor[135] );
       hsv2rgb_spectrum( CHSV( huestar[55], 255, brightstar[55]) , starcolor[152] );
       hsv2rgb_spectrum( CHSV( huestar[55], 255, brightstar[55]) , starcolor[153] );
      
       hsv2rgb_spectrum( CHSV( huestar[56], 255, brightstar[56]) , starcolor[136] );
       hsv2rgb_spectrum( CHSV( huestar[56], 255, brightstar[56]) , starcolor[150] );
       hsv2rgb_spectrum( CHSV( huestar[56], 255, brightstar[56]) , starcolor[151] );
      
       hsv2rgb_spectrum( CHSV( huestar[57], 255, brightstar[57]) , starcolor[137] );
       hsv2rgb_spectrum( CHSV( huestar[57], 255, brightstar[57]) , starcolor[149] );
      
       hsv2rgb_spectrum( CHSV( huestar[58], 255, brightstar[58]) , starcolor[138] );
       hsv2rgb_spectrum( CHSV( huestar[58], 255, brightstar[58]) , starcolor[139] );
       hsv2rgb_spectrum( CHSV( huestar[58], 255, brightstar[58]) , starcolor[147] );
       hsv2rgb_spectrum( CHSV( huestar[58], 255, brightstar[58]) , starcolor[148] );
      
       hsv2rgb_spectrum( CHSV( huestar[59], 255, brightstar[59]) , starcolor[140] );
       hsv2rgb_spectrum( CHSV( huestar[59], 255, brightstar[59]) , starcolor[144] );
       hsv2rgb_spectrum( CHSV( huestar[59], 255, brightstar[59]) , starcolor[145] );
       hsv2rgb_spectrum( CHSV( huestar[59], 255, brightstar[59]) , starcolor[146] );
      
       hsv2rgb_spectrum( CHSV( huestar[60], 255, brightstar[60]) , starcolor[141] );
       hsv2rgb_spectrum( CHSV( huestar[60], 255, brightstar[60]) , starcolor[142] );
       hsv2rgb_spectrum( CHSV( huestar[60], 255, brightstar[60]) , starcolor[143] );

   //  Snare                                          
       hsv2rgb_spectrum( CHSV( huestar[60], 255, brightstar[61]) , starcolor[167] ); // changed to 61!

       hsv2rgb_spectrum( CHSV( huestar[61], 255, brightstar[61]) , starcolor[164] );
       hsv2rgb_spectrum( CHSV( huestar[61], 255, brightstar[61]) , starcolor[165] );
       hsv2rgb_spectrum( CHSV( huestar[61], 255, brightstar[61]) , starcolor[166] );
       hsv2rgb_spectrum( CHSV( huestar[61], 255, brightstar[61]) , starcolor[168] );
      
       hsv2rgb_spectrum( CHSV( huestar[62], 255, brightstar[62]) , starcolor[162] );
       hsv2rgb_spectrum( CHSV( huestar[62], 255, brightstar[62]) , starcolor[163] );
       hsv2rgb_spectrum( CHSV( huestar[62], 255, brightstar[62]) , starcolor[169] );
      
       hsv2rgb_spectrum( CHSV( huestar[63], 255, brightstar[63]) , starcolor[160] );
       hsv2rgb_spectrum( CHSV( huestar[63], 255, brightstar[63]) , starcolor[161] );
       hsv2rgb_spectrum( CHSV( huestar[63], 255, brightstar[63]) , starcolor[170] );
      
       hsv2rgb_spectrum( CHSV( huestar[64], 255, brightstar[64]) , starcolor[171] );
       hsv2rgb_spectrum( CHSV( huestar[64], 255, brightstar[64]) , starcolor[203] );
       hsv2rgb_spectrum( CHSV( huestar[64], 255, brightstar[64]) , starcolor[204] );
      
       hsv2rgb_spectrum( CHSV( huestar[65], 255, brightstar[65]) , starcolor[202] );
      
       hsv2rgb_spectrum( CHSV( huestar[66], 255, brightstar[66]) , starcolor[172] );
       hsv2rgb_spectrum( CHSV( huestar[66], 255, brightstar[66]) , starcolor[201] );
      
       hsv2rgb_spectrum( CHSV( huestar[67], 255, brightstar[67]) , starcolor[173] );
       hsv2rgb_spectrum( CHSV( huestar[67], 255, brightstar[67]) , starcolor[200] );
      
       hsv2rgb_spectrum( CHSV( huestar[68], 255, brightstar[68]) , starcolor[199] );
      
       hsv2rgb_spectrum( CHSV( huestar[69], 255, brightstar[69]) , starcolor[174] );
       hsv2rgb_spectrum( CHSV( huestar[69], 255, brightstar[69]) , starcolor[198] );
      
       hsv2rgb_spectrum( CHSV( huestar[70], 255, brightstar[70]) , starcolor[196] );
       hsv2rgb_spectrum( CHSV( huestar[70], 255, brightstar[70]) , starcolor[197] );
      
       hsv2rgb_spectrum( CHSV( huestar[71], 255, brightstar[71]) , starcolor[175] );
      
       hsv2rgb_spectrum( CHSV( huestar[72], 255, brightstar[72]) , starcolor[194] );
       hsv2rgb_spectrum( CHSV( huestar[72], 255, brightstar[72]) , starcolor[195] );
      
       hsv2rgb_spectrum( CHSV( huestar[73], 255, brightstar[73]) , starcolor[176] );
       hsv2rgb_spectrum( CHSV( huestar[73], 255, brightstar[73]) , starcolor[193] );
      
       hsv2rgb_spectrum( CHSV( huestar[74], 255, brightstar[74]) , starcolor[177] );
       hsv2rgb_spectrum( CHSV( huestar[74], 255, brightstar[74]) , starcolor[192] );
      
       hsv2rgb_spectrum( CHSV( huestar[75], 255, brightstar[75]) , starcolor[191] );
      
       hsv2rgb_spectrum( CHSV( huestar[76], 255, brightstar[76]) , starcolor[178] );
       hsv2rgb_spectrum( CHSV( huestar[76], 255, brightstar[76]) , starcolor[189] );
       hsv2rgb_spectrum( CHSV( huestar[76], 255, brightstar[76]) , starcolor[190] );
      
       hsv2rgb_spectrum( CHSV( huestar[77], 255, brightstar[77]) , starcolor[179] );
       hsv2rgb_spectrum( CHSV( huestar[77], 255, brightstar[77]) , starcolor[188] );
      
       hsv2rgb_spectrum( CHSV( huestar[78], 255, brightstar[78]) , starcolor[180] );
       hsv2rgb_spectrum( CHSV( huestar[78], 255, brightstar[78]) , starcolor[186] );
       hsv2rgb_spectrum( CHSV( huestar[78], 255, brightstar[78]) , starcolor[187] );
      
       hsv2rgb_spectrum( CHSV( huestar[79], 255, brightstar[79]) , starcolor[181] );
       hsv2rgb_spectrum( CHSV( huestar[79], 255, brightstar[79]) , starcolor[183] );
       hsv2rgb_spectrum( CHSV( huestar[79], 255, brightstar[79]) , starcolor[184] );
       hsv2rgb_spectrum( CHSV( huestar[79], 255, brightstar[79]) , starcolor[185] );
      
       hsv2rgb_spectrum( CHSV( huestar[80], 255, brightstar[80]) , starcolor[182] );
}

void brightset () {
      //brightstar[0] = rgb2hsv_approximate ( leds[19] ).v;
      //brightstar[0] = max(max(leds[19].r,leds[19].g),leds[19].b);
// obne starcolor column löschen, dann
// cat input.raw2 | tr -d "," | awk '{print $6 " = max(max(max(leds"$1".r,leds"$1".g),leds"$1".b),minbright);"}' | awk '/^bright/' | sort -u -k1,1 | sort -n -t[ -k2,2
// cat input.raw2 | tr -d "," | awk '{print $6 " = max(leds"$1".r+leds"$1".g+leds"$1".b,(unsigned char)minbright);"}' | awk '/^bright/' | sort -u -k1,1 | sort -n -t[ -k2,2
// brightness conversion loses about 1 - 4 (e.g. 255 -> 251), luckily this lead to a fade!

brightstar[0] = max(leds[19].r+leds[19].g+leds[19].b,minbright);
brightstar[1] = max(leds[16].r+leds[16].g+leds[16].b,minbright);
brightstar[2] = max(leds[14].r+leds[14].g+leds[14].b,minbright);
brightstar[3] = max(leds[13].r+leds[13].g+leds[13].b,minbright);
brightstar[4] = max(leds[12].r+leds[12].g+leds[12].b,minbright);
brightstar[5] = max(leds[11].r+leds[11].g+leds[11].b,minbright);
brightstar[6] = max(leds[9].r+leds[9].g+leds[9].b,minbright);
brightstar[7] = max(leds[8].r+leds[8].g+leds[8].b,minbright);
brightstar[8] = max(leds[6].r+leds[6].g+leds[6].b,minbright);
brightstar[9] = max(leds[5].r+leds[5].g+leds[5].b,minbright);
brightstar[10] = max(leds[3].r+leds[3].g+leds[3].b,minbright);
brightstar[11] = max(leds[2].r+leds[2].g+leds[2].b,minbright);
brightstar[12] = max(leds[1].r+leds[1].g+leds[1].b,minbright);
brightstar[13] = max(leds[0].r+leds[0].g+leds[0].b,minbright);
brightstar[14] = max(leds[33].r+leds[33].g+leds[33].b,minbright);
brightstar[15] = max(leds[34].r+leds[34].g+leds[34].b,minbright);
brightstar[16] = max(leds[35].r+leds[35].g+leds[35].b,minbright);
brightstar[17] = max(leds[36].r+leds[36].g+leds[36].b,minbright);
brightstar[20] = max(leds[58].r+leds[58].g+leds[58].b,minbright);
brightstar[21] = max(leds[55].r+leds[55].g+leds[55].b,minbright);
brightstar[22] = max(leds[53].r+leds[53].g+leds[53].b,minbright);
brightstar[23] = max(leds[52].r+leds[52].g+leds[52].b,minbright);
brightstar[24] = max(leds[49].r+leds[49].g+leds[49].b,minbright);
brightstar[25] = max(leds[48].r+leds[48].g+leds[48].b,minbright);
brightstar[26] = max(leds[47].r+leds[47].g+leds[47].b,minbright);
brightstar[27] = max(leds[46].r+leds[46].g+leds[46].b,minbright);
brightstar[28] = max(leds[45].r+leds[45].g+leds[45].b,minbright);
brightstar[29] = max(leds[88].r+leds[88].g+leds[88].b,minbright);
brightstar[30] = max(leds[68].r+leds[68].g+leds[68].b,minbright);
brightstar[31] = max(leds[69].r+leds[69].g+leds[69].b,minbright);
brightstar[32] = max(leds[70].r+leds[70].g+leds[70].b,minbright);
brightstar[33] = max(leds[71].r+leds[71].g+leds[71].b,minbright);
brightstar[34] = max(leds[72].r+leds[72].g+leds[72].b,minbright);
brightstar[35] = max(leds[74].r+leds[74].g+leds[74].b,minbright);
brightstar[36] = max(leds[75].r+leds[75].g+leds[75].b,minbright);
brightstar[38] = max(leds[98].r+leds[98].g+leds[98].b,minbright);
brightstar[39] = max(leds[95].r+leds[95].g+leds[95].b,minbright);
brightstar[40] = max(leds[93].r+leds[93].g+leds[93].b,minbright);
brightstar[41] = max(leds[92].r+leds[92].g+leds[92].b,minbright);
brightstar[42] = max(leds[90].r+leds[90].g+leds[90].b,minbright);
brightstar[43] = max(leds[124].r+leds[124].g+leds[124].b,minbright);
brightstar[44] = max(leds[105].r+leds[105].g+leds[105].b,minbright);
brightstar[45] = max(leds[106].r+leds[106].g+leds[106].b,minbright);
brightstar[46] = max(leds[107].r+leds[107].g+leds[107].b,minbright);
brightstar[47] = max(leds[108].r+leds[108].g+leds[108].b,minbright);
brightstar[48] = max(leds[109].r+leds[109].g+leds[109].b,minbright);
brightstar[49] = max(leds[111].r+leds[111].g+leds[111].b,minbright);
brightstar[50] = max(leds[127].r+leds[127].g+leds[127].b,minbright);
brightstar[51] = max(leds[125].r+leds[125].g+leds[125].b,minbright);
brightstar[52] = max(leds[131].r+leds[131].g+leds[131].b,minbright);
brightstar[53] = max(leds[132].r+leds[132].g+leds[132].b,minbright);
brightstar[54] = max(leds[134].r+leds[134].g+leds[134].b,minbright);
brightstar[55] = max(leds[135].r+leds[135].g+leds[135].b,minbright);
brightstar[56] = max(leds[136].r+leds[136].g+leds[136].b,minbright);
brightstar[57] = max(leds[137].r+leds[137].g+leds[137].b,minbright);
brightstar[58] = max(leds[138].r+leds[138].g+leds[138].b,minbright);
brightstar[59] = max(leds[140].r+leds[140].g+leds[140].b,minbright);
brightstar[60] = max(leds[141].r+leds[141].g+leds[141].b,minbright);
brightstar[61] = max(leds[164].r+leds[164].g+leds[164].b,minbright);
brightstar[62] = max(leds[162].r+leds[162].g+leds[162].b,minbright);
brightstar[63] = max(leds[160].r+leds[160].g+leds[160].b,minbright);
brightstar[64] = max(leds[171].r+leds[171].g+leds[171].b,minbright);
brightstar[65] = max(leds[202].r+leds[202].g+leds[202].b,minbright);
brightstar[66] = max(leds[172].r+leds[172].g+leds[172].b,minbright);
brightstar[67] = max(leds[173].r+leds[173].g+leds[173].b,minbright);
brightstar[68] = max(leds[199].r+leds[199].g+leds[199].b,minbright);
brightstar[69] = max(leds[174].r+leds[174].g+leds[174].b,minbright);
brightstar[70] = max(leds[196].r+leds[196].g+leds[196].b,minbright);
brightstar[71] = max(leds[175].r+leds[175].g+leds[175].b,minbright);
brightstar[72] = max(leds[194].r+leds[194].g+leds[194].b,minbright);
brightstar[73] = max(leds[176].r+leds[176].g+leds[176].b,minbright);
brightstar[74] = max(leds[177].r+leds[177].g+leds[177].b,minbright);
brightstar[75] = max(leds[191].r+leds[191].g+leds[191].b,minbright);
brightstar[76] = max(leds[178].r+leds[178].g+leds[178].b,minbright);
brightstar[77] = max(leds[179].r+leds[179].g+leds[179].b,minbright);
brightstar[78] = max(leds[180].r+leds[180].g+leds[180].b,minbright);
brightstar[79] = max(leds[181].r+leds[181].g+leds[181].b,minbright);
brightstar[80] = max(leds[182].r+leds[182].g+leds[182].b,minbright);
}
