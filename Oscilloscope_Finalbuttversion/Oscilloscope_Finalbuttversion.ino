#include <Wire.h>              // librería para iniciar la comunicación I2C con el arduino.         
#include <Adafruit_SSD1306.h>  // librería para el uso la pantalla OLED SSD1306 128x64              
#include <Adafruit_GFX.h>      // librería de funciones gráficas para visualizar las señales        
#include <EEPROM.h>            // librería para guardar ajustes de configuración durante el tiempo de ejecución      



#define SCREEN_WIDTH 128         // anchura de la pantalla
#define SCREEN_HEIGHT 64        // altura de la pantalla
#define OLED_RESET     -1      // inicialización del pin reset para algunos tipos de pantallas

  // Declaración del dispositivo SSD1306 para su comunicación I2C (la variable Wire de la librería habilita los pines SDA y SCL)
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);   
/////Adafruit_SH1106 oled(OLED_RESET);        // si requiere pantalla SH1106

#define REC_LENG 200                    // tamaño del buffer de datos de la onda                         
#define MIN_TRIG_SWING 5                // valor mínimo de inestabilidad para sincronización 



  // Rango de matrices con valores que serán guardados en memoria flash
const char vRangeName[10][5] PROGMEM = {"A50V", "A 5V", " 50V", " 20V", " 10V", "  5V", "  2V", "  1V", "0.5V", "0.2V"}; // matriz con caracteres para visualización en el eje y. 
const char * const vstring_table[] PROGMEM = {vRangeName[0], vRangeName[1], vRangeName[2], vRangeName[3], vRangeName[4], vRangeName[5], vRangeName[6], vRangeName[7], vRangeName[8], vRangeName[9]};
const char hRangeName[10][6] PROGMEM = {"200ms", "100ms", " 50ms", " 20ms", " 10ms", "  5ms", "  2ms", "  1ms", "500us", "200us"};  // matriz con caracteres para visualización en el eje x.
const char * const hstring_table[] PROGMEM = {hRangeName[0], hRangeName[1], hRangeName[2], hRangeName[3], hRangeName[4], hRangeName[5], hRangeName[6], hRangeName[7], hRangeName[8], hRangeName[9]};
const PROGMEM float hRangeValue[] = { 0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001, 0.5e-3, 0.2e-3}; // matriz con valores para visualización en el eje x. ( = 25pix on screen)

int waveBuff[REC_LENG];        // inicia el buffer para señales con su tamaño          wave form buffer (RAM remaining capacity is barely)
char chrBuff[8];               // display string buffer         inicio
char hScale[] = "xxxAs";       // horizontal scale character
char vScale[] = "xxxx";        // vartical scale

float lsb5V = 0.00566826;      // Coeficiente de sensibilidad para rango 5V, servirá para dividir los valores de voltaje hasta este rango [1.1*630/(1024*120)]
float lsb50V = 0.05243212;     // Coeficiente de sensibilidad para rango 50V, servirá para dividir la visualización hasta este rango [1.1*520.91/(1024*10.91)]

volatile int vRange;           // contendrá codificados los caracteres para los valores en el eje y. 0:A50V,  1:A 5V,  2:50V,  3:20V,  4:10V,  5:5V,  6:2V,  7:1V,  8:0.5V,  9:0.2V
volatile int hRange;           // contendrá codificados los caracteres para los valores en el eje x. 0:200ms, 1:100ms, 2:50ms, 3:20ms, 4:10ms, 5:5ms, 6;2ms, 7:1ms, 8:500us, 9;200us
volatile boolean switchPushed;
volatile int trigD;            // trigger slope flag,     0:positive 1:negative
volatile int scopeP;           // operation scope position number. 0:Veratical, 1:Hrizontal, 2:Trigger slope

volatile int saveTimer;        // Tiempo hasta guardar en la EEPROM
int timeExec;                  // Tiempo de ejecución para los ajustes definidos           approx. execution time of current range setting (ms)

int holdButt = 11;
volatile boolean pressedButt = false; 
int dataMin;                   // buffer minimum value (smallest=0)
int dataMax;                   //        maximum value (largest=1023)
int dataAve;                   // 10 x average value (use 10x value to keep accuracy. so, max=10230)
int rangeMax;                  // buffer value to graph full swing
int rangeMin;                  // buffer value of graph botto
int rangeMaxDisp;              // display value of max. (100x value)
int rangeMinDisp;              // display value if min.
int trigP;                     // trigger position pointer on data buffer
boolean trigSync;              // flag of trigger detected
int att10x;                    // 10x attenetor ON (effective when 1)

float waveFreq;                // Frecuencia de la onda en Hz
float waveDuty;                // porcentaje del ciclo de trabajo

void setup() {
  //pinMode(2, INPUT);             // interrupción si alguno de los botones es pulsado
  //pinMode(8, INPUT_PULLUP);             // botón de selección
  //pinMode(9, INPUT_PULLUP);             // Subida
 // pinMode(10, INPUT_PULLUP);            // Bajada
  pinMode(holdButt, INPUT_PULLUP);            // botón de mantener
  pinMode(12, INPUT);                   // atenuador * 1/10 (Off=High-Z, Enable=Output Low)
  pinMode(13, OUTPUT);                  // LED de arduino, parpadea cada vez que se actualizan los datos
  

  oled.begin(SSD1306_SWITCHCAPVCC,  0x3C); //iniciar pantalla

  //auxFunctions();                       // Voltage measure (never return)
  loadEEPROM();                         // read last settings from EEPROM
  analogReference(INTERNAL);            // ADC full scale = 1.1V
  
  
  startScreen();                        // display start message
}

void loop() {
  

  setConditions();                      // set measurment conditions
  digitalWrite(13, HIGH);               // flash LED
  readWave();                           // función para leer la forma de onda y guardarla en la memoria del buffer
  digitalWrite(13, LOW);                // stop LED
  setConditions();                      // set measurment conditions again (reflect change during measure)
  dataAnalize();                        // analize data
  writeCommonImage();                   // write fixed screen image (2.6ms)
  plotData();                           // plot waveform (10-18ms)
  dispInf();                            // display information (6.5-8.5ms)
  oled.display();                       // send screen buffer to OLED (37ms)
  saveEEPROM();                         // save settings to EEPROM if necessary
  pressedButt=buttonDetector();
  if(pressedButt){
  while(buttonDetector()){
    
    dispHold();
    delay(10);
  } 
  } else {
    pressedButt = false;
  }

  
}

void setConditions() {           // ajustes en la medida
  // get range name from PROGMEM
  strcpy_P(hScale, (char*)pgm_read_word(&(hstring_table[hRange])));  // mapea los posibles caracteres en el eje x con sus valores
  strcpy_P(vScale, (char*)pgm_read_word(&(vstring_table[vRange])));  // mapea los posibles caracteres en el eje y con sus valores
  rangeMax = 20 / lsb50V;  // set full scale pixcel count number
  rangeMaxDisp = 2000;
  rangeMin = 0;
  rangeMinDisp = 0;
  att10x = 1;
  
}

void writeCommonImage() {                 // Función para representar la malla y parámetros del osciloscopio
  oled.clearDisplay();                    
  oled.setTextColor(WHITE);               
  oled.setCursor(85, 0);                  // caracteres en la primera fila
  oled.println(F("av    v"));             
  oled.drawFastVLine(26, 9, 55, WHITE);   // Línea vertical eje y
  oled.drawFastVLine(127, 9, 3, WHITE);   
  oled.drawFastVLine(127, 61, 3, WHITE);  // línea vertical eje x

  oled.drawFastHLine(24, 9, 7, WHITE);    // marcadores del valor máximo 
  oled.drawFastHLine(24, 36, 2, WHITE);
  oled.drawFastHLine(24, 63, 7, WHITE);

  oled.drawFastHLine(51, 9, 3, WHITE);    
  oled.drawFastHLine(51, 63, 3, WHITE);

  oled.drawFastHLine(76, 9, 3, WHITE);   
  oled.drawFastHLine(76, 63, 3, WHITE);

  oled.drawFastHLine(101, 9, 3, WHITE);  
  oled.drawFastHLine(101, 63, 3, WHITE);

  oled.drawFastHLine(123, 9, 5, WHITE);  
  oled.drawFastHLine(123, 63, 5, WHITE);

  for (int x = 26; x <= 128; x += 5) {         // líneas discontinuas horizontales
    oled.drawFastHLine(x, 36, 2, WHITE);  
  }
  for (int x = (127 - 25); x > 30; x -= 25) {  // líneas discontinuas verticales
    for (int y = 10; y < 63; y += 5) {
      oled.drawFastVLine(x, y, 2, WHITE); 
    }
  }
}



bool buttonDetector() {
  
  if (digitalRead(holdButt) == 0) {           // if HOLD button(pin11) pushed
    pressedButt = true;                 // revers the flag
  } else {
    pressedButt=false;
  }
  return pressedButt; // Devuelve falso si no hay pulsación
  
}

void readWave() {                            // Función para guardar la forma de onda en el array de memoria
  
  if (att10x == 1) {                         // cuando necesite atenuador * 1/10
    pinMode(12, OUTPUT);                     // inicio del pin al que se conecta la resistencia de 10k
    digitalWrite(12, LOW);                   // se cierra el circuito con una fuente a 0V
  } else {                                   //
    pinMode(12, INPUT);                      // Realiza función de alta impedancia 
  }
  switchPushed = false;                      // Clear switch operation flag
  timeExec = 160 + 60;                 
        ADCSRA = ADCSRA & 0xf8;              
        ADCSRA = ADCSRA | 0x07;              
        for (int i = 0; i < REC_LENG; i++) { 
          waveBuff[i] = analogRead(0);       
          // delayMicroseconds(688);           
          delayMicroseconds(686);
        }
}

void dataAnalize() {                       // obtención de información a partir de la señal de entrada
  int d;
  long sum = 0;

  // buscar los valores máximo y mínimo
  dataMin = 1023;                          // inicio de la variable en el mayor valor posible para recorrer todos 
  dataMax = 0;                             // inicio de la variable en el menor valor 
  for (int i = 0; i < REC_LENG; i++) {     
    d = waveBuff[i];
    sum = sum + d;
    if (d < dataMin) {                     // actualiza el mínimo
      dataMin = d;
    }
    if (d > dataMax) {                     // actualiza el máximo
      dataMax = d;
    }
  }

  dataAve = (sum + 10) / 20;               // cáculo de la media (10 veces para mayor precisión)

  //asignación de valores máximo y mínimo a los ejes
  if (vRange <= 1) {                       
    rangeMin = dataMin - 20;               // mantendremos de margen 20 valores por arriba y por abajo
    rangeMin = (rangeMin / 10) * 10;       
    if (rangeMin < 0) {
      rangeMin = 0;                        
    }
    rangeMax = dataMax + 20;               
    rangeMax = ((rangeMax / 10) + 1) * 10; 
    if (rangeMax > 1020) {
      rangeMax = 1023;                     // si se sale de escala cogerá el máximo valor
    }

    if (att10x == 1) {                            // si necesita atenuador *1/10
      rangeMaxDisp = 100 * (rangeMax * lsb50V);   // el rango máximo estará determinado por el valor máximo leido y limitado por la escala de valores del ADC
      rangeMinDisp = 100 * (rangeMin * lsb50V);   // >=0
    } else {                                       
      rangeMaxDisp = 100 * (rangeMax * lsb5V);
      rangeMinDisp = 100 * (rangeMin * lsb5V);
    }
  } else {                                       
    // 
  }

  // Trigger position search
  for (trigP = ((REC_LENG / 2) - 51); trigP < ((REC_LENG / 2) + 50); trigP++) { // Find the points that straddle the median at the center ± 50 of the data range
    if (trigD == 0) {                             // if trigger direction is positive
      if ((waveBuff[trigP - 1] < (dataMax + dataMin) / 2) && (waveBuff[trigP] >= (dataMax + dataMin) / 2)) {
        break;                                    // positive trigger position found !
      }
    } else {                                      // trigger direction is negative
      if ((waveBuff[trigP - 1] > (dataMax + dataMin) / 2) && (waveBuff[trigP] <= (dataMax + dataMin) / 2)) {
        break;
      }                                           // negative trigger poshition found !
    }
  }
  trigSync = true;
  if (trigP >= ((REC_LENG / 2) + 50)) {           // If the trigger is not found in range
    trigP = (REC_LENG / 2);                       // Set it to the center for the time being
    trigSync = false;                             // set Unsync display flag
  }
  if ((dataMax - dataMin) <= MIN_TRIG_SWING) {    // amplitude of the waveform smaller than the specified value
    trigSync = false;                             // set Unsync display flag
  }
  freqDuty();
}

void freqDuty() {                               // función para anaizar el ciclo de trabajo
  int swingCenter;                              // punto medio
  float p0 = 0;                                 // primer ciclo positivo
  float p1 = 0;                                 // duración total de ciclos
  float p2 = 0;                                 // duración total de pulso a nivel alto
  float pFine = 0;                              // fine position (0-1.0)
  float lastPosiEdge;                           // last positive edge position

  float pPeriod;                                // Periodo del pulso
  float pWidth;                                 // anchura del pulso

  int p1Count = 0;                              // contador de ciclos
  int p2Count = 0;                              // contador de tiempo en alto

  boolean a0Detected = false;
  boolean posiSerch = true;                      

  swingCenter = (3 * (dataMin + dataMax)) / 2;   // calculo del valor medio

  for (int i = 1; i < REC_LENG - 2; i++) {       // scan all over the buffer
    if (posiSerch == true) {   // posi slope (frequency serch)
      if ((sum3(i) <= swingCenter) && (sum3(i + 1) > swingCenter)) {  // if across the center when rising (+-3data used to eliminate noize)
        pFine = (float)(swingCenter - sum3(i)) / ((swingCenter - sum3(i)) + (sum3(i + 1) - swingCenter) );  // fine cross point calc.
        if (a0Detected == false) {               // if 1-st cross
          a0Detected = true;                     // 
          p0 = i + pFine;                        // guardado de la posición como punto de partida
        } else {
          p1 = i + pFine - p0;                   // record length (length of n*cycle time)
          p1Count++;
        }
        lastPosiEdge = i + pFine;                // posición guardada para calcular el ancho del pulso
        posiSerch = false;
      }
    } else {   // nega slope serch (duration serch)
      if ((sum3(i) >= swingCenter) && (sum3(i + 1) < swingCenter)) {  // if across the center when falling (+-3data used to eliminate noize)
        pFine = (float)(sum3(i) - swingCenter) / ((sum3(i) - swingCenter) + (swingCenter - sum3(i + 1)) );
        if (a0Detected == true) {
          p2 = p2 + (i + pFine - lastPosiEdge);  // calucurate pulse width and accumurate it
          p2Count++;
        }
        posiSerch = true;
      }
    }
  }

  pPeriod = p1 / p1Count;                 
  pWidth = p2 / p2Count;                  

  waveFreq = 1.0 / ((pgm_read_float(hRangeValue + hRange) * pPeriod) / 25.0); // cálculo de la frecuencia de la onda, f=1/T
  waveDuty = 100.0 * pWidth / pPeriod;                                      // cálculo del ciclo de trabajo (duración del pulso/periodo)
}

int sum3(int k) {       // Sum of before and after and own value
  int m = waveBuff[k - 1] + waveBuff[k] + waveBuff[k + 1];
  return m;
}

void startScreen() {                      // Transición de inicio
  oled.clearDisplay();
  oled.setTextSize(1);                    
  oled.setTextColor(WHITE);
  oled.setCursor(55, 0);
  oled.println(F("Mini"));  
  oled.setCursor(30, 20);
  oled.println(F("Oscilloscope")); 
  oled.setCursor(55, 42);            
  oled.println(F("v1.1"));                
  oled.display();                         
  delay(1500);
  oled.clearDisplay();
  oled.setTextSize(1);                    
}

void dispHold() {                         // display "Hold"
  oled.fillRect(42, 11, 24, 8, BLACK);    // black paint 4 characters
  oled.setCursor(42, 11);
  oled.print(F("Hold"));                  // Hold
  oled.display();                         //
}

void dispInf() {                          // Display of various information
  float voltage;
  // display vertical sensitivity
  oled.setCursor(2, 0);                   // around top left
  oled.print(vScale);                     // vertical sensitivity value
  if (scopeP == 0) {                      // if scoped
    oled.drawFastHLine(0, 7, 27, WHITE);  // display scoped mark at the bottom
    oled.drawFastVLine(0, 5, 2, WHITE);
    oled.drawFastVLine(26, 5, 2, WHITE);
  }

  // horizontal sweep speed
  oled.setCursor(34, 0);                  //
  oled.print(hScale);                     // display sweep speed (time/div)
  if (scopeP == 1) {                      // if scoped
    oled.drawFastHLine(32, 7, 33, WHITE); // display scoped mark at the bottom
    oled.drawFastVLine(32, 5, 2, WHITE);
    oled.drawFastVLine(64, 5, 2, WHITE);
  }

  // trigger polarity
  oled.setCursor(75, 0);                  // at top center
  if (trigD == 0) {                       // if positive
    oled.print(char(0x18));               // up mark
  } else {
    oled.print(char(0x19));               // down mark              ↓
  }
  if (scopeP == 2) {                      // if scoped
    oled.drawFastHLine(71, 7, 13, WHITE); // display scoped mark at the bottom
    oled.drawFastVLine(71, 5, 2, WHITE);
    oled.drawFastVLine(83, 5, 2, WHITE);
  }

  // average voltage
  if (att10x == 1) {                         // if 10x attenuator is used
    voltage = dataAve * lsb50V / 10.0;       // 50V range value
  } else {                                   // no!
    voltage = dataAve * lsb5V / 10.0;        // 5V range value
  }
  if (voltage < 10.0) {                      // if less than 10V
    dtostrf(voltage, 4, 2, chrBuff);         // format x.xx
  } else {                                   // no!
    dtostrf(voltage, 4, 1, chrBuff);         // format xx.x
  }
  oled.setCursor(98, 0);                     // around the top right
  oled.print(chrBuff);                       // display average voltage圧の平均値を表示
  //  oled.print(saveTimer);                 // use here for debugging

  // vartical scale lines
  voltage = rangeMaxDisp / 100.0;            // convart Max voltage
  if (vRange == 1 || vRange > 4) {           // if range below 5V or Auto 5V
    dtostrf(voltage, 4, 2, chrBuff);         // format *.**
  } else {                                   // no!
    dtostrf(voltage, 4, 1, chrBuff);         // format **.*
  }
  oled.setCursor(0, 9);
  oled.print(chrBuff);                       // display Max value

  voltage = (rangeMaxDisp + rangeMinDisp) / 200.0; // center value calculation
  if (vRange == 1 || vRange > 4) {           // if range below 5V or Auto 5V
    dtostrf(voltage, 4, 2, chrBuff);         // format *.**
  } else {                                   // no!
    dtostrf(voltage, 4, 1, chrBuff);         // format **.*
  }
  oled.setCursor(0, 33);
  oled.print(chrBuff);                       // display the value

  voltage = rangeMinDisp / 100.0;            // convart Min vpltage
  if (vRange == 1 || vRange > 4) {           // if range below 5V or Auto 5V
    dtostrf(voltage, 4, 2, chrBuff);         // format *.**
  } else {                                   // no!
    dtostrf(voltage, 4, 1, chrBuff);         // format **.*
  }
  oled.setCursor(0, 57);
  oled.print(chrBuff);                       // display the value

  // print de la frecuencia, el ciclo de trabajo o mensaje de no sincronizado
  if (trigSync == false) {                   // If trigger point can't found
    oled.fillRect(92, 14, 24, 8, BLACK);     
    oled.setCursor(92, 14);                  
    oled.print(F("unSync"));                 
  } else {
    oled.fillRect(90, 12, 25, 9, BLACK);    // si se ha sincronizado: borrado en la posición de la frecuencia
    oled.setCursor(91, 13);                  
    if (waveFreq < 100.0) {                  // if less than 100Hz
      oled.print(waveFreq, 1);               // display 99.9Hz
      oled.print(F("Hz"));
    } else if (waveFreq < 1000.0) {          // if less than 1000Hz
      oled.print(waveFreq, 0);               // display 999Hz
      oled.print(F("Hz"));
    } else if (waveFreq < 10000.0) {         // if less than 10kHz
      oled.print((waveFreq / 1000.0), 2);    // display 9.99kH
      oled.print(F("kH"));
    } else {                                 // if more
      oled.print((waveFreq / 1000.0), 1);    // display 99.9kH
      oled.print(F("kH"));
    }
    oled.fillRect(96, 21, 25, 10, BLACK);    // erase Freq area (as small as possible)
    oled.setCursor(97, 23);                  // set location
    oled.print(waveDuty, 1);                 // print del ciclo de trabajo
    oled.print(F("%"));
  }
}

void plotData() {                    // plot wave form on OLED
  long y1, y2;
  for (int x = 0; x <= 98; x++) {
    y1 = map(waveBuff[x + trigP - 50], rangeMin, rangeMax, 63, 9); // convert to plot address
    y1 = constrain(y1, 9, 63);                                     // Crush(Saturate) the protruding part
    y2 = map(waveBuff[x + trigP - 49], rangeMin, rangeMax, 63, 9); // to address calucurate
    y2 = constrain(y2, 9, 63);                                     //
    oled.drawLine(x + 27, y1, x + 28, y2, WHITE);                  // connect between point
  }
}

void saveEEPROM() {                    // Save the setting value in EEPROM after waiting a while after the button operation.
  if (saveTimer > 0) {                 // If the timer value is positive,
    saveTimer = saveTimer - timeExec;  // Timer subtraction
    if (saveTimer < 0) {               // if time up
      EEPROM.write(0, vRange);         // save current status to EEPROM
      EEPROM.write(1, hRange);
      EEPROM.write(2, trigD);
      EEPROM.write(3, scopeP);
    }
  }
}

void loadEEPROM() {                    // Read setting values from EEPROM (abnormal values will be corrected to default)
  int x;
  x = EEPROM.read(0);                  // vRange
  if ((x < 0) || (9 < x)) {            // if out side 0-9
    x = 3;                             // default value
  }
  vRange = x;

  x = EEPROM.read(1);                  // hRange
  if ((x < 0) || (9 < x)) {            // if out of 0-9
    x = 3;                             // default value
  }
  hRange = x;
  x = EEPROM.read(2);                  // trigD
  if ((x < 0) || (1 < x)) {            // if out of 0-1
    x = 1;                             // default value
  }
  trigD = x;
  x = EEPROM.read(3);                  // scopeP
  if ((x < 0) || (2 < x)) {            // if out of 0-2
    x = 1;                             // default value
  }
  scopeP = x;
}



  
