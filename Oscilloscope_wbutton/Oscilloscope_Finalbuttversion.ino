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

int waveBuff[REC_LENG];        // inicia el buffer para señales con su tamaño  
char chrBuff[8];               
char hScale[] = "xxxAs";       
char vScale[] = "xxxx";        

float lsb5V = 0.00566826;      
float lsb50V = 0.05243212;     

volatile int vRange;           // contendrá codificados los caracteres para los valores en el eje y. 0:A50V,  1:A 5V,  2:50V,  3:20V,  4:10V,  5:5V,  6:2V,  7:1V,  8:0.5V,  9:0.2V
volatile int hRange;           // contendrá codificados los caracteres para los valores en el eje x. 0:200ms, 1:100ms, 2:50ms, 3:20ms, 4:10ms, 5:5ms, 6;2ms, 7:1ms, 8:500us, 9;200us
volatile boolean switchPushed;
volatile int trigD;            
volatile int scopeP;           

volatile int saveTimer;        // Tiempo hasta guardar en la EEPROM
int timeExec;                  // Tiempo de ejecución para los ajustes definidos        

int holdButt = 11;
volatile boolean pressedButt = false; 
int dataMin;                   
int dataMax;                   
int dataAve;                   
int rangeMax;                  
int rangeMin;                  
int rangeMaxDisp;              
int rangeMinDisp;              
int trigP;                     
boolean trigSync;              
int att10x;                   

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

  //auxFunctions();                       
  loadEEPROM();                        
  analogReference(INTERNAL);         
  
  
  startScreen();                        
}

void loop() {
  

  setConditions();                     
  digitalWrite(13, HIGH);              
  readWave();                           // función para leer la forma de onda y guardarla en la memoria del buffer
  digitalWrite(13, LOW);             
  setConditions();                     
  dataAnalize();                      
  writeCommonImage();                  
  plotData();                      
  dispInf();                       
  oled.display();                  
  saveEEPROM();                      
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

  strcpy_P(hScale, (char*)pgm_read_word(&(hstring_table[hRange])));  // mapea los posibles caracteres en el eje x con sus valores
  strcpy_P(vScale, (char*)pgm_read_word(&(vstring_table[vRange])));  // mapea los posibles caracteres en el eje y con sus valores
  rangeMax = 20 / lsb50V; 
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
  
  if (digitalRead(holdButt) == 0) {          
    pressedButt = true;                
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
  switchPushed = false;                
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

 
  for (trigP = ((REC_LENG / 2) - 51); trigP < ((REC_LENG / 2) + 50); trigP++) { 
    if (trigD == 0) {                           
      if ((waveBuff[trigP - 1] < (dataMax + dataMin) / 2) && (waveBuff[trigP] >= (dataMax + dataMin) / 2)) {
        break;                                  
      }
    } else {                                    
      if ((waveBuff[trigP - 1] > (dataMax + dataMin) / 2) && (waveBuff[trigP] <= (dataMax + dataMin) / 2)) {
        break;
      }                                       
    }
  }
  trigSync = true;
  if (trigP >= ((REC_LENG / 2) + 50)) {          
    trigP = (REC_LENG / 2);                       
    trigSync = false;                           
  }
  if ((dataMax - dataMin) <= MIN_TRIG_SWING) {    
    trigSync = false;                             
  }
  freqDuty();
}

void freqDuty() {                               // función para anaizar el ciclo de trabajo
  int swingCenter;                              // punto medio
  float p0 = 0;                                 // primer ciclo positivo
  float p1 = 0;                                 // duración total de ciclos
  float p2 = 0;                                 // duración total de pulso a nivel alto
  float pFine = 0;                            
  float lastPosiEdge;                       

  float pPeriod;                                // Periodo del pulso
  float pWidth;                                 // anchura del pulso

  int p1Count = 0;                              // contador de ciclos
  int p2Count = 0;                              // contador de tiempo en alto

  boolean a0Detected = false;
  boolean posiSerch = true;                      

  swingCenter = (3 * (dataMin + dataMax)) / 2;   // calculo del valor medio

  for (int i = 1; i < REC_LENG - 2; i++) {     
    if (posiSerch == true) {  
      if ((sum3(i) <= swingCenter) && (sum3(i + 1) > swingCenter)) { 
        pFine = (float)(swingCenter - sum3(i)) / ((swingCenter - sum3(i)) + (sum3(i + 1) - swingCenter) );  
        if (a0Detected == false) {              
          a0Detected = true;                    
          p0 = i + pFine;                        // guardado de la posición como punto de partida
        } else {
          p1 = i + pFine - p0;                  
          p1Count++;
        }
        lastPosiEdge = i + pFine;                // posición guardada para calcular el ancho del pulso
        posiSerch = false;
      }
    } else {  
      if ((sum3(i) >= swingCenter) && (sum3(i + 1) < swingCenter)) {  
        pFine = (float)(sum3(i) - swingCenter) / ((sum3(i) - swingCenter) + (swingCenter - sum3(i + 1)) );
        if (a0Detected == true) {
          p2 = p2 + (i + pFine - lastPosiEdge); 
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

int sum3(int k) {       
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

void dispHold() {                        
  oled.fillRect(42, 11, 24, 8, BLACK);   
  oled.setCursor(42, 11);
  oled.print(F("Hold"));                  
  oled.display();                        
}

void dispInf() {                          
  float voltage;
  // display vertical sensitivity
  oled.setCursor(2, 0);                   
  oled.print(vScale);                    
  if (scopeP == 0) {                    
    oled.drawFastHLine(0, 7, 27, WHITE);  
    oled.drawFastVLine(0, 5, 2, WHITE);
    oled.drawFastVLine(26, 5, 2, WHITE);
  }

  // horizontal sweep speed
  oled.setCursor(34, 0);                
  oled.print(hScale);                    
  if (scopeP == 1) {                   
    oled.drawFastHLine(32, 7, 33, WHITE);
    oled.drawFastVLine(32, 5, 2, WHITE);
    oled.drawFastVLine(64, 5, 2, WHITE);
  }

  
  oled.setCursor(75, 0);                 
  if (trigD == 0) {                     
    oled.print(char(0x18));               
  } else {
    oled.print(char(0x19));                  
  }
  if (scopeP == 2) {                     
    oled.drawFastHLine(71, 7, 13, WHITE); 
    oled.drawFastVLine(71, 5, 2, WHITE);
    oled.drawFastVLine(83, 5, 2, WHITE);
  }


  if (att10x == 1) {                       
    voltage = dataAve * lsb50V / 10.0;     
  } else {                                
    voltage = dataAve * lsb5V / 10.0;      
  }
  if (voltage < 10.0) {                    
    dtostrf(voltage, 4, 2, chrBuff);        
  } else {                                  
    dtostrf(voltage, 4, 1, chrBuff);         
  }
  oled.setCursor(98, 0);                    
  oled.print(chrBuff);                      
  //  oled.print(saveTimer);              


  voltage = rangeMaxDisp / 100.0;           
  if (vRange == 1 || vRange > 4) {        
    dtostrf(voltage, 4, 2, chrBuff);        
  } else {                                 
    dtostrf(voltage, 4, 1, chrBuff);       
  }
  oled.setCursor(0, 9);
  oled.print(chrBuff);                     

  voltage = (rangeMaxDisp + rangeMinDisp) / 200.0;
  if (vRange == 1 || vRange > 4) {          
    dtostrf(voltage, 4, 2, chrBuff);         
  } else {                                   
    dtostrf(voltage, 4, 1, chrBuff);        
  }
  oled.setCursor(0, 33);
  oled.print(chrBuff);                      

  voltage = rangeMinDisp / 100.0;           
  if (vRange == 1 || vRange > 4) {          
    dtostrf(voltage, 4, 2, chrBuff);       
  } else {                                   
    dtostrf(voltage, 4, 1, chrBuff);        
  }
  oled.setCursor(0, 57);
  oled.print(chrBuff);                     

  // print de la frecuencia, el ciclo de trabajo o mensaje de no sincronizado
  if (trigSync == false) {                   
    oled.fillRect(92, 14, 24, 8, BLACK);     
    oled.setCursor(92, 14);                  
    oled.print(F("unSync"));                 
  } else {
    oled.fillRect(90, 12, 25, 9, BLACK);    // si se ha sincronizado: borrado en la posición de la frecuencia
    oled.setCursor(91, 13);                  
    if (waveFreq < 100.0) {                  
      oled.print(waveFreq, 1);              
      oled.print(F("Hz"));
    } else if (waveFreq < 1000.0) {         
      oled.print(waveFreq, 0);              
      oled.print(F("Hz"));
    } else if (waveFreq < 10000.0) {        
      oled.print((waveFreq / 1000.0), 2);    
      oled.print(F("kH"));
    } else {                                
      oled.print((waveFreq / 1000.0), 1);   
      oled.print(F("kH"));
    }
    oled.fillRect(96, 21, 25, 10, BLACK);   
    oled.setCursor(97, 23);                 
    oled.print(waveDuty, 1);              
    oled.print(F("%"));
  }
}

void plotData() {                  
  long y1, y2;
  for (int x = 0; x <= 98; x++) {
    y1 = map(waveBuff[x + trigP - 50], rangeMin, rangeMax, 63, 9); 
    y1 = constrain(y1, 9, 63);                                   
    y2 = map(waveBuff[x + trigP - 49], rangeMin, rangeMax, 63, 9); 
    y2 = constrain(y2, 9, 63);                                    
    oled.drawLine(x + 27, y1, x + 28, y2, WHITE);                  
  }
}

void saveEEPROM() {                    
  if (saveTimer > 0) {                
    saveTimer = saveTimer - timeExec;  
    if (saveTimer < 0) {               
      EEPROM.write(0, vRange);        
      EEPROM.write(1, hRange);
      EEPROM.write(2, trigD);
      EEPROM.write(3, scopeP);
    }
  }
}

void loadEEPROM() {                  
  int x;
  x = EEPROM.read(0);                  
  if ((x < 0) || (9 < x)) {            
    x = 3;                             
  }
  vRange = x;

  x = EEPROM.read(1);                 
  if ((x < 0) || (9 < x)) {           
    x = 3;                             
  }
  hRange = x;
  x = EEPROM.read(2);                 
  if ((x < 0) || (1 < x)) {            
    x = 1;                            
  }
  trigD = x;
  x = EEPROM.read(3);                  
  if ((x < 0) || (2 < x)) {           
    x = 1;                            
  }
  scopeP = x;
}



  
