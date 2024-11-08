#include <Wire.h>              // librería para iniciar la comunicación I2C con el arduino.         
#include <Adafruit_SSD1306.h>  // librería para el uso la pantalla OLED SSD1306 128x64              
#include <Adafruit_GFX.h>      // librería de funciones gráficas para visualizar las señales        
#include <EEPROM.h>            // librería para guardar ajustes de configuración durante el tiempo de ejecución      

#define SCREEN_WIDTH 128         // anchura de la pantalla
#define SCREEN_HEIGHT 64        // altura de la pantalla
#define OLED_RESET     -1      // inicialización del pin reset para algunos tipos de pantallas

  // Declaración del dispositivo SSD1306 para su comunicación I2C (la variable Wire de la librería habilita los pines SDA y SCL)
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);   
//Adafruit_SH1106 oled(OLED_RESET);     // si requiere pantalla SH1106

#define REC_LENG 200                    // tamaño del buffer de datos de la onda                         
#define MIN_TRIG_SWING 5                // valor mínimo de inestabilidad para sincronización 


  // Rango de matrices con valores que serán guardados en memoria flash, en esta versión usaremos exclusivamente la configuración de 20V 20ms, si desea cambiarla, línea 98 y 155
const char vRangeName[10][5] PROGMEM = {"A50V", "A 5V", " 50V", " 20V", " 10V", "  5V", "  2V", "  1V", "0.5V", "0.2V"}; // matriz con caracteres para visualización en el eje y. 
const char * const vstring_table[] PROGMEM = {vRangeName[0], vRangeName[1], vRangeName[2], vRangeName[3], vRangeName[4], vRangeName[5], vRangeName[6], vRangeName[7], vRangeName[8], vRangeName[9]};
const char hRangeName[10][6] PROGMEM = {"200ms", "100ms", " 50ms", " 20ms", " 10ms", "  5ms", "  2ms", "  1ms", "500us", "200us"};  // matriz con caracteres para visualización en el eje x.
const char * const hstring_table[] PROGMEM = {hRangeName[0], hRangeName[1], hRangeName[2], hRangeName[3], hRangeName[4], hRangeName[5], hRangeName[6], hRangeName[7], hRangeName[8], hRangeName[9]};
const PROGMEM float hRangeValue[] = { 0.2, 0.1, 0.05, 0.02, 0.01, 0.005, 0.002, 0.001, 0.5e-3, 0.2e-3}; // matriz con valores para visualización en el eje x. ( = 25pix on screen)

int waveBuff[REC_LENG];        // declara el buffer para señales con su tamaño          
char chrBuff[8];               // declara el buffer de caracteres que guardará los valores del rango
char hScale[] = "xxxAs";       // caracter para escala horizontal
char vScale[] = "xxxx";        // caracter para escala vertical

float lsb5V = 0.00566826;      // Coeficiente de sensibilidad para rango 5V, servirá para dividir los valores de voltaje hasta este rango [1.1*630/(1024*120)]
float lsb50V = 0.05243212;     // Coeficiente de sensibilidad para rango 50V, servirá para dividir la visualización hasta este rango [1.1*520.91/(1024*10.91)]

volatile int vRange;           // contendrá codificados los caracteres para los valores en el eje y. 0:A50V,  1:A 5V,  2:50V,  3:20V,  4:10V,  5:5V,  6:2V,  7:1V,  8:0.5V,  9:0.2V
volatile int hRange;           // contendrá codificados los caracteres para los valores en el eje x. 0:200ms, 1:100ms, 2:50ms, 3:20ms, 4:10ms, 5:5ms, 6;2ms, 7:1ms, 8:500us, 9;200us
volatile int trigD;            // trigger bandera que servirá de referencia para saber si el trigger de posición se encuentra hacia delante o atrás en el vector. 0=pos 1=neg
volatile int scopeP;           // trigger de posición que se usará de referencia para sincronizar la señal. 0 = Veratical, 1 = Hrizontal, 2 = scope

volatile int saveTimer;        // Tiempo hasta guardar en la EEPROM
int timeExec;                  // Tiempo de ejecución para los ajustes definidos           approx. execution time of current range setting (ms)

int holdButt = 11;             // pin para el botón de mantener
volatile boolean pressedButt = false; //bandera para la interacción de la pulsación con el programa
int dataMin;                   // menor valor que tomará el buffer, límite 0
int dataMax;                   // mayor valor que tomará el buffer, límite 1023 
int dataAve;                   // valor medio, (por factor de 10 para tener más precisión) límite 10230
int rangeMax;                  // limite superior para graficar
int rangeMin;                  // límite inferior ppara graficar
int rangeMaxDisp;              // valor máximo para el display de pantalla
int rangeMinDisp;              // valor mínimo para el display de pantalla
int trigP;                     // trigger de position que funcionará como puntero en el buffer de datos
boolean trigSync;              // bandera de detección del trigger de sincronización


float waveFreq;                // Frecuencia de la onda en Hz
float waveDuty;                // porcentaje del ciclo de trabajo

void setup() {

  pinMode(holdButt, INPUT_PULLUP);      // botón de mantener
  pinMode(12, INPUT);                   // atenuador *1/10 (apagado=alta impedancia, desactivado=salida nv bajo)
  pinMode(13, OUTPUT);                  // LED de arduino, parpadea cada vez que se actualizan los datos
  oled.begin(SSD1306_SWITCHCAPVCC,  0x3C); //iniciar pantalla
  loadEEPROM();                         // lee ultima configuración de la EEPROM
  analogReference(INTERNAL);            // Establece el voltaje  referencia usado por entradas analógicas (1.1V)
  startScreen();                        // muestra el mensaje e inicio
}

void loop() {

  setConditions();                      // establecer los rangos de visualización
  digitalWrite(13, HIGH);               // enciende el LED flash del arduino cada vez que se ejecuta el bucle
  readWave();                           // función para leer la forma de onda y guardarla en la memoria del buffer
  digitalWrite(13, LOW);                // al leer la onda apaga el LED
  setConditions();                      // actualiza condiciones
  dataAnalize();                        // llama a la función que se queda con datos de la onda de valores máximo, mínimo y medio
  writeCommonImage();                   // dibuja los márgenes y valores de los ejes en la pantalla
  plotData();                           // representa la señal sobre los ejes
  dispInf();                            // dibuja los valores calculados de voltaje
  oled.display();                       // manda a la pantalla la matriz gráfica configurada
  saveEEPROM();                         // si es necesario se guardarán datos en la EEPROM
  pressedButt=buttonDetector();         // con botón pulsado la función devolverá true, entrará en el bucle hasta que se suelte
  if(pressedButt){
  while(buttonDetector()){
    dispHold();                         // paraliza la pantalla con los valores en ese instante
    delay(10);
  } 
  } else {
    pressedButt = false;
  }
}


void setConditions() {           // ajustes en la medida
  // obtención de los nombres de los rangos de la memoria flash
  strcpy_P(hScale, (char*)pgm_read_word(&(hstring_table[hRange])));  // mapea los posibles caracteres en el eje x con sus valores
  strcpy_P(vScale, (char*)pgm_read_word(&(vstring_table[vRange])));  // mapea los posibles caracteres en el eje y con sus valores
 
  rangeMax = 20 / lsb50V;  //configuración 20V, si se desea otra dentro de las declaradas, sustituir 20 por x && 2000 por x*10^2
  rangeMaxDisp = 2000;
  rangeMin = 0;
  rangeMinDisp = 0;
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
  
  if (digitalRead(holdButt) == 0) {           // se cumple si el botón está pulsado (0 ya que funciona como pull up)
    pressedButt = true;                 
  } else {
    pressedButt=false;
  }
  return pressedButt;                         // Devuelve verdadero si el botón está siendo pulsado y falso cuando no
  
}


void readWave() {                            // Función para guardar la forma de onda en el array de memoria
  
  pinMode(12, INPUT);                        ///  configuración de 20ms  
  timeExec = 160 + 60;                 
  ADCSRA = ADCSRA & 0xf8;              
  ADCSRA = ADCSRA | 0x07;              
  for (int i = 0; i < REC_LENG; i++) { 
    waveBuff[i] = analogRead(0);       
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
    rangeMaxDisp = 100 * (rangeMax * lsb5V);
      rangeMinDisp = 100 * (rangeMin * lsb5V);
    
  }

  // Buscar el trigger de posición en el rango de datos
  for (trigP = ((REC_LENG / 2) - 51); trigP < ((REC_LENG / 2) + 50); trigP++) { // buscar puntos en el centro con margen superior e inferior de 50 posiciones
    if (trigD == 0) {                             // dirección positiva
      if ((waveBuff[trigP - 1] < (dataMax + dataMin) / 2) && (waveBuff[trigP] >= (dataMax + dataMin) / 2)) {
        break;                                    // trigger encontrado por encima de la mitad de valores
      }
    } else {                                      // dirección negativa
      if ((waveBuff[trigP - 1] > (dataMax + dataMin) / 2) && (waveBuff[trigP] <= (dataMax + dataMin) / 2)) {
        break;
      }                                           // trigger encontrado por debajo de la mitad de valores
    }
  }
  trigSync = true;
  if (trigP >= ((REC_LENG / 2) + 50)) {           // trigger no encontrado en el rango
    trigP = (REC_LENG / 2);                       // establecido exactamente en el centro
    trigSync = false;                             // no sincronizado
  }
  if ((dataMax - dataMin) <= MIN_TRIG_SWING) {    // amplitud de la onda de entrada por debajo del umbral 
    trigSync = false;                             // no sincronizado
  freqDuty();
  }
}


void freqDuty() {                               // función para anaizar el ciclo de trabajo
  int swingCenter;                              // punto medio
  float p0 = 0;                                 // primer ciclo positivo
  float p1 = 0;                                 // duración total de ciclos
  float p2 = 0;                                 // duración total de pulso a nivel alto
  float pFine = 0;                              // fine position (0-1.0)                                   ////
  float lastPosiEdge;                           // last positive edge position                                   ////

  float pPeriod;                                // Periodo del pulso
  float pWidth;                                 // anchura del pulso

  int p1Count = 0;                              // contador de ciclos
  int p2Count = 0;                              // contador de tiempo en alto

  boolean a0Detected = false;
  boolean posiSerch = true;                      

  swingCenter = (3 * (dataMin + dataMax)) / 2;   // calculo del valor medio

  for (int i = 1; i < REC_LENG - 2; i++) {       // recorriendo todo el buffer
    if (posiSerch == true) {   // si tienes que buscar hacia delante
      if ((sum3(i) <= swingCenter) && (sum3(i + 1) > swingCenter)) {  // aumentendo posición a través del vector
        pFine = (float)(swingCenter - sum3(i)) / ((swingCenter - sum3(i)) + (sum3(i + 1) - swingCenter) );  // fine cross point calc.                                   ////
        if (a0Detected == false) {               // if 1-st cross                                   ////
          a0Detected = true;                     // 
          p0 = i + pFine;                        // guardado de la posición como punto de partida
        } else {
          p1 = i + pFine - p0;                   //gurda la longitud de valores para el vector (n*ciclo)
          p1Count++;
        }
        lastPosiEdge = i + pFine;                // posición guardada para calcular el ancho del pulso
        posiSerch = false;
      }
    } else {   // si tienes que buscar hacia atrás
      if ((sum3(i) >= swingCenter) && (sum3(i + 1) < swingCenter)) {  // if across the center when falling (+-3data used to eliminate noize)                                   ////
        pFine = (float)(sum3(i) - swingCenter) / ((sum3(i) - swingCenter) + (swingCenter - sum3(i + 1)) );
        if (a0Detected == true) {
          p2 = p2 + (i + pFine - lastPosiEdge);  // cálculo del ancho de pulso y acumulación
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


int sum3(int k) {       // suma del valor + el anterior y el siguiente
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
  oled.println(F("Osciloscopio")); 
  oled.setCursor(55, 42);            
  oled.println(F("V.f"));                
  oled.display();                         
  delay(1500);
  oled.clearDisplay();
  oled.setTextSize(1);                    
}


void dispHold() {                         // muestra por pantalla que se está manteniendo la señal
  oled.fillRect(42, 11, 24, 8, BLACK);    // borra lo que había en esa posición
  oled.setCursor(42, 11);
  oled.print(F("Hold"));                  // escribe hold
  oled.display();                         
}


void dispInf() {                          // representa información calculada de la señal
  float voltage;
  // display vertical sensitivity
  oled.setCursor(2, 0);                   // arriba a la izquierda
  oled.print(vScale);                     // muestra el valor de voltaje del rango
  if (scopeP == 0) {                      // si vertical
    oled.drawFastHLine(0, 7, 27, WHITE);  
    oled.drawFastVLine(0, 5, 2, WHITE);
    oled.drawFastVLine(26, 5, 2, WHITE);
  }

  // horizontal sweep speed
  oled.setCursor(34, 0);                  // arriba en el centro
  oled.print(hScale);                     // muestra el tiempo por división
  if (scopeP == 1) {                      // si horizontal
    oled.drawFastHLine(32, 7, 33, WHITE); 
    oled.drawFastVLine(32, 5, 2, WHITE);
    oled.drawFastVLine(64, 5, 2, WHITE);
  }

  // polaridad del trigger, graficar en la posición de pantalla correspondiente según indique el trigger bandera
  oled.setCursor(75, 0);                  // arriba en el centro
  if (trigD == 0) {                       // positivo
    oled.print(char(0x18));               // arriba
  } else {
    oled.print(char(0x19));               // negativo, abajo            
  }
  if (scopeP == 2) {                      
    oled.drawFastHLine(71, 7, 13, WHITE); //
    oled.drawFastVLine(71, 5, 2, WHITE);
    oled.drawFastVLine(83, 5, 2, WHITE);
  }

  // cálculo del voltaje medio en la entrada
  voltage = dataAve * lsb5V / 10.0;
 
  if (voltage < 10.0) {                      // si es <= 10V
    dtostrf(voltage, 4, 2, chrBuff);         // formato x.xx
  } else {                                   
    dtostrf(voltage, 4, 1, chrBuff);         // formato xx.x
  }
  oled.setCursor(98, 0);                     // arriba a la derecha
  oled.print(chrBuff);                       // mostrar el voltage medio

  // líneas de escala vertical
  voltage = rangeMaxDisp / 100.0;            // conversión del voltaje máximo 
  if (vRange == 1 || vRange > 4) {           // rango por debajo de 5V o auto5V
    dtostrf(voltage, 4, 2, chrBuff);         // formato *.**
  } else {                                   
    dtostrf(voltage, 4, 1, chrBuff);         // formato **.*
  }
  oled.setCursor(0, 9);
  oled.print(chrBuff);                       // mostrar valor máximo

  voltage = (rangeMaxDisp + rangeMinDisp) / 200.0; // cálculo de valor central
  if (vRange == 1 || vRange > 4) {           // rango por debajo de 5V o auto5V
    dtostrf(voltage, 4, 2, chrBuff);         // formato *.**
  } else {                                   
    dtostrf(voltage, 4, 1, chrBuff);         // formato **.*
  }
  oled.setCursor(0, 33);
  oled.print(chrBuff);                       

  voltage = rangeMinDisp / 100.0;            // conversión del voltaje mínimo
  if (vRange == 1 || vRange > 4) {           // rango por debajo de 5V o auto5V
    dtostrf(voltage, 4, 2, chrBuff);         // formato *.**
  } else {                                   
    dtostrf(voltage, 4, 1, chrBuff);         // formato **.*
  }
  oled.setCursor(0, 57);
  oled.print(chrBuff);                      

  // print de la frecuencia, el ciclo de trabajo o mensaje de no sincronizado
  if (trigSync == false) {                   // Si no se ha podido sincronizar
    oled.fillRect(92, 14, 24, 8, BLACK);     
    oled.setCursor(92, 14);                  
    oled.print(F("unSync"));                 
  } else {
    oled.fillRect(90, 12, 25, 9, BLACK);    // si se ha sincronizado: borrado en la posición de la frecuencia
    oled.setCursor(91, 13);                  
    if (waveFreq < 100.0) {                  // menos de 100Hz
      oled.print(waveFreq, 1);               // muestra 99.9Hz
      oled.print(F("Hz"));
    } else if (waveFreq < 1000.0) {          // menos de 1000Hz
      oled.print(waveFreq, 0);               // muestra  999Hz
      oled.print(F("Hz"));
    } else if (waveFreq < 10000.0) {         // menos de 10kHz
      oled.print((waveFreq / 1000.0), 2);    // muestra 9.99kH
      oled.print(F("kH"));
    } else {                                 // superior
      oled.print((waveFreq / 1000.0), 1);    // muestra 99.9kH
      oled.print(F("kH"));
    }
    oled.fillRect(96, 21, 25, 10, BLACK);    // borrado en la frecuencia
    oled.setCursor(97, 23);                  // posición del dato de ciclo de trabajo
    oled.print(waveDuty, 1);                 // print del ciclo de trabajo
    oled.print(F("%"));
  }
}



void plotData() {                    // representación de la señal en OLED
  long y1, y2;
  for (int x = 0; x <= 98; x++) {
    y1 = map(waveBuff[x + trigP - 50], rangeMin, rangeMax, 63, 9); // mapea los valores guardados en la pantalla          
    y1 = constrain(y1, 9, 63);                                     // satura los puntos que sobresalgan del margen vertical
    y2 = map(waveBuff[x + trigP - 49], rangeMin, rangeMax, 63, 9); 
    y2 = constrain(y2, 9, 63);                                     
    oled.drawLine(x + 27, y1, x + 28, y2, WHITE);                  // une los puntos graficados con una línea
  }
}


void saveEEPROM() {                    // guardar configuración en EEPROM cada tiempo de guardado
  if (saveTimer > 0) {                 // cuando no ha pasado aún irá decreciendo
    saveTimer = saveTimer - timeExec;  
    if (saveTimer < 0) {               // al llega a 0
      EEPROM.write(0, vRange);         // guardado
      EEPROM.write(1, hRange);
      EEPROM.write(2, trigD);
      EEPROM.write(3, scopeP);
    }
  }
}


void loadEEPROM() {                    // lectura de valores de la EEPROM
  int x;
  x = EEPROM.read(0);                  // vRange, rango de valores posibles para el eje y (V)
  if ((x < 0) || (9 < x)) {            // si está fuera del rango de valores declarados
    x = 3;                             // valor por defecto
  }
  vRange = x;

  x = EEPROM.read(1);                  // hRange, rango de valores posibles para eje x (ms)
  if ((x < 0) || (9 < x)) {            // si está fuera del rango de valores declarados
    x = 3;                             // default value
  }
  hRange = x;
  x = EEPROM.read(2);                  // trigger D (bandera para encontrar trigger de posición)
  if ((x < 0) || (1 < x)) {            // si es distinto de 0/1
    x = 1;                             // valor por defecto
  }
  trigD = x;
  x = EEPROM.read(3);                  // scopeP 
  if ((x < 0) || (2 < x)) {            // si está fuera del intervalo 0-2
    x = 1;                             // valor por defecto
  }
  scopeP = x;
}

