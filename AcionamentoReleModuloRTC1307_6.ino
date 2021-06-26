#include <Wire.h>
#include "RTClib.h"
#include <avr/sleep.h>

const int alarmPin = 2;

RTC_DS3231 rtc;

unsigned long keyPrevMillis = 0;

// Intervalo mínimo de checagem do estado do botão em milisegundos.
const unsigned long keySampleIntervalMs = 25;

// Número de ciclos de keySampleIntervalMs para considerar como pressão longa do botão.
byte longKeyPressCountMax = 80;    // 80 * 25 = 2000 ms

// Ciclos em milisegundos de keySampleIntervalMs executados.
byte longKeyPressCount = 0;

// Button default state.
byte prevKeyState = LOW;

// Pino ao qual o botão está ligado.
const byte keyPin = 5;

char daysOfTheWeek[7][13] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// pino ao qual o módulo de MOSFET está conectado.
#define mosfetPin 8

// true para iniciar contagem do tempo.
bool temporizadorAcionado = false; 

// HORARIOS ACIONAMENTO/DESLIGAMENTO 1.
int horarioAcionamento1 = 22;
int minutoAcionamento1 = 22;
int horarioDesligamento1 = 22;
int minutoDesligamento1 = 28;

// HORARIOS ACIONAMENTO/DESLIGAMENTO 2.
int horarioAcionamento2 = 16;
int minutoAcionamento2 = 35;
int horarioDesligamento2 = 16;
int minutoDesligamento2 = 41;

// Quantos segundos anteriores ao acionamento da bomba sair do sleep mode.
const int tsDesativarSleepModeAcionarBomba = 5;

//RTC Module global variables
const int time_interval=5;// Sets the wakeup intervall in minutes.

// ESPECIFICA PARAMETROS DE SENSOR DE UMIDADE SOLO.
#define sensorUmidadeVCCPin 7
#define sensorUmidadeAnalogInput A1

// Valor mínimo do sensor de umidade. Se valor lido no sensor for maior que este, deve-se acionar a bomba.
const int valorMinimoSensorUmidade = 100;

DateTime dtHorarioAcionamento1;
DateTime dtHorarioDesligamento1;
DateTime dtHorarioAcionamento2;
DateTime dtHorarioDesligamento2;

String formatTime(DateTime dt){
  String formattedTime = "";
  formattedTime.concat(dt.day());
  formattedTime.concat("/");
  formattedTime.concat(dt.month());
  formattedTime.concat("/");
  formattedTime.concat(dt.year());
  formattedTime.concat(" ");
  formattedTime.concat(dt.hour());
  formattedTime.concat(":");
  formattedTime.concat(dt.minute());
  formattedTime.concat(":");
  formattedTime.concat(dt.second());
  return formattedTime;
}

void setup () {
    Serial.begin(9600);
    Serial.println("Teste");
    
    pinMode(LED_BUILTIN, OUTPUT);

    // Define pin VCC sensor umidade como LOW.
    //pinMode(sensorUmidadeVCCPin, LOW);

    // seta o pino como saída para acionar o modulo MOSFET.
    pinMode(mosfetPin, OUTPUT);
    digitalWrite(mosfetPin, LOW);
  
    // Set the alarm pin as pullup.
    pinMode(alarmPin, INPUT_PULLUP);

    if (!rtc.begin()){
      Serial.println("Couldn't find RTC");
      Serial.flush();
      abort();
    }

    // If required set time
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // To compiled time
    //rtc.adjust(DateTime(2021, 5, 1, 16, 21, 0)); // Or explicitly, e.g. July 3, 2020 at 8pm

    // Disable and clear both alarms
    rtc.disableAlarm(1);
    rtc.disableAlarm(2);
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);

    // Place SQW pin into alarm interrupt mode.
    rtc.writeSqwPinMode(DS3231_OFF);

    Serial.println("Starting");

    // Define o pino do botão de acionamento do sistema.
    pinMode(keyPin, INPUT_PULLUP);
}

void configurarAlarmes() {
  Serial.println(formatTime(rtc.now()) + " configurando alarmes...");
  
  // Disable and clear both alarms alarm
  rtc.disableAlarm(1);
  rtc.clearAlarm(1);
  rtc.disableAlarm(2);
  rtc.clearAlarm(2);
  
  DateTime currentTime = rtc.now();

  // Especifica horários de acionamento/desligamento.
  dtHorarioAcionamento1 = DateTime(currentTime.year(), currentTime.month(), currentTime.day(), horarioAcionamento1, minutoAcionamento1, 0);
  dtHorarioDesligamento1 = DateTime(currentTime.year(), currentTime.month(), currentTime.day(), horarioDesligamento1, minutoDesligamento1, 0);
  dtHorarioAcionamento2 = DateTime(currentTime.year(), currentTime.month(), currentTime.day(), horarioAcionamento2, minutoAcionamento2, 0);
  dtHorarioDesligamento2 = DateTime(currentTime.year(), currentTime.month(), currentTime.day(), horarioDesligamento2, minutoDesligamento2, 0);

  DateTime dtHorarioDespertar1;
  DateTime dtHorarioDespertar2;
  
  // Verifica se o horário atual é depois do horário do acionamento do alarme 1. 
  // Caso seja, programa alarme 1 para acionar no dia seguinte.
  if (dtHorarioAcionamento1.unixtime() > currentTime.unixtime()){
    dtHorarioDespertar1 = dtHorarioAcionamento1 - TimeSpan(tsDesativarSleepModeAcionarBomba);
    rtc.setAlarm1(dtHorarioDespertar1, DS3231_A1_Hour);
  } else {
    dtHorarioAcionamento1 = DateTime::DateTime(currentTime.year(), currentTime.month(), currentTime.day() + 1, horarioAcionamento1, minutoAcionamento1, 0);
    dtHorarioDesligamento1 = DateTime::DateTime(currentTime.year(), currentTime.month(), currentTime.day() + 1, horarioDesligamento1, minutoDesligamento1, 0);
    dtHorarioDespertar1 = dtHorarioAcionamento1 - TimeSpan(tsDesativarSleepModeAcionarBomba);
    rtc.setAlarm1(dtHorarioDespertar1, DS3231_A1_Hour);
  }

  // Verifica se o horário atual é depois do horário do acionamento do alarme 2. 
  // Caso seja, programa alarme 2 para acionar no dia seguinte.
  if (dtHorarioAcionamento2.unixtime() > currentTime.unixtime()){
    dtHorarioDespertar2 = dtHorarioAcionamento2 - TimeSpan(tsDesativarSleepModeAcionarBomba);
    Serial.println("Horario despertar 2 2 " + formatTime(dtHorarioDespertar2));
    rtc.setAlarm2(dtHorarioDespertar2, DS3231_A2_Hour);
  } else {
    dtHorarioAcionamento2 = DateTime::DateTime(currentTime.year(), currentTime.month(), currentTime.day() + 1, horarioAcionamento2, minutoAcionamento2, 0);
    dtHorarioDesligamento2 = DateTime::DateTime(currentTime.year(), currentTime.month(), currentTime.day() + 1, horarioDesligamento2, minutoDesligamento2, 0);
    dtHorarioDespertar2 = dtHorarioAcionamento2 - TimeSpan(tsDesativarSleepModeAcionarBomba);
    rtc.setAlarm2(dtHorarioDespertar2, DS3231_A2_Hour);
  }

  Serial.println("Próximo horário despertar 1: " + formatTime(dtHorarioDespertar1));
  Serial.println("Próximo horário desligamento 1: " + formatTime(dtHorarioDesligamento1 ));
  Serial.println("Próximo horário despertar 2: " + formatTime(dtHorarioDespertar2));
  Serial.println("Próximo horário desligamento 2: " + formatTime(dtHorarioDesligamento2));
}

bool soloUmido() {
  // Realiza leitura de umidade do solo.
  float sensorValue = 0;
  for (int i = 0; i <= 100; i++){
    sensorValue = sensorValue + analogRead(sensorUmidadeAnalogInput);
    delay(1);
  }
  sensorValue = sensorValue/100.0;

  // Desativa sensir de umidade.
  digitalWrite(sensorUmidadeVCCPin, LOW);
  
  // Caso umidade esteja baixa, retorna false.
  if (sensorValue < valorMinimoSensorUmidade) {
    return true;
  } else {
    return false;
  }
}

bool horarioLigarBomba(DateTime currentTime) {
  // Ativa sensor de umidade.
  digitalWrite(sensorUmidadeVCCPin, HIGH);
  delay(10);
  
  long unixTimeCurrentTime = currentTime.unixtime();
  long unixTimeHorarioAcionamento1 = dtHorarioAcionamento1.unixtime();
  long unixTimeHorarioDesligamento1 = dtHorarioDesligamento1.unixtime();

  long unixTimeHorarioAcionamento2 = dtHorarioAcionamento2.unixtime();
  long unixTimeHorarioDesligamento2 = dtHorarioDesligamento2.unixtime();
  
  if ((unixTimeCurrentTime >= unixTimeHorarioAcionamento1 && unixTimeCurrentTime < unixTimeHorarioDesligamento1) ||
      (unixTimeCurrentTime >= unixTimeHorarioAcionamento2 && unixTimeCurrentTime < unixTimeHorarioDesligamento2)){
    return true;
  }
  else{
    return false;
  }
}

bool horarioDesligarBomba(DateTime currentTime){  
  long unixTimeCurrentTime = currentTime.unixtime();
  long unixTimeHorarioDesligamento1 = dtHorarioDesligamento1.unixtime();

  
  long unixTimeHorarioAcionamento2 = dtHorarioAcionamento2.unixtime();
  long unixTimeHorarioDesligamento2 = dtHorarioDesligamento2.unixtime();
  
  if ((unixTimeCurrentTime >= unixTimeHorarioDesligamento1 && unixTimeCurrentTime < unixTimeHorarioAcionamento2) ||
      (unixTimeCurrentTime >= unixTimeHorarioDesligamento2)){
        
    if (unixTimeCurrentTime >= unixTimeHorarioDesligamento1) { 
      Serial.println("unixTimeCurrentTime >= unixTimeHorarioDesligamento1 is TRUE"); 
    } else {
      Serial.println("unixTimeCurrentTime >= unixTimeHorarioDesligamento1 is FALSE");
    }
    if (unixTimeCurrentTime < unixTimeHorarioAcionamento2) { 
      Serial.println("unixTimeCurrentTime < unixTimeHorarioAcionamento2 is TRUE"); 
    } else {
      Serial.println("unixTimeCurrentTime < unixTimeHorarioAcionamento2 is FALSE"); 
    }
    if (unixTimeCurrentTime >= unixTimeHorarioDesligamento2) {
      Serial.println("unixTimeCurrentTime >= unixTimeHorarioDesligamento2 is TRUE");
    } else {
      Serial.println("unixTimeCurrentTime >= unixTimeHorarioDesligamento2 is FALSE");
    }

    Serial.println("currentTime: " + formatTime(currentTime));
    Serial.println("dtHorarioDesligamento2: " + formatTime(dtHorarioDesligamento2));
    Serial.print("unixTimeCurrentTime: ");
    Serial.println(unixTimeCurrentTime);
    Serial.print("unixTimeHorarioDesligamento2: ");
    Serial.println(unixTimeHorarioDesligamento2);
    
    return true;
  }
  else{
    return false;
  }
}

void alarm_ISR() {
  // This runs when SQW pin is low. It will wake up the microcontroller.

  sleep_disable(); // Disable sleep mode.
  detachInterrupt(digitalPinToInterrupt(alarmPin)); // Detach the interrupt to stop it firing.
}

void enterSleep(){
  Serial.println(formatTime(rtc.now()) + " sleep mode acionado. Arduino dorme.");
  delay(100);
  
  sleep_enable();                         // Enabling sleep mode.

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // Setting sleep mode, in this case full sleep.

  noInterrupts();                         // Disable interrupts.
  
  attachInterrupt(digitalPinToInterrupt(alarmPin), alarm_ISR, LOW);

  Serial.flush();

  interrupts();                           // Allow interrupts again.
  sleep_cpu();                            // Enter sleep mode.

  /* The program will continue from here when it wakes */

  Serial.println(formatTime(rtc.now()) + " Arduino acordou!");
}



void loop () {
    // IDENTIFICAÇÃO DO ACIONAMENTO DO BOTÃO.
    // millis() -> milisegundos desde o início da execução do programa.
    // keySampleIntervalMs -> intervalo mínimo de checagem do estado do botão.
    // keyPrevMillis -> milisegundos do início da execução até a última checagem do estado do botão.
    if (!temporizadorAcionado) {
      if (millis() - keyPrevMillis >= keySampleIntervalMs) {
        keyPrevMillis = millis();

        // Leitura do estado do botão (pressionado ou solto).
        byte currKeyState = digitalRead(keyPin);

        // Estado anterior: solto; estado atual: pressionado; significa que botão foi pressionado.
        if ((prevKeyState == LOW) && (currKeyState == HIGH)) {
          Serial.println("Key pressed");
          longKeyPressCount = 0;
        }

        // Estado anterior: pressionado; estado atual: solto; significa que botão foi pressionado depois foi solto.
        else if ((prevKeyState == HIGH) && (currKeyState == LOW)) {
          Serial.println("Key released");

          if (longKeyPressCount >= longKeyPressCountMax){
            Serial.println("Long key press");
            temporizadorAcionado = true; // Aciona temporizador na próxima iteração.
            digitalWrite(LED_BUILTIN, HIGH);
            delay(1000); // Delay para acionamento do sleep mode.
            digitalWrite(LED_BUILTIN, LOW);

            // CONFIGURA ALARMES PARA PRIMEIRO ACIONAMENTO DA BOMBA.
            DateTime currentTime = rtc.now();
            
            configurarAlarmes();
            delay(500);
            
            enterSleep();
          }
          else {
            Serial.println("Short key press");
          }
        }
        // Enquanto botão se mantém pressionado incrementa contagem de ciclos de keySampleIntervalMs.
        else if (currKeyState == HIGH) {
          longKeyPressCount++;
        }
        prevKeyState = currKeyState;
      }
    }

    // Temporizador.
    if (temporizadorAcionado) {
        
        DateTime currentTime = rtc.now();

        if (horarioLigarBomba(currentTime) && digitalRead(mosfetPin) == LOW) {
          Serial.println(formatTime(currentTime) + " verificando umidade do solo...");
          delay(1000);

          if (!soloUmido()) {
            Serial.println(formatTime(currentTime) + " acionando bomba...");
            delay(1500);
            
            // LIGAR BOMBA.
            digitalWrite(mosfetPin, HIGH);
            Serial.println(formatTime(currentTime) + " bomba acionada.");
          } else {
            Serial.println(formatTime(currentTime) + " solo suficientemente irrigado.");
          }
        }

        if (horarioDesligarBomba(currentTime) && digitalRead(mosfetPin) == HIGH) {

          Serial.println(formatTime(rtc.now()) + " desligando bomba...");
          delay(1000);
          
          // DESLIGAR BOMBA.
          digitalWrite(mosfetPin, LOW);
          Serial.println(formatTime(rtc.now()) + " bomba desligada.");

          // TRABALHO FINALIZADO. VOLTA A DORMIR.
          configurarAlarmes();
          delay(500);

          enterSleep();
        }
      
        delay(1000);
        
   }
}
    
