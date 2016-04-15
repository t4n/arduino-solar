// ---------------------------------------------------------------------------
// Arduino-kompatibles Solarlicht mit ATTINY85
// t4n, 15. April 2016
// ---------------------------------------------------------------------------

#include <avr/sleep.h>
#include <avr/wdt.h>

// ---------------------------------------------------------------------------
// Einzelne Bits in einem Register setzen oder löschen
// ---------------------------------------------------------------------------
#undef sbi
#undef cbi
#define sbi(ADDR, BIT) ((ADDR) |=  (1 << (BIT)))
#define cbi(ADDR, BIT) ((ADDR) &= ~(1 << (BIT)))

// ---------------------------------------------------------------------------
// Mögliche Zustände, die das Solarlicht annehmen kann
// ---------------------------------------------------------------------------
#define WARTE_AUF_TAG 0
#define WARTE_AUF_NACHT 1
#define SEQUENZ_A 2
#define SEQUENZ_B 3
#define SEQUENZ_C 4

// ---------------------------------------------------------------------------
// Anschlüsse der LEDs und der Fotodiode
// ---------------------------------------------------------------------------
#define FOTODIODE A2
#define LED_BLAU 0
#define LED_ROT 1
#define LED_WEISS 2
#define LED_EIN LOW   // Achtung, invertiert:
#define LED_AUS HIGH  // LOW = ein, HIGH = aus

// ---------------------------------------------------------------------------
// Schwellwerte für Helligkeit (Erkennung Tag/Nacht).
// ---------------------------------------------------------------------------
#define SCHWELLWERT_HELL 25
#define SCHWELLWERT_DUNKEL 4

// ---------------------------------------------------------------------------
// Um Strom zu sparen, werden die LEDs mittels PWM nur zu maximal ca. 40%
// eingeschaltet - das ist auch schon hell genug. Der genaue Maximalwert wird
// durch die folgende Konstante festgelegt (0 = ganz an, 255 = ganz aus).
// ---------------------------------------------------------------------------
#define LED_MAX 150

// ---------------------------------------------------------------------------
// Funktion für Watchdog-Interrupt (wird alle 8 Sekunden ausgelöst,
// um den Mikrocontroller wieder aus dem Schlafmodus aufzuwecken).
// ---------------------------------------------------------------------------
ISR(WDT_vect)
{
  // Dummy
}

// ---------------------------------------------------------------------------
// Einzelne PWM-Periode an weißer LED ausgeben (am zweiten Digital-I/O
// ist kein "automatisches" PWM-Signal möglich, daher zu Fuß programmiert).
// ---------------------------------------------------------------------------
void pwmWeissAusgeben(int pwm)
{
  // pwm = 255 --> weiße LED ganz aus
  // pwm = 0 ----> weiße LED ganz ein
  for(int count = 0; count <= 255; count++)
  {
    if(pwm >= count)
      sbi(PORTB, LED_WEISS);  // LED aus
    else
      cbi(PORTB, LED_WEISS);  // LED ein
  }
}

// ---------------------------------------------------------------------------
// Fotodiode abfragen; der Rückgabewert ist umso größer, je heller es ist.
// ---------------------------------------------------------------------------
int aktuelleHelligkeit(void)
{
  // Es werden immer die letzten vier Helligkeitswerte gemittelt.
  static int hell1 = SCHWELLWERT_DUNKEL;
  static int hell2 = SCHWELLWERT_DUNKEL;
  static int hell3 = SCHWELLWERT_DUNKEL;
  static int hell4 = SCHWELLWERT_DUNKEL;
  
  hell4 = hell3, hell3 = hell2, hell2 = hell1;
  hell1 = analogRead(FOTODIODE);
  return (hell1 + hell2 + hell3 + hell4) / 4;
}

// ---------------------------------------------------------------------------
// Der Mikrocontroller wird in den Schlafmodus versetzt, Analog-Komparator
// und AD-Wandler werden deaktiviert um den Stromverbrauch weiter zu senken.
// ---------------------------------------------------------------------------
void controller_in_schlafmodus_versetzen(void)
{
  // IO-Ports für LEDs deaktivieren --> Strom sparen!
  pinMode(LED_BLAU,  INPUT_PULLUP);
  pinMode(LED_ROT,   INPUT_PULLUP);
  pinMode(LED_WEISS, INPUT_PULLUP);
  
  // Analog-Komparator und AD-Wandler deaktivieren --> Strom sparen!
  ACSR |= _BV(ACD);      // Komparator aus
  ADCSRA &= ~_BV(ADEN);  // AD-Wandler aus
  PRR = 0xFF;            // "Power Reduction Register" --> alles aus!

  // Jetzt wird der Mikrocontroller in den Schlafmodus versetzt.
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();

  // Nach dem nächsten Watchdog-Interrupt (8 Sekunden) geht es hier weiter.
  sleep_disable();

  // Timer und AD-Wandler werden wieder aktiviert.
  PRR = 0x00;
  ADCSRA |= (1 << ADEN);

  // Die erste AD-Wandlung nach dem Aufwachen kann evtl. ungenau
  // sein, sie wird daher einfach verworfen.
  /* int dummy = */ aktuelleHelligkeit();
}

// ---------------------------------------------------------------------------
// Vorbereitungen direkt nach dem Start des Mikrocontrollers.
// ---------------------------------------------------------------------------
void setup(void)
{
  // Erst einmal abwarten, ob der Mikrocontroller wirklich stabil
  // läuft oder ob evtl. gleich wieder ein BOD-Reset erfolgt, weil
  // die Supercaps nicht genügend aufgeladen sind.
  delay(1000);

  // Interne 1V1-Spannungsreferenz für den AD-Wandler benutzen,
  // weil damit die Auflösung bei geringer Helligkeit höher ist.
  ADMUX |= (1 << REFS1);

  // Watchdog-Timer so einstellen, dass immer nach 8 Sekunden ein
  // Interrupt ausgelöst wird; dies ist notwendig, um später den
  // Mikrocontroller wieder aus dem Schlafmodus aufzuwecken.
  MCUSR &= ~(1 << WDRF);                // reset status flag
  WDTCR |=  (1 << WDCE) | (1 << WDE);   // enable configuration changes
  WDTCR =   (1 << WDP3) | (1 << WDP0);  // interval 8 sec
  WDTCR |=  (1 << WDIE);                // enable interrupt mode
}

// ---------------------------------------------------------------------------
// Hauptschleife
// ---------------------------------------------------------------------------
void loop(void)
{
  static int zustand = WARTE_AUF_TAG;
  static int sequenz_zaehler = 0;
  int i, j, pwm;

  // Bei nachlassender Versorgungsspannung wäre theoretisch folgendes
  // Verhalten möglich: Der Schwellenwert des Brownout-Detektors (BOD)
  // wird unterschritten, der Mikrocontroller wird gestoppt. Dadurch sinkt
  // der Stromverbrauch und die Versorgungsspannung steigt wieder über den
  // BOD-Schwellenwert an. Der Mikrocontroller startet erneut und stellt
  // fest, dass es (immer noch) dunkel ist. Er schaltet die LEDs ein,
  // wodurch die Versorgungsspannung wieder unter den BOD-Schwellenwert
  // absinkt usw. Die Folge wäre ein unkontrolliertes Flackern der LEDs.
  //
  // Um dies zu verhindern, wartet die Solarleuchte nach einem Neustart
  // erst einmal ab, bis es in der Umgebung wieder hell geworden ist
  // (Zustand: WARTE_AUF_TAG), bevor sie dann bei wieder nachlassender
  // Helligkeit die LEDs einschaltet (Zustand: WARTE_AUF_NACHT).
  
  switch(zustand)
  {
    // *****************************
    case WARTE_AUF_TAG:
      if(aktuelleHelligkeit() >= SCHWELLWERT_HELL)
        zustand = WARTE_AUF_NACHT;
      else
        controller_in_schlafmodus_versetzen();  // Abwarten und Strom sparen
      break;

    // *****************************
    case WARTE_AUF_NACHT:
      if(aktuelleHelligkeit() <= SCHWELLWERT_DUNKEL)
        zustand = SEQUENZ_A;
      else
        controller_in_schlafmodus_versetzen();  // Abwarten und Strom sparen
      break;

    // *****************************
    case SEQUENZ_A:
      // Hinweis: Alle angeschlossenen LEDs werden dadurch eingeschaltet,
      // dass der entsprechende Port auf LOW geschaltet wird. Zum Ausschalten
      // einer LED muss der entsprechende Port auf HIGH gesetzt werden.
      pinMode(LED_BLAU,  OUTPUT); analogWrite (LED_BLAU,  255);
      pinMode(LED_ROT,   OUTPUT); analogWrite (LED_ROT,   255);
      pinMode(LED_WEISS, OUTPUT); digitalWrite(LED_WEISS, LED_AUS);

      // Blaue LED einblenden und 30 Sekunden warten
      for(pwm = 255; pwm >= LED_MAX; pwm--)
      { analogWrite(LED_BLAU, pwm); delay(100); }
      delay(30000);
      
      // Rote LED einblenden, blaue LED ausblenden, 30 Sekunden warten
      for(pwm = 255; pwm >= LED_MAX; pwm--)
      { analogWrite(LED_ROT, pwm); delay(100); }
      for(pwm = LED_MAX; pwm <= 255; pwm++)
      { analogWrite(LED_BLAU, pwm); delay(100); }
      delay(30000);

      // Rote LED wieder ausblenden
      for(pwm = LED_MAX; pwm <= 255; pwm++)
      { analogWrite(LED_ROT, pwm); delay(100); }

      // Testen, ob es noch dunkel ist
      if(aktuelleHelligkeit() >= SCHWELLWERT_HELL)
        zustand = WARTE_AUF_NACHT;
      else
        zustand = SEQUENZ_B;
      break;

    // *****************************
    case SEQUENZ_B:
      // Weiße LED langsam einblenden
      for(pwm = 255; pwm >= LED_MAX; pwm--)
        for(i = 0; i < 10; ++i)
          pwmWeissAusgeben(pwm);

      // Etwas warten
      for(j = 0; j <= 10000; j++)
        pwmWeissAusgeben(pwm);
      
      // Weiße LED wieder ausblenden
      for(pwm = LED_MAX; pwm <= 255; pwm++)
        for(i = 0; i < 10; ++i)
          pwmWeissAusgeben(pwm);

      // Testen, ob es noch dunkel ist
      zustand = WARTE_AUF_NACHT;
      if(aktuelleHelligkeit() < SCHWELLWERT_HELL)
      {
        // Normalerweise geht es jetzt weiter mit dem Ein-/Ausblenden
        // der blauen und roten LEDs. Nur hin und wieder gehen wir nach
        // SEQUENZ_C und lassen alle LEDs für einige Sekunden blinken...
        ++sequenz_zaehler;
        if(sequenz_zaehler % 5 == 0)
          zustand = SEQUENZ_C;
        else
          zustand = SEQUENZ_A;
      }
      break;

    // *****************************
    case SEQUENZ_C:
      // Alle Farben blinken für wenige Sekunden
      for(i = 1; i < 10; ++i)
      {
        digitalWrite(LED_BLAU, LED_EIN);
        delay(100);
        digitalWrite(LED_BLAU, LED_AUS);
        digitalWrite(LED_ROT, LED_EIN);
        delay(100);
        digitalWrite(LED_ROT, LED_AUS);
        digitalWrite(LED_WEISS, LED_EIN);
        delay(100);
        digitalWrite(LED_WEISS, LED_AUS);
      }
      
      // Testen, ob es noch dunkel ist
      if(aktuelleHelligkeit() >= SCHWELLWERT_HELL)
        zustand = WARTE_AUF_NACHT;
      else
        zustand = SEQUENZ_A;
      break;
  }      
}

