/*
  ASCENSOR 3 PISOS - STM32F411 Black Pill (core STM32duino, Arduino IDE / VS Code)

  - Motor 28BYJ-48: movimiento por PASOS (sin sensores). NO MODIFICADO.
  - Servo SG90 (puerta): estado inicial ABIERTA (brazo al norte). Cierra antes de moverse y
    abre al llegar. Cerrada = brazo al oeste (~80 grados). (Logica de servo de la version que funcionaba)
  - Sensores TCRT5000: se leen (irLeer) pero el LCD muestra el piso segun los PASOS.
  - LEDs: verde = reposo, amarillo = en movimiento, rojo = emergencia.
  - LCD 16x2 I2C: "Piso -" al pulsar un boton y mientras se mueve; al frenar muestra el piso
    que corresponde a los pasos.

  NOVEDADES DE ESTA VERSION
  1) Emergencia: al liberarla (2a pulsacion) el ascensor RETOMA el destino que tenia antes de
     la emergencia. Las llamadas pendientes ya no se borran.
  2) Prioridad por cercania: se atiende primero el piso solicitado mas CERCANO.
     Desempate (ej: en piso 2 con llamadas 1 y 3): gana el que siga el sentido de la ultima
     marcha (llego subiendo -> va al 3, llego bajando -> va al 1).
  3) SERVO: vuelve a la logica original que funcionaba (pulso directo, 660 us, 600 ms).

  MODO_SERVO 1   -> herramienta para CALIBRAR los pulsos del servo (P1 menos / P2 mas).
  MODO_PRUEBA 0  -> CALIBRACION de pasos entre pisos (como antes).
  MODO_PRUEBA 1  -> ASCENSOR normal.

  PINES
    Pulsadores (a GND, pull-up interno): P1 = PA1, P2 = PA2, P3 = PA3, EMERGENCIA = PA4
    Motor ULN2003 IN1..IN4:              PB12, PB13, PB14, PB15
    LCD I2C:                             SDA = PB7, SCL = PB6
    Servo SG90 (senal naranja):          PA8 (TIM1_CH1, PWM 50 Hz)
    Sensores IR TCRT5000 (DO):           PA5 (piso 1), PA6 (piso 2), PA7 (piso 3)
    LEDs (con 220 ohm a GND):            PB8 verde (reposo), PB9 amarillo (movimiento), PB10 rojo (emergencia)
*/
#include <Arduino.h>
#include <Wire.h>

/* ================= CONFIGURACION ================= */
#define MODO_SERVO         0        /* 1 = calibrar servo (ver arriba) */
#define MODO_PRUEBA        1        /* 0 = calibrar pasos, 1 = ascensor */
#define USAR_SERIAL        0        /* 1 = tambien imprime por Serial */

#define SENTIDO_SUBIDA     1        /* pon -1 si al pulsar SUBIR la cabina baja */
#define T_PASO_US          2000UL   /* us entre pasos a velocidad normal (2 ms). Sube a 3000 si pierde pasos */
#define RAMP_PASOS         150      /* pasos de arranque/frenado suave (modo prueba) */
#define RAMP_INC_US        10UL     /* cuanto mas lento por cada paso de rampa */
#define LIMITE_PASOS       120000L  /* tope de seguridad en calibracion (|pos| maximo) */
#define PASOS_POR_VUELTA   4096.0f  /* 28BYJ-48 en medio paso */
#define DEB_MS             30       /* antirrebote */
#define LARGO_MS           2000     /* pulsacion larga de P3 */
#define PASOS_LLEGANDO     3000     /* en los ultimos pasos del viaje el LCD muestra "Llegando" */

/* Pasos medidos (indice 1..3). Piso 1 = 0. */
static const int32_t PASOS_PISO[4] = {0, 0, 21500, 41525};   /* piso1=0, piso2=21500, piso3=41525 */

/* ---- Servo de la puerta ----
   SG90: ~10.5 us por grado. Abierta = brazo al NORTE. Cerrada = brazo al OESTE (~80 grados).
   CALIBRA con MODO_SERVO 1: primero ajusta SERVO_ABIERTO_US hasta que el brazo apunte al norte,
   luego encuentra el pulso donde apunta al oeste y ponlo en SERVO_CERRADO_US.
   Si al cerrar gira hacia el ESTE en vez del oeste, cambia SERVO_CERRADO_US a 2340 (ABIERTO + 840). */
#define SERVO_ABIERTO_US   1500
#define SERVO_CERRADO_US   660
#define SERVO_CON_LIBRERIA 0        /* 0 = PWM por hardware TIM1 (pulso limpio), 1 = libreria Servo */
#define T_SERVO_MS         600      /* tiempo para que el servo llegue */
#define T_LLEGANDO_MS      1000     /* la puerta abre 1 s DESPUES de parar en el piso */
#define T_PUERTA_ABIERTA_MS 3000    /* solo si PUERTA_ABIERTA_EN_REPOSO = 0 */
/* 1 = la puerta queda ABIERTA en reposo y solo se cierra cuando hay que moverse (estado inicial abierta)
   0 = despues de 3 s abierta, se cierra sola (como pide el PDF del proyecto) */
#define PUERTA_ABIERTA_EN_REPOSO 1

/* ---- Sensores IR ---- */
#define SENSOR_ACTIVO      LOW      /* TCRT5000 con LM393: DO baja al detectar */
#define IR_FILTRO_MS       20       /* el cambio debe durar 20 ms para aceptarse */

#define MOSTRAR_PASOS      0        /* 1 = la fila 1 del LCD tambien muestra los pasos (para depurar) */

#if SERVO_CON_LIBRERIA
#include <Servo.h>
#endif

/* ================= PINES ================= */
static const uint32_t PIN_BTN[3] = {PA1, PA2, PA3};   /* P1, P2, P3 */
#define PIN_EMG            PA4
static const uint32_t PIN_MOTOR[4] = {PB12, PB13, PB14, PB15};  /* IN1..IN4 */
/* Las 4 bobinas deben ser 4 pines SEGUIDOS del mismo puerto (PB12..PB15) */
#define COIL_PORT          GPIOB
#define COIL_SHIFT         12

#define PIN_SERVO          PA8
static const uint32_t PIN_IR[3] = {PA5, PA6, PA7};    /* piso 1, 2, 3 */
#define PIN_LED_REPOSO     PB8      /* verde    */
#define PIN_LED_MOV        PB9      /* amarillo */
#define PIN_LED_EMG        PB10     /* rojo     */

/* Si USAR_SERIAL = 0, Serial se sustituye por un objeto vacio (no imprime ni bloquea) */
#if !USAR_SERIAL
struct NullSerial {
  template<class T> void print(T) {}
  template<class T, class U> void print(T, U) {}
  template<class T> void println(T) {}
  template<class T, class U> void println(T, U) {}
  void println() {}
  void begin(long) {}
};
static NullSerial NullSer;
#define Serial NullSer
#endif

/* ================= LCD I2C (PCF8574) ================= */
static uint8_t lcdAddr = 0x27;
static bool    lcdOk = false;
static char    lcdDes[32];           /* lo que queremos mostrar */
static char    lcdAct[32];           /* lo que la pantalla ya muestra */
static int     lcdCur = -1;
static char    msgTmp[17];
static uint32_t msgFin = 0;

static void lcdNibble(uint8_t n)
{
  uint8_t b = (n & 0xF0) | 0x08;
  uint8_t d[2] = { (uint8_t)(b | 0x04), b };
  Wire.beginTransmission(lcdAddr);
  Wire.write(d, 2);
  Wire.endTransmission();
}

static void lcdEnviar(uint8_t val, uint8_t rs)
{
  uint8_t hi = val & 0xF0;
  uint8_t lo = (uint8_t)((val << 4) & 0xF0);
  uint8_t d[4] = { (uint8_t)(hi | 0x0C | rs), (uint8_t)(hi | 0x08 | rs),
                   (uint8_t)(lo | 0x0C | rs), (uint8_t)(lo | 0x08 | rs) };
  Wire.beginTransmission(lcdAddr);
  Wire.write(d, 4);
  Wire.endTransmission();
}

static void lcdInit(void)
{
  /* Flechas personalizadas: codigo 1 = arriba, codigo 2 = abajo */
  static const uint8_t flechaArriba[8] = {0x04,0x0E,0x15,0x04,0x04,0x04,0x04,0x00};
  static const uint8_t flechaAbajo[8]  = {0x04,0x04,0x04,0x04,0x15,0x0E,0x04,0x00};

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(100000);

  Wire.beginTransmission(0x27);
  if (Wire.endTransmission() == 0) lcdAddr = 0x27;
  else {
    Wire.beginTransmission(0x3F);
    if (Wire.endTransmission() == 0) lcdAddr = 0x3F;
    else { lcdOk = false; return; }
  }
  lcdOk = true;

  delay(50);
  lcdNibble(0x30); delay(5);
  lcdNibble(0x30); delay(1);
  lcdNibble(0x30); delay(1);
  lcdNibble(0x20); delay(1);
  lcdEnviar(0x28, 0); delay(1);   /* 4 bits, 2 lineas */
  lcdEnviar(0x0C, 0); delay(1);   /* display ON, sin cursor */
  lcdEnviar(0x06, 0); delay(1);
  lcdEnviar(0x01, 0); delay(3);   /* limpiar */

  lcdEnviar(0x48, 0);             /* CGRAM, caracter 1 */
  for (uint8_t i = 0; i < 8; i++) lcdEnviar(flechaArriba[i], 1);
  for (uint8_t i = 0; i < 8; i++) lcdEnviar(flechaAbajo[i], 1);
  lcdEnviar(0x80, 0);             /* volver a DDRAM */

  memset(lcdDes, ' ', sizeof(lcdDes));
  memset(lcdAct, ' ', sizeof(lcdAct));
  lcdCur = -1;
}

static void lcdPoner(uint8_t fila, const char *txt)
{
  char tmp[17];
  snprintf(tmp, sizeof(tmp), "%-16.16s", txt);
  memcpy(&lcdDes[fila * 16], tmp, 16);
}

/* Manda como maximo UN caracter por llamada: el motor nunca se queda bloqueado */
static void lcdPoll(void)
{
  static uint8_t inicio = 0;
  if (!lcdOk) return;
  for (uint8_t n = 0; n < 32; n++) {
    uint8_t k = (uint8_t)((inicio + n) & 31);
    if (lcdDes[k] != lcdAct[k]) {
      if (lcdCur != k)
        lcdEnviar((uint8_t)(0x80 | ((k / 16) ? 0x40 : 0x00) | (k % 16)), 0);
      lcdEnviar((uint8_t)lcdDes[k], 1);
      lcdAct[k] = lcdDes[k];
      lcdCur = ((k % 16) == 15) ? -1 : (k + 1);
      inicio = (uint8_t)((k + 1) & 31);
      return;
    }
  }
}

/* Mensaje temporal en la fila 2 */
static void lcdMensaje(const char *t, uint32_t ms)
{
  strncpy(msgTmp, t, 16);
  msgTmp[16] = 0;
  msgFin = millis() + ms;
}

/* ================= SERVO SG90 (puerta) ================= */
/*
   Control por posicion, igual que en el codigo de referencia:
   - servoInit(): configura PWM de 50 Hz y manda la posicion ABIERTA.
   - puerta_abrir()/puerta_cerrar(): mandan un pulso solo al cambiar de estado.
   - El PWM permanece estable mientras el servo sostiene la posicion.
*/
static bool puerta_abierta = true;           /* servo inicializado en ABIERTA */
static uint16_t servo_pulso_actual = SERVO_ABIERTO_US;

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

static uint16_t servoLimitarPulso(uint16_t us)
{
  if (us < SERVO_MIN_US) us = SERVO_MIN_US;
  if (us > SERVO_MAX_US) us = SERVO_MAX_US;
  return us;
}

#if SERVO_CON_LIBRERIA
static Servo servoPuerta;

static void servoInit(void)
{
  servo_pulso_actual = servoLimitarPulso(SERVO_ABIERTO_US);
  servoPuerta.attach(PIN_SERVO, SERVO_MIN_US, SERVO_MAX_US);
  servoPuerta.writeMicroseconds(servo_pulso_actual);
  puerta_abierta = true;
}

static void servoUs(uint16_t us)
{
  us = servoLimitarPulso(us);
  if (us == servo_pulso_actual) return;
  servoPuerta.writeMicroseconds(us);
  servo_pulso_actual = us;
}
#else
static HardwareTimer *servoTim = NULL;

static void servoInit(void)
{
  servo_pulso_actual = servoLimitarPulso(SERVO_ABIERTO_US);
  servoTim = new HardwareTimer(TIM1);
  if (servoTim == NULL) return;

  servoTim->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PIN_SERVO);
  servoTim->setOverflow(20000, MICROSEC_FORMAT);  /* 20 ms = 50 Hz */
  servoTim->setCaptureCompare(1, servo_pulso_actual, MICROSEC_COMPARE_FORMAT);
  servoTim->resume();
  puerta_abierta = true;
}

static void servoUs(uint16_t us)
{
  us = servoLimitarPulso(us);
  if (servoTim == NULL || us == servo_pulso_actual) return;
  servoTim->setCaptureCompare(1, us, MICROSEC_COMPARE_FORMAT);
  servo_pulso_actual = us;
}
#endif

static void puerta_abrir(void)
{
  if (puerta_abierta) return;
  servoUs(SERVO_ABIERTO_US);
  puerta_abierta = true;
}

static void puerta_cerrar(void)
{
  if (!puerta_abierta) return;
  servoUs(SERVO_CERRADO_US);
  puerta_abierta = false;
}

/* ================= SENSORES IR ================= */
static uint8_t  irCrudo[4], irEst[4];
static uint32_t irT[4];
static uint8_t  pisoSensor = 0;              /* ultimo piso detectado por un sensor (0 = ninguno) */

static uint8_t sensorRaw(uint8_t p)
{
  return (digitalRead(PIN_IR[p - 1]) == SENSOR_ACTIVO) ? 1 : 0;
}

static void irIniciar(void)
{
  delay(20);
  for (uint8_t p = 1; p <= 3; p++) { irCrudo[p] = irEst[p] = sensorRaw(p); irT[p] = millis(); }
  for (uint8_t p = 1; p <= 3; p++) if (irEst[p]) { pisoSensor = p; break; }
}

static void irLeer(void)
{
  uint32_t ahora = millis();
  for (uint8_t p = 1; p <= 3; p++) {
    uint8_t l = sensorRaw(p);
    if (l != irCrudo[p]) { irCrudo[p] = l; irT[p] = ahora; }
    if ((ahora - irT[p]) >= IR_FILTRO_MS && irEst[p] != irCrudo[p]) {
      irEst[p] = irCrudo[p];
      if (irEst[p]) pisoSensor = p;
    }
  }
}

/* ================= LEDs ================= */
static void leds(bool mov, bool emg)
{
  static int8_t ult = -1;
  int8_t est = emg ? 2 : (mov ? 1 : 0);
  if (est == ult) return;
  ult = est;
  digitalWrite(PIN_LED_REPOSO, est == 0);
  digitalWrite(PIN_LED_MOV,    est == 1);
  digitalWrite(PIN_LED_EMG,    est == 2);
}

/* ================= MOTOR ================= */
static const uint8_t SEQ[8] = {0x1, 0x3, 0x2, 0x6, 0x4, 0xC, 0x8, 0x9};  /* medio paso */
static uint8_t fase = 0;

volatile uint8_t  emergencia = 0;
volatile uint32_t tEmg = 0;

/* Escribe las bobinas. Devuelve false si la emergencia esta activa (no energiza nada) */
static bool coils(uint8_t v)
{
  bool ok;
  noInterrupts();
  ok = !emergencia;
  if (ok) {
    COIL_PORT->BSRR = ((uint32_t)(~v & 0xF) << (COIL_SHIFT + 16)) |
                      ((uint32_t)(v & 0xF) << COIL_SHIFT);
  }
  interrupts();
  return ok;
}

static void motor_apagar(void)
{
  COIL_PORT->BSRR = (0xFUL << (COIL_SHIFT + 16));
}

/* d = +1 sube, -1 baja (sentido LOGICO). Devuelve true si el paso se dio */
static bool motor_paso(int8_t d)
{
  if (emergencia) return false;
  int8_t f = d * SENTIDO_SUBIDA;
  fase = (f > 0) ? ((fase + 1) & 7) : ((fase + 7) & 7);
  return coils(SEQ[fase]);
}

/* EMERGENCIA: lo primero es apagar las bobinas */
void emergenciaISR(void)
{
  motor_apagar();
  uint32_t ahora = millis();
  if ((ahora - tEmg) >= 250UL) {
    tEmg = ahora;
    emergencia ^= 1;                 /* 1 = detenido, 0 = liberar */
  }
}

/* ================= BOTONES (con antirrebote) ================= */
static bool crudo[3], estable[3], flancoP[3], soltado[3];
static uint32_t tCambio[3], tPulso[3];

static void botonesActualizar(void)
{
  uint32_t ahora = millis();
  for (uint8_t i = 0; i < 3; i++) {
    bool r = (digitalRead(PIN_BTN[i]) == LOW);
    if (r != crudo[i]) { crudo[i] = r; tCambio[i] = ahora; }
    if ((ahora - tCambio[i]) >= DEB_MS && estable[i] != crudo[i]) {
      estable[i] = crudo[i];
      if (estable[i]) { flancoP[i] = true; tPulso[i] = ahora; }
      else            { soltado[i] = true; }
    }
  }
}

static void botonesLimpiar(void)
{
  memset(flancoP, 0, sizeof(flancoP));
  memset(soltado, 0, sizeof(soltado));
}

/* ================= POSICION ================= */
static int32_t pos = 0;              /* pasos desde el arranque (+ = sube) */
static uint32_t tPaso = 0;

static void avisoEmergencia(void)
{
  static uint8_t prev = 0;
  if (emergencia != prev) {
    prev = emergencia;
    if (emergencia) {
      Serial.print(">>> EMERGENCIA: motor detenido. pos = ");
      Serial.println((long)pos);
    } else {
      Serial.println(">>> Emergencia liberada.");
    }
  }
}

#if MODO_SERVO
/* =====================================================================
   MODO SERVO: calibrar los pulsos de la puerta
   P1 = menos us | P2 = mas us (mantener). El LCD muestra el pulso actual.
   Anota el valor para "norte" (ABIERTO) y para "oeste" (CERRADO).
   ===================================================================== */
static uint16_t servoPulso = SERVO_ABIERTO_US;

static void servoCal(void)
{
  static uint32_t t = 0, tl = 0;
  uint32_t ahora = millis();

  if ((ahora - t) >= 25) {
    t = ahora;
    if (estable[1] && !estable[0] && servoPulso < 2500) servoPulso += 5;
    if (estable[0] && !estable[1] && servoPulso > 400)  servoPulso -= 5;
    servoUs(servoPulso);
  }
  if ((ahora - tl) >= 100) {
    tl = ahora;
    char a[24];
    snprintf(a, sizeof(a), "Servo: %u us", (unsigned)servoPulso);
    lcdPoner(0, a);
    lcdPoner(1, "P1 menos P2 mas");
  }
  lcdPoll();
  leds(false, false);
}
#endif /* MODO_SERVO */

#if !MODO_PRUEBA
/* =====================================================================
   MODO CALIBRACION DE PASOS
   ===================================================================== */
static int32_t marcas[4];
static uint8_t siguiente = 1;        /* proximo piso a marcar (1..3), 4 = listo */
static bool moviendo = false;
static bool avisoLimite = false;
static int8_t ultDir = 1;

static void imprimirResultado(void)
{
  int32_t a = marcas[2] - marcas[1];
  int32_t b = marcas[3] - marcas[2];
  Serial.println("=========== RESULTADO ===========");
  Serial.print("Piso 1 -> 2 : "); Serial.print((long)a); Serial.println(" pasos");
  Serial.print("Piso 2 -> 3 : "); Serial.print((long)b); Serial.println(" pasos");
  Serial.print("Piso 1 -> 3 : "); Serial.print((long)(a + b)); Serial.println(" pasos");
  Serial.print("static const int32_t PASOS_PISO[4] = {0, 0, ");
  Serial.print((long)a); Serial.print(", "); Serial.print((long)(a + b)); Serial.println("};");
}

static void reiniciarMarcas(void)
{
  siguiente = 1;
  lcdMensaje("Marcas borradas", 2000);
  Serial.println(">>> Marcas borradas.");
}

static void marcarPiso(void)
{
  if (siguiente > 3) { imprimirResultado(); return; }

  if (siguiente > 1 && pos <= marcas[siguiente - 1]) {
    lcdMensaje("ERROR: suba mas", 2000);
    Serial.println("ERROR: el piso debe quedar MAS ARRIBA que el anterior.");
    return;
  }
  marcas[siguiente] = pos;
  char t[17];
  snprintf(t, sizeof(t), "Piso %d marcado", siguiente);
  lcdMensaje(t, 1500);
  Serial.print("Piso "); Serial.print(siguiente);
  Serial.print(" marcado en pos = "); Serial.println((long)pos);
  siguiente++;
  if (siguiente > 3) imprimirResultado();
}

static void calibracion(void)
{
  if (emergencia) { moviendo = false; return; }

  bool baja = estable[0] && !estable[1];
  bool sube = estable[1] && !estable[0];
  uint32_t us = micros();

  if (sube || baja) {
    if (!moviendo) { moviendo = true; tPaso = us - T_PASO_US; }
    if ((uint32_t)(us - tPaso) >= T_PASO_US) {
      tPaso = us;
      int8_t d = sube ? 1 : -1;
      ultDir = d;
      if (labs((long)(pos + d)) > LIMITE_PASOS) {
        if (!avisoLimite) { avisoLimite = true; lcdMensaje("LIMITE alcanzado", 2000); }
      } else {
        avisoLimite = false;
        if (motor_paso(d)) pos += d;
      }
    }
  } else if (moviendo) {
    moviendo = false;
    motor_apagar();
    Serial.print("Parado en pos = "); Serial.println((long)pos);
  }

  if (!moviendo && soltado[2]) {
    if ((millis() - tPulso[2]) >= LARGO_MS) reiniciarMarcas();
    else                                    marcarPiso();
  }
}

static void ui(void)
{
  static uint32_t t = 0;
  uint32_t ahora = millis();
  leds(moviendo, emergencia);
  if ((ahora - t) >= 100) {
    t = ahora;
    char a[24], b[24];
    if (emergencia) {
      snprintf(a, sizeof(a), "MODO EMERGENCIA");
      snprintf(b, sizeof(b), "SISTEMA DETENIDO");
    } else {
      snprintf(a, sizeof(a), "CAL Pos:%ld", (long)pos);
      if ((int32_t)(msgFin - ahora) > 0) {
        snprintf(b, sizeof(b), "%s", msgTmp);
      } else if (siguiente > 3) {
        long x = (long)(marcas[2] - marcas[1]);
        long y = (long)(marcas[3] - marcas[2]);
        switch ((ahora / 2500) % 3) {
          case 0:  snprintf(b, sizeof(b), "1>2:%ld", x);     break;
          case 1:  snprintf(b, sizeof(b), "2>3:%ld", y);     break;
          default: snprintf(b, sizeof(b), "1>3:%ld", x + y); break;
        }
      } else if (moviendo) {
        snprintf(b, sizeof(b), "%s", (ultDir > 0) ? "Subiendo" : "Bajando");
      } else {
        snprintf(b, sizeof(b), "Marque piso %d", siguiente);
      }
    }
    lcdPoner(0, a);
    lcdPoner(1, b);
  }
  lcdPoll();
}
#endif /* !MODO_PRUEBA */

#if MODO_PRUEBA
/* =====================================================================
   MODO ASCENSOR: ir a cada piso contando pasos + puerta
   ===================================================================== */
enum Estado { E_ESPERA, E_MOVIENDO, E_EMERG, E_LLEGANDO, E_ABRIENDO, E_PUERTA, E_CERRANDO };
static Estado estado = E_ESPERA;
static uint8_t llamadas[4] = {0, 0, 0, 0};
static int8_t  dir = 1;                /* sentido de la ultima marcha (+1 sube, -1 baja) */
static uint8_t destino = 0;
static uint8_t destinoGuardado = 0;    /* destino interrumpido por una emergencia */
static int32_t posInicio = 0;
static uint32_t tEstado = 0;

static uint8_t pisoPorPos(void)
{
  for (uint8_t p = 1; p <= 3; p++) if (pos == PASOS_PISO[p]) return p;
  return 0;
}

/* true si hay una llamada pendiente a un piso distinto de donde esta la cabina */
static bool llamadaPendiente(void)
{
  uint8_t aqui = pisoPorPos();
  for (uint8_t p = 1; p <= 3; p++)
    if (llamadas[p] && p != aqui) return true;
  return false;
}

/* Piso que se muestra en el LCD (0 = se imprime "-"):
   - Al pulsar un boton (llamada pendiente) o mientras se mueve: "Piso -"
   - Cuando el motor frena: el piso que corresponde a los pasos (pos) */
static uint8_t pisoMostrado(void)
{
  if (estado == E_MOVIENDO) return 0;
  if ((estado == E_ESPERA || estado == E_CERRANDO) && llamadaPendiente()) return 0;
  return pisoPorPos();          /* 0 si los pasos no coinciden con ningun piso */
}

/* Llamada mas cercana POR DELANTE en el sentido dado */
static uint8_t buscar(int8_t sentido, bool inclusivo)
{
  if (sentido > 0) {
    for (uint8_t p = 1; p <= 3; p++)
      if (llamadas[p] && (PASOS_PISO[p] > pos || (inclusivo && PASOS_PISO[p] == pos))) return p;
  } else {
    for (uint8_t p = 3; p >= 1; p--)
      if (llamadas[p] && (PASOS_PISO[p] < pos || (inclusivo && PASOS_PISO[p] == pos))) return p;
  }
  return 0;
}

/* Piso mas cercano a la posicion actual (por pasos) */
static uint8_t pisoCercano(void)
{
  uint8_t mejor = 1;
  int32_t md = labs((long)(pos - PASOS_PISO[1]));
  for (uint8_t p = 2; p <= 3; p++) {
    int32_t d = labs((long)(pos - PASOS_PISO[p]));
    if (d < md) { md = d; mejor = p; }
  }
  return mejor;
}

/* Elige el proximo destino:
   1) Si una emergencia interrumpio un viaje, retoma ese destino.
   2) Si no, el piso solicitado mas CERCANO (medido en pisos).
   3) Empate (ej: en piso 2 con llamadas 1 y 3): gana el que siga el sentido
      de la ultima marcha (dir). */
static uint8_t elegirDestino(void)
{
  if (destinoGuardado) {
    if (llamadas[destinoGuardado]) return destinoGuardado;
    destinoGuardado = 0;                 /* ya no esta pendiente */
  }

  uint8_t ref = pisoCercano();
  uint8_t mejor = 0, mejorD = 255;

  for (uint8_t p = 1; p <= 3; p++) {
    if (!llamadas[p] || PASOS_PISO[p] == pos) continue;
    uint8_t d = (p > ref) ? (p - ref) : (ref - p);
    int8_t  sentidoP = (p > ref) ? 1 : -1;
    if (d < mejorD || (d == mejorD && sentidoP == dir)) {
      mejor = p;
      mejorD = d;
    }
  }
  return mejor;
}

/* Rampa: lento al salir y al llegar, rapido en medio */
static uint32_t periodoRampa(int32_t desde, int32_t hasta)
{
  int32_t hecho = labs((long)(pos - desde));
  int32_t falta = labs((long)(hasta - pos));
  int32_t m = (hecho < falta) ? hecho : falta;
  if (m > RAMP_PASOS) m = RAMP_PASOS;
  return T_PASO_US + (uint32_t)(RAMP_PASOS - m) * RAMP_INC_US;
}

static void prueba(void)
{
  if (emergencia) {
    if (estado != E_EMERG) {
      if (estado == E_MOVIENDO) destinoGuardado = destino;   /* recuerda el viaje */
      motor_apagar();
      estado = E_EMERG;
      /* las llamadas NO se borran: se conservan para reanudar */
    }
    return;
  }
  if (estado == E_EMERG) estado = E_ESPERA;    /* al liberar, vuelve a decidir */

  for (uint8_t i = 0; i < 3; i++) {
    if (flancoP[i]) {
      llamadas[i + 1] = 1;
      Serial.print("Llamada piso "); Serial.println(i + 1);
    }
  }

  switch (estado) {
  case E_ESPERA: {
    uint8_t aqui = pisoPorPos();
    if (aqui && llamadas[aqui]) {
      llamadas[aqui] = 0;
      lcdMensaje("Ya esta aqui", 1500);
      if (!puerta_abierta) {                       /* puerta cerrada: la abre */
        puerta_abrir();
        estado = E_ABRIENDO;
        tEstado = millis();
        break;
      }
    }
    uint8_t d = elegirDestino();
    if (!d) {
#if PUERTA_ABIERTA_EN_REPOSO
      if (aqui && !puerta_abierta) {               /* en reposo en un piso: puerta abierta */
        puerta_abrir();
        estado = E_ABRIENDO;
        tEstado = millis();
      }
#endif
      break;
    }
    if (puerta_abierta) {                          /* hay que moverse: primero cierra la puerta */
      puerta_cerrar();
      estado = E_CERRANDO;
      tEstado = millis();
      break;
    }
    destino = d;
    dir = (PASOS_PISO[d] > pos) ? 1 : -1;
    posInicio = pos;
    tPaso = micros();
    estado = E_MOVIENDO;
    destinoGuardado = 0;
    break;
  }

  case E_MOVIENDO: {
    uint8_t meta = buscar(dir, true);      /* una llamada intermedia se atiende primero */
    if (!meta) { motor_apagar(); estado = E_ESPERA; break; }
    destino = meta;
    if (pos == PASOS_PISO[meta]) {         /* llego por pasos: parar */
      motor_apagar();
      llamadas[meta] = 0;
      Serial.print("LLEGO piso "); Serial.print(meta);
      Serial.print(" | pos = "); Serial.println((long)pos);
      estado = E_LLEGANDO;                 /* 1 s despues abre la puerta */
      tEstado = millis();
      break;
    }
    uint32_t us = micros();
    if ((uint32_t)(us - tPaso) >= periodoRampa(posInicio, PASOS_PISO[meta])) {
      tPaso = us;
      if (motor_paso(dir)) pos += dir;
    }
    break;
  }

  case E_LLEGANDO:
    if ((millis() - tEstado) >= T_LLEGANDO_MS) {
      puerta_abrir();
      estado = E_ABRIENDO;
      tEstado = millis();
    }
    break;

  case E_ABRIENDO:
    if ((millis() - tEstado) >= T_SERVO_MS) {
#if PUERTA_ABIERTA_EN_REPOSO
      estado = E_ESPERA;                   /* se queda abierta en reposo */
#else
      estado = E_PUERTA;
      tEstado = millis();
#endif
    }
    break;

  case E_PUERTA: {                         /* solo si PUERTA_ABIERTA_EN_REPOSO = 0 */
    uint8_t aqui = pisoPorPos();
    if (aqui) llamadas[aqui] = 0;          /* ignora re-pulsaciones del mismo piso */
    if ((millis() - tEstado) >= T_PUERTA_ABIERTA_MS) {
      puerta_cerrar();
      estado = E_CERRANDO;
      tEstado = millis();
    }
    break;
  }

  case E_CERRANDO:
    if ((millis() - tEstado) >= T_SERVO_MS) estado = E_ESPERA;
    break;

  default: break;
  }
}

static void ui(void)
{
  static uint32_t t = 0;
  uint32_t ahora = millis();
  leds(estado == E_MOVIENDO, emergencia);
  if ((ahora - t) >= 100) {
    t = ahora;
    char a[24], b[24];
    uint8_t p = pisoMostrado();
    if (emergencia) {
      snprintf(a, sizeof(a), "MODO EMERGENCIA");
      snprintf(b, sizeof(b), "SISTEMA DETENIDO");
    } else {
#if MOSTRAR_PASOS
      snprintf(a, sizeof(a), "Piso %c P:%ld", p ? (char)('0' + p) : '-', (long)pos);
#else
      snprintf(a, sizeof(a), "Piso %c", p ? (char)('0' + p) : '-');
#endif
      if ((int32_t)(msgFin - ahora) > 0) {
        snprintf(b, sizeof(b), "%s", msgTmp);
      } else {
        switch (estado) {
          case E_MOVIENDO:
            if (labs((long)(PASOS_PISO[destino] - pos)) <= PASOS_LLEGANDO)
              snprintf(b, sizeof(b), "Llegando a P%d", destino);
            else if (dir > 0)
              snprintf(b, sizeof(b), "Subiendo \x01 a P%d", destino);
            else
              snprintf(b, sizeof(b), "Bajando \x02 a P%d", destino);
            break;
          case E_LLEGANDO: snprintf(b, sizeof(b), "Llegando a P%d", destino); break;
          case E_ABRIENDO: snprintf(b, sizeof(b), "Abriendo puerta");         break;
          case E_PUERTA:   snprintf(b, sizeof(b), "Puerta abierta");          break;
          case E_CERRANDO: snprintf(b, sizeof(b), "Cerrando puerta");         break;
          default:         snprintf(b, sizeof(b), "%s", puerta_abierta ? "Puerta abierta" : "En reposo"); break;
        }
      }
    }
    lcdPoner(0, a);
    lcdPoner(1, b);
  }
  lcdPoll();
}
#endif /* MODO_PRUEBA */

/* ================= SETUP / LOOP ================= */
void setup(void)
{
  Serial.begin(115200);
  for (uint8_t i = 0; i < 3; i++) pinMode(PIN_BTN[i], INPUT_PULLUP);
  pinMode(PIN_EMG, INPUT_PULLUP);
  for (uint8_t i = 0; i < 4; i++) { pinMode(PIN_MOTOR[i], OUTPUT); digitalWrite(PIN_MOTOR[i], LOW); }
  motor_apagar();

  for (uint8_t i = 0; i < 3; i++) pinMode(PIN_IR[i], INPUT_PULLUP);
  pinMode(PIN_LED_REPOSO, OUTPUT);
  pinMode(PIN_LED_MOV, OUTPUT);
  pinMode(PIN_LED_EMG, OUTPUT);
  leds(false, false);

  servoInit();                                   /* arranca con la puerta ABIERTA */

  attachInterrupt(digitalPinToInterrupt(PIN_EMG), emergenciaISR, FALLING);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);       /* emergencia: prioridad NVIC 0 */

  lcdInit();
  irIniciar();
  if (lcdOk) {
#if MODO_SERVO
    lcdPoner(0, "MODO SERVO");
    lcdPoner(1, "Calibrar puerta");
#elif MODO_PRUEBA
    lcdPoner(0, "ASCENSOR LISTO");
    lcdPoner(1, "Cabina piso 1");
#else
    lcdPoner(0, "MODO CALIBRACION");
    lcdPoner(1, "P1 baja P2 sube");
#endif
    for (uint16_t i = 0; i < 80; i++) lcdPoll();   /* dibuja todo el mensaje */
  }
  delay(1500);
}

void loop(void)
{
  botonesActualizar();
  irLeer();
  avisoEmergencia();
#if MODO_SERVO
  servoCal();
#elif MODO_PRUEBA
  prueba();
  ui();
#else
  calibracion();
  ui();
#endif
  botonesLimpiar();
}
