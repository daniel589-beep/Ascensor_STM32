#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

#define MODO_SERVO         0
#define MODO_PRUEBA        1
#define USAR_SERIAL        0

#define SENTIDO_SUBIDA     1
#define T_PASO_US          2000UL
#define RAMP_PASOS         150
#define RAMP_INC_US        10UL
#define LIMITE_PASOS       120000L
#define PASOS_POR_VUELTA   4096.0f
#define DEB_MS             30
#define LARGO_MS           2000
#define PASOS_LLEGANDO     3000

static const int32_t PASOS_PISO[4] = {0, 0, 21500, 41525};

#define SERVO_ABIERTO_US   1500
#define SERVO_CERRADO_US   660
#define SERVO_CON_LIBRERIA 1
#define T_SERVO_MS         600
#define T_LLEGANDO_MS      1000
#define T_PUERTA_ABIERTA_MS 3000

#define PUERTA_ABIERTA_EN_REPOSO 1

#define SENSOR_ACTIVO      LOW
#define IR_FILTRO_MS       20

#define MOSTRAR_PASOS      0

static const uint32_t PIN_BTN[3] = {PA1, PA2, PA3};
#define PIN_EMG            PA4
static const uint32_t PIN_MOTOR[4] = {PB12, PB13, PB14, PB15};

#define COIL_PORT          GPIOB
#define COIL_SHIFT         12

#define PIN_SERVO          PA8
static const uint32_t PIN_IR[3] = {PA5, PA6, PA7};
#define PIN_LED_REPOSO     PB8
#define PIN_LED_MOV        PB9
#define PIN_LED_EMG        PB10

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

static uint8_t lcdAddr = 0x27;
static bool    lcdOk = false;
static char    lcdDes[32];
static char    lcdAct[32];
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
  lcdEnviar(0x28, 0); delay(1);
  lcdEnviar(0x0C, 0); delay(1);
  lcdEnviar(0x06, 0); delay(1);
  lcdEnviar(0x01, 0); delay(3);

  lcdEnviar(0x48, 0);
  for (uint8_t i = 0; i < 8; i++) lcdEnviar(flechaArriba[i], 1);
  for (uint8_t i = 0; i < 8; i++) lcdEnviar(flechaAbajo[i], 1);
  lcdEnviar(0x80, 0);

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

static void lcdMensaje(const char *t, uint32_t ms)
{
  strncpy(msgTmp, t, 16);
  msgTmp[16] = 0;
  msgFin = millis() + ms;
}

static bool puerta_abierta = true;

#if SERVO_CON_LIBRERIA
static Servo servoPuerta;

static void servoInit(void)
{
  servoPuerta.attach(PIN_SERVO, 500, 2500);
  servoPuerta.writeMicroseconds(SERVO_ABIERTO_US);
}

static void servoUs(uint16_t us)
{
  servoPuerta.writeMicroseconds(us);
}
#else
static HardwareTimer *servoTim = NULL;

static void servoInit(void)
{
  servoTim = new HardwareTimer(TIM1);
  servoTim->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PIN_SERVO);
  servoTim->setOverflow(20000, MICROSEC_FORMAT);
  servoTim->setCaptureCompare(1, SERVO_ABIERTO_US, MICROSEC_COMPARE_FORMAT);
  servoTim->resume();
}

static void servoUs(uint16_t us)
{
  servoTim->setCaptureCompare(1, us, MICROSEC_COMPARE_FORMAT);
}
#endif

static void puerta_abrir(void)  { servoUs(SERVO_ABIERTO_US);  puerta_abierta = true;  }
static void puerta_cerrar(void) { servoUs(SERVO_CERRADO_US);  puerta_abierta = false; }

static uint8_t  irCrudo[4], irEst[4];
static uint32_t irT[4];
static uint8_t  pisoSensor = 0;

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

static const uint8_t SEQ[8] = {0x1, 0x3, 0x2, 0x6, 0x4, 0xC, 0x8, 0x9};
static uint8_t fase = 0;

volatile uint8_t  emergencia = 0;
volatile uint32_t tEmg = 0;

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

static bool motor_paso(int8_t d)
{
  if (emergencia) return false;
  int8_t f = d * SENTIDO_SUBIDA;
  fase = (f > 0) ? ((fase + 1) & 7) : ((fase + 7) & 7);
  return coils(SEQ[fase]);
}

void emergenciaISR(void)
{
  motor_apagar();
  uint32_t ahora = millis();
  if ((ahora - tEmg) >= 250UL) {
    tEmg = ahora;
    emergencia ^= 1;
  }
}

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

static int32_t pos = 0;
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
#endif

#if !MODO_PRUEBA

static int32_t marcas[4];
static uint8_t siguiente = 1;
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
#endif

#if MODO_PRUEBA

enum Estado { E_ESPERA, E_MOVIENDO, E_EMERG, E_LLEGANDO, E_ABRIENDO, E_PUERTA, E_CERRANDO };
static Estado estado = E_ESPERA;
static uint8_t llamadas[4] = {0, 0, 0, 0};
static int8_t  dir = 1;
static uint8_t destino = 0;
static int32_t posInicio = 0;
static uint32_t tEstado = 0;

static uint8_t pisoPorPos(void)
{
  for (uint8_t p = 1; p <= 3; p++) if (pos == PASOS_PISO[p]) return p;
  return 0;
}

static uint8_t pisoMostrado(void)
{
  return pisoSensor ? pisoSensor : pisoPorPos();
}

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
      motor_apagar();
      estado = E_EMERG;
      llamadas[1] = llamadas[2] = llamadas[3] = 0;
    }
    return;
  }
  if (estado == E_EMERG) estado = E_ESPERA;

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
      if (!puerta_abierta) {
        puerta_abrir();
        estado = E_ABRIENDO;
        tEstado = millis();
        break;
      }
    }
    uint8_t d = buscar(dir, false);
    if (!d) d = buscar(-dir, false);
    if (!d) {
#if PUERTA_ABIERTA_EN_REPOSO
      if (aqui && !puerta_abierta) {
        puerta_abrir();
        estado = E_ABRIENDO;
        tEstado = millis();
      }
#endif
      break;
    }
    if (puerta_abierta) {
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
    break;
  }

  case E_MOVIENDO: {
    uint8_t meta = buscar(dir, true);
    if (!meta) { motor_apagar(); estado = E_ESPERA; break; }
    destino = meta;
    if (pos == PASOS_PISO[meta]) {
      motor_apagar();
      llamadas[meta] = 0;
      Serial.print("LLEGO piso "); Serial.print(meta);
      Serial.print(" | pos = "); Serial.println((long)pos);
      estado = E_LLEGANDO;
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
      estado = E_ESPERA;
#else
      estado = E_PUERTA;
      tEstado = millis();
#endif
    }
    break;

  case E_PUERTA: {
    uint8_t aqui = pisoPorPos();
    if (aqui) llamadas[aqui] = 0;
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
#endif

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

  servoInit();

#if !MODO_SERVO
  delay(800);
  puerta_cerrar();  delay(1200);
  puerta_abrir();   delay(1200);
#endif

  attachInterrupt(digitalPinToInterrupt(PIN_EMG), emergenciaISR, FALLING);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);

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
    for (uint16_t i = 0; i < 80; i++) lcdPoll();
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
