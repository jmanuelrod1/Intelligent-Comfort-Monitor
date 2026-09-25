/*
  =====================================================================
  HIDRASPORT  -  ESP32-S3
  =====================================================================
  Sistema de monitoreo ambiental para espacios deportivos.

  Mide temperatura, humedad e iluminacion, y a partir de esos datos
  calcula un porcentaje de confort ambiental, una recomendacion de
  hidratacion, un nivel de alerta (semaforo) y una accion recomendada.

  Espacio: 1 = Gimnasio | 2 = Cancha Sintetica | 3 = Skatepark

  Salidas:
    Temp_C          -> temperatura reportada (C)
    Vaso_Agua_ml    -> ml de agua recomendados cada 20 min
    Confort_Pct     -> % de confort ambiental (0-100)
    Alerta_Codigo   -> 1=Verde  2=Amarillo  3=Rojo
    Accion_ID       -> 1..4 segun tasa de sudor estimada
  =====================================================================
*/

#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>

// =====================================================================
//  CONFIGURACION
// =====================================================================

#define PIN_SDA            8
#define PIN_SCL            9
#define PIN_TOUCH          4
#define TOUCH_ACTIVO_ALTO  true

#define INTERVALO_LECTURA  1000
#define INTERVALO_REPORTE  10000
#define MS_TOQUE_LARGO     1500

#define ALFA_EMA           0.25    // filtro de media movil exponencial

#define MODO_SIMULACION    false   // true = ruido gaussiano sintetico, sin sensores
#define SIGMA_T            0.30
#define SIGMA_H            2.00
#define SIGMA_L            8.00

#define MODO_CSV           false

// =====================================================================
//  ESTADO: ESPACIO  (1=Gimnasio 2=CanchaSintetica 3=Skatepark)
// =====================================================================

int espacio = 1;

const char* nombreEspacio(int e) {
  switch (e) {
    case 1: return "Gimnasio";
    case 2: return "Cancha Sintetica";
    case 3: return "Skatepark";
    default: return "Desconocido";
  }
}

// =====================================================================
//  OBJETOS Y VARIABLES DE SENSOR
// =====================================================================

Adafruit_AHTX0  aht;
Adafruit_BMP280 bmp;
BH1750          luxometro(0x23);
bool okAHT = false, okBMP = false, okBH = false;

float T_crudo = NAN, H_crudo = NAN, L_crudo = NAN, Pr_crudo = NAN;
float T = NAN, H = NAN, L = NAN, Pr = NAN;
bool  emaIniciado = false;

// Salidas del algoritmo
float Temp_C        = 0;
int   Vaso_Agua_ml  = 0;
float Confort_Pct   = 0;
int   Alerta_Codigo = 1;
int   Accion_ID     = 1;
float Tasa_Sudor    = 0;   // se deja visible porque es la base de Accion_ID

unsigned long tLectura = 0, tReporte = 0, tInicioToque = 0;
bool estadoTouchPrev = false, toqueLargoHecho = false;

// =====================================================================
//  SETUP
// =====================================================================

void setup() {
  Serial.begin(115200);
  delay(2500);
  pinMode(PIN_TOUCH, INPUT);

  Serial.println();
  Serial.println(F("############################################"));
  Serial.println(F("#  HIDRASPORT  -  ESP32-S3                  #"));
  Serial.println(F("#  Monitoreo de espacios deportivos          #"));
  Serial.println(F("############################################"));
  Serial.println();

  Wire.begin(PIN_SDA, PIN_SCL, 100000);
  delay(100);

  if (MODO_SIMULACION) {
    Serial.println(F(">> MODO SIMULACION: datos sinteticos + ruido gaussiano"));
    randomSeed(esp_random());
  } else {
    iniciarSensores();
  }

  if (MODO_CSV) {
    Serial.println(F("ms,espacio,T,H,L,Temp_C,Vaso_Agua_ml,Confort_Pct,Alerta_Codigo,Accion_ID"));
  } else {
    Serial.print(F("\nEspacio activo: "));
    Serial.println(nombreEspacio(espacio));
    Serial.println(F("Toque corto = reporte  |  Toque largo = cambiar espacio\n"));
  }

  muestrear();
  ejecutarAlgoritmo();
  imprimirReporte();
  tReporte = millis();
}

// =====================================================================
//  LOOP
// =====================================================================

void loop() {
  unsigned long ahora = millis();

  if (ahora - tLectura >= INTERVALO_LECTURA) {
    tLectura = ahora;
    muestrear();
    ejecutarAlgoritmo();
  }

  gestionarTouch();

  if (ahora - tReporte >= INTERVALO_REPORTE) {
    tReporte = ahora;
    if (MODO_CSV) imprimirCSV();
    else          imprimirReporte();
  }
}

// =====================================================================
//  SENSORES
// =====================================================================

void iniciarSensores() {
  okAHT = aht.begin(&Wire);
  Serial.print(F("AHT20  ... "));  Serial.println(okAHT ? F("OK") : F("FALLO"));

  okBMP = bmp.begin(0x77);
  if (!okBMP) okBMP = bmp.begin(0x76);
  Serial.print(F("BMP280 ... "));  Serial.println(okBMP ? F("OK") : F("FALLO"));
  if (okBMP) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }

  okBH = luxometro.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  Serial.print(F("BH1750 ... "));  Serial.println(okBH ? F("OK") : F("FALLO"));
  Serial.println();
}

float ruidoGauss(float sigma) {
  float u1 = (random(1, 10000)) / 10000.0f;
  float u2 = (random(0, 10000)) / 10000.0f;
  return sigma * sqrt(-2.0f * log(u1)) * cos(2.0f * PI * u2);
}

void muestrear() {
  if (MODO_SIMULACION) {
    float t = millis() / 1000.0f;
    T_crudo  = 22.0f + 4.0f * sin(t / 40.0f)  + ruidoGauss(SIGMA_T);
    H_crudo  = 45.0f + 8.0f * sin(t / 55.0f)  + ruidoGauss(SIGMA_H);
    L_crudo  = 350.0f + 150.0f * sin(t / 30.0f) + ruidoGauss(SIGMA_L);
    Pr_crudo = 750.0f + ruidoGauss(0.5f);
    if (H_crudo < 0) H_crudo = 0;
    if (L_crudo < 0) L_crudo = 0;
  } else {
    if (okAHT) {
      sensors_event_t eh, et;
      aht.getEvent(&eh, &et);
      T_crudo = et.temperature;
      H_crudo = eh.relative_humidity;
    }
    if (okBMP) {
      Pr_crudo = bmp.readPressure() / 100.0f;
      if (!okAHT) T_crudo = bmp.readTemperature();
    }
    if (okBH) {
      float l = luxometro.readLightLevel();
      if (l >= 0) L_crudo = l;
    }
  }

  // Filtro de media movil exponencial: y[k] = alfa*x[k] + (1-alfa)*y[k-1]
  if (!emaIniciado) {
    T = T_crudo;  H = H_crudo;  L = L_crudo;  Pr = Pr_crudo;
    emaIniciado = true;
  } else {
    if (!isnan(T_crudo))  T  = ALFA_EMA * T_crudo  + (1 - ALFA_EMA) * T;
    if (!isnan(H_crudo))  H  = ALFA_EMA * H_crudo  + (1 - ALFA_EMA) * H;
    if (!isnan(L_crudo))  L  = ALFA_EMA * L_crudo  + (1 - ALFA_EMA) * L;
    if (!isnan(Pr_crudo)) Pr = ALFA_EMA * Pr_crudo + (1 - ALFA_EMA) * Pr;
  }
}

// =====================================================================
//  ALGORITMO DE DECISION
// =====================================================================

void ejecutarAlgoritmo() {

  // ---- 1. Salida directa de temperatura ----
  Temp_C = T;

  // ---- 2. Factores segun el tipo de espacio ----
  float Factor_Actividad, Radiacion_Extra;
  switch (espacio) {
    case 1:  // Gimnasio Cerrado
      Factor_Actividad = 0.85f;
      Radiacion_Extra  = 0.0f;
      break;
    case 2:  // Cancha Sintetica (calor radiante del caucho)
      Factor_Actividad = 1.25f;
      Radiacion_Extra  = (L > 500) ? ((L - 500) * 0.005f) : 0.0f;
      break;
    case 3:  // Skatepark (radiacion reflejada en concreto)
      Factor_Actividad = 1.05f;
      Radiacion_Extra  = (L > 300) ? ((L - 300) * 0.004f) : 0.0f;
      break;
    default:
      Factor_Actividad = 0.85f;
      Radiacion_Extra  = 0.0f;
      break;
  }

  float T_efectiva = T + (0.05f * H) + Radiacion_Extra;

  // ---- 3. Confort ambiental global ----
  float Penalizacion = fabs(T_efectiva - 21.0f) * 3.5f + fabs(H - 50.0f) * 0.4f;
  Confort_Pct = 100.0f - Penalizacion;
  if (Confort_Pct < 0)   Confort_Pct = 0;
  if (Confort_Pct > 100) Confort_Pct = 100;

  // ---- 4. Semaforo multivariable ----
  bool es_temp_opt  = (T > 18.0f) && (T <= 24.0f);
  bool es_hum_opt   = (H > 40.0f) && (H <= 60.0f);
  bool es_lux_opt   = (L <= 600.0f);

  bool es_temp_crit = (T > 30.0f);
  bool es_hum_crit  = (H > 75.0f);
  bool es_lux_crit  = (L > 900.0f);

  if (es_temp_opt && es_hum_opt && es_lux_opt) {
    Alerta_Codigo = 1;   // VERDE
  } else if (es_temp_crit || es_hum_crit || es_lux_crit) {
    Alerta_Codigo = 3;   // ROJO
  } else {
    Alerta_Codigo = 2;   // AMARILLO
  }

  // ---- 5. Tasa de sudor y rehidratacion ----
  float exceso = T_efectiva - 20.0f;
  if (exceso < 0) exceso = 0;
  Tasa_Sudor = 0.4f + exceso * 0.04f * Factor_Actividad;
  Vaso_Agua_ml = (int)round((Tasa_Sudor * 1000.0f * 0.80f) / 3.0f);

  // ---- 6. Accion recomendada segun tasa de sudor ----
  if      (Tasa_Sudor < 0.7f) Accion_ID = 1;
  else if (Tasa_Sudor < 1.2f) Accion_ID = 2;
  else if (Tasa_Sudor < 1.8f) Accion_ID = 3;
  else                        Accion_ID = 4;
}

const char* textoAlerta(int codigo) {
  switch (codigo) {
    case 1: return "VERDE - condiciones ideales";
    case 2: return "AMARILLO - precaucion";
    case 3: return "ROJO - al menos una variable en nivel critico";
    default: return "-";
  }
}

const char* textoAccion(int id) {
  switch (id) {
    case 1: return "Condicion Top: apto para records y maxima potencia.";
    case 2: return "Perdiendo grip: aplica magnesio en manos o seca suelas.";
    case 3: return "Ropa saturada: cambia de camiseta y consume suero/electrolitos.";
    case 4: return "Riesgo de calambre: reduce la carga un 30% e ir a la sombra.";
    default: return "-";
  }
}

// =====================================================================
//  IMPRESION
// =====================================================================

void imprimirReporte() {
  Serial.println();
  Serial.println(F("=================================================="));
  Serial.print  (F("  ESPACIO "));
  Serial.print  (espacio);
  Serial.print  (F(" - "));
  Serial.println(nombreEspacio(espacio));
  Serial.print  (F("  t = "));
  Serial.print  (millis() / 1000);
  Serial.println(F(" s"));
  Serial.println(F("=================================================="));

  Serial.println(F("  ENTRADAS (crudo -> filtrado)"));
  Serial.print(F("    T ... ")); Serial.print(T_crudo, 2); Serial.print(F("  ->  ")); Serial.print(T, 2); Serial.println(F(" C"));
  Serial.print(F("    H ... ")); Serial.print(H_crudo, 2); Serial.print(F("  ->  ")); Serial.print(H, 2); Serial.println(F(" %HR"));
  Serial.print(F("    L ... ")); Serial.print(L_crudo, 1); Serial.print(F("  ->  ")); Serial.print(L, 1); Serial.println(F(" lx"));

  Serial.println(F("--------------------------------------------------"));
  Serial.println(F("  SALIDAS DEL ALGORITMO"));
  Serial.print(F("    Temp_C ......... ")); Serial.print(Temp_C, 2); Serial.println(F(" C"));
  Serial.print(F("    Vaso_Agua_ml ... ")); Serial.print(Vaso_Agua_ml); Serial.println(F(" ml / 20 min"));
  Serial.print(F("    Confort_Pct .... ")); Serial.print(Confort_Pct, 2); Serial.println(F(" %"));
  Serial.print(F("    Alerta_Codigo .. ")); Serial.print(Alerta_Codigo); Serial.print(F("  -> ")); Serial.println(textoAlerta(Alerta_Codigo));
  Serial.print(F("    Accion_ID ...... ")); Serial.print(Accion_ID); Serial.print(F("  -> ")); Serial.println(textoAccion(Accion_ID));
  Serial.print(F("    (Tasa_Sudor .... ")); Serial.print(Tasa_Sudor, 2); Serial.println(F(" L/h, referencia)"));

  Serial.println(F("=================================================="));
  Serial.println();
}

void imprimirCSV() {
  Serial.print(millis());        Serial.print(',');
  Serial.print(espacio);         Serial.print(',');
  Serial.print(T, 3);            Serial.print(',');
  Serial.print(H, 3);            Serial.print(',');
  Serial.print(L, 1);            Serial.print(',');
  Serial.print(Temp_C, 2);       Serial.print(',');
  Serial.print(Vaso_Agua_ml);    Serial.print(',');
  Serial.print(Confort_Pct, 2);  Serial.print(',');
  Serial.print(Alerta_Codigo);   Serial.print(',');
  Serial.println(Accion_ID);
}

// =====================================================================
//  TOUCH
// =====================================================================

void gestionarTouch() {
  bool crudo  = digitalRead(PIN_TOUCH);
  bool tocado = TOUCH_ACTIVO_ALTO ? crudo : !crudo;

  if (tocado && !estadoTouchPrev) {
    tInicioToque = millis();
    toqueLargoHecho = false;
  }

  if (tocado && estadoTouchPrev && !toqueLargoHecho &&
      (millis() - tInicioToque >= MS_TOQUE_LARGO)) {
    toqueLargoHecho = true;
    espacio = (espacio % 3) + 1;    // 1 -> 2 -> 3 -> 1
    Serial.println();
    Serial.print(F(">>> ESPACIO CAMBIADO A "));
    Serial.print(espacio);
    Serial.print(F(" - "));
    Serial.println(nombreEspacio(espacio));
    Serial.println();
    ejecutarAlgoritmo();
  }

  if (!tocado && estadoTouchPrev) {
    unsigned long dur = millis() - tInicioToque;
    if (!toqueLargoHecho && dur > 50) {
      muestrear();
      ejecutarAlgoritmo();
      if (MODO_CSV) imprimirCSV();
      else          imprimirReporte();
      tReporte = millis();
    }
  }

  estadoTouchPrev = tocado;
}
