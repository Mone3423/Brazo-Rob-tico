#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Servo.h>
#include <math.h>
#include <IRremote.h>

// Configuración WiFi
const char* ssid = "ASMODEUS 6003";
const char* password = "polo4989";

// Definir los pines GPIO a los que conectar los servos
#define SERVO_PIN_1 2
#define SERVO_PIN_2 4
#define SERVO_PIN_3 5
#define SERVO_PIN_4 16
#define SERVO_PIN_5 17
#define SERVO_PIN_6 18

const float d1 = 8.5; // Altura de la base d1 en cm
const float l2 = 12;  // Longitud del eslabón 12 en cm
const float l3 = 12;  // Longitud del eslabón 13 en cm
const float l4 = 16;  // Longitud del eslabón 14 en cm

float XO, YO, ZO, Q;
float q1, q2, q3, q4;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;

AsyncWebServer server(80);

void setup() {
  Serial.begin(115200);

  // Inicializar WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a WiFi...");
  }
  Serial.println("Conectado a WiFi");
  Serial.println(WiFi.localIP());

  // Inicializar servomotores
  servo1.attach(SERVO_PIN_1);
  servo2.attach(SERVO_PIN_2);
  servo3.attach(SERVO_PIN_3);
  servo4.attach(SERVO_PIN_4);
  servo5.attach(SERVO_PIN_5);
  servo6.attach(SERVO_PIN_6);

  // Configurar rutas del servidor
  server.on("/move", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("type") && request->hasParam("value")) {
      String type = request->getParam("type")->value();
      String value = request->getParam("value")->value();
      String response = "";

      response += "Received type: " + type + "\n";
      response += "Received value: " + value + "\n";
      Serial.print("Received type: ");
      Serial.println(type);
      Serial.print("Received value: ");
      Serial.println(value);

      if (type == "coord") {
        // Parsear valor y asignar a XO, YO, ZO, Q
        int xIndex = value.indexOf("X=");
        int yIndex = value.indexOf("Y=");
        int zIndex = value.indexOf("Z=");
        int qIndex = value.indexOf("Q=");

        int firstSemicolon = value.indexOf(';', xIndex);
        int secondSemicolon = value.indexOf(';', yIndex);
        int thirdSemicolon = value.indexOf(';', zIndex);

        XO = value.substring(xIndex + 2, firstSemicolon).toFloat();
        YO = value.substring(yIndex + 2, secondSemicolon).toFloat();
        ZO = value.substring(zIndex + 2, thirdSemicolon).toFloat();
        Q = value.substring(qIndex + 2).toFloat();

        response += "XO: " + String(XO) + "\n";
        response += "YO: " + String(YO) + "\n";
        response += "ZO: " + String(ZO) + "\n";
        response += "Q: " + String(Q) + "\n";
        Serial.print("XO: ");
        Serial.println(XO);
        Serial.print("YO: ");
        Serial.println(YO);
        Serial.print("ZO: ");
        Serial.println(ZO);
        Serial.print("Q: ");
        Serial.println(Q);

        float Modulo = sqrt(pow(XO, 2) + pow(YO, 2)) + 1.4;

        // Cálculo de X1, Z1 y h
        float q = radians(Q); // El ángulo de cabeceo hay que pasarlo a radianes
        float lx = l4 * cos(q);
        float lz = l4 * sin(q);
        float X1 = Modulo - lx;
        float Z1 = ZO + lz - d1;
        float h = sqrt(pow(X1, 2) + pow(Z1, 2));

        // Cálculo de a1, a2 y a3
        float a1 = atan(Z1 / X1) * (180.0 / PI);
        float a2 = acos((pow(l2, 2) - pow(l3, 2) + pow(h, 2)) / (2 * l2 * h)) * (180.0 / PI);
        float a3 = acos((pow(l2, 2) + pow(l3, 2) - pow(h, 2)) / (2 * l2 * l3)) * (180.0 / PI);

        // Cálculo de los ángulos Q1, Q2, Q3 y Q4
        float Q1;
        if (XO < 0) {
          Q1 = abs(atan(XO / YO)) * (180.0 / PI) + 90;
        } else {
          Q1 = atan(YO / XO) * (180.0 / PI);
        }
        float Q2 = a1 + a2;
        float Q3 = -(180 - a3);
        float Q4 = Q - Q2 - Q3 - 90;

        response += "Ángulo Q1 calculado en grados: " + String(Q1) + "\n";
        response += "Ángulo Q2 calculado en grados: " + String(Q2) + "\n";
        response += "Ángulo Q3 calculado en grados: " + String(Q3) + "\n";
        response += "Ángulo Q4 calculado en grados: " + String(Q4) + "\n";
        Serial.print("Ángulo Q1 calculado en grados: ");
        Serial.println(Q1);
        Serial.print("Ángulo Q2 calculado en grados: ");
        Serial.println(Q2);
        Serial.print("Ángulo Q3 calculado en grados: ");
        Serial.println(Q3);
        Serial.print("Ángulo Q4 calculado en grados: ");
        Serial.println(Q4);

        q1 = Q1;
        q2 = Q2;
        q3 = Q3 + 142;
        q4 = abs((int)Q4 - 41);

        bool datosValidos = true;
        if (q1 < 0 || q1 > 180) {
          response += "Advertencia: Q1 fuera de rango: " + String(Q1) + "\n";
          Serial.print("Advertencia: Q1 fuera de rango: ");
          Serial.println(Q1);
          datosValidos = false;
        }
        if (q2 < 0 || q2 > 180) {
          response += "Advertencia: Q2 fuera de rango: " + String(Q2) + "\n";
          Serial.print("Advertencia: Q2 fuera de rango: ");
          Serial.println(Q2);
          datosValidos = false;
        }
        if (q3 < 0 || q3 > 180) {
          response += "Advertencia: Q3 fuera de rango: " + String(Q3) + "\n";
          Serial.print("Advertencia: Q3 fuera de rango: ");
          Serial.println(Q3);
          datosValidos = false;
        }
        if (q4 < 0 || q4 > 180) {
          response += "Advertencia: Q4 fuera de rango: " + String(Q4) + "\n";
          Serial.print("Advertencia: Q4 fuera de rango: ");
          Serial.println(Q4);
          datosValidos = false;
        }

        if (!datosValidos) {
          response += "Por favor, introduzca nuevamente las coordenadas.\n";
          Serial.println("Por favor, introduzca nuevamente las coordenadas.");
          request->send(400, "text/plain", response);
          return;
        }

        response += "Moviendo servos.\n";
        response += "Ángulo Servo1 en grados: " + String(q1) + "\n";
        response += "Ángulo Servo2 en grados: " + String(q2) + "\n";
        response += "Ángulo Servo3 en grados: " + String(q3) + "\n";
        response += "Ángulo Servo4 en grados: " + String(q4) + "\n";
        response += "Ángulo Servo5 en grados: 90\n";

        Serial.println("Moviendo servos.");
        Serial.print("Ángulo Servo1 en grados: ");
        Serial.println(q1);
        Serial.print("Ángulo Servo2 en grados: ");
        Serial.println(q2);
        Serial.print("Ángulo Servo3 en grados: ");
        Serial.println(q3);
        Serial.print("Ángulo Servo4 en grados: ");
        Serial.println(q4);
        Serial.print("Ángulo Servo5 en grados: ");
        Serial.println(90);

        moverServosSimultaneamente1(q1, q2, q3, q4);
        servo5.write(90);

        request->send(200, "text/plain", response);
      } else if (type == "grado") {
        int q1Index = value.indexOf("q1=");
        int q2Index = value.indexOf("q2=");
        int q3Index = value.indexOf("q3=");
        int q4Index = value.indexOf("q4=");
        int q5Index = value.indexOf("q5=");

        int firstSemicolon = value.indexOf(';', q1Index);
        int secondSemicolon = value.indexOf(';', q2Index);
        int thirdSemicolon = value.indexOf(';', q3Index);
        int fourthSemicolon = value.indexOf(';', q4Index);

        q1 = value.substring(q1Index + 3, firstSemicolon).toFloat();
        q2 = value.substring(q2Index + 3, secondSemicolon).toFloat();
        q3 = value.substring(q3Index + 3, thirdSemicolon).toFloat();
        q4 = value.substring(q4Index + 3, fourthSemicolon).toFloat();
        float q5 = value.substring(q5Index + 3).toFloat();

        response += "q1: " + String(q1) + "\n";
        response += "q2: " + String(q2) + "\n";
        response += "q3: " + String(q3) + "\n";
        response += "q4: " + String(q4) + "\n";
        response += "q5: " + String(q5) + "\n";
        Serial.print("q1: ");
        Serial.println(q1);
        Serial.print("q2: ");
        Serial.println(q2);
        Serial.print("q3: ");
        Serial.println(q3);
        Serial.print("q4: ");
        Serial.println(q4);
        Serial.print("q5: ");
        Serial.println(q5);

        moverServosSimultaneamente(q1, q2, q3, q4);
        servo5.write(q5);

        request->send(200, "text/plain", response);
      } else {
        request->send(400, "text/plain", "Faltan parámetros");
      }
    } else {
      request->send(400, "text/plain", "Faltan parámetros");
    }
  });

  server.on("/toggle_gripper", HTTP_GET, [](AsyncWebServerRequest *request){
    static bool gripperClosed = false;
    gripperClosed = !gripperClosed;
    servo6.write(gripperClosed ? 89 : 0); // Ajustado para abrir y cerrar la pinza

    String gripperState = gripperClosed ? "Cerrada" : "Abierta";
    String message = "Estado de la pinza: " + gripperState;

    Serial.println(message);
    request->send(200, "text/plain", message+ "\n");
  });

  server.begin();
}

void loop() {
  // No se requiere código en el loop
}

void moverServosSimultaneamente(float q1, float q2, float q3, float q4) {
  servo4.write(q4);
  servo1.write(q1);
  servo2.write(q2);
  servo3.write(q3);
}
void moverServosSimultaneamente1(int angulo1, int angulo2, int angulo3, int angulo4) {
  int anguloActual1 = servo1.read();
  int anguloActual2 = servo2.read();
  int anguloActual3 = servo3.read();
  int anguloActual4 = servo4.read();

  int delta1 = angulo1 - anguloActual1;
  int delta2 = angulo2 - anguloActual2;
  int delta3 = angulo3 - anguloActual3;
  int delta4 = angulo4 - anguloActual4;

  int pasos = max(max(abs(delta1), abs(delta2)), max(abs(delta3), abs(delta4)));
  if (pasos == 0) return; // No hay movimiento necesario

  float incremento1 = delta1 / float(pasos);
  float incremento2 = delta2 / float(pasos);
  float incremento3 = delta3 / float(pasos);
  float incremento4 = delta4 / float(pasos);

  for (int i = 0; i <= pasos; i++) {
    servo1.write(anguloActual1 + round(incremento1 * i));
    servo2.write(anguloActual2 + round(incremento2 * i));
    servo3.write(anguloActual3 + round(incremento3 * i));
    servo4.write(anguloActual4 + round(incremento4 * i));
    delay(15); // Ajustar el delay para suavizar el movimiento
    yield(); // Alimentar el watchdog timer
  }
}
void loop() {
  // No se requiere código en el loop
}

void parseAndMoveCoordinates(String value) {
  // Aquí debes parsear el valor y asignarlo a XO, YO, ZO, Q
  // Formato esperado: "X=<value>;Y=<value>;Z=<value>;Q=<value>"
  
  int xIndex = value.indexOf("X=");
  int yIndex = value.indexOf("Y=");
  int zIndex = value.indexOf("Z=");
  int qIndex = value.indexOf("Q=");

  // Encontrar los puntos y comas
  int firstSemicolon = value.indexOf(';', xIndex);
  int secondSemicolon = value.indexOf(';', yIndex);
  int thirdSemicolon = value.indexOf(';', zIndex);

  // Extraer y convertir los valores
  XO = value.substring(xIndex + 2, firstSemicolon).toFloat();
  YO = value.substring(yIndex + 2, secondSemicolon).toFloat();
  ZO = value.substring(zIndex + 2, thirdSemicolon).toFloat();
  Q = value.substring(qIndex + 2).toFloat();

  Serial.print("XO: ");
  Serial.println(XO);
  Serial.print("YO: ");
  Serial.println(YO);
  Serial.print("ZO: ");
  Serial.println(ZO);
  Serial.print("Q: ");
  Serial.println(Q);

  moverBrazo();
}


void parseAndMoveGrados(String value) {
  // Aquí debes parsear el valor y asignarlo a q1, q2, q3, q4, q5
  // Formato esperado: "q1=<value>;q2=<value>;q3=<value>;q4=<value>;q5=<value>"
  
  int q1Index = value.indexOf("q1=");
  int q2Index = value.indexOf("q2=");
  int q3Index = value.indexOf("q3=");
  int q4Index = value.indexOf("q4=");
  int q5Index = value.indexOf("q5=");

  // Encontrar los puntos y comas
  int firstSemicolon = value.indexOf(';', q1Index);
  int secondSemicolon = value.indexOf(';', q2Index);
  int thirdSemicolon = value.indexOf(';', q3Index);
  int fourthSemicolon = value.indexOf(';', q4Index);

  // Extraer y convertir los valores
  q1 = value.substring(q1Index + 3, firstSemicolon).toFloat();
  q2 = value.substring(q2Index + 3, secondSemicolon).toFloat();
  q3 = value.substring(q3Index + 3, thirdSemicolon).toFloat();
  q4 = value.substring(q4Index + 3, fourthSemicolon).toFloat();
  float q5 = value.substring(q5Index + 3).toFloat(); // q5 es siempre 90, lo usaremos directo en el servo

  Serial.print("q1: ");
  Serial.println(q1);
  Serial.print("q2: ");
  Serial.println(q2);
  Serial.print("q3: ");
  Serial.println(q3);
  Serial.print("q4: ");
  Serial.println(q4);
  Serial.print("q5: ");
  Serial.println(q5);

  moverServosSimultaneamente(q1, q2, q3, q4);
  servo5.write(q5); // q5
}


void moverBrazo() {
  // Cálculo del Módulo
  float Modulo = sqrt(pow(XO, 2) + pow(YO, 2)) + 1.4;

  // Cálculo de X1, Z1 y h
  float q = radians(Q); // El ángulo de cabeceo hay que pasarlo a radianes
  float lx = l4 * cos(q);
  float lz = l4 * sin(q);
  float X1 = Modulo - lx;
  float Z1 = ZO + lz - d1;
  float h = sqrt(pow(X1, 2) + pow(Z1, 2));

  // Cálculo de a1, a2 y a3
  float a1 = atan(Z1 / X1) * (180.0 / PI);
  float a2 = acos((pow(l2, 2) - pow(l3, 2) + pow(h, 2)) / (2 * l2 * h)) * (180.0 / PI);
  float a3 = acos((pow(l2, 2) + pow(l3, 2) - pow(h, 2)) / (2 * l2 * l3)) * (180.0 / PI);

  // Cálculo de los ángulos Q1, Q2, Q3 y Q4
  float Q1;
  if (XO < 0) {
    Q1 = abs(atan(XO / YO)) * (180.0 / PI) + 90;
  } else {
    Q1 = atan(YO / XO) * (180.0 / PI);
  }
  float Q2 = a1 + a2;
  float Q3 = -(180 - a3);
  float Q4 = Q - Q2 - Q3 - 90;

  // Imprimir los ángulos calculados
  Serial.print("Ángulo Q1 calculado en grados: ");
  Serial.println(Q1);
  Serial.print("Ángulo Q2 calculado en grados: ");
  Serial.println(Q2);
  Serial.print("Ángulo Q3 calculado en grados: ");
  Serial.println(Q3);
  Serial.print("Ángulo Q4 calculado en grados: ");
  Serial.println(Q4);

  // Calcular los ángulos de los servomotores
  q1 = Q1;
  q2 = Q2;
  q3 = Q3 + 142;
  q4 = abs((int)Q4 - 41); // Conversión a int para abs

  // Verificar si los ángulos están dentro del rango permitido
  bool datosValidos = true;
  if (q1 < 0 || q1 > 180) {
    Serial.print("Advertencia: Q1 fuera de rango: ");
    Serial.println(Q1);
    datosValidos = false;
  }
  if (q2 < 0 || q2 > 180) {
    Serial.print("Advertencia: Q2 fuera de rango: ");
    Serial.println(Q2);
    datosValidos = false;
  }
  if (q3 < 0 || q3 > 180) {
    Serial.print("Advertencia: Q3 fuera de rango: ");
    Serial.println(Q3);
    datosValidos = false;
  }
  if (q4 < 0 || q4 > 180) {
    Serial.print("Advertencia: Q4 fuera de rango: ");
    Serial.println(Q4);
    datosValidos = false;
  }

  if (!datosValidos) {
    Serial.println("Por favor, introduzca nuevamente las coordenadas.");
    return;
  }

  // Imprimir los ángulos de los servos
  Serial.print("Ángulo Servo1 en grados: ");
  Serial.println(q1);
  Serial.print("Ángulo Servo2 en grados: ");
  Serial.println(q2);
  Serial.print("Ángulo Servo3 en grados: ");
  Serial.println(q3);
  Serial.print("Ángulo Servo4 en grados: ");
  Serial.println(q4);
  Serial.print("Ángulo Servo5 en grados: ");
  Serial.println(90);

  moverServosSimultaneamente(q1, q2, q3, q4);
  servo5.write(90); // Servo 5 siempre a 90 grados
}

void moverServosSimultaneamente(float q1, float q2, float q3, float q4) {
  servo1.write(q1);
  servo2.write(q2);
  servo3.write(q3);
  servo4.write(q4);
}

void toggleGripper() {
  static bool gripperClosed = false;
  gripperClosed = !gripperClosed;
  servo6.write(gripperClosed ? 89 : 0); // Ajustado para abrir y cerrar la pinza
  Serial.print("Estado de la pinza: ");
  Serial.println(gripperClosed ? "Cerrada" : "Abierta");
}