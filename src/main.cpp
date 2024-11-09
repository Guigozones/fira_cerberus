#include <Arduino.h>
#include <PID_v1.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <HardwareSerial.h>

    

#define ENA 5 // ENA PWM Motor Esquerdo
#define ENB 6 // ENB PWM Motor Direito

#define IN1 7 // DIR Motor Esquerdo
#define IN2 7 // DIR Motor Esquerdo

#define IN3 8 // DIR Motor Direito
#define IN4 8 // DIR Motor Direito

#define xshutPinsE 12
#define xshutPinsC 9
#define xshutPinsD 4

// SoftwareSerial BTSerial(A0, A1); // RX, TX

VL53L1X sensorE;
VL53L1X sensorC;
VL53L1X sensorD;

// Variáveis Globais
float tamanho_carrinho = 13.5;
float tamanho_pista;

float distanciaE;
double distanciaC;
float distanciaD;

unsigned long time;
/*Parâmetros para ajustar*/

// Robocore - fonte

//-----------------------------------------------

// Valor máximo 255 para potência total
// float VEL_MAX = 90;

/* Ajuste de alinhamento em reta */

float MAX_DELTA = 40;

float MAX_VOLTAGE = 100;    // em voltagem
float MIN_PERCENT_DIR = 35; // em porcentagem
float MIN_PERCENT_ESQ = 50;

double delta;

double DIS_MAX = 7.1;
double DIS_MIN = 0.1;

// PID PARA O SENSOR CENTRAL ((DES)ACELERAÇÃO)
double OutputC;
double kp = 0.2;
double ki = 0.2;
double kd = 0.005;
double valor_porcentagemD;
double valor_porcentagemE;

void ler_sensores()
{
  distanciaE = (sensorE.read() - 20) / 10;
  if (distanciaE > 400)
  {
    distanciaE = (sensorE.read() - 20) / 10;
  }
  sensorE.timeoutOccurred() ? distanciaE = 400 : distanciaE = distanciaE;

  distanciaD = (sensorD.read() - 20) / 10;
  sensorD.timeoutOccurred() ? distanciaD = 400 : distanciaD = distanciaD;

  distanciaC = (sensorC.read()) / 10;
  sensorC.timeoutOccurred() ? distanciaC = 400 : distanciaC = distanciaC;

  delta = distanciaE - distanciaD;
}

// Robocore - Fonte

// Robocore - Fonte

void imprimeDistancias()
{
  Serial.print("Dis Esq: ");
  Serial.print(distanciaE);
  Serial.print(" mm  /  ");
  Serial.print("Dis Cen: ");
  Serial.print(distanciaC);
  Serial.print(" mm   /  ");
  Serial.print("Dis Dir: ");
  Serial.print(distanciaD);
  Serial.print(" mm     /   ");
}

float tratamento(float vel)
{
  vel = min(vel, 100);
  vel = max(vel, 0);
  vel = (vel)*MAX_VOLTAGE / 100;
  return vel;
}

void acelera(float vel_esquerda, float vel_direita, int ativa = 0)
{
  // int vel_direita_int = ceil(tratamento((vel_direita* OutputC / MAX_VOLTAGE), MIN_VOLTAGE_DIR));
  // int vel_esquerda_int = ceil(tratamento((vel_esquerda* OutputC / MAX_VOLTAGE), MIN_VOLTAGE_ESQ));

  int vel_direita_int = round(tratamento((vel_direita)));
  int vel_esquerda_int = round(tratamento((vel_esquerda)));

  analogWrite(ENA, vel_esquerda_int);

  analogWrite(ENB, vel_direita_int);
}

void back()
{
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, MAX_VOLTAGE);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, MAX_VOLTAGE);
}

void parar()
{
  acelera(0, 0);
}
void ajuste3(float delta)
{
  int i = 0;
  int limite = 30;
  int limite_b = 20;
  while (delta > 0 && distanciaE < tamanho_pista && i < limite_b)
  {
    ler_sensores();
    delta = distanciaE - distanciaD;
    acelera(75, 50 + i);
    i += 2;
  }
  while (delta < 0 && distanciaD < tamanho_pista && i < limite)
  {
    ler_sensores();
    delta = distanciaE - distanciaD;
    acelera(70 + i, 55 - i / 2);
    i += 2;
  }
}

void frente(int *vector = NULL)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);
}

void ajuste2(float delta)
{

  // mais a esquerda
  frente();

  if (delta < 0)
  {
    if (delta < -tamanho_pista * 0.3)
    {
      acelera(85, 58);
      delay(250);
      parar();
      delay(1000);
    }
    if (delta > -tamanho_pista * 0.3 && delta < -tamanho_pista * 0.15)
    {
      acelera(80, 58);
      delay(250);
      parar();
      delay(1000);
    }
    else if (delta > -tamanho_pista * 0.15)
    {
      acelera(75, 58);
      delay(250);
      parar();
      delay(1000);
    }
  }

  // mais a direita
  if (delta > 0)
  {
    if (delta > tamanho_pista * 0.3)
    {
      acelera(65, 90);
      delay(250);
      parar();
      delay(1000);
    }
    if (delta < tamanho_pista * 0.3 && delta > tamanho_pista * 0.15)
    {
      acelera(65, 85);
      delay(250);
      parar();
      delay(1000);
    }
    else if (delta < tamanho_pista * 0.15)
    {
      acelera(65, 75);
      delay(250);
      parar();
      delay(150);
    }
  }
}

void acompanha_parede()
{
  if (distanciaD > 15)
  {
    acelera(0, 80);
    delay(150);
    acelera(95, 0);
    delay(250);
    parar();
    delay(150);
  }
  else if (distanciaD >= 10)
  {
    acelera(95, 60);
    delay(200);
    parar();
    delay(100);
    ler_sensores();
    if (distanciaC > 4)
    {
      acelera(95, 70);
      delay(50);
      parar();
      delay(100);
    }
    else
    {
      back();
      acelera(80, 60);
      delay(150);
      parar();
      delay(200);
      frente();
    }
  }
  else if (distanciaD >= 7)
  {
    acelera(95, 65);
    delay(175);
    parar();
    delay(100);
  }
  else if (distanciaD <= 3)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN3, LOW);
    acelera(95, 95);
    delay(100);
    parar();
    delay(150);
    frente();
  }
  else
  {
    acelera(70, 90);
    delay(200);
    parar();
    delay(100);
  }
}


void sentido_esquerdo()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, LOW);
}

void sentido_direito()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, HIGH);
}

void setup()
{
  while (!Serial)
  {
  }

  Serial.begin(9600); // Comunicação Serial com o Computador
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); // definição dos pinos entradas e saidas
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); // OUTPUT = Saída
  pinMode(IN4, OUTPUT); // INPUT = Entrada

  Wire.begin();
  Wire.setClock(400000);

  pinMode(xshutPinsD, OUTPUT);
  digitalWrite(xshutPinsD, LOW);

  pinMode(xshutPinsC, OUTPUT);
  digitalWrite(xshutPinsC, LOW);

  pinMode(xshutPinsE, OUTPUT);
  digitalWrite(xshutPinsE, LOW);
  pinMode(A3, OUTPUT);
  pinMode(A2, INPUT);
  digitalWrite(A3, HIGH);

  pinMode(xshutPinsE, INPUT);
  delay(10);

  sensorE.setTimeout(500);
  if (!sensorE.init())
  {
    Serial.print("Failed to detect and initialize sensor ");
    Serial.println("E");
    while (1)
      ;
  }

  sensorE.setAddress(0x2A);

  sensorE.startContinuous(50);

  pinMode(xshutPinsD, INPUT);
  delay(10);

  sensorD.setTimeout(500);
  if (!sensorD.init())
  {
    Serial.print("Failed to detect and initialize sensor ");
    Serial.println("D");
    while (1)
      ;
  }

  sensorD.setAddress(0x2A + 1);

  sensorD.startContinuous(50);

  pinMode(xshutPinsC, INPUT);
  delay(10);

  sensorC.setTimeout(500);
  if (!sensorC.init())
  {
    Serial.print("Failed to detect and initialize sensor ");
    Serial.println("C");
    while (1)
      ;
  }

  sensorC.setAddress(0x2A + 2);

  sensorC.startContinuous(50);

  delay(2000);
  acelera(75, 75);
  delay(350);
  ler_sensores();
  imprimeDistancias();
  tamanho_pista = distanciaD + distanciaE + tamanho_carrinho;
  if (tamanho_pista > 100)
  {
    acelera(0, 0);
    delay(100);
    ler_sensores();
    tamanho_pista = distanciaD + distanciaE + tamanho_carrinho;
  }

  Serial.print("Tamanho da pista: ");
  Serial.println(tamanho_pista);
  DIS_MAX = (distanciaD + distanciaE) / 2;

  time = millis();
  // last_time = time;
  // acelera(90,90);
  // delay(500);
  // tamanho_pista = distanciaD + distanciaE + tamanho_carrinho;
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  ler_sensores();
  imprimeDistancias();

  ler_sensores();

  if (distanciaC > 6)
  {
    acompanha_parede();
  }
  else if (distanciaE > 15)
  {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN3, LOW);
    while (distanciaC < 15)
    {
      ler_sensores();
      acelera(95, 95);
      delay(100);
      parar();
      delay(75);
    }
    parar();
    delay(250);
    frente();
  }
  // else if (distanciaD < ((tamanho_pista - tamanho_carrinho) / 2) + 5 && distanciaC < 6 && distanciaE < ((tamanho_pista - tamanho_carrinho) / 2) + 5)
  else if (distanciaD < ((tamanho_pista - tamanho_carrinho) / 2) +10  && distanciaE < ((tamanho_pista - tamanho_carrinho) / 2) + 10)
  {
    acelera(0, 0);
    digitalWrite(LED_BUILTIN, HIGH);
    // while(1);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN3, LOW);
    while (distanciaE < 15)
    {
      ler_sensores();
      acelera(95, 95);
      delay(100);

      parar();
      delay(100);
    }
    frente();
    parar();
    delay(75);
  }
  else
  {
    back();
    acelera(95, 50);
    delay(150);
    acelera(70, 95);
    delay(75);
    parar();
    delay(150);

    frente();
  }
}