#include <Arduino.h>
#include <PID_v1.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <HardwareSerial.h>

int rotacao_RPM_sensor2();
int rotacao_RPM_sensor1();

#define PINO_CH1 3
#define PINO_CH2 2
#define PINO_CH3 10
#define PINO_CH4 11

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
int soma_esquerda = 0;
int soma_direita = 0;
float tamanho_carrinho = 13.5;
float tamanho_pista;

float distanciaE;
double distanciaC;
float distanciaD;

// Velocidade em rpm da roda esquerda e direita
double velocidadeD = 0;
double velocidadeE = 0;

unsigned long time;
/*Parâmetros para ajustar*/

// Robocore - fonte
// Declaracao das variaveis auxiliares para a verificacao do sentido
int estado_sensor1;
int ultimo_estado_sensor1;
boolean sentido_sensor1;

int estado_sensor2;
int ultimo_estado_sensor2;
boolean sentido_sensor2;

// Declaracao das variaveis auxiliares para o calculo da velocidade
unsigned long contador1_sensor1;
unsigned long contador2_sensor1;

unsigned long contador1_sensor2;
unsigned long contador2_sensor2;

const int NUMERO_CONTADORES = 2;
const int NUMERO_LEITURAS = 2;
// Variavel de numero de dentes do disco de leitura
const int NUMERO_DENTES = 10; // Altere se necessario

// Declaracao das variaveis auxiliares para a temporizacao de um minuto
unsigned long tempo_antes_sensor1 = 0;
unsigned long tempo_antes_sensor2 = 0;
const long MINUTO = 1000;
// Robocore - fonte

// Declaracao das variaveis auxiliares para a temporizacao de um minuto
unsigned long tempo_antes_direita_sensor1 = 0;
unsigned long tempo_antes_esquerda_sensor1 = 0;

unsigned long tempo_antes_direita_sensor2 = 0;
unsigned long tempo_antes_esquerda_sensor2 = 0;

long tempo_atual = 100;
long tempo_anterior = 0;

//-----------------------------------------------

// Valor máximo 255 para potência total
// float VEL_MAX = 90;

/* Ajuste de alinhamento em reta */

float MAX_DELTA = 40;

float MAX_VOLTAGE = 100; // em voltagem
float MIN_PERCENT_DIR = 35; // em porcentagem
float MIN_PERCENT_ESQ = 50;

double delta;

double DIS_MAX = 7.1;
double DIS_MIN = 0.1;

// PID PARA O SENSOR CENTRAL ((DES)ACELERAÇÃO)
double OutputC;
double velocidadeSetpoint_D = velocidadeE;
double velocidadeSetpoint_E = velocidadeD;
double kp = 0.2;
double ki = 0.2;
double kd = 0.005;
double valor_porcentagemD;
double valor_porcentagemE;

PID PIDRodaD(&velocidadeD, &valor_porcentagemD, &velocidadeSetpoint_D, kp, ki, kd, DIRECT);
PID PIDRodaE(&velocidadeE, &valor_porcentagemE, &velocidadeSetpoint_E, kp, ki, kd, DIRECT);

void ler_sensores()
{
  distanciaE = (sensorE.read() - 20)/10;
    if(distanciaE > 400){
      distanciaE = (sensorE.read() - 20)/10;
    }
  sensorE.timeoutOccurred() ? distanciaE = 400 : distanciaE = distanciaE;

  distanciaD = (sensorD.read() - 20)/10;
  sensorD.timeoutOccurred() ? distanciaD = 400 : distanciaD = distanciaD;

  distanciaC = (sensorC.read())/10;
  sensorC.timeoutOccurred() ? distanciaC = 400 : distanciaC = distanciaC;

  delta = distanciaE - distanciaD;

  velocidadeE = rotacao_RPM_sensor2();
  velocidadeD = rotacao_RPM_sensor1();
  velocidadeSetpoint_D = velocidadeE;
  velocidadeSetpoint_E = velocidadeD;
}

// Robocore - Fonte

// Funcao de interrupcao
void contador_pulso2_sensor1()
{

  // Incrementa o contador
  contador2_sensor1++;

  // Verifica o sentido de rotacao do motor
  estado_sensor1 = digitalRead(PINO_CH2);
  if (ultimo_estado_sensor1 == LOW && estado_sensor1 == HIGH)
  {
    if (digitalRead(PINO_CH1) == LOW)
    {
      sentido_sensor1 = true;
    }
    else
    {
      sentido_sensor1 = false;
    }
  }
  ultimo_estado_sensor1 = estado_sensor1;
}

// Funcao de interrupcao
void contador_pulso1_sensor1()
{

  // Incrementa o contador
  contador1_sensor1++;
}

void contador_pulso2_sensor2()
{

  // Incrementa o contador
  contador2_sensor2++;

  // Verifica o sentido de rotacao do motor
  estado_sensor1 = digitalRead(PINO_CH4);
  if (ultimo_estado_sensor2 == LOW && estado_sensor2 == HIGH)
  {
    if (digitalRead(PINO_CH3) == LOW)
    {
      sentido_sensor2 = true;
    }
    else
    {
      sentido_sensor2 = false;
    }
  }
  ultimo_estado_sensor2 = estado_sensor2;
}

// Funcao de interrupcao
void contador_pulso1_sensor2()
{

  // Incrementa o contador
  contador1_sensor2++;
}

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
  Serial.print("Vel Esq: ");
  Serial.print(velocidadeE);
  Serial.print("rpm   /    ");
  Serial.print("Vel Dir: ");
  Serial.print(velocidadeD);
  Serial.println("rpm   /    ");
}

float tratamento(float vel)
{
  vel = min(vel, 100);
  vel = max(vel, 0);
  vel = (vel)*MAX_VOLTAGE / 100;
  return vel;
}

int ajuste_rpm(int velocidade_melhor_real = velocidadeD, int velocidade_pior_real = velocidadeE);

void acelera(float vel_esquerda, float vel_direita, int ativa = 0)
{
  // int vel_direita_int = ceil(tratamento((vel_direita* OutputC / MAX_VOLTAGE), MIN_VOLTAGE_DIR));
  // int vel_esquerda_int = ceil(tratamento((vel_esquerda* OutputC / MAX_VOLTAGE), MIN_VOLTAGE_ESQ));

  int vel_direita_int = round(tratamento((vel_direita)));
  int vel_esquerda_int = round(tratamento((vel_esquerda)));

  if (ativa && distanciaD > 6.5 && distanciaE > 6.5 && millis() - tempo_atual >= 1)
  {
    // while (1)
    // {
    //   acelera(0,0);
    //   digitalWrite(LED_BUILTIN, LOW);
    //   delay(100);
    //   digitalWrite(LED_BUILTIN, HIGH);
    //   delay(100);
    // }
    
    if(velocidadeD > velocidadeE)
    { 
        if (abs(vel_esquerda_int - MIN_PERCENT_ESQ) >= abs(vel_esquerda_int - MAX_VOLTAGE))
        {
          soma_esquerda = 0;
          soma_direita--;
        }
        else
        {
        soma_direita = 0;
          soma_esquerda++;
        }
      
      vel_direita_int += soma_direita;
      vel_esquerda_int += soma_esquerda;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
        if (abs(vel_direita_int - MIN_PERCENT_DIR) >= abs(vel_direita_int - MAX_VOLTAGE))
        {
          soma_direita = 0;
          soma_esquerda--;
        }
        else
        {
          soma_esquerda = 0;
          soma_direita++;
        }
        vel_direita_int += soma_direita;
        vel_esquerda_int += soma_esquerda;

        digitalWrite(LED_BUILTIN, LOW);
    }
    vel_direita_int = max(vel_direita_int, MIN_PERCENT_DIR);
    vel_esquerda_int = max(vel_esquerda_int, MIN_PERCENT_ESQ);
    vel_direita_int = min(vel_direita_int, MAX_VOLTAGE);
    vel_esquerda_int = min(vel_esquerda_int, MAX_VOLTAGE);

    Serial.print("Soma Esq: ");
    Serial.print(soma_esquerda);
    Serial.print("      Soma Dir: ");
    Serial.print(soma_direita);
    Serial.print("      Vel Esq");
    Serial.print(vel_esquerda_int);
    Serial.print("      Vel Dir");
    Serial.println(vel_direita_int);
    tempo_atual = millis();
  }

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

double output_func_math(int choice, float MIN_PERCENT, double referencia,
                        double referencia_2 = 0)
{
  double c;
  double a;
  switch (choice)
  {
  // cosseno
  case 0:
    return cos(referencia * acos(MIN_PERCENT / MAX_VOLTAGE) / DIS_MAX) *
           MAX_VOLTAGE; 

  // quadratica
  case 1:
    c = MAX_VOLTAGE;
    a = (MIN_PERCENT - MAX_VOLTAGE) / DIS_MAX * DIS_MAX;
    return referencia * referencia * a + c;

  // linear
  case 2:
    // valores iniciais a = -50.0 e c = 15.5
    a = -70;
    c = 16;
    return ceil((a / c) * referencia + 100.0);

  // 3 variaveis, varia em relacao ao tempo
  case 3:
    c = MAX_VOLTAGE;
    a = (MIN_PERCENT - MAX_VOLTAGE) / DIS_MAX * DIS_MAX;
    if (a < 1)
    {
      return referencia * referencia * a * referencia_2 + c;
    }
    else
    {
      return referencia * referencia * a / referencia_2 + c;
    }
  //retorna o valor da roda direita a partir do valor da roda esquerda, dentr
  case 4:
    return 0.001333091 * referencia*referencia +0.474922601 * referencia;
  // erro
  default:
    return -1;
  }
}

void parar()
{
  acelera(0,0);
}
void ajuste3(float delta){
  int i = 0;
  int limite = 30;
  int limite_b = 20;
    while(delta > 0 && distanciaE < tamanho_pista && i < limite_b){
      ler_sensores();
      delta = distanciaE - distanciaD;
      acelera(75 , 50 + i);
      i += 2;
      
      }
      while(delta < 0 && distanciaD < tamanho_pista && i < limite){
        ler_sensores();
        delta = distanciaE - distanciaD;
        acelera(70+i, 55-i/2);
        i+= 2;
        
      }
  }

  void frente(int *vector = NULL)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);
}

void ajuste2(float delta){
  // if(velocidadeD ==0 && velocidadeE ==0){
  //   acelera(90,90);{
  //     delay(75);
  //   }
  // }
  //mais a esquerda
  frente();
  // if(velocidadeD == 0 && velocidadeE == 0){
  //   acelera(80, 80);
  //   delay(25);
  // }
  if(delta < 0){
    if(delta < -tamanho_pista * 0.3){
      acelera(85, 58);
      delay(250);
      parar();
      delay(1000);
    }if(delta> -tamanho_pista *0.3 && delta < -tamanho_pista * 0.15){
      acelera(80, 58);
      delay(250);
      parar();
      delay(1000);
    }else if(delta > -tamanho_pista * 0.15){
      acelera(75, 58);
      delay(250);
      parar();
      delay(1000);
    }
  }

  //mais a direita
  if (delta > 0)
  {
    if(delta > tamanho_pista * 0.3){
      acelera(65, 90);
      delay(250);
      parar();
      delay(1000);
    }if(delta< tamanho_pista *0.3 && delta > tamanho_pista * 0.15){
      acelera(65, 85);
      delay(250);
      parar();
      delay(1000);
    }else if(delta < tamanho_pista * 0.15){
      acelera(65, 75);
      delay(250);
      parar();
      delay(150);
    }
  }
}

void acompanha_parede(){
  if (distanciaD > tamanho_pista){
    acelera(0, 80);
    delay(150);
    acelera(100, 0);
    delay(250);
    parar();
    delay(150);
  }else if(distanciaD >= 10){
    acelera(100, 60);
    delay(150);
    parar();
    delay(150);
    ler_sensores();
    if(distanciaC> 6){
      acelera(100, 70);
      delay(75);
      parar();
      delay(150);
    }else{
      back();
      acelera(80, 60);
      delay(350);
      parar();
      delay(200);
      frente();
    }
  }
  else if(distanciaD >= 7){
    acelera(100, 65);
    delay(150);
    parar();
    delay(150);
  }else if(distanciaD <= 2){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN3, LOW);
    acelera(80, 80);
    delay(50);
    parar();
    delay(150);
    frente();
  }
  else{
    acelera(70, 90);
    delay(150);
    parar();
    delay(150);
  }
}
void ajuste(float delta)
{
  // variável que calcula o quanto uma roda deverá diminuir para ajustar o
  // carrinho
  double valor_acelera;

  // CONTROLE PID

  // float valor_acelera =
  //  mais a direita
  if (delta > 0)
  {
    //Serial.println("Delta: ");
    //SErial.println(delta);
    valor_acelera = output_func_math(0, MIN_PERCENT_ESQ, abs(delta));
    valor_acelera = min(valor_acelera, 100);
    valor_acelera = max(valor_acelera, 0);

    digitalWrite(LED_BUILTIN, HIGH);
    // time_here_right += (millis() - last_time);
    // time_here_left = 0;
    //  digitalWrite(led_amarelo_esquerda, HIGH);
    //  digitalWrite(led_azul_direita, LOW);
  }

  // mais a esquerda
  else if (delta < 0)
  {
    valor_acelera = output_func_math(0, MIN_PERCENT_DIR, abs(delta));
    valor_acelera = min(100, valor_acelera);
    valor_acelera = max(100, valor_acelera);

    digitalWrite(LED_BUILTIN, LOW);

    // time_here_right += (millis() - last_time);
    // time_here_left = 0;
    //  digitalWrite(led_amarelo_esquerda, HIGH);
    //  digitalWrite(led_azul_direita, LOW);
  }
  else
  {
    // time_here_left = 0;
    // time_here_right = 0;
    //  digitalWrite(led_amarelo_esquerda, LOW);
    //  digitalWrite(led_azul_direita, LOW);
    acelera(70, 60);
  }
  // last_time = millis();
}

void ajuste4(float delta)
{
  double valor_acelera = 60;
  if(velocidadeE < 1 && velocidadeD < 1 ) acelera(80, 70);
  else{
  //mais a aesquerda
    if (delta > 0)
    {
    //Serial.println("Delta: ");
    //SErial.println(delta);
      for(int i = valor_acelera; delta > 0; i = i + 4){
        acelera(70 , valor_acelera + i);
        delta = distanciaE - distanciaD;
      }
    }

  // mais a direita
    else if (delta < 0)
    {
      for(int i = valor_acelera; delta > 0; i = i + 5){
        acelera(valor_acelera + i, 70);
        delta = distanciaE - distanciaD;
      }
    }
    else
    {
      acelera(70, 60);
      return;
    }
  }
}

int rotacao_RPM_sensor1()
{
  // Verifica a contagem de tempo e exibe as informacoes coletadas do motor
  // if ((millis() - tempo_antes) > MINUTO)
  // { // A cada minuto

  // Verifica a variavel "sentido"
  //  if (sentido) { //Se ela for verdadeira ("true")
  //    Serial.print("Sentido: Horario");
  //    Serial.print("       |  ");
  //  } else { //Se ela for falsa ("false")
  //    Serial.print("Sentido: Anti-Horario");
  //    Serial.print("  |  ");
  //  }

  // Calcula a velocidade e exibe no monitor
  float media = (contador1_sensor1 + contador2_sensor1) / (NUMERO_CONTADORES); // Calcula a media dos contadores
  float velocidade = media / (NUMERO_DENTES * NUMERO_LEITURAS);                // Calcula a velocidade de acordo com o numero de dentes do disco
  velocidade *= 60;
  // Serial.print("Velocidade: ");
  // Serial.print(velocidade);
  // Serial.println(" RPM");

  // Zera os contadores e reinicia a contagem de tempo.
  contador1_sensor1 = 0;
  contador2_sensor1 = 0;

  return round(velocidade);
  // tempo_antes = millis();
  // }
}

int rotacao_RPM_sensor2()
{
  // Verifica a contagem de tempo e exibe as informacoes coletadas do motor
  // if ((millis() - tempo_antes) > MINUTO)
  // { // A cada minuto

  // Verifica a variavel "sentido"
  //  if (sentido) { //Se ela for verdadeira ("true")
  //    Serial.print("Sentido: Horario");
  //    Serial.print("       |  ");
  //  } else { //Se ela for falsa ("false")
  //    Serial.print("Sentido: Anti-Horario");
  //    Serial.print("  |  ");
  //  }

  // Calcula a velocidade e exibe no monitor
  float media = (contador1_sensor2 + contador2_sensor2) / (NUMERO_CONTADORES); // Calcula a media dos contadores
  float velocidade = media / (NUMERO_DENTES * NUMERO_LEITURAS);                // Calcula a velocidade de acordo com o numero de dentes do disco
  velocidade *= 60;
  // Serial.print("Velocidade: ");
  // Serial.print(velocidade);
  // Serial.println(" RPM");

  // Zera os contadores e reinicia a contagem de tempo.
  contador1_sensor2 = 0;
  contador2_sensor2 = 0;

  return round(velocidade);
  // tempo_antes = millis();
  // }
}

// posição 0 esquerda, 1 centro, 2 direita
int *livre()
{
  int *vector = new int[3];
  // 0 não está livre
  //  1 está livre
  distanciaE/10 < tamanho_pista * 1.5 ? vector[0] = 0 : vector[0] = 1;

  distanciaC/10 <= (tamanho_pista - tamanho_carrinho) / 2 ? vector[1] = 0 : vector[1] = 1;

  distanciaD/10 < tamanho_pista * 1.5 ? vector[2] = 0 : vector[2] = 1;

  return vector;
}

// melhor roda
double rpm_direta(float x)
{
  return 5.2822e-9 * pow(x, 6) -
         8.30698e-7 * pow(x, 5) -
         3.02739e-5 * pow(x, 4) +
         0.009833676 * pow(x, 3) -
         0.372287095 * pow(x, 2) +
         3.496110355 * x;
}

double rpm_esquerdo(float x)
{
  return -6.39077e-9 * pow(x, 6) +
         2.67143e-6 * pow(x, 5) -
         0.000410143 * pow(x, 4) +
         0.027383752 * pow(x, 3) -
         0.6778225 * pow(x, 2) +
         5.256018147 * x -
         3.550308947;
}



int ajuste_rpm(int velocidade_melhor_real, int velocidade_pior_real)
{

  int diferenca_real = abs(velocidade_pior_real - velocidade_melhor_real);
  int menor_diferenca = 0;

  for (int i = 0; i <= 100; i++)
  {
    if (abs(rpm_esquerdo(menor_diferenca) - velocidade_melhor_real) > abs(rpm_esquerdo(i) - velocidade_melhor_real))
    {
      menor_diferenca = i;
    }
  }

  return abs(rpm_esquerdo(menor_diferenca) - velocidade_melhor_real) < diferenca_real ? menor_diferenca : -1;
}
void sentido_esquerdo(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, LOW);
}

void sentido_direito(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, HIGH);
}

void virar_direita(int *vector = NULL)
{
  acelera(70,50);
  delay(200);
  parar();
  delay(500);
  int distancia_desejada = distanciaD;
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, HIGH);
  
  acelera(70,70);
  
  delay(500);
  
  while(distanciaC < tamanho_pista*1.5){
    digitalWrite(IN1, LOW);
    digitalWrite(IN3, HIGH);
    parar();
    delay(100);
    ler_sensores();
    imprimeDistancias();
    acelera(90, 90);
    delay(75);
    if (velocidadeD == 0 && velocidadeE == 0 && (distanciaC < 5 || distanciaE < 5 || distanciaD < 5))
    {
      back();
      delay(100);
      frente();
      digitalWrite(LED_BUILTIN, HIGH);
    }

    if(velocidadeD != 0 || velocidadeE != 0)
    {
      tempo_anterior = millis();
    }
    else if (millis() - tempo_anterior >= 5000)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      back();
      delay( 100);
      frente();
      acelera(80,50);
      delay(100);
    }
    ler_sensores();
    
    
  }
  frente();
  acelera(70, 60);
  delay(800);
  ler_sensores();
  if(distanciaE < tamanho_pista/2 && distanciaD < tamanho_pista/2 && distanciaC > 10){
    ajuste2(delta);
  }else if(distanciaE < tamanho_pista/2 || distanciaD < tamanho_pista/2){
    if(distanciaE < tamanho_pista/2){
      frente();
      acelera(80, 50);
      delay(100);
    }else{
      frente();
      acelera(60, 65);
    }
  }else{
    frente();
    parar();
    delay(100);
  }
  
}

void virar_esquerda(int *vector = NULL)
{
  acelera(70,60);
  delay(100);
  int distancia_desejada = distanciaE;
  parar();
  delay(500);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, LOW);
  acelera(70,70);
  delay(500);
  
  while(distanciaC < tamanho_pista * 2){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN3, LOW);
    parar();
    delay(50);
    ler_sensores();
    imprimeDistancias();
    acelera(80, 80);
    delay(150);
    
    if (velocidadeD == 0 && velocidadeE == 0 && (distanciaC < 5 || distanciaE < 5 || distanciaD < 51))
    {
      back();
      delay(100);
      frente();
    }

    if(velocidadeD != 0 || velocidadeE != 0)
    {
      tempo_anterior = millis();
    }
    else if (millis() - tempo_anterior >= 1000)
    {
      back();
      delay(100);
      
      frente();
      acelera(80,50);
      delay(100);
    }
    ler_sensores();
  }
  frente();
  acelera(75, 55);
  delay(800);

}

void virar_direita2(){
  if(distanciaC> 10){
    acelera(70, 50);
    delay(200);
  }
  parar();
  delay(1000);
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, HIGH);
  acelera(65,65);
  delay(150);
  frente();
  acelera(70, 50);
  delay(200);
  while(distanciaD > tamanho_pista/2|| distanciaE > tamanho_pista/2){
    ler_sensores();
    acelera(90,72);
    delay(100);
    parar();
    delay(100);
    ler_sensores();
  }
}
void virar_esquerda2(){
  if(distanciaC> 10){
    acelera(70, 50);
    delay(500);
  }
  parar();
  delay(1000);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, LOW);
  acelera(90,90);
  delay(500);
  frente();
  while(distanciaD > tamanho_pista|| distanciaE > tamanho_pista){
    ler_sensores();
    acelera(90,72);
    delay(100);
    parar();
    delay(100);
    ler_sensores();
  }
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

  attachInterrupt(digitalPinToInterrupt(PINO_CH1), contador_pulso1_sensor1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PINO_CH2), contador_pulso2_sensor1, CHANGE);

  attachPCINT(digitalPinToPCINT(PINO_CH3), contador_pulso1_sensor2, CHANGE);
  attachPCINT(digitalPinToPCINT(PINO_CH4), contador_pulso2_sensor2, CHANGE);

  pinMode(xshutPinsD, OUTPUT);
  digitalWrite(xshutPinsD, LOW);

  pinMode(xshutPinsC, OUTPUT);
  digitalWrite(xshutPinsC, LOW);

  pinMode(xshutPinsE, OUTPUT);
  digitalWrite(xshutPinsE, LOW);
  pinMode(A3, OUTPUT);
  pinMode(A2, INPUT);
  digitalWrite(A3, HIGH);

  PIDRodaD.SetSampleTime(10);
  PIDRodaD.SetMode(AUTOMATIC);
  PIDRodaD.SetTunings(kp, ki, kd);

  PIDRodaD.SetOutputLimits(MIN_PERCENT_DIR, 100);

  PIDRodaE.SetSampleTime(10);
  PIDRodaE.SetMode(AUTOMATIC);
  PIDRodaE.SetTunings(kp, ki, kd);

  PIDRodaE.SetOutputLimits(MIN_PERCENT_ESQ, 100);

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
    acelera(0,0);
    delay(100);
    ler_sensores();
    tamanho_pista = distanciaD + distanciaE + tamanho_carrinho;
  }
  
  Serial.print("Tamanho da pista: ");
  Serial.println(tamanho_pista);
  DIS_MAX = (distanciaD + distanciaE) /2;
  
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
  float delta = distanciaE - distanciaD;
  //tamanho_pista = distanciaE+
  int delta_velocidade = velocidadeE - velocidadeD;

  // if(distanciaD > tamanho_pista || distanciaE > tamanho_pista){
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   if(delta < 0){ 
  //     virar_direita();
  //     frente();
  //     acelera(70,70);
  //     delay(500);
  //   }else{
  //     virar_esquerda();
  //     frente();
  //     acelera(70,70);
  //     delay(500);
  //   }
  // }else 


  // if(abs(delta) > tamanho_pista){
  //   if(delta > 0){
  //     acelera(80, 65);
  //     delay(600);
  //     virar_esquerda();
  //     frente();
  //     acelera(80, 65);
  //     delay(900);
  //   }else{
  //     acelera(80, 65);
  //     delay(600);
  //     virar_direita();
  //     frente();
  //     acelera(80, 65);
  //     delay(900);
  //   }
  // }else
  //  if(distanciaC > 7){
  //   digitalWrite(LED_BUILTIN, LOW);
  //   ajuste3(delta);
  // }
  // else
  // {
  //   acelera(0,0);
  //   for (int i = 0; i < 10; i++)
  //   {
  //     digitalWrite(LED_BUILTIN, LOW);
  //     delay(100);
  //     digitalWrite(LED_BUILTIN, HIGH);
  //     delay(100);
  //   }
  // }
  // if (velocidadeD == 0 && velocidadeE == 0 && (distanciaC < 5 || distanciaE < 5 || distanciaD < 5))
  //   {
  //     back();
  //     delay(100);
  //     frente();
  //   }

  //   if(velocidadeD != 0 || velocidadeE != 0)
  //   {
  //     tempo_anterior = millis();
  //   }
  //   else if (millis() - tempo_anterior >= 10000)
  //   {
  //     back();
  //     delay(100);
  //     frente();
  //     acelera(80,50);
  //     delay(100);
  //   }
    ler_sensores();
  // if(distanciaD > tamanho_pista || distanciaE > tamanho_pista){
  //   if(delta < 0){
  //     virar_direita();
  //     ler_sensores();
  //     if(distanciaC > 10){
  //       acelera(70, 60);
  //       delay(150);
  //     }
  //   }else if(delta >= 0){
  //     virar_esquerda();
  //     ler_sensores();
  //     if(distanciaC > 10){
  //       acelera(70, 60);
  //       delay(150);
  //     }
  //   }
  // }else if(distanciaC > 9){
  //   ajuste2(delta);
  // }
  // else{
  //   back();
  //   delay(150);
  //   frente();
  // }

  //estavamos usando essa no dia anterior ao evento
// if(distanciaD > tamanho_pista){
//   virar_direita2();
  
// }else if (distanciaD < tamanho_pista && distanciaE < tamanho_pista && distanciaC > 3)
// { 
//   frente();
//   ajuste2(delta);
  
// }else if(distanciaE > tamanho_pista){
//   virar_esquerda2();
// }

  if(distanciaC > 6){
    acompanha_parede();
  }else if (distanciaE > tamanho_pista){
    
      // back();
      // acelera(100, 75);
      // delay(75);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN3, LOW);
      while(distanciaC< 15){
        ler_sensores();
        acelera(100, 100);
        delay(100);
        parar();
        delay(75);
        
      }
      parar();
      delay(250);
      frente();
      // acelera(70, 90);
      // delay(200);
      // parar();
      // delay(150);
    
  }else if(distanciaD < ((tamanho_pista-tamanho_carrinho)/2)+5&& distanciaC < 6 && distanciaE < ((tamanho_pista-tamanho_carrinho)/2)+5 ){
    acelera(0, 0);
    digitalWrite(LED_BUILTIN, HIGH);
    // while(1);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN3, LOW);
    while(distanciaE < 15){
      ler_sensores();
      acelera(80, 80);
      delay(75);
      
      parar();
      delay(75);
      
    }
    frente();
    parar();
    delay(75);
    
  }else{
    back();
    acelera(100, 50);
    delay(150);
    acelera(70, 100);
    delay(75);
    parar();
    delay(150);
    

    frente();
  }
  

  // PIDRodaD.Compute();
  // PIDRodaE.Compute();
  // acelera(valor_porcentagemE, 60);

  
  // }else if(distanciaD > tamanho_pista * 1.2){
  //   virar_direita();
  // }else if(distanciaE > tamanho_pista * 1.2){
  //   virar_esquerda();
  // }

  // int *vector = livre();
  // imprimeDistancias();
  // if (distanciaD > 15){
  //   frente(vector);
  //   delay(500);
  //   virar_direita(vector);
    
  // }else if(distanciaE < 15 && distanciaC < 10 && distanciaD < 15) {
  //   virar_direita(vector);
  // }else if(distanciaC > 10){
  //     if(distanciaD > distanciaE){
  //     acelera(60, 80);
  //   }else{
  //     acelera(80, 60);
  //   }
  // }else if(distanciaE > 15){
  //   virar_esquerda(vector); 
  // }
  // if(vector[1])
  // {
  //   frente(vector);
  //   if (!vector[2] && !vector[0])
  //   {
  //     ajuste(delta);
  //   }

  // }
  // else
  // {
  //   if(vector[0])
  //   {
  //     virar_esquerda(vector);
  //   }
  //   else
  //   {
  //     virar_direita(vector);
  //   }
  // }

  // int vector[3] = {0, 0, 1};
  // virar_esquerda(vector);
  // while (true)
  // {
  //   acelera(0,0);
  // }

  // capturar valores das rodas
  //  while (true)
  //  {
  //    unsigned long tempo_limite = 1000 * 32;

  //   for (int i = 0; i <= 100; i += 5)
  //   {

  //     acelera(100, 100);
  //     delay(100);

  //     acelera(i,i);
  //     delay(100);

      // unsigned long tempo_atual = millis();
  //     while (millis() - tempo_atual < tempo_limite)
  //     {
  //       /* code */

  //       ler_sensores();
  //       Serial.print("i: ");
  //       Serial.println(i);
  //       Serial.print("Velocidade Esquerda: ");
  //       Serial.println(rotacao_RPM_sensor2());
  //       Serial.print("Velocidade Direita: ");
  //       Serial.println(rotacao_RPM_sensor1());
  //     }
  //     //  delay(1000);
  //   }

  //   while (true)
  //   {
  //     acelera(0, 0);
  //   }
  // }
}