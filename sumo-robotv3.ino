/*
 * Projeto: Controle de Motores com Controle PS4 via ESP32
 * Autor: Victor Mielitz
 * Data da última atualização: 06 de novembro de 2024
 *
 * Descrição:
 * Este código foi desenvolvido para controlar dois motores DC (esquerdo e direito) 
 * utilizando um controle PS4 via Bluetooth, conectado a um microcontrolador ESP32. 
 * O código utiliza a biblioteca PS4Controller para gerenciar a comunicação com o controle PS4
 * e a biblioteca para controle PWM dos motores.
 *
 * O controle do movimento é feito através dos analógicos do controle PS4:
 * - O eixo Y do analógico esquerdo controla a aceleração/redução de velocidade (throttle).
 * - O eixo X do analógico direito controla a direção (steering).
 *
 * Funções:
 * - Quando o controle PS4 é conectado, os motores são inicializados e a comunicação é estabelecida.
 * - O valor dos analógicos do controle PS4 é mapeado para controlar a velocidade e direção dos motores.
 * - Os motores podem girar para frente, para trás ou parar, dependendo dos valores de entrada do controle PS4.
 *
 * Dependências:
 * - Biblioteca PS4Controller (https://github.com/meren/PS4-esp32)
 *
 * Obs.: Para o funcionamento correto do código, é necessário configurar o endereço MAC do controle PS4
 * no comando PS4.begin("MAC_ADDRESS_AQUI");.
 */
 
#include <PS4Controller.h>

// Definição dos pinos para controle dos motores
#define ENA_PIN 32 // PWM Motor A
#define IN1_PIN 26 // Pino IN1 Motor A
#define IN2_PIN 27 // Pino IN2 Motor A
#define ENB_PIN 33 // PWM Motor B
#define IN3_PIN 14 // Pino IN3 Motor B
#define IN4_PIN 12 // Pino IN4 Motor B

const int PWM_FREQ = 1000;       // Frequência PWM para controle de velocidade
const int PWM_RESOLUTION = 8;    // Resolução PWM de 8 bits

// Variáveis para controlar a velocidade máxima
int maxSpeedPercentage = 100; // Velocidade máxima inicial (100%)

bool leftMotorStopped = true;
bool rightMotorStopped = true;

// Função para configurar os pinos dos motores
void setUpPinModes() {
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  ledcAttach(ENA_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB_PIN, PWM_FREQ, PWM_RESOLUTION);
  
  rotateMotor(0, 0);  // Inicializa os motores parados
}

// Função chamada quando o controle PS4 é conectado
void onConnect() {
  Serial.println("Controle PS4 Conectado!");
}

// Função chamada quando o controle PS4 é desconectado
void onDisConnect() {
  rotateMotor(0, 0);  // Para os motores
  Serial.println("Controle PS4 Desconectado!");    
}

// Função para processar as entradas do controle PS4
void notify() {
  int yAxisValue = PS4.LStickY();   // Valor do eixo Y do analógico esquerdo (acelerador)
  int xAxisValue = PS4.RStickX();   // Valor do eixo X do analógico direito (direção)

  // Mapeia os valores dos analógicos para um intervalo de -255 a 255
  int throttle = map(yAxisValue, -127, 127, -255, 255);
  int steering = map(xAxisValue, -127, 127, -255, 255);

  // Ajusta a velocidade máxima com base no estado atual
  int maxSpeed = map(maxSpeedPercentage, 0, 100, 0, 255);

  // Aplica o throttle dentro dos limites de velocidade máxima
  int motorDirection = (throttle < 0) ? -1 : 1;
  int rightMotorSpeed = constrain(abs(throttle) - steering, 0, maxSpeed);
  int leftMotorSpeed = constrain(abs(throttle) + steering, 0, maxSpeed);

  // Chama a função para girar os motores com as velocidades calculadas
  rotateMotor(rightMotorSpeed * motorDirection, leftMotorSpeed * motorDirection);

  // Atualiza a iluminação do controle PS4 de acordo com o nível de velocidade
  updatePS4LED(maxSpeedPercentage);
}

// Função para controlar a rotação dos motores com base na velocidade
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  leftMotorStopped = false;  
  rightMotorStopped = false;  

  // Controla a direção do motor esquerdo
  if (leftMotorSpeed < -20) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);    
  } else if (leftMotorSpeed > 20) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);      
  } else {
    leftMotorStopped = true;      
  }

  // Controla a direção do motor direito
  if (rightMotorSpeed < -20) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);   
  } else if (rightMotorSpeed > 20) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);      
  } else {
    rightMotorStopped = true;      
  }

  ledcWrite(ENA_PIN, abs(leftMotorSpeed));   
  ledcWrite(ENB_PIN, abs(rightMotorSpeed));
}

// Função para alterar a cor do LED do controle PS4 com base na velocidade
void updatePS4LED(int speedPercentage) {
  if (speedPercentage == 50) {
    PS4.setLed(0, 0, 255);  // Azul para 50% de velocidade
  } else if (speedPercentage == 70) {
    PS4.setLed(0, 255, 0);  // Verde para 70% de velocidade
  } else if (speedPercentage == 100) {
    PS4.setLed(255, 0, 0);  // Vermelho para 100% de velocidade
  }
}

// Função de setup que inicializa o sistema
void setup() {
  setUpPinModes();
  Serial.begin(115200);
  PS4.begin("2c:33:7a:bb:ed:cc");  // Substitua com o MAC do seu controle PS4
  PS4.attach(notify);  
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);

  while (PS4.isConnected() == false) { 
    Serial.println("Controle PS4 Não encontrado");
    delay(300);  // Aguarda até o controle ser conectado
  } 
  Serial.println("Controle PS4 Pronto.");
}

// Função principal de loop
void loop() {
  // Verifica os botões pressionados e ajusta a velocidade máxima
  if (PS4.Circle()) {
    maxSpeedPercentage = 100;  // 100% de velocidade
    Serial.println("Velocidade máxima: 100%");
  } else if (PS4.Triangle()) {
    maxSpeedPercentage = 70;  // 70% de velocidade
    Serial.println("Velocidade máxima: 70%");
  } else if (PS4.Square()) {
    maxSpeedPercentage = 50;  // 50% de velocidade
    Serial.println("Velocidade máxima: 50%");
  }

  // Exibe os valores dos eixos para diagnóstico (a cada 6 segundos)
  Serial.printf("Y (Acelerador) = %d, X (Direção) = %d\n", PS4.LStickY(), PS4.RStickX());
  Serial.printf("Throttle = %d, Steering = %d\n", map(PS4.LStickY(), -127, 127, -255, 255), map(PS4.RStickX(), -127, 127, -255, 255));
  Serial.printf("Velocidade Motor Esquerdo = %d, Velocidade Motor Direito = %d\n", abs(PS4.LStickY()) - map(PS4.RStickX(), -127, 127, -255, 255), abs(PS4.LStickY()) + map(PS4.RStickX(), -127, 127, -255, 255));

  // Diagnóstico do estado dos motores
  if (leftMotorStopped && rightMotorStopped) {
    Serial.println("Ambos os motores estão parados");
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
  }

  // Diagnóstico da direção dos motores
  if (digitalRead(IN1_PIN) && !digitalRead(IN2_PIN)) {
    Serial.println("Motor Esquerdo em Sentido Anti-Horário");
  }
  if (digitalRead(IN3_PIN) && !digitalRead(IN4_PIN)) {
    Serial.println("Motor Direito em Sentido Anti-Horário");
  }
  if (!digitalRead(IN1_PIN) && digitalRead(IN2_PIN)) { 
    Serial.println("Motor Esquerdo em Sentido Horário");
  }
  if (!digitalRead(IN3_PIN) && digitalRead(IN4_PIN)) {
    Serial.println("Motor Direito em Sentido Horário");
  }

  // Verifica se ambos os motores estão parados
  if ((!digitalRead(IN1_PIN) && !digitalRead(IN2_PIN)) && (!digitalRead(IN3_PIN) && !digitalRead(IN4_PIN))) {
    Serial.println("Motores Parados");
  }

  // Diagnóstico do estado dos pinos de controle
  Serial.println(digitalRead(IN1_PIN));
  Serial.println(digitalRead(IN2_PIN));
  Serial.println(digitalRead(IN3_PIN));
  Serial.println(digitalRead(IN4_PIN));

  delay(1000);  // Atraso de 6 segundos entre as atualizações
}
