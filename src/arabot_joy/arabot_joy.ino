#include <AFMotor.h>

int motorSpeedL = 0;
int motorSpeedR = 0;
AF_DCMotor motorL(3);
AF_DCMotor motorR(4);

void setup() {
  motorL.setSpeed(0);
  motorR.setSpeed(0);
  motorL.run(RELEASE);
  motorR.run(RELEASE);
}

void loop() {
  int xAxis = analogRead(A4); // Leitura do eixo X do Joystick
  int yAxis = analogRead(A5); // Leitura do eixo Y do Joystick

  // Eixo Y usado para controle para frente e para trás
  if (yAxis > 550) {
    motorL.run(FORWARD);
    motorR.run(FORWARD);
    // Converter as leituras crescentes do eixo Y para andar para frente ao posicionar o joystick de 550 até 1023, ou equivalentemente, enviando um sinal PWM de 0 até 255 para aumentar a velocidade do motor
    motorSpeedL = map(yAxis, 550, 1023, 0, 255);
    motorSpeedR = map(yAxis, 550, 1023, 0, 255);
  }
  else if (yAxis < 470) {
    motorL.run(BACKWARD);
    motorR.run(BACKWARD);
    // Converter as leituras decrescentes do eixo Y para andar para trás ao posicionar o joystick de 470 até 0, ou equivalentemente, enviando um sinal PWM de 0 até 255 para aumentar a velocidade do motor
    motorSpeedL = map(yAxis, 470, 0, 0, 255);
    motorSpeedR = map(yAxis, 470, 0, 0, 255);
  }
  // Se o joystick ficar no meio, os motores não se movem
  else {
    motorSpeedL = 0;
    motorSpeedR = 0;
  }

  // Eixo X usado para controle direção esquerda/direita
  if (xAxis < 470) {
    // Converter as leituras decrescentes de 470 a 0 referentes ao eixo X do joystick em valores crescentes de sinal PWM, respectivamente de 0 a 255
    int xMapped = map(xAxis, 470, 0, 0, 255);
    // Girar para a esquerda - diminuir a velocidade do motor da esquerda, aumentar a velocidade do motor da direita
    motorSpeedL = motorSpeedL - xMapped;
    motorSpeedR = motorSpeedR + xMapped;
    // Confinar o sinal PWM no intervalo de 0 a 255
    if (motorSpeedL < 0) {
      motorSpeedL = 0;
    }
    if (motorSpeedR > 255) {
      motorSpeedR = 255;
    }
  }
  if (xAxis > 550) {
    // Converter as leituras crescentes de 550 a 1023 referentes ao do eixo X do joystick em valores crescentes de sinal PWM, respectivamente de 0 a 255
    int xMapped = map(xAxis, 550, 1023, 0, 255);
    // Girar para a direita - diminuir a velocidade do motor da direita, aumentar a velocidade do motor da esquerda
    motorSpeedL = motorSpeedL + xMapped;
    motorSpeedR = motorSpeedR - xMapped;
    // Confinar o sinal PWM no intervalo de 0 a 255
    if (motorSpeedL > 255) {
      motorSpeedL = 255;
    }
    if (motorSpeedR < 0) {
      motorSpeedR = 0;
    }
  }
  // Evitar zumbidos em baixas velocidades (ajuste de acordo com seus motores. Meus motores não começariam a se mover se o valor de PWM estivesse abaixo do valor de 70)
  if (motorSpeedL < 70) {
    motorSpeedL = 0;
  }
  if (motorSpeedR < 70) {
    motorSpeedR = 0;
  }
  motorL.setSpeed(motorSpeedL); // Enviar sinal PWM para o motor da esquerda
  motorR.setSpeed(motorSpeedR); // Enviar sinal PWM para o motor da direita
}
