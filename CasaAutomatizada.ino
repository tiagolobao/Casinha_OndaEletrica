//Falta LCD e Sensor do alarme

#include <Servo.h>                      //Biblioteca para controlar servo motores
#include <Wire.h>                       // para uso da comunicação I2C
#include <SoftwareSerial.h>             // Biblioteca para a comunicação serial com bluetooth
//#include <LiquidCrystal.h>              // permite usar o lcd
#include <LiquidCrystal_I2C.h>          // LCD com comunicação I2C (modulo)


#define timer 20 //Tempo de espera na hora de atualizar o app

// LED => Piscina, Gramado, Garagem, Sala, Cozinha, Corredor, Banheiro, Quarto 1, Quarto 2. 
// MOTORES => Janela, Porta da Frente
// PERIFERICOS => LCD (2 pinos), Bluetooth (2 Pinos), Buzzer (1 Pino), Sensor (1 Pino)

//Definindo dos pinos das lampadas (LEDS)
//Pino 0    //Lampada da cozinha 
//Pino 1    //Lampada da sala
//Pino 2    //Lampada do banheiro
//Pino 3   //Lampada do quarto1
//Pino 4   //Lampada do quarto2
//Pino 10    //Lampada da corredor
//Pino 11    //Lampada da piscina
//Pino 12    //Lampada da gramado
//Pino 13    //Lampada da garagem


//definindo código vindo do bluetooth para mudar o estado das lâmpadas
//Lâmpada da cozinha 'a'
//Lâmpada da Sala 'b'
//Lâmpada da Banheiro 'c'
//Lâmpada da Quarto 1 'd'
//Lâmpada da Quarto 2 'e'
//Lâmpada da Corredor 'f'
//Lâmpada da Piscina 'g'
//Lâmpada da Gramado 'h'
//Lâmpada da Garagem 'i'


//Definindo pinos dos motores
#define PORTA 5     //Porta da frente
#define JANELA 6    //Janela do quarto
#define TV 7        //Televisão

//Definindo outros pinos
#define BUZZER A2 //Barulinho chato
#define SENSOR A3 //Sensor de presença
#define BT_RX 8    //Bluetooth RX
#define BT_TX 9    //Bluetooth TX


//definindo código vindo do bluetooth para ligar e desligar motores
// PORTA 'j'
// JANELA 'k'
// TELEVISÃO 'l'


Servo servoPorta;  // Cria um objeto de controle para o servo
Servo servoJanela;
SoftwareSerial BT(BT_RX, BT_TX); // RX, TX   PINOS BLUETOOTH
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // Addr, En, Rw, Rs, d4, d5, d6, d7, backlighpin, polarity

//Constantes
char ligado[13] = "ABCDEFGHIJKL";
char desligado[13] = "abcdefghijkl";
int pinos[13] = {0,1,2,3,4,10,11,12,13,5,6,7}; //Pinos que dependem das variável state

//Variáveis
boolean state[13] = {0,0,0,0,0,0,0,0,0,0,0,0}; //A variável que armazena o estado de cada led e porta/janela
int i; // usado nos comandos for



void setup(){
  //Inicialiando o display
  
  //Inicializando as comunicações serial
  Serial.begin(9600); // Descomente para debugar
  BT.begin(9600); //Comunicação bluetooth
//  Serial.println("Casinha");
 
  //Definindo os pinos dos servos
  servoPorta.attach(PORTA); 
  servoJanela.attach(JANELA);
//Inicializando os pinos das lâmpadas
  for(i=0;i<13;i++){
    pinMode(pinos[i],OUTPUT);
  }

//Inicializando os pinos das lâmpadas
  pinMode(BUZZER,OUTPUT);
  pinMode(SENSOR,INPUT);  
}

void loop(){
  if (Serial.available()){
    char comando = '0';
    comando = Serial.read();
    Serial.println(comando); //Debug
    if(comando!='m'){  
      i=0;
      while(comando != desligado[i]){  //Utilizei o array desligado pois o app manda as letas minusculas para ligar e desligar  
        i++;
      }
      if(state[i]){
        turn_off(pinos[i]);
        state[i] = 0;     
        }
        else{
        turn_on(pinos[i]);
        state[i] = 1;
      }
    }
    else{
      char caractere;
      String nome = "";
      delay(timer); //espera um tempo para o bluetooth ficar disponível
      while(Serial.available()){
        caractere = Serial.read();
        nome = nome + caractere;    
      }
    lcd.clear();  
    lcd.print(nome);
    Serial.println(nome); //Debug
    }
  update_app(); //Sempre ao terminar um comando ou receber qualquer letra, atualize o app
  }

  if(digitalRead(SENSOR)){
    while(Serial.read()!='n'){
      Serial.write('n'); //Comando para informar o app que o alarme foi ativado
      alarme();  //ativa o buzzer
    }
    Serial.write('N'); //Comando para informar o app que o alarme acabou
  }
}



void alarme(){
  digitalWrite(BUZZER,HIGH);
  delay(200);
  digitalWrite(BUZZER,LOW);
  delay(200);
}

void update_app(){
  for(i=0;i<13;i++){
    delay(timer);
    if(state[i]){
      Serial.write(ligado[i]);
 //     Serial.println(ligado[i]);
    }
    else{
      Serial.write(desligado[i]);
  //    Serial.println(desligado[i]);
    }
  }
}

void turn_on(int pin){
  if(pin==PORTA){
    //Serial.println(i);
    servoPorta.write(10);
  }
  else if(pin==JANELA){
    servoJanela.write(10);     
  }
  else if(pin==TV){
    digitalWrite(TV,HIGH);
    delay(500);
    lcd.begin(16,2);
    lcd.clear();     
    lcd.print("CASINHA");
    lcd.setCursor(0,1);
    lcd.print("ONDA ELETRICA");
  }
  else{
    digitalWrite(pin,HIGH);
  }
}

void turn_off(int pin){
  if(pin==PORTA){
    servoPorta.write(100);
  }
  else if(pin==JANELA){
    servoJanela.write(100);
  }
  else if(pin==TV){
    digitalWrite(TV,LOW);
  }
  else{
    digitalWrite(pin,LOW);
  }
}
