//=====================================
//--------- frequência da CPU ---------
#define F_CPU 16000000


//=====================================
//----------- Bibliotecas -------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
#include <Wire.h>
#include "ADS1X15.h"
#include <PID_v1.h>

//=====================================
//------ Mapeamento de Hardware -------
#define   pwm1        (1<<PB1)
#define   pwm2        (1<<PB2)
#define   freq_PWM    60000
#define   tempo_morto 4  


unsigned int maximo = 0;

//=============================================================
//--------- Função principal ----------
void setup()
{

  //define as saidas PWM
  DDRB  |=  pwm1;
  DDRB  |=  pwm2;

  //incializamos os bits do PWM em LOW
  PORTB  &=  ~pwm1;
  PORTB  &=  ~pwm2;

 
  //configura o PWM do timer1
  TCCR1A  = 0b10110010;   //configura o PWM no modo Fast PWM (0xA3 em hexadecimal)
  TCCR1B  = 0b00011001;   //Pre Scaler 1:1, setando a frequência em 62.4kHz (0x01 em hexadecimal)
  //TCCR1C  = 0b00000000;

  maximo = (F_CPU / (freq_PWM)) - 1; // define a frequência do PWM
  ICR1 = maximo; 

  //OCR1A   = maximo >> 2;          //dutty cicle do pino PWM PB1
  //OCR1B   = maximo >> 2;          //dutty cicle do pino PWM PB2

  //Configuração das Interrupções.
  cli();                  //desliga interrupções
  
  //Configuração das interrupções do TIMER1
  TIMSK1 = 0b00100000;    //habilita interrupção do Timer1 por comparação com ICR1

  //Configuração das interrupções por estouro do TIMER2
  TCCR2B = 0b00000010;    //prescaler 1:8
  TIMSK2 = 0b00000001;    //habilita interrupção do Timer2
  TCNT2  = 6;           //inicia Timer2 para contar a partir de 193 e estabelecer uma frequencia de 8kHz
    
  sei();


} //FIM DA FUNÇÃO PRINCIPAL



//---------- LOOP INFINITO ----------
void loop()
{ //INÍCIO DO LOOP INFINITO



} //FIM DO LOOP INFINITO


// ============================================================================
// ------------------- Interrupções -------------------

//---------Interrupção da máquina de estados------------

ISR(TIMER1_CAPT_vect) 
{
  unsigned int dutty = 129;

  OCR1A = dutty;// - tempo_morto;
  OCR1B = dutty;// + tempo_morto;
}

//---------Interrupção do algoritmo MPPT---------------

ISR(TIMER2_OVF_vect) 
{

}
