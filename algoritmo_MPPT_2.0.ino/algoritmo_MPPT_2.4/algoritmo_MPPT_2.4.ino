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





//=====================================
//------ Mapeamento de Hardware -------
#define   pwm1    (1<<PD5)
#define   pwm2    (1<<PD6)

#define   pwm3    (1<<PB2)
#define   pwm4    (1<<PB3)

ADS1115 ADS(0x48);

//===========================================================
//----------- Variaveis Globais -------------

float     Tensao_fonte          = 0.0,
          Corrente_fonte        = 0.0,
          Tensao_circ_aberto    = 40.0,
          Tensao_ref_fonte      = 50,  //dutty do PWM com resolução de 8bits
          Tensao_ref_maxima     = 155.0,
          Tensao_ref_minima     = 5.0;

int8_t    PASSO_MPPT            = 2;

// ============================================================================
// --- Protótipo das Funções ---
void Algoritmo_MPPT();


// ============================================================================
// ------------------- Interrupções -------------------

//---------Interrupção da máquina de estados------------

ISR(TIMER1_OVF_vect) 
{
  //_delay_us(1);
}

//---------Interrupção do algoritmo MPPT---------------

ISR(TIMER2_OVF_vect) 
{
  Algoritmo_MPPT(Corrente_fonte, Tensao_fonte);

  OCR0A = Tensao_ref_fonte;
}



//=============================================================
//--------- Função principal ----------
void setup()
{


  //define as saidas PWM
  DDRD  |=  pwm1;
  DDRD  |=  pwm2;

  //incializamos os bits do PWM em LOW
  PORTD  &=  ~pwm1;
  PORTD  &=  ~pwm2;

 
  //configura o PWM do timer0
  TCCR0A  = 0b10100011;   //configura o PWM no modo Fast PWM (0xA3 em hexadecimal)
  TCCR0B  = 0b00010001;   //Pre Scaler 1:1, setando a frequência em 62.4kHz (0x01 em hexadecimal)
  OCR0A   = 0;            //dutty cicle do pino PWM PD5
  OCR0B   = 100;            //dutty cicle do pino PWM PD6

  //Configuração das Interrupções.
  cli();                  //desliga interrupções
  
  //Configuração das interrupções por estouro do TIMER1
  TCCR1B = 0b00000100;    //prescaler 1:256
  TIMSK1 = 0b00000001;    //habilita interrupção do Timer1
  TCNT1  = 65286;         //inicia Timer1 para contar a partir de 60539 e estabelecer uma frequencia de 25Hz

  //Configuração das interrupções por estouro do TIMER2
  TCCR2B = 0b00000110;    //prescaler 1:256
  TIMSK2 = 0b00000001;    //habilita interrupção do Timer2
  TCNT2  = 131;           //inicia Timer2 para contar a partir de 131 e estabelecer uma frequencia de 500Hz
    
  sei();



  Wire.begin();
  //Serial.begin(9600);

  //Serial.println(__FILE__);
  //Serial.print("ADS1X15_LIB_VERSION: ");
  //Serial.println(ADS1X15_LIB_VERSION);

  ADS.begin();
  ADS.setGain(2);      // 4.096V volt
  ADS.setDataRate(7);  // 0 = slow   4 = medium   7 = fast
  ADS.setMode(1);      // continuous mode
  ADS.readADC(0);      // first read to trigger
  
  // set the thresholds to Trigger RDY pin
  ADS.setComparatorThresholdLow(0x0000);
  ADS.setComparatorThresholdHigh(0x0200);
  ADS.setComparatorQueConvert(0);             // enable RDY pin !!
  ADS.setComparatorLatch(0);

} //FIM DA FUNÇÃO PRINCIPAL



//---------- LOOP INFINITO ----------
void loop()
{ //INÍCIO DO LOOP INFINITO

  ADS.setGain(1);
  float voltage = 0.0,
        f       = ADS.toVoltage(1);   // voltage factor

  static int  cont = 0;

  Tensao_fonte    = 0.0;
  voltage = ADS.readADC(0);
  Tensao_fonte = ((voltage * f ) - 0.0031) * 250;       // calculo baseado na relação de 4mV/V na saída do sensor de tensão
  
  
  Corrente_fonte    = 0.0;
  voltage = ADS.readADC(1);
  Corrente_fonte = ((voltage * f ) - 1.626) / 0.0187 ;       // calculo baseado na relação de 18.7mV/A na saída do sensor de corrente


delay(500);


} //FIM DO LOOP INFINITO





//===========================================================
//--------- Desenvolvimento das Funções ----------


void Algoritmo_MPPT(float corrente, float tensao) 
{
  //variaveis locais da função
  static float  deltaV              = 0,
                tensao_anterior     = 0,
                Potencia_anterior   = 0,
                potencia_fonte      = 0;

  int           Incializacao_MPPT   = 0;
  
  //Calculo do valor atual da potência a partir da variação da tensão 
  potencia_fonte = corrente * tensao;
  deltaV = tensao - tensao_anterior;

  if((tensao > (Tensao_circ_aberto - 3)) && (Incializacao_MPPT == 0))
    {
      Tensao_ref_fonte = Tensao_ref_fonte - 2;
    }
  else
    {
      Incializacao_MPPT = 1;
    }
  
  //Algoritmo MPPT
  if(potencia_fonte > Potencia_anterior)
    {
      if(deltaV > 0) Tensao_ref_fonte += PASSO_MPPT;
      else Tensao_ref_fonte -= PASSO_MPPT;
    }
  else if(potencia_fonte < Potencia_anterior)
    {
      if(deltaV > 0) Tensao_ref_fonte -= PASSO_MPPT;
      else Tensao_ref_fonte += PASSO_MPPT;
    }
  
  /*
  Serial.print(tensao);
  Serial.print("V ");

  Serial.print(corrente);
  Serial.print("A ");

  Serial.print(potencia_fonte);
  Serial.print("W ");

  Serial.print(Tensao_ref_fonte);
  Serial.println("%  ");
  */

  //Limites para estabelecer a saturação do volores máximos do MPPT
  if(Tensao_ref_fonte > Tensao_ref_maxima) Tensao_ref_fonte = Tensao_ref_maxima;
  else if(Tensao_ref_fonte < Tensao_ref_minima) Tensao_ref_fonte = Tensao_ref_minima;

  //Armazenamento dos volores presentes nas variaveis de valores do passado
  tensao_anterior = tensao;
  Potencia_anterior = potencia_fonte;   // Armazena o valor de potência calculado anteriormente
  
}

