//CÓDIGO PERFEITAMENTE FUNCIONAL: pode ser aplicado no conversor BOOST.

// ------ ***** Versão com PWM de 16kHz do Timer1 no modo Phase Correct com interrupção de evento na flag ICR1. Tempo morto do PWM: 520ns ***** -------
// Data: 03/10/2023
// Versão: 3.2


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

//=====================================
//------ Definições importantes -------
#define   Freq_PWM    16000   //frequência do PWM
#define   Tempo_morto 4       //valor do tempo morto em ciclos do timer

ADS1115 ADS(0x48);

//===========================================================
//----------- Variaveis Globais -------------

float Tensao_fonte,
      Corrente_fonte,
      Tensao_filtrada,
      Corrente_filtrada;

int   Tensao_ref_fonte;  //dutty do PWM com resolução de 8bits

unsigned long Tempo_anterior  = 0,
              Tempo_atual     = 0;

static unsigned int   PASSO_MPPT         = 1,
                      Maximo_ICR1        = 0,
                      Tensao_circ_aberto = 42,
                      Tensao_ref_maxima  = 0,
                      Tensao_ref_minima  = 0;


// ============================================================================
// --- Protótipo das Funções ---
void Algoritmo_MPPT();

void Desloca_vetor();

void Filtro_PB_tensao();

void Filtro_PB_corrente();


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

 
  //configuração do PWM do timer1
  TCCR1A  = 0b10110010;   //configura o PWM no modo Phase Correct com Interrupt Flag Set ICR1 habilitada
  TCCR1B  = 0b00010001;   //Pre Scaler 1:1

  Maximo_ICR1 = (F_CPU / (2 * Freq_PWM)) - 1;  // define o valor máximo da Interrupt Flag Set ICR1 para a frequência do PWM escolhida
  Tensao_ref_maxima  = Maximo_ICR1 * 0.95,     // limita o valor máximo do dutty cicle do PWM em 95% 
  Tensao_ref_minima  = Maximo_ICR1 * 0.03;     // limita o valor mínimo do dutty cicle do PWM em 03%
  
  ICR1 = Maximo_ICR1; //atribui o valor máximo de contagem calculado à flag ICR1


  //Configuração das Interrupções.
  cli();                  //desliga interrupções
  
  //Configuração das interrupções do TIMER1
  TIMSK1 = 0b00100000;    //habilita interrupção do Timer1 por comparação com a Interrupt Flag Set ICR1

  //Configuração das interrupções por estouro do TIMER2
  TCCR2B = 0b00000010;    //prescaler 1:8
  TIMSK2 = 0b00000001;    //habilita interrupção do Timer2
  TCNT2  = 6;             //inicia Timer2 para contar a partir de 193 e estabelecer uma frequencia de 8kHz
    
  sei();


  Wire.begin();        //inicializa o protocolo de comunicação I2C
  Serial.begin(9600);

  ADS.begin();         //inicializa a comunicação com o ADS1115
  ADS.setGain(2);      //determina a resolução do ADS1115 para um limite de 4.096V
  ADS.setDataRate(7);  //determina a velociade de aquisição das medidas convertidas no ADS1115 (0 = slow   4 = medium   7 = fast)
  ADS.setMode(1);      //determina o modo de operação do ADS1115 para "single mode"
  ADS.readADC(0);      //first read to trigger
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

  Tensao_fonte = ((ADS.readADC(0) * f ) - 0.0031) * 250;       // calculo baseado na relação de 4mV/V na saída do sensor de tensão
  Tensao_filtrada = Filtro_PB_tensao(Tensao_fonte);
  
  Corrente_fonte = ((ADS.readADC(1) * f ) - 1.626) / 0.0187 ;       // calculo baseado na relação de 18.7mV/A na saída do sensor de corrente
  Corrente_filtrada = Filtro_PB_corrente(Corrente_fonte);


} //FIM DO LOOP INFINITO




// ============================================================================
// ------------------- Interrupções -------------------

//---------Interrupção da máquina de estados------------

ISR(TIMER1_CAPT_vect) 
{
   

}


//---------Interrupção do algoritmo MPPT---------------

ISR(TIMER2_OVF_vect) 
{
  Algoritmo_MPPT(Corrente_filtrada, Tensao_filtrada);
  Serial.println("Laço 0");
}



//===========================================================
//--------- Desenvolvimento das Funções ----------


//>>>>>>>>>>>> ALGORITMO MPPT <<<<<<<<<<<<<
void Algoritmo_MPPT(float corrente, float tensao) 
{
  //variaveis locais da função
  static float  deltaV              = 0,
                tensao_anterior     = 0,
                potencia_anterior   = 0,
                potencia_fonte      = 0;

  int           incializacao_MPPT   = 0;
   
  potencia_fonte = corrente * tensao;   //Calculo do valor atual da potência
  deltaV = tensao - tensao_anterior;    //cálculo da variação da tensão 

  if((tensao > (Tensao_circ_aberto - 3)) && (incializacao_MPPT == 0))
    {
      Tensao_ref_fonte = Tensao_ref_fonte - 2;
      Serial.println("Laço 1");
    }
  else
    {
      incializacao_MPPT = 1;
    }
  
  //Algoritmo MPPT
  if(potencia_fonte > potencia_anterior)
    {
      if(deltaV > 0) Tensao_ref_fonte += PASSO_MPPT;
      else Tensao_ref_fonte -= PASSO_MPPT;
      Serial.println("Laço 2");
    }
  else if(potencia_fonte < potencia_anterior)
    {
      if(deltaV > 0) Tensao_ref_fonte -= PASSO_MPPT;
      else Tensao_ref_fonte += PASSO_MPPT;
      Serial.println("Laço 3");
    }

  OCR1A = Tensao_ref_fonte - Tempo_morto; //atualização do dutty do PWM do pino PB1 (pino D09 no Arduino Nano)
  OCR1B = Tensao_ref_fonte + Tempo_morto; //atualização do dutty do PWM do pino PB2 (pino D10 no Arduino Nano)

  /*
  Serial.print(tensao);
  Serial.print("V ");

  Serial.print(corrente);
  Serial.print("A ");

  Serial.print(potencia_fonte);
  Serial.print("W ");

  Serial.print(Tensao_ref_fonte);
  Serial.println("%");
  */

  //Limites para estabelecer a saturação do volores máximos do MPPT
  if(Tensao_ref_fonte > Tensao_ref_maxima) Tensao_ref_fonte = Tensao_ref_maxima;
  else if(Tensao_ref_fonte < Tensao_ref_minima) Tensao_ref_fonte = Tensao_ref_minima;

  //Armazenamento dos volores presentes nas variaveis de valores do passado
  tensao_anterior = tensao;
  potencia_anterior = potencia_fonte;   // Armazena o valor de potência calculado anteriormente
  
}


//>>>>>>>>>>>> FUNÇÃO QUE ARMAZENA E DESLOCA DADOS EM UM VETOR <<<<<<<<<<<<<
void Desloca_vetor(float *vetorAddr, int dimensao, float valor)
{
  static int k = dimensao - 1;

  for(k ; k > 0; k--)
  {
    *(vetorAddr + k) = *(vetorAddr + k - 1);
  }
  *vetorAddr = valor;
}

//>>>>>>>>>>>> FILTRO PASSA BAIXAS IIR PARA A TENSÃO <<<<<<<<<<<<<
float Filtro_PB_tensao(float x)
{
  static float  y,
                y_pass1[2] = {0,0},
                x_pass1[2] = {0,0};

  static const float    a = 0.45,
                        b = 0.50;
  
  y = ((a + b) * y_pass1[0]) - (a * b * y_pass1[1]) + ((1 - a - b + (a * b)) * x_pass1[1]);

  Desloca_vetor(y_pass1, 2, y);
  Desloca_vetor(x_pass1, 2, x);
  
  return y;
}


//>>>>>>>>>>>> FILTRO PASSA BAIXAS IIR PARA A CORRENTE <<<<<<<<<<<<<
float Filtro_PB_corrente(float x)
{
  static float  y_pass2[2] = {0,0},
                x_pass2[2] = {0,0};

  static const float    a = 0.45,
                        b = 0.50;
  
  float y = ((a + b) * y_pass2[0]) - (a * b * y_pass2[1]) + ((1 - a - b + (a * b)) * x_pass2[1]);

  Desloca_vetor(y_pass2, 2, y);
  Desloca_vetor(x_pass2, 2, x);
  
  return y;
}

