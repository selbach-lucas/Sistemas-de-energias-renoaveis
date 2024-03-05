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
#define   pwm1        (1<<PD5)
#define   pwm2        (1<<PD6)
#define   TSAMPLE     50000
#define   tempo_morto 3


ADS1115 ADS(0x48);

//===========================================================
//----------- Variaveis Globais -------------

float Tensao_fonte,
      Corrente_fonte,
      Tensao_filtrada,
      Corrente_filtrada,
      Kp = 10.0,
      Ki = 1.0,
      Kd = 10.0;

int   Tensao_ref_fonte;  //dutty do PWM com resolução de 8bits

unsigned long Tempo_anterior  = 0,
              Tempo_atual     = 0;

static int  PASSO_MPPT         = 1,
            Tensao_circ_aberto = 42,
            Tensao_ref_maxima  = 155,
            Tensao_ref_minima  = 5;


// ============================================================================
// --- Protótipo das Funções ---
void Algoritmo_MPPT();

void Desloca_vetor();

void Filtro_PB_tensao();

void Filtro_PB_corrente();

//PID controlador(&Tensao_ref_fonte, &OCR0A, &Tensao_ref_maxima, Kp, Ki, Kd, 1, 0);

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
  Algoritmo_MPPT(Corrente_filtrada, Tensao_filtrada);

  OCR0A = Tensao_ref_fonte - tempo_morto;
  OCR0B = Tensao_ref_fonte + tempo_morto;
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
  TCCR0A  = 0b10110011;   //configura o PWM no modo Fast PWM (0xA3 em hexadecimal)
  TCCR0B  = 0b00010001;   //Pre Scaler 1:1, setando a frequência em 62.4kHz (0x01 em hexadecimal)
  OCR0A   = 10;            //dutty cicle do pino PWM PD5
  OCR0B   = 10;          //dutty cicle do pino PWM PD6

  //Configuração das Interrupções.
  cli();                  //desliga interrupções
  
  //Configuração das interrupções por estouro do TIMER1
  TCCR1B = 0b00000100;    //prescaler 1:256
  TIMSK1 = 0b00000001;    //habilita interrupção do Timer1
  TCNT1  = 65286;         //inicia Timer1 para contar a partir de 60539 e estabelecer uma frequencia de 25Hz

  //Configuração das interrupções por estouro do TIMER2
  TCCR2B = 0b00000111;    //prescaler 1:8
  TIMSK2 = 0b00000001;    //habilita interrupção do Timer2
  TCNT2  = 60;           //inicia Timer2 para contar a partir de 193 e estabelecer uma frequencia de 8kHz
    
  sei();



  Wire.begin();
  Serial.begin(9600);

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

  //conrtrolador.SetMode(AUTOMATIC);

} //FIM DA FUNÇÃO PRINCIPAL


//---------- LOOP INFINITO ----------
void loop()
{ //INÍCIO DO LOOP INFINITO

  ADS.setGain(1);
  float voltage = 0.0,
        f       = ADS.toVoltage(1);   // voltage factor

  static int  cont = 0;


  Tensao_fonte = ((ADS.readADC(0) * f ) - 0.0031) * 250;       // calculo baseado na relação de 4mV/V na saída do sensor de tensão
  Tensao_filtrada = Filtro_PB_tensao(Tensao_fonte);
  
  Corrente_fonte = ((ADS.readADC(1) * f ) - 1.626) / 0.0187 ;       // calculo baseado na relação de 18.7mV/A na saída do sensor de corrente
  Corrente_filtrada = Filtro_PB_corrente(Corrente_fonte);


  //delay(4);

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
  
 
  Serial.print(tensao);
  Serial.print("V ");

  Serial.print(corrente);
  Serial.print("A ");

  Serial.print(potencia_fonte);
  Serial.print("W ");

  Serial.print(Tensao_ref_fonte);
  Serial.println("%");
  

  //Limites para estabelecer a saturação do volores máximos do MPPT
  if(Tensao_ref_fonte > Tensao_ref_maxima) Tensao_ref_fonte = Tensao_ref_maxima;
  else if(Tensao_ref_fonte < Tensao_ref_minima) Tensao_ref_fonte = Tensao_ref_minima;

  //Armazenamento dos volores presentes nas variaveis de valores do passado
  tensao_anterior = tensao;
  Potencia_anterior = potencia_fonte;   // Armazena o valor de potência calculado anteriormente
  
}


void Desloca_vetor(float *vetorAddr, int tam, float valor)
{
  for(int k = tam - 1; k > 0; k--)
  {
    *(vetorAddr + k) = *(vetorAddr + k - 1);
  }
  *vetorAddr = valor;
}


float Filtro_PB_tensao(float x)
{
  static float  y_pass1[2] = {0,0},
                x_pass1[2] = {0,0};

  const float   a = 0.45,
                b = 0.50;
  
  float y = ((a + b) * y_pass1[0]) - (a * b * y_pass1[1]) + ((1 - a - b + (a * b)) * x_pass1[1]);

  Desloca_vetor(y_pass1, 2, y);
  Desloca_vetor(x_pass1, 2, x);
  
  return y;
}


float Filtro_PB_corrente(float x)
{
  static float  y_pass2[2] = {0,0},
                x_pass2[2] = {0,0};

  const float   a = 0.65,
                b = 0.70;
  
  float y = ((a + b) * y_pass2[0]) - (a * b * y_pass2[1]) + ((1 - a - b + (a * b)) * x_pass2[1]);

  Desloca_vetor(y_pass2, 2, y);
  Desloca_vetor(x_pass2, 2, x);
  
  return y;
}

