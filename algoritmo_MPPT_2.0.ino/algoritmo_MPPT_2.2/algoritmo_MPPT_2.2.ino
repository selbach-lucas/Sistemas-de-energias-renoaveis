//=====================================
//--------- frequência da CPU ---------
#define F_CPU 16000000


//=====================================
//----------- Bibliotecas -------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
#include <Wire.h>
#include <ADS1115_WE.h>





//=====================================
//------ Mapeamento de Hardware -------
#define   pwm1    (1<<PD5)
#define   pwm2    (1<<PD6)

#define I2C_ADDRESS 0x48
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

//===========================================================
//----------- Variaveis Globais -------------

float     tensao_anterior       = 0,
          potencia_anterior     = 0,
          PASSO_MPPT            = 1,
          Tensao_circ_aberto    = 256,
          tensao_ref_fonte      = 0,  //dutty de 50% do PWM com resolução de 8bits
          Tensao_maxima_fonte   = 200.0,
          Tensao_minima_fonte   = 90.0,
          Tensao_fonte          = 0.0,
          Corrente_fonte        = 0.0;

// ============================================================================
// --- Protótipo das Funções ---
void Algoritmo_MPPT();


// ============================================================================
// ------------------- Interrupções -------------------

//---------Interrupção da máquina de estados------------


ISR(TIMER1_OVF_vect) 
{
  _delay_us(1);
}

//---------Interrupção do algoritmo MPPT---------------

ISR(TIMER2_OVF_vect) 
{
  _delay_us(1);
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
  TCCR0A  = 0b10100001;   //configura o PWM no modo Fast PWM (0xA3 em hexadecimal)
  TCCR0B  = 0b00000011;   //Pre Scaler 1:1, setando a frequência em 62.4kHz (0x01 em hexadecimal)
  OCR0A   = 0;            //dutty cicle do pino PWM PD5
  OCR0B   = 100;            //dutty cicle do pino PWM PD6


  //Configuração das Interrupções.
  cli();                  //desliga interrupções

  //Configuração das interrupções por estouro do TIMER1
  TCCR1B = 0b00000011;    //prescaler 1:64
  TIMSK1 = 0b00000001;    //habilita interrupção do Timer1
  TCNT1  = 60539;         //inicia Timer1 para contar a partir de 60539 e estabelecer uma frequencia de 25Hz

  //Configuração das interrupções por estouro do TIMER2
  TCCR2B = 0b00000110;    //prescaler 1:256
  TIMSK2 = 0b00000001;    //habilita interrupção do Timer2
  TCNT2  = 131;           //inicia Timer2 para contar a partir de 131 e estabelecer uma frequencia de 250Hz
  
  sei();



  Wire.begin();
  Serial.begin(115200);

  if(!adc.init())
  {
    Serial.println("ADS1115 não conectado!");
  }

  adc.setVoltageRange_mV(ADS1115_RANGE_4096);
  adc.setCompareChannels(ADS1115_COMP_0_GND);
  adc.setMeasureMode(ADS1115_CONTINUOUS);

  Serial.println("ADS1115 - Continuous Mode");
  Serial.println();

} //FIM DA FUNÇÃO PRINCIPAL



//---------- LOOP INFINITO ----------
void loop()
{ //INÍCIO DO LOOP INFINITO

  float voltage = 0.0;


  Tensao_fonte    = 0.0;
  voltage = readChannel(ADS1115_COMP_0_GND);
  Tensao_fonte = (voltage - 0.0029) * 250;       // calculo baseado na relação de 4mV/V na saída do sensor de tensão
  Serial.print(Tensao_fonte);
  Serial.print("V  ");
  
  
  Corrente_fonte    = 0.0;
  voltage = readChannel(ADS1115_COMP_1_GND);
  Corrente_fonte = (voltage - 1.6280) / 0.0187 ;       // calculo baseado na relação de 18.7mV/A na saída do sensor de corrente
  Serial.print(Corrente_fonte);
  Serial.print("A  ");

  Algoritmo_MPPT(Corrente_fonte, Tensao_fonte);

  OCR0A   = tensao_ref_fonte;
  
  Serial.println();

  _delay_ms(250);
} //FIM DO LOOP INFINITO





//===========================================================
//--------- Desenvolvimento das Funções ----------


float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}




void Algoritmo_MPPT(float corrente, float tensao) 
{
  //variaveis locais da função
  float   deltaV              = 0,
          Incializacao_MPPT   = 0,
          potencia_fonte      = 0;

  
  //Calculo do valor atual da potência a partir da variação da tensão 
  potencia_fonte = corrente * tensao;
  deltaV = tensao - tensao_anterior;

  //Serial.print(" |||| Potência: ");
  Serial.print(potencia_fonte);
  Serial.print("W  ");

  //Serial.print(" --- Potência anterior: ");
  //Serial.println(potencia_anterior);
  //Serial.print("W ");

  //Serial.print(" --- Tensão anterior: ");
  //Serial.println(tensao_anterior);
  //Serial.print("V ");

  //Serial.print(" Delta V: ");
  //Serial.println(deltaV);
  //Serial.println();
  //Serial.print("V ||||");


  if((tensao > (Tensao_circ_aberto)) && (Incializacao_MPPT == 0))
    {
      tensao_ref_fonte = tensao_ref_fonte - 5;
    }
  else
    {
      Incializacao_MPPT = 1;
    }
  
  //Algoritmo MPPT
  if(potencia_fonte > potencia_anterior)
    {
      if(deltaV > 0) tensao_ref_fonte += PASSO_MPPT;
      else tensao_ref_fonte -= PASSO_MPPT;
    }
  else if(potencia_fonte < potencia_anterior)
    {
      if(deltaV > 0) tensao_ref_fonte -= PASSO_MPPT;
      else tensao_ref_fonte += PASSO_MPPT;
    }

  Serial.print(tensao_ref_fonte);
  Serial.print("%  ");

  //Limites para estabelecer a saturação do volores máximos do MPPT
  if(tensao_ref_fonte > Tensao_maxima_fonte) tensao_ref_fonte = Tensao_maxima_fonte;
  else if(tensao_ref_fonte < Tensao_minima_fonte) tensao_ref_fonte = Tensao_minima_fonte;

  //Armazenamento dos volores presentes nas variaveis de valores do passado
  tensao_anterior = tensao;
  potencia_anterior = potencia_fonte;   // Armazena o valor de potência calculado anteriormente
  
}

