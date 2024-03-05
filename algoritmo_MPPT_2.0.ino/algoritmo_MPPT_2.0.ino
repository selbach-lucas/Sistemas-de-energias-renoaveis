
//=====================================
//--------- frequência da CPU ---------
//#define F_CPU 16000000


//=====================================
//----------- Bibliotecas -------------
//#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <util/delay.h> 
#include <ADS1115_WE.h>
#include <Wire.h>



//=====================================
//------ Mapeamento de Hardware -------

#define   pwm1    (1<<PD5)
#define   pwm2    (1<<PD6)
#define   bt_1    (1<<PB1)
#define   I2C_ADDRESS 0x48
#define   cpl_bit(y, bit) (y^= (1<<bit))  //troca o estado lógico do bit x da variável y


//===========================================================
//----------- Variaveis Globais -------------

float     Potencia_fonte        = 0,
          Tensao_anterior       = 0,
          Tensao_circ_aberto    = 350,
          tensao_ref_fonte      = 0,
          Incializacao_MPPT     = 0,
          PASSO_MPPT            = 0,
          Tensao_maxima_fonte   = 400,
          Tensao_minima_fonte   = 100,
          Tensao_fonte          = 0.0,
          Corrente_fonte        = 0.0;

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

/*
int16_t val[4] = { 0, 0, 0, 0 };
volatile bool RDY = false;
//uint8_t channel = 0;

int SPS = 0;
uint32_t lastTime = 0;


// ============================================================================
// --- Protótipo das Funções ---
void Algoritmo_MPPT();

// ============================================================================
// ------------------- Interrupções -------------------

//---------Interrupção da máquina de estados------------

ISR(TIMER1_OVF_vect) 
{
  _delay_us(1);
  TCNT1  = 60539;
  cpl_bit(PORTD, PD4);
  //OCR0B++;
}

//---------Interrupção do algoritmo MPPT---------------

ISR(TIMER2_OVF_vect) 
{
  TCNT2  = 131;
  cpl_bit(PORTD, PD7);
}


*/


//=============================================================
//--------- Função principal ----------
int main(void)
{
  
  
  //define as saidas PWM
  DDRD  |=  pwm1;
  DDRD  |=  pwm2;

  //incializamos os bits do PWM em LOW
  PORTD  &=  ~pwm1;
  PORTD  &=  ~pwm2;

 
  //configura o PWM do timer0
  TCCR0A  = 0b10100011;   //configura o PWM no modo Fast PWM (0xA3 em hexadecimal)
  TCCR0B  = 0b00010001;   //Pre Scaler 1:1, setando a frequência em 62.5kHz (0x01 em hexadecimal)
  OCR0A   = 100;            //dutty cicle do pino PWM PD5
  OCR0B   = 100;            //dutty cicle do pino PWM PD6


  //Configuração das Interrupções.
  cli();                  //desliga interrupções

  //Configuração das interrupções por estouro do TIMER2
  TCCR2B = 0b00000110;    //prescaler 1:256
  TIMSK2 = 0b00000001;    //habilita interrupção do Timer2
  TCNT2  = 131;           //inicia Timer2 para contar a partir de 131 e estabelecer uma frequencia de 250Hz
  
  //Configuração das interrupções por estouro do TIMER1
  TCCR1B = 0b00000011;    //prescaler 1:64
  TIMSK1 = 0b00000001;    //habilita interrupção do Timer1
  TCNT1  = 60539;         //inicia Timer1 para contar a partir de 60539 e estabelecer uma frequencia de 25Hz
  
  sei(); 


  //Configurações do hardware de comunicação
  Wire.begin();
  //Wire.setClock(400000);
  Serial.begin(115200);


  //Configurações da comunicação e parâmetros de aquisição do conversor ADS1115
  if(!adc.init())
  {
    Serial.println("ADS1115 not connected!");
  }

  adc.setVoltageRange_mV(ADS1115_RANGE_4096); //Set the voltage range of the ADC to adjust the gain. ADS1115_RANGE_4096  ->  +/- 4096 mV
  adc.setCompareChannels(ADS1115_COMP_0_GND); //Set the inputs to be compared. ADS1115_COMP_0_GND  ->  compares 0 with GND
  adc.setMeasureMode(ADS1115_CONTINUOUS);     //Set the conversion rate in SPS (samples per second)
  
  Serial.println("ADS1115 Example Sketch - Continuous Mode");
  Serial.println("All values in volts");
  Serial.println();



  //---------- LOOP INFINITO ----------
  while(1)
  { //INÍCIO DO LOOP INFINITO
    
   
    float voltage;// = 0.0;

    /*
    //voltage1        = 0.0;
    Tensao_fonte    = 0.0;
    Serial.print("0: ");
    voltage1 = readChannel(ADS1115_COMP_0_GND);
    Tensao_fonte = (voltage1 - 0.0027) * 250;       // calculo baseado na relação de 4mV/V na saída do sensor de tensão
    Serial.print(Tensao_fonte);
    Serial.print("V  ");
    */
 
  
  
    //Tensao_fonte    = TENSAO_ENTRADA(ADS1115_COMP_0_GND);
    //Serial.println(Tensao_fonte);
    //Serial.print(" ");    
    //Serial.print("Tensão da fonte: ");
    //Serial.print(Tensao_fonte, 4);

    //Serial.print("V --- ");
  
    /*Corrente_fonte    = 0.0;
    Serial.print("1: ");
    voltage1 = readChannel(ADS1115_COMP_1_GND);
    Corrente_fonte = (voltage1 - 1.6210) / 0.0187 ;       // calculo baseado na relação de 18.7mV/A na saída do sensor de corrente
    Serial.print(Corrente_fonte);
    //Corrente_fonte  = CORRENTE_ENTRADA(ADS1115_COMP_1_GND);
    //Serial.print("   Corrente da fonte: ");
    //Serial.println(Corrente_fonte);
    Serial.print("A");
    */

    
    //Serial.print("\n ");
  

    Serial.print(",   0: ");
    voltage = readChannel(ADS1115_COMP_0_GND);
    Serial.print(voltage);

    Serial.print(",   1: ");
    voltage = readChannel(ADS1115_COMP_1_GND);
    Serial.print(voltage);
    
    Serial.print(",   2: ");
    voltage = readChannel(ADS1115_COMP_2_GND);
    Serial.print(voltage);

    Serial.print(",   3: ");
    voltage = readChannel(ADS1115_COMP_3_GND);
    Serial.println(voltage);

    //OCR0A++;
    _delay_ms(1000);

    //Algoritmo_MPPT(Corrente_fonte, Tensao_fonte);

  } //FIM DO LOOP INFINITO





return 0;

} //FIM DA FUNÇÃO PRINCIPAL






//===========================================================
//--------- Desenvolvimento das Funções ----------


float readChannel(ADS1115_MUX channel) {
  float vv = 0.0;
  adc.setCompareChannels(channel);
  vv = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  //Serial.print(vv);
  return vv;
}



void Algoritmo_MPPT(float Corrente_fonte, float Tensao_fonte) 
{
  //variaveis locais da função
  int16_t deltaV              = 0,
          potencia_anterior   = 0;

  potencia_anterior = Potencia_fonte;   // Armazena o valor de potência calculado anteriormente

  // Calculo do valor atual da potência a partir da variação da tensão 
  Potencia_fonte = Corrente_fonte * Tensao_fonte;
  deltaV = Tensao_fonte - Tensao_anterior;


  if((Tensao_fonte > (Tensao_circ_aberto - 1966)) && (Incializacao_MPPT == 0))
    {
      tensao_ref_fonte = tensao_ref_fonte - 200;
    }
  else
    {
      Incializacao_MPPT = 1;
    }
  
  //Algoritmo MPPT
  if(Potencia_fonte > potencia_anterior)
    {
      if(deltaV > 0) tensao_ref_fonte += PASSO_MPPT;
      else tensao_ref_fonte -= PASSO_MPPT;
    }
  else if(Potencia_fonte < potencia_anterior)
    {
      if(deltaV > 0) tensao_ref_fonte -= PASSO_MPPT;
      else tensao_ref_fonte += PASSO_MPPT;
    }

  //Limites para estabelecer a saturação do volores máximos do MPPT
  if(tensao_ref_fonte > Tensao_maxima_fonte) tensao_ref_fonte = Tensao_maxima_fonte;
  else if(tensao_ref_fonte < Tensao_minima_fonte) tensao_ref_fonte = Tensao_minima_fonte;

  // Armazenamento dos volores presentes nas variaveis de valores do passado
  potencia_anterior = Potencia_fonte;
  Tensao_anterior = Tensao_fonte;
  
}




