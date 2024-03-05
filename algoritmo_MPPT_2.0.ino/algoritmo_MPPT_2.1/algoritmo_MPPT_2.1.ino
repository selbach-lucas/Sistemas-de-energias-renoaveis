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

#define   pwm3    (1<<PB2)
#define   pwm4    (1<<PB3)

#define I2C_ADDRESS 0x48
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

//===========================================================
//----------- Variaveis Globais -------------

float     tensao_anterior       = 0,
          potencia_anterior     = 0,
          PASSO_MPPT            = 1,
          Tensao_circ_aberto    = 256,
          tensao_ref_fonte      = 0,  //dutty de 50% do PWM com resolução de 8bits
          Tensao_maxima_fonte   = 360.0,
          Tensao_minima_fonte   = 90.0,
          Tensao_fonte          = 0.0,
          Corrente_fonte        = 0.0;

// ============================================================================
// --- Protótipo das Funções ---
void Algoritmo_MPPT();


// ============================================================================
// ------------------- Interrupções -------------------

//---------Interrupção da máquina de estados------------



//---------Interrupção do algoritmo MPPT---------------





//=============================================================
//--------- Função principal ----------
void setup()
{



  Wire.begin();
  Serial.begin(115200);

  if(!adc.init())
  {
    Serial.println("ADS1115 not connected!");
  }

  adc.setVoltageRange_mV(ADS1115_RANGE_4096);
  adc.setCompareChannels(ADS1115_COMP_0_GND);
  adc.setMeasureMode(ADS1115_CONTINUOUS);

  Serial.println("ADS1115 Example Sketch - Continuous Mode");
  Serial.println("All values in volts");
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

