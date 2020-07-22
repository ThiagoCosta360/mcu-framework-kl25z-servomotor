/*!
 * @copyright   © 2020 UFAM - Universidade Federal do Amazonas.
 *
 * @brief       Implementação da classe "mkl_ServoMotor180".
 *
 * @file        mkl_ServoMotor180.cpp
 * @version     1.0
 * @date        20 Janeiro 2020
 *
 * @section     HARDWARES & SOFTWARES
 *              +board        FRDM-KL25Z da NXP.
 *              +processor    MKL25Z128VLK4 - ARM Cortex-M0+.
 *              +compiler     Kinetis� Design Studio IDE.
 *              +manual       L25P80M48SF0RM, Rev.3, September 2012.
 *              +revisions    Versão (data): Descrição breve.
 *                             ++ 1.0 (20 Janeiro 2020): Versão inicial.
 *
 * @section     AUTHORS & DEVELOPERS
 *              +institution  Universidade Federal do Amazonas.
 *              +courses      Engenharia da Computação / Engenharia Elétrica.
 *              +teacher      Miguel Grimm <miguelgrimm@gmail.com>
 *              +student      Versão inicial:
 *                             ++ Thiago Costa Antunes Afonso <tcaa@icomp.ufam.edu.br>
 *
 * @section     LICENSE
 *
 *              GNU General Public License (GNU GPL).
 *
 *              Este programa é um software livre; Você pode redistribuí-lo
 *              e/ou modificálo de acordo com os termos do "GNU General Public
 *              License" como publicado pela Free Software Foundation; Seja a
 *              versão 3 da licença, ou qualquer versão posterior.
 *
 *              Este programa é distribuído na esperança de que seja útil,
 *              mas SEM QUALQUER GARANTIA; Sem a garantia implícita de
 *              COMERCIALIZA��O OU USO PARA UM DETERMINADO PROPÓSITO.
 *              Veja o site da "GNU General Public License" para mais detalhes.
 *
 * @htmlonly    http://www.gnu.org/copyleft/gpl.html
 */


#include "mkl_ServoMotor.h"
#include "mkl_TPMPulseWidthModulation.h"
#include "stdio.h"
#include "math.h"

/*!
 *   @fn         ServoMotor.
 *
 *   @brief      Construtor padrão da classe mkl_TPMPulseWidthModulation.
 *
 *   Este método é o construtor padrão da classe, que realiza a o controle
 *   periférico servo motor. O construtor inicia o motor na posição 0.
 *
 *   @param[in]  tpmPin - pino de saída do sinal PWM gerado para o motor.
 *
 */
    mkl_ServoMotor::mkl_ServoMotor(tpm_Pin tmpPin) : pwmServoMotor(tmpPin)
    {

		// Ajusta os valores default para um ServoMotor
		pulseWidthMin =  0.45;
		pulseWidthMax =  2.55;
		fullPeriod = 20;
		divBase = tpm_div8;
		modRegister = 52427;
		maxAngle = 180;

    	//Ajusta a Frequência do PWM para 50Hz ( T = 20ms )
    	pwmServoMotor.setFrequency(divBase, modRegister);

    	writeAngle(0);
    }

/*!
 *   @fn         turnOn.
 *
 *   @brief     Habilita o sinal PWM.
 *
 *   Este método é utilizado como interface ao enableOperation
 *   da classe mkl_TPMPulseWidthModulation
 *
 */
    void mkl_ServoMotor::turnOn()
    {
    	pwmServoMotor.enableOperation();
    }

/*!
 *   @fn         setPWM.
 *
 *   @brief     Configura o sinal PWM.
 *
 *   	Este método é utilizado para definir as configurações específicas do
 *   servo motor. 
 * 
 * 	 pwmMin é o tempo de alto do PWM em ms referente a posição 0 do servo.
 * 	 pwmMax é o tempo de alto do PWM em ms referente a posição angleMax do servo.
 *	 period é o período de atuação do PWM.
 *   angleMax é o angulo em graus que o motor abrange.
 * 
 */

	void mkl_ServoMotor::setPWM(double pwmMin, double pwmMax, double period, uint16_t angleMax){

		pulseWidthMin = pwmMin;
		pulseWidthMax = pwmMax;
		fullPeriod = period;
		maxAngle = angleMax;
		int counter = 0;

		double divider = (20971520/(1000/period));

		while ( ((divider/65536) > pow(2,counter)) && counter < 7){
			counter++;
		}
		divBase = (tpm_Div)counter;

		modRegister = (divider/ pow(2,counter));

		pwmServoMotor.setFrequency(divBase, modRegister);
		
	}

/*!
 *   @fn		turnOff.
 *
 *   @brief		Desabilita o sinal PWM.
 *
 *   Este método é utilizado como interface ao disableOperation
 *   da classe mkl_TPMPulseWidthModulation
 *
 */
    void mkl_ServoMotor::turnOff()
    {
    	pwmServoMotor.disableOperation();
    }

/*!
 *   @fn         writeAngle.
 *
 *   @brief      Ajusta o duty cycle do PWM com base no ângulo do motor.
 *
 *   Este método ajusta o duty cycle considerando a seguinte express�o 
 * 	que serve basicamente para mapear os valores de angulo que variam 
 * 	de 0 a maxAngle em valores de duty_cycle.
 *
 *   @param[in]  angle - Valor desejado para a posição relativa da haste do motor.
 *
 */
    void mkl_ServoMotor::writeAngle(int angle){
    	// duty = 1048 + ((7340 - 1048)* angle) / 180;
		double t0 = (pulseWidthMin*modRegister)/fullPeriod;
		double t180 = (pulseWidthMax*modRegister)/fullPeriod;

      	duty = t0 + (t180 - t0) * angle / maxAngle;

    	pwmServoMotor.setDutyCycle(duty);
    	turnOn();

    }
