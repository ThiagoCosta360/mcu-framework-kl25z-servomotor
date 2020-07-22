/*!
 * @copyright   � 2017 UFAM - Universidade Federal do Amazonas.
 *
 * @brief       Exemplo de uso da classe mkl_ServoMotor180.
 *
 * @example     main.cpp
 * @version     1.0
 * @date        20 Janeiro 2020
 *
 * @section     HARDWARES & SOFTWARES
 *              +board        FRDM-KL25Z da NXP.
 *              +processor    MKL25Z128VLK4 - ARM Cortex-M0+
 *              +compiler     Kinetis Design Studio IDE
 *              +manual       L25P80M48SF0RM, Rev.3, September 2012
 *
 * @section     AUTHORS & DEVELOPERS
 *              +institution  Universidade Federal do Amazonas
 *              +courses      Engenharia da Computação / Engenharia Elétrica
 *              +teacher      Miguel Grimm <miguelgrimm@gmail.com>
 *              +student      Versão inicial:
 *                             ++ Thiago Costa Antunes Afonso <tcaa@icomp.ufam.edu.br>
 *
 * @section     LICENSE
 *
 *              GNU General Public License (GNU GPL).
 *
 *              Este programa é um software livre; Você pode redistribuí-lo
 *              e/ou modificá-lo de acordo com os termos do "GNU General Public
 *              License" como publicado pela Free Software Foundation; Seja a
 *              versão 3 da licença, ou qualquer versão posterior.
 *
 *              Este programa é distribudo na esperança de que seja útil,
 *              mas SEM QUALQUER GARANTIA; Sem a garantia implícita de
 *              COMERCIALIZAÇÃO OU USO PARA UM DETERMINADO PROPÓSITO.
 *              Veja o site da "GNU General Public License" para mais detalhes.
 *
 * @htmlonly    http://www.gnu.org/copyleft/gpl.html
 */

#include "mkl_TPMPulseWidthModulation.h"
#include "mkl_ServoMotor.h"
#include "MKL25Z4.h"

/*!
 * Objeto para controle do servo via sinal PWM no pino PT41.
 */
mkl_ServoMotor servoMotor(tpm_PTD4);

/*!
 *   @fn         delayMs.
 *
 *   @brief      Implementa delay de código de cycle_number milissegundos.
 *
 *   @param[in]  time - tempo em milissegundos de delay desejado.
 *
 */
void delayMs(int time) {
  int i;
  int j;

  for (i = 0 ; i < time; i++) {
    for (j = 0; j < 7000; j++) {}
  }
}

/*!
 *   @fn       main.
 *
 *   @brief    Implementa o programa exemplo do TPM no modo PWM.
 *
 *   Este programa realiza o teste da classe do periférico servo motor
 *   180 graus mudando o ângulo do servo entre 0 e 180 graus a cada 150 ms.
 *
 *   @return  Sempre retorna 0.
 */


int main(void) {
 	int angle = 0;

	servoMotor.turnOn();
	servoMotor.setPWM(0.45 , 2.55, 20, 180); //Valores Default

	while (true)
	{
    	servoMotor.writeAngle(angle);
		delayMs(150);
    	angle += 20;
    	if (angle >= 180)
			angle = 0;

	}
	servoMotor.turnOff();

	return 0;
}
