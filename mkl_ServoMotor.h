/*!
 * @copyright   © 2020 UFAM - Universidade Federal do Amazonas.
 *
 * @brief       Implementação da classe "mkl_ServoMotor".
 *
 * @file        mkl_ServoMotor.cpp
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
 *              COMERCIALIZAÇÃO OU USO PARA UM DETERMINADO PROPÓSITO.
 *              Veja o site da "GNU General Public License" para mais detalhes.
 *
 * @htmlonly    http://www.gnu.org/copyleft/gpl.html
 */

#ifndef MKL_SERVOMOTOR_H_
#define MKL_SERVOMOTOR_H_

#include "mkl_TPMPulseWidthModulation.h"


/*!
 *  @class    mkl_ServoMotor180.
 *
 *  @brief    A classe permite a utilização do servo motor.
 *
 *  @details  Esta classe implementa o serviço PWM utilizando os
 *            periféricos TPM0, TPM1 ou TPM2 e os pinos correspondentes e
 *            herdando da classe m�e "mkl_TPM".
 *
 *  @section  EXAMPLES USAGE
 *
 *            Uso dos métodos para utilização do servo motor.
 *
 *              +fn turnOn();
 *              +fn writeAngle(90);
 *              +fn turnOff();
 */

class mkl_ServoMotor {

	private:
		mkl_TPMPulseWidthModulation pwmServoMotor;

		int duty;

		double pulseWidthMin;

		double pulseWidthMax;

		double fullPeriod;

		tpm_Div divBase;

		uint16_t modRegister;

		uint16_t maxAngle;

	public:

		  /*!
		   * Método construtor padrão da classe.
		   */
		explicit mkl_ServoMotor(tpm_Pin tmpPin);
		  
		  /*!
		   * Método para definir as especificações do motor.
		   */
		void setPWM(double pwmMin, double pwmMax, double period, uint16_t angleMax);

		  /*!
		   * Método de habilitar a operação PWM.
		   */
		void turnOn();

		  /*!
		   * Método de disabilitar a operação PWM.
		   */
		void turnOff();

		  /*!
		   * Método de seleção do ângulo relativo do motor.
		   */
		void writeAngle(int newAngle);
};

#endif  //  MKL_SERVOMOTOR_H_
