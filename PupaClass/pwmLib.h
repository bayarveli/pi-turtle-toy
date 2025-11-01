/*
 * PWM.h
 *
 *  Created on: Aug 13, 2015
 *      Author: veli
 */

#ifndef PWM_H_
#define PWM_H_

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

class PWM
{
	public:

	PWM();

	PWM(double d_Hz, unsigned int u_i_counts, double d_duty, int i_m);

	~PWM();

	unsigned int SetFrequency(const double & c_d_hz);

	unsigned int SetCounts(const unsigned int & c_u_i_counts);

	unsigned int SetDutyCycle(const double &c_d_duty, int i_pwm_no);

	unsigned int SetDutyCycleCount(const unsigned int &c_u_i_counts, int i_pwm_no);

	unsigned int SetDutyCycleForce(const double &c_d_duty, const int &c_i_m, int i_pwm_no);

	unsigned int SetMode(const int &c_i_m);

	double GetFrequency() const;

	double GetDutyCycle() const;

	int GetCounts() const;

	int GetDivisor() const;

	int GetMode() const;

	//Public constants
	// PWM MODES
	static const int PWMMODE = 1;
	static const int MSMODE = 2;
	// Error Codes
	static const int ERRFREQ = 1;
	static const int ERRCOUNT = 2;
	static const int ERRDUTY = 3;
	static const int ERRMODE = 4;

	//Private constants
	//Base register addresses
	static const int BCM2708_PERI_BASE = 0x3F000000;//0x20000000;
	static const int PWM_BASE = (BCM2708_PERI_BASE + 0x20C000); /* PWM controller */
	static const int CLOCK_BASE = (BCM2708_PERI_BASE + 0x101000); /* Clock controller */
	static const int GPIO_BASE = (BCM2708_PERI_BASE + 0x200000); /* GPIO controller */
	static const int PWM_CTL = 0;
	static const int PWM_RNG1 = 4;
	static const int PWM_DAT1 = 5;
	static const int PWM_RNG2 = 8;
	static const int PWM_DAT2 = 9;
	static const int PWMCLK_CNTL= 40;
	static const int PWMCLK_DIV = 41;
	// Register addresses offsets divided by 4 (register addresses are word (32-bit) aligned
	static const int BLOCK_SIZE = 4096;
	// Block size.....every time mmap() is called a 4KB
	//section of real physical memory is mapped into the memory of
	//the process

	volatile unsigned *MapRegisterAddres(unsigned long u_l_base_address);

	void ConfigPWMPin();

	void ConfigPWM();

	double d_frequency; // PWM frequency
	double d_dutyCycle; //PWM duty Cycle (%)
	unsigned int u_i_counts; // PWM resolution
	unsigned int u_i_divisor; // divisor value
	int i_mode; // PWM mode
	volatile unsigned *p_v_u_clk, *p_v_u_pwm, *p_v_u_gpio; // pointers to the memory mapped sections
	//of our process memory
};

#endif /* PWM_H_ */
