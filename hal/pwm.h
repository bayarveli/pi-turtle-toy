// PWM control for Raspberry Pi using memory-mapped registers.

#ifndef HAL_PWM_H_
#define HAL_PWM_H_

class MemoryMappedPin;  // Forward declaration

class PWM {
 public:
  // Constructors accepting pre-configured MemoryMappedPin for PWM channels
  PWM(MemoryMappedPin& channel0_pin, MemoryMappedPin& channel1_pin);
  PWM(MemoryMappedPin& channel0_pin, MemoryMappedPin& channel1_pin, 
      double d_Hz, unsigned int u_i_counts, double d_duty, int i_m);
  ~PWM();

  // Non-copyable, non-movable (owns hardware resources)
  PWM(const PWM&) = delete;
  PWM& operator=(const PWM&) = delete;
  PWM(PWM&&) = delete;
  PWM& operator=(PWM&&) = delete;

  unsigned int set_frequency(const double& hz);
  unsigned int set_counts(const unsigned int& counts);
  unsigned int set_duty_cycle(const double& duty_percent, int channel);
  unsigned int set_duty_cycle_count(const unsigned int& count_value, int channel);
  unsigned int set_duty_cycle_force(const double& duty_percent, const int& mode, int channel);
  unsigned int set_mode(const int& mode);

  double frequency() const;
  double duty_cycle() const;
  int counts() const;
  int divisor() const;
  int mode() const;

  // Public constants
  static const int PWMMODE = 1; // PWM mode
  static const int MSMODE = 2;  // Mark-Space mode

  // Error Codes
  static const int ERRFREQ = 1;
  static const int ERRCOUNT = 2;
  static const int ERRDUTY = 3;
  static const int ERRMODE = 4;

 private:
  // Hardware register addresses (private implementation detail)
  static const int BCM2708_PERI_BASE = 0x3F000000; // 0x20000000 on older Pi
  static const int PWM_BASE = (BCM2708_PERI_BASE + 0x20C000); /* PWM controller */
  static const int CLOCK_BASE = (BCM2708_PERI_BASE + 0x101000); /* Clock controller */
  static const int PWM_CTL = 0;
  static const int PWM_RNG1 = 4;
  static const int PWM_DAT1 = 5;
  static const int PWM_RNG2 = 8;
  static const int PWM_DAT2 = 9;
  static const int PWMCLK_CNTL = 40;
  static const int PWMCLK_DIV = 41;
  // Register addresses offsets divided by 4 (register addresses are word (32-bit) aligned)
  static const int BLOCK_SIZE = 4096; // each mmap() maps 4KB

  volatile unsigned* map_register_address(unsigned long base_address);
  void config_pwm();

  MemoryMappedPin* channel0_pin_;  // Injected pins for PWM channels
  MemoryMappedPin* channel1_pin_;
  double d_frequency;          // PWM frequency
  double d_dutyCycle;          // PWM duty Cycle (%)
  unsigned int u_i_counts;     // PWM resolution
  unsigned int u_i_divisor;    // divisor value
  int i_mode;                  // PWM mode
  // pointers to the memory mapped sections of our process memory
  volatile unsigned* p_v_u_clk;
  volatile unsigned* p_v_u_pwm;
};

#endif /* HAL_PWM_H_ */
