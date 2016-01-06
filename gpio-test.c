#include <linux/module.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/fs.h>

// Sortie sur broche 12 (GPIO 18)
#define RPI_GPIO_OUT 18

// Entree sur broche 16 (GPIO 23)
#define RPI_GPIO_IN  23

#define BCM2708_PERI_BASE 	0x20000000
#define GPIO_BASE 			(BCM2708_PERI_BASE + 0x200000) /* Controleur GPIO*/
#define PWM_BASE 			(BCM2708_PERI_BASE + 0x20C000) /* Controleur PWM*/
#define CLOCK_BASE 			(BCM2708_PERI_BASE + 0x101000) 

#define PWM_CTL		0x0
#define PWM_RNG1	0x10
#define PWM_DAT1	0x14

#define PWMCLK_CNTL	40
#define PWMCLK_DIV	41

static struct timer_list gpio_test_timer;




static void setServo(int percent)
{
	int bitCount;
	unsigned int bits = 0;

	// 32 bits = 2 milliseconds
	bitCount = 16 + 16 * percent / 100;
	if (bitCount > 32) bitCount = 32;
	if (bitCount < 1) bitCount = 1;
	bits = 0;
	while (bitCount) {
		bits <<= 1;
		bits |= 1;
		bitCount--;
	}
	*(pwm + PWM_DAT1) = bits;
}



static void gpio_test_function (unsigned long unused)
{
  static int value = 1;
  value = 1 - value;
  if (gpio_get_value(RPI_GPIO_IN) == 0)
    value = 0;
  gpio_set_value(RPI_GPIO_OUT, value);
  mod_timer(& gpio_test_timer, jiffies+ (HZ >> 3));
}

static int __init gpio_test_init (void)
{
  int err;

  if ((err = gpio_request(RPI_GPIO_IN,THIS_MODULE->name)) != 0)
    return err;
  if ((err = gpio_request(RPI_GPIO_OUT,THIS_MODULE->name)) != 0) {
    gpio_free(RPI_GPIO_IN);
    return err;
  }
  if ((err = gpio_direction_input(RPI_GPIO_IN)) != 0) {
    gpio_free(RPI_GPIO_OUT);
    gpio_free(RPI_GPIO_IN);
    return err;
  }
  if ((err = gpio_direction_output(RPI_GPIO_OUT,1)) != 0) {
    gpio_free(RPI_GPIO_OUT);
    gpio_free(RPI_GPIO_IN);
    return err;
  }
    
  init_timer(& gpio_test_timer);
  gpio_test_timer.function = gpio_test_function;
  gpio_test_timer.data = 0; // non utilise
  gpio_test_timer.expires = jiffies + (HZ >> 3);
  add_timer(& gpio_test_timer);
  
  /*
  // Active le mode PWM pour le pin 18
  SET_GPIO_ALT(18,5);
  //GPFSEL1 = 010; Bit 26-24, décalage de 24, y mettre 010 (fonction 5), correspondant à 2 << 24
  // GPFSEL1 est à l'adresse 0x7E200004
  
  // Kill clock
  *(clk + PWMCLK_CNTL) = 0x5A000000 | (1 << 5);
  usleep(10);

	// set frequency
	// DIVI is the integer part of the divisor
	// the fractional part (DIVF) drops clock cycles to get the output frequency, bad for servo motors
	// 320 bits for one cycle of 20 milliseconds = 62.5 us per bit = 16 kHz
	int idiv = (int) (19200000.0f / 16000.0f);
	if (idiv < 1 || idiv > 0x1000) {
		printf("idiv out of range: %x\n", idiv);
		exit(-1);
	}
	*(clk + PWMCLK_DIV)  = 0x5A000000 | (idiv<<12);
	
	// source=osc and enable clock
	*(clk + PWMCLK_CNTL) = 0x5A000011;

	// disable PWM
	*(pwm + PWM_CTL) = 0;
	
	// needs some time until the PWM module gets disabled, without the delay the PWM module crashs
	usleep(10);  
	
	// filled with 0 for 20 milliseconds = 320 bits
	*(pwm + PWM_RNG1) = 320;
	
	// 32 bits = 2 milliseconds, init with 1 millisecond
	setServo(0);
	
	// start PWM1 in serializer mode
	*(pwm + PWM_CTL) = 3;*/

  return 0; 
}

static void __exit gpio_test_exit (void)
{
  del_timer(& gpio_test_timer);
  gpio_free(RPI_GPIO_OUT);
  gpio_free(RPI_GPIO_IN);
}

module_init(gpio_test_init);
module_exit(gpio_test_exit);
MODULE_LICENSE("GPL");