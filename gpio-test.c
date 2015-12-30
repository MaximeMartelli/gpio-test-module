#include <linux/module.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/fs.h>

// Sortie sur broche 18 (GPIO 24)
#define RPI_GPIO_OUT 24

// Entree sur broche 16 (GPIO 23)
#define RPI_GPIO_IN  23

static int __init gpio-test_init (void)
{
  printk(KERN_INFO "HELLO \n");
  gpio_set_value(RPI_GPIO_OUT, 1);
  
  return 0; 
}

static void __exit gpio-test_exit (void)
{
  gpio_free(RPI_GPIO_OUT);
  gpio_free(RPI_GPIO_IN);
}

module_init(gpio-test_init);
module_exit(gpio-test_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maxime MARTELLI");