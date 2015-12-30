#include <linux/module.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/fs.h>

// Sortie sur broche 18 (GPIO 24)
#define RPI_GPIO_OUT 24

// Entree sur broche 16 (GPIO 23)
#define RPI_GPIO_IN  23

static struct timer_list gpio_test_timer;

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