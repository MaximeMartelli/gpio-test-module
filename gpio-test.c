#include <linux/module.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/fs.h>

// Sortie sur broche 12 (GPIO 18)
#define RPI_GPIO_OUT 18

// GPIO sur broche 16 (GPIO 23)
#define RPI_GPIO_1  23
#define RPI_GPIO_2  24
#define RPI_GPIO_3  27
#define RPI_GPIO_4  22


#define BCM2708_PERI_BASE 	0x20000000
#define GPIO_BASE 			(BCM2708_PERI_BASE + 0x200000) /* Controleur GPIO*/
#define PWM_BASE 			(BCM2708_PERI_BASE + 0x20C000) /* Controleur PWM*/
#define CLOCK_BASE 			(BCM2708_PERI_BASE + 0x101000) 

#define PWM_CTL		0x0
#define PWM_RNG1	0x10
#define PWM_DAT1	0x14

#define PWMCLK_CNTL	40
#define PWMCLK_DIV	41


/*int minor = MINOR(inode->i_redev);
int major = MAJOR(inode->i_redev);*/


static int gpio_test_init (struct inode *inode, struct file *file);
static int gpio_test_exit (struct inode *inode, struct file *file);
static ssize_t gpio_test_read (struct file *file, const char *buf, size_t count, loff_t *ppos);
static ssize_t gpio_test_write (struct file *file, const char *buf, size_t count, loff_t *ppos);


/* File operation structure */
static struct file_operations fops = {
	.open = gpio_test_init, 
	.release = gpio_test_exit, 
	.read = gpio_test_read, 
	.write = gpio_test_write,
};



static int gpio_test_init (struct inode *inode, struct file *file)
{
  int err;
  
  int ret;
  ret = register_chrdev(major, "gpio-test",&fops);

  if ((err = gpio_request(RPI_GPIO_1,THIS_MODULE->name)) != 0)
    return err;
  if ((err = gpio_request(RPI_GPIO_2,THIS_MODULE->name)) != 0) {
    gpio_free(RPI_GPIO_1);
    return err;
  }
  if ((err = gpio_request(RPI_GPIO_3,THIS_MODULE->name)) != 0) {
    gpio_free(RPI_GPIO_1);
    gpio_free(RPI_GPIO_2);
    return err;
  }
  if ((err = gpio_request(RPI_GPIO_4,THIS_MODULE->name)) != 0) {
    gpio_free(RPI_GPIO_1);
    gpio_free(RPI_GPIO_2);
    gpio_free(RPI_GPIO_3);
    return err;
  }
  
  printk("Driver perso ajouté\n");

  return 0; 
}

static int gpio_test_exit (struct inode *inode, struct file *file)
{
    gpio_free(RPI_GPIO_1);
    gpio_free(RPI_GPIO_2);
    gpio_free(RPI_GPIO_3);
    gpio_free(RPI_GPIO_4);

	unregister_chrdev(major, "gpio-test");

  	printk("Driver perso supprimé\n");

    return 0;
}

static ssize_t gpio_test_read (struct file *file, const char *buf, size_t count, loff_t *ppos)
{


	printk("Driver read\n");
	return 0;
}

static ssize_t gpio_test_write (struct file *file, const char *buf, size_t count, loff_t *ppos)
{


	printk("Driver write\n");
	return 0;
}



module_init(gpio_test_init);
module_exit(gpio_test_exit);
MODULE_LICENSE("GPL");