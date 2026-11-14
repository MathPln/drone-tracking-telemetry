#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
    printk("[Info]: Main thread started\n");

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}