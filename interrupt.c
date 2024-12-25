/*
 * xyd_driver.c - 基于平台总线的LED和按键驱动
 *
 * 该驱动程序用于控制两个LED和一个按键
 * 通过按键中断来控制LED的亮灭状态
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/timer.h>

#define DRIVER_NAME     "xyd_driver"
#define DRIVER_VERSION  "1.0"
#define DEBOUNCE_DELAY  15 // 消抖延迟，单位为毫秒

/* 驱动设备结构体 */
struct xyd_dev {
    int led21_gpio;                     /* LED21的GPIO编号 */
    int led22_gpio;                     /* LED22的GPIO编号 */
    int key_gpio;                       /* 按键的GPIO编号 */
    int irq;                            /* 中断号 */
    bool led_state;                     /* LED状态 */
    bool initialized;                   /* 初始化完成标志 */
    struct device *dev;                 /* 设备结构体指针 */
    struct timer_list debounce_timer;   /* 定时器结构体 */
};

/* 全局设备结构体指针 */
static struct xyd_dev *xyd_device;



/**
 * xyd_key_isr - 软件定时回调服务程序
 *
 * 当按键触发中断时，切换LED状态
 */
static void xyd_debounce_timer_callback(struct timer_list *t)
{
    struct xyd_dev *dev = from_timer(dev, t, debounce_timer);

    int key_state;

    /* 读取按键状态 */
    key_state = gpio_get_value(dev->key_gpio);

    /* 检查按键是否仍然按下 */
    if (key_state == 0) { // 假设按键按下时GPIO状态为低电平
        /* 切换LED状态 */
        dev->led_state = !dev->led_state;

        /* 同时控制两个LED */
        gpio_set_value(dev->led21_gpio, dev->led_state);
        gpio_set_value(dev->led22_gpio, dev->led_state);
        dev_info(dev->dev, "LEDs state changed to %s\n", dev->led_state ? "ON" : "OFF");
    }

}


/**
 * xyd_key_isr - 按键中断服务程序
 * @irq: 中断号
 * @dev_id: 设备ID（这里是xyd_device结构体指针）
 *
 * 当按键触发中断时，切换LED状态
 */
static irqreturn_t xyd_key_isr(int irq, void *dev_id)
{
    struct xyd_dev *dev = (struct xyd_dev *)dev_id;
    
    /* 确保所有设备都已初始化 */
    if (!dev->initialized)
        return IRQ_HANDLED;
    
    /* 重新启动定时器，消抖处理 */
    mod_timer(&dev->debounce_timer, jiffies + msecs_to_jiffies(DEBOUNCE_DELAY));

    return IRQ_HANDLED;
}

/**
 * xyd_init_led21 - 初始化LED21
 * @dev: 平台设备结构体指针
 * @xyd: 驱动设备结构体指针
 *
 * 返回: 成功返回0，失败返回负错误码
 */
static int xyd_init_led21(struct device *dev, struct xyd_dev *xyd)
{
    int ret;

    /* 获取LED21的GPIO */
    xyd->led21_gpio = of_get_named_gpio(dev->of_node, "led21_gpios", 0);
    if (!gpio_is_valid(xyd->led21_gpio)) {
        dev_err(dev, "Invalid LED21 GPIO\n");
        return -EINVAL;
    }

    /* 申请GPIO资源 */
    ret = devm_gpio_request(dev, xyd->led21_gpio, "led21");
    if (ret) {
        dev_err(dev, "Failed to request LED21 GPIO: %d\n", ret);
        return ret;
    }

    /* 设置GPIO为输出，初始状态为低电平 */
    gpio_direction_output(xyd->led21_gpio, 0);
    dev_info(dev, "LED21 GPIO initialized\n");

    return 0;
}

/**
 * xyd_init_led22 - 初始化LED22
 * @dev: 平台设备结构体指针
 * @xyd: 驱动设备结构体指针
 *
 * 返回: 成功返回0，失败返回负错误码
 */
static int xyd_init_led22(struct device *dev, struct xyd_dev *xyd)
{
    int ret;

    /* 获取LED22的GPIO */
    xyd->led22_gpio = of_get_named_gpio(dev->of_node, "led22_gpios", 0);
    if (!gpio_is_valid(xyd->led22_gpio)) {
        dev_err(dev, "Invalid LED22 GPIO\n");
        return -EINVAL;
    }

    /* 申请GPIO资源 */
    ret = devm_gpio_request(dev, xyd->led22_gpio, "led22");
    if (ret) {
        dev_err(dev, "Failed to request LED22 GPIO: %d\n", ret);
        return ret;
    }

    /* 设置GPIO为输出，初始状态为低电平 */
    gpio_direction_output(xyd->led22_gpio, 0);
    dev_info(dev, "LED22 GPIO initialized\n");

    return 0;
}

/**
 * xyd_init_key - 初始化按键和中断
 * @dev: 平台设备结构体指针
 * @xyd: 驱动设备结构体指针
 *
 * 返回: 成功返回0，失败返回负错误码
 */
static int xyd_init_key(struct device *dev, struct xyd_dev *xyd)
{
    int ret;

    /* 获取按键的GPIO */
    xyd->key_gpio = of_get_named_gpio(dev->of_node, "key_gpios", 0);
    if (!gpio_is_valid(xyd->key_gpio)) {
        dev_err(dev, "Invalid KEY GPIO\n");
        return -EINVAL;
    }

    /* 申请GPIO资源 */
    ret = devm_gpio_request(dev, xyd->key_gpio, "key");
    if (ret) {
        dev_err(dev, "Failed to request KEY GPIO: %d\n", ret);
        return ret;
    }

    /* 设置GPIO为输入 */
    gpio_direction_input(xyd->key_gpio);

    /* 获取中断号 */
    xyd->irq = gpio_to_irq(xyd->key_gpio);
    if (xyd->irq < 0) {
        dev_err(dev, "Failed to get IRQ number: %d\n", xyd->irq);
        return xyd->irq;
    }

    /* 申请中断 */
    ret = devm_request_irq(dev, xyd->irq, xyd_key_isr,
                          IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                          "xyd_key", xyd);
    if (ret) {
        dev_err(dev, "Failed to request IRQ: %d\n", ret);
        return ret;
    }

    timer_setup(&(xyd->debounce_timer), xyd_debounce_timer_callback, 0);

    dev_info(dev, "KEY GPIO and IRQ initialized\n");
    return 0;
}

/**
 * xyd_probe - 驱动探测函数
 * @pdev: 平台设备结构体指针
 *
 * 返回: 成功返回0，失败返回负错误码
 */
static int xyd_probe(struct platform_device *pdev)
{
    struct device *dev = &(pdev->dev);
    int ret = 0;

    /* 首次probe时分配设备结构体 */
    if (!xyd_device) {
        xyd_device = devm_kzalloc(dev, sizeof(*xyd_device), GFP_KERNEL);
        if (!xyd_device)
            return -ENOMEM;
        
        xyd_device->initialized = false;
        xyd_device->dev = dev;
    }

    /* 根据设备树节点的compatible属性初始化相应设备 */
    if (of_device_is_compatible(dev->of_node, "xyd-led21")) {
        ret = xyd_init_led21(dev, xyd_device);
    } else if (of_device_is_compatible(dev->of_node, "xyd-led22")) {
        ret = xyd_init_led22(dev, xyd_device);
    } else if (of_device_is_compatible(dev->of_node, "xyd_key")) {
        ret = xyd_init_key(dev, xyd_device);
    }

    if (ret) {
        dev_err(dev, "Device initialization failed\n");
        return ret;
    }

    /* 检查所有设备是否都已初始化 */
    if (gpio_is_valid(xyd_device->led21_gpio) &&
        gpio_is_valid(xyd_device->led22_gpio) &&
        gpio_is_valid(xyd_device->key_gpio)) {
        xyd_device->initialized = true;
        dev_info(dev, "All devices initialized successfully\n");
    }

    return 0;
}

/**
 * xyd_remove - 驱动移除函数
 * @pdev: 平台设备结构体指针
 *
 * 返回: 始终返回0
 */
static int xyd_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    
    /* 如果是按键设备，清除初始化标志 */
    if (of_device_is_compatible(dev->of_node, "xyd_key")) {
        xyd_device->initialized = false;
    }

    del_timer_sync(&xyd_device->debounce_timer);
    
    dev_info(dev, "Device removed\n");
    return 0;
}

/* 设备树匹配表 */
static const struct of_device_id xyd_of_match[] = {
    { .compatible = "xyd-led21" },
    { .compatible = "xyd-led22" },
    { .compatible = "xyd_key" },
    { },
};
MODULE_DEVICE_TABLE(of, xyd_of_match);

/* 平台驱动结构体 */
static struct platform_driver xyd_driver = {
    .probe  = xyd_probe,
    .remove = xyd_remove,
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = xyd_of_match,
    },
};

/**
 * xyd_init - 模块初始化函数
 */
static int __init xyd_init(void)
{
    pr_info("XYD driver initializing...\n");
    return platform_driver_register(&xyd_driver);
}

/**
 * xyd_exit - 模块退出函数
 */
static void __exit xyd_exit(void)
{
    pr_info("XYD driver exiting...\n");
    platform_driver_unregister(&xyd_driver);
}

/* 模块入口和出口 */
module_init(xyd_init);
module_exit(xyd_exit);

/* 模块信息 */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("XYD LED and Key driver");
MODULE_VERSION(DRIVER_VERSION);

