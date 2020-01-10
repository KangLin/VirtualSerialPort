
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/amba/bus.h>
#include <linux/amba/serial.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/tty_flip.h>
#include <linux/circ_buf.h>

#include <asm/io.h>
#include <linux/platform_device.h>

#define drintk printk

#define virtual_SERIAL_NR         4

typedef     char                CHAR;
typedef     unsigned char       BYTE;
typedef     short               SHORT;
typedef     unsigned short      WORD;
typedef     long                LONG;
typedef     unsigned long       DWORD;
typedef     int                 INT;

/****************************************************************************************************
struct uart_ops {                                                                                 
  unsigned int    (*tx_empty)(struct uart_port *);                                                
  void        (*set_mctrl)(struct uart_port *, unsigned int mctrl);                               
  unsigned int    (*get_mctrl)(struct uart_port *);                                               
  void        (*stop_tx)(struct uart_port *);                                                     
  void        (*start_tx)(struct uart_port *);                                                    
  void        (*send_xchar)(struct uart_port *, char ch);                                         
  void        (*stop_rx)(struct uart_port *);                                                     
  void        (*enable_ms)(struct uart_port *);                                                   
  void        (*break_ctl)(struct uart_port *, int ctl);                                          
  int     (*startup)(struct uart_port *);                                                         
  void        (*shutdown)(struct uart_port *);                                                    
  void        (*flush_buffer)(struct uart_port *);                                                
  void        (*set_termios)(struct uart_port *, struct ktermios *new,                            
                     struct ktermios *old);                                                       
  void        (*set_ldisc)(struct uart_port *);                                                   
  void        (*pm)(struct uart_port *, unsigned int state,                                       
                unsigned int oldstate);                                                           
  int     (*set_wake)(struct uart_port *, unsigned int state);                                    
                                                                                                  
  // Return a string describing the type of the port

  const char *(*type)(struct uart_port *);                                                       

  // Release IO and memory resources used by the port.                                         
  // This includes iounmap if necessary.                                                       

  void        (*release_port)(struct uart_port *);                                               
                                             
  // Request IO and memory resources used by the port.                                         
  // This includes iomapping the port if necessary.                                            
                                                                                               
  int     (*request_port)(struct uart_port *);                                                   
  void        (*config_port)(struct uart_port *, int);                                           
  int     (*verify_port)(struct uart_port *, struct serial_struct *);                            
  int     (*ioctl)(struct uart_port *, unsigned int, unsigned long);                              
#ifdef CONFIG_CONSOLE_POLL                                                                        
  void    (*poll_put_char)(struct uart_port *, unsigned char);                                    
  int     (*poll_get_char)(struct uart_port *);                                                   
#endif                                                                                            
***************************************************************************************************/

static unsigned int virtual_tx_empty(struct uart_port *port);
static void virtual_set_mctrl(struct uart_port *port, unsigned int mctrl);
static unsigned int virtual_get_mctrl(struct uart_port *port);
static void virtual_stop_tx(struct uart_port *port);
static void virtual_start_tx(struct uart_port *port);
static void virtual_stop_rx(struct uart_port *port);
static void virtual_enable_ms(struct uart_port *port);
static void virtual_break_ctl(struct uart_port *port, int break_state);
static int virtual_startup(struct uart_port *port);
static void virtual_shutdown(struct uart_port *port);
static void virtual_flush_buffer(struct uart_port *port);
static void
virtual_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old);
static const char *virtual_type(struct uart_port *port);
static void virtual_release_port(struct uart_port *port);
static int virtual_request_port(struct uart_port *port);
static void virtual_config_port(struct uart_port *port, int flags);
static int virtual_verify_port(struct uart_port *port, struct serial_struct *ser);

/****************************************************************************************************/
/*struct file_operations {                                                                          */
/*struct module *owner;                                                                             */
/*loff_t (*llseek) (struct file *, loff_t, int);                                                    */
/*ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);                                 */
/*ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);                          */
/*ssize_t (*aio_read) (struct kiocb *, const struct iovec *, unsigned long, loff_t);                */
/*ssize_t (*aio_write) (struct kiocb *, const struct iovec *, unsigned long, loff_t);               */
/*int (*readdir) (struct file *, void *, filldir_t);                                                */
/*unsigned int (*poll) (struct file *, struct poll_table_struct *);                                 */
/*int (*ioctl) (struct inode *, struct file *, unsigned int, unsigned long);                        */
/*long (*unlocked_ioctl) (struct file *, unsigned int, unsigned long);                              */
/*long (*compat_ioctl) (struct file *, unsigned int, unsigned long);                                */
/*int (*mmap) (struct file *, struct vm_area_struct *);                                             */
/*int (*open) (struct inode *, struct file *);                                                      */
/*int (*flush) (struct file *, fl_owner_t id);                                                      */
/*int (*release) (struct inode *, struct file *);                                                   */
/*int (*fsync) (struct file *, struct dentry *, int datasync);                                      */
/*int (*aio_fsync) (struct kiocb *, int datasync);                                                  */
/*int (*fasync) (int, struct file *, int);                                                          */
/*int (*lock) (struct file *, int, struct file_lock *);                                             */
/*ssize_t (*sendpage) (struct file *, struct page *, int, size_t, loff_t *, int);                   */
/*unsigned long (*get_unmapped_area)(struct file *, unsigned long, unsigned long, unsigned long, unsigned long);*/
/*int (*check_flags)(int);                                                                          */
/*int (*flock) (struct file *, int, struct file_lock *);                                            */
/*ssize_t (*splice_write)(struct pipe_inode_info *, struct file *, loff_t *, size_t, unsigned int); */
/*ssize_t (*splice_read)(struct file *, loff_t *, struct pipe_inode_info *, size_t, unsigned int);  */
/*int (*setlease)(struct file *, long, struct file_lock **);                                        */
/*}                                                                                                 */
/****************************************************************************************************/

int virtual_open(struct inode *i, struct file *file);
int virtual_release(struct inode *i, struct file *file);
ssize_t virtual_read(struct file *file, char __user *buf, size_t size, loff_t *offset);
ssize_t virtual_write(struct file *file, const char __user *buf, size_t size, loff_t *offset);
unsigned int virtual_poll(struct file *file, struct poll_table_struct *pwait);
int virtual_fasync(int fd, struct file *filp, int mode) ;
int create_manager_device(struct platform_device *pdev, int index);
static int serial_virtual_probe(struct platform_device *pdev);
static int serial_virtual_remove(struct platform_device *dev);
void virtual_serial_release(struct device *dev);
long virtual_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static int virtual_serial_major = 0;
static int virtual_serial_minor_start = 0;
static struct virtual_uart_port *virtual_array[virtual_SERIAL_NR];

unsigned int virtual_serial_nr = 1;
module_param(virtual_serial_nr, uint, S_IRUGO);

int virtual_fasync(int fd, struct file *filp, int mode);
static struct virtual_serial_data
{
    DWORD tx_fifo_size;
    DWORD rx_fifo_size;
};

/****************************************************************************************************/
/*struct platform_driver {                                                                          */
/*  int (*probe)(struct platform_device *);         /* 设备检测，软硬件匹配成功后调用                 */
/*  int (*remove)(struct platform_device *);        /* 设备卸载时调用                                */
/*  void (*shutdown)(struct platform_device *);                                                     */
/*  int (*suspend)(struct platform_device *, pm_message_t state);                                   */
/*  int (*resume)(struct platform_device *);                                                        */
/*  struct device_driver driver;                                                                    */
/*  struct platform_device_id *id_table;                                                            */
/*};                                                                                                */
/****************************************************************************************************/

static struct platform_driver virtual_serial_driver = {
    .probe      = serial_virtual_probe,             /* 设备检测函数，所以需要先注册platform_device    */
    .remove     = serial_virtual_probe,             /* 设备删除函数                                  */
    .driver     = {
        .name   = "virtual_serial",
        .owner  = THIS_MODULE,
    }
};

static struct virtual_serial_data virtual_serial_dev_data = {
        .tx_fifo_size = 2 * 1024,
        .rx_fifo_size = 2 * 1024,
};

/****************************************************************************************************/
/*struct platform_device {                                                                          */
/*  const char  * name;                             /* 设备名                                       */                  
/*  int     id;                                     /* 决定是否对name编号，值为-1表示，直接使用上诉   */
/*                                                  /* name命名，如果不是-1则对name进行编号          */
/*  struct device   dev;                            /* 设备的详细信息，碍于篇幅，这里就不做过多解释   */
/*  u32     num_resources;                          /* 设备用到的资源个数                            */
/*  struct resource * resource;                     /* 用到的资源                                    */
/*                                                                                                  */
/*  struct platform_device_id   *id_entry;          /* 设备ID                                       */
/*                                                                                                  */
/*  /* arch specific additions                                                                      */
/*  struct pdev_archdata    archdata;                                                               */
/*};                                                                                                */
/****************************************************************************************************/
static struct platform_device virtual_serial_device = {
    .name       = "virtual_serial",                 /* 设备名                                      */
    .dev        = {                                 /* struct device dev 设备描述的详细信息         */
        .release = virtual_serial_release,
        .platform_data = ( void *)( &virtual_serial_dev_data ),
    }
};

/****************************************************************************************************/
/*struct uart_driver {                                                                              */
/*  struct module   *owner;                         /* 拥有该uart_driver的模块                        */
/*  const char      *driver_name;                   /* 驱动名                                      */
/*  const char      *dev_name;                      /* 设备名                                      */
/*  int             major;                          /* 主设备号                                     */
/*  int             minor;                          /* 次设备号                                     */
/*  int             nr;                             /* 支持的串口个数                              */
/*  struct console  *cons;                          /* 若不支持serial console，则为NULL                */
/*  /* 私有数据，底层驱动不应该访问这些成员，应该被初始化为空                                      */
/*  struct uart_state   *state;                                                                     */
/*  struct tty_driver   *tty_driver;                                                                */
/*};                                                                                                */
/****************************************************************************************************/

static struct uart_driver virtual_driver = {
    .owner          = THIS_MODULE,
    .driver_name    = "virtualSerial",
    .dev_name       = "virtualSerial",
    .major          = 0,                            /* 设为0，表示由系统分配设备号                   */
    .minor          = 0,
    .nr             = virtual_SERIAL_NR,
    .cons           = NULL,                         /* 不支持serial console                            */
};

static struct uart_ops virtual_uart_ops = {
    .tx_empty       = virtual_tx_empty,             /* 检测发送FIFO缓冲区是否为空                  */
    .set_mctrl      = virtual_set_mctrl,            /* 设置串口流控                                   */
    .get_mctrl      = virtual_get_mctrl,            /* 检测串口流控                                   */
    .stop_tx        = virtual_stop_tx,              /* 停止发送                                     */
    .start_tx       = virtual_start_tx,             /* 开始发送                                     */
    .stop_rx        = virtual_stop_rx,              /* 停止接收                                     */
    .enable_ms      = virtual_enable_ms,            /* 空函数                                      */
    .break_ctl      = virtual_break_ctl,            /* 发送break信号                                */
    .startup        = virtual_startup,              /* 串口发送/接收，以及中断初始函数             */
    .shutdown       = virtual_shutdown,             /* 关闭串口                                     */
    .flush_buffer   = virtual_flush_buffer,         /* 刷新缓冲区，并丢弃任何剩下的数据             */
    .set_termios    = virtual_set_termios,          /* 设置串口波特率，数据位等                     */
    .type           = virtual_type,                 /* 串口类型                                     */
    .release_port   = virtual_release_port,         /* 释放串口                                     */
    .request_port   = virtual_request_port,         /* 申请串口                                     */
    .config_port    = virtual_config_port,          /* 串口的一些配置                              */
    .verify_port    = virtual_verify_port,          /* 串口检测                                     */
#ifdef CONFIG_CONSOLE_POLL                          /*                                              */
    .poll_get_char  = NULL,                         /* 设备阻塞与非阻塞访问相关                     */
    .poll_put_char  = NULL,
#endif
};

struct file_operations virtual_fops = {
    .open           = virtual_open,                 /* 对应上层系统调用，打开串口                    */
    .release        = virtual_release,              /* 对应上层系统调用，释放串口                    */
    .read           = virtual_read,                 /* 对应上层系统调用，从串口读数据              */
    .write          = virtual_write,                /* 对应上层系统调用，往串口写入                   */
    .unlocked_ioctl = virtual_ioctl,                /* 对应上层系统调用，串口控制                    */
    .poll           = virtual_poll,
    .fasync         = virtual_fasync,
};

struct virtual_uart_port {
    struct uart_port port;

    struct virtual_port_data *port_data;

    char type[12];

    unsigned char *tx_fifo;
    unsigned char *rx_fifo;

    unsigned long tx_len;
    unsigned long rx_len;

    unsigned int mctrl;
    unsigned int baud;

    struct ktermios termios;

    struct fasync_struct *async_queue;
    struct semaphore async_sem;

    struct completion manager_activie;
    struct completion write_ok; 
    wait_queue_head_t poll_wq; 

    int manager_reset;

    int is_default_termios : 1;
    unsigned long status;

    struct cdev c_dev;
    int index;
};

static struct class *virtual_class;

static unsigned int virtual_tx_empty(struct uart_port *port)
{

    struct virtual_uart_port *virtual = (struct virtual_uart_port *)port;

    drintk("virtual_tx_empty %d\n", virtual->tx_len);

    return virtual->tx_len > 0 ? 0 : TIOCSER_TEMT;
}

static void virtual_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
    struct virtual_uart_port *virtual = (struct virtual_uart_port *)port;
    virtual->mctrl = mctrl;

    drintk("virtual_set_mctrl!\n");   
}

static unsigned int virtual_get_mctrl(struct uart_port *port)
{
    struct virtual_uart_port *virtual = (struct virtual_uart_port *)port;

    drintk("virtual_get_mctrl!\n");
    return virtual->mctrl;
}

static void virtual_stop_tx(struct uart_port *port)
{
    drintk("virtual_stop_tx!\n");
}

static void virtual_start_tx(struct uart_port *port)
{   
    struct virtual_uart_port *virtual = (struct virtual_uart_port*)port;
    struct virtual_serial_data *data = (struct virtual_serial_data *)virtual->port_data;
    struct circ_buf *xmit = &port->state->xmit;
    int i = 0;

    drintk("virtual_start_tx!\n");

    // virtual->tx_len = 0;
    do {        
        virtual->tx_fifo[virtual->tx_len++] = xmit->buf[xmit->tail];

        xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
        port->icount.tx++;  

        if (uart_circ_empty(xmit)) {
            break;
        }

    } while (virtual->tx_len < data->tx_fifo_size); 

    for (i = 0; i < virtual->tx_len; i++) {
        drintk("%c ", virtual->tx_fifo[i]);
    }
    drintk("\n");

    wake_up(&virtual->poll_wq);

    init_completion(&virtual->write_ok);
    wait_for_completion(&virtual->write_ok);
}

static void virtual_stop_rx(struct uart_port *port)
{
    drintk("virtual_stop_rx!\n");
}

static void virtual_enable_ms(struct uart_port *port)
{
    drintk("virtual_enable_ms!\n");
}

static void virtual_break_ctl(struct uart_port *port, int break_state)
{
    drintk("virtual_break_ctl!\n");
}

static int virtual_startup(struct uart_port *port)
{
    drintk("virtual_startup!\n");

    return 0;
}

static void virtual_shutdown(struct uart_port *port)
{
    drintk("virtual_shutdown!\n");
}

static void virtual_flush_buffer(struct uart_port *port)
{
    drintk("virtual_flush_buffer!\n");
}


static void
virtual_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
    unsigned int baud = 0;
    struct virtual_uart_port *virtual = (struct virtual_uart_port*)port;

    baud = uart_get_baud_rate(port, termios, old, 0, 460800);
    virtual->baud = baud;

    memcpy(&virtual->termios, termios, sizeof(struct ktermios));

    drintk("set baudrate to %d\n", baud);

#if 1
    down(&virtual->async_sem);
    if (virtual->async_queue == NULL) {   
        up(&virtual->async_sem);

        init_completion(&virtual->manager_activie);
        wait_for_completion(&virtual->manager_activie);

    } else {
        up(&virtual->async_sem);
    }

    kill_fasync(&virtual->async_queue, SIGIO, POLL_IN);
#endif

}

static const char *virtual_type(struct uart_port *port)
{
    struct virtual_uart_port *virtual = (struct virtual_uart_port*)port;
    drintk("virtual_type\n");             
    return virtual->type;
}

static void virtual_release_port(struct uart_port *port)
{
    drintk("virtual_release_port\n");
}

static int virtual_request_port(struct uart_port *port)
{
    drintk("virtual_request_port\n");
    return 0;
}

static void virtual_config_port(struct uart_port *port, int flags)
{
    drintk("virtual_config_port\n");
    virtual_request_port(port);
    drintk("port->type = %d\n", port->type);

    port->type = PORT_AMBA;
}

static int virtual_verify_port(struct uart_port *port, struct serial_struct *ser)
{

    drintk("virtual_verify_port\n");

    return 0;
}

int virtual_open(struct inode *i, struct file *file)
{
    int minor = iminor(i);
    int index = minor - virtual_serial_minor_start;

    file->private_data = virtual_array[index];

    drintk("minor %d index %d private_data %p\n", minor, index, file->private_data);

    return 0;
}

int virtual_release(struct inode *i, struct file *file)
{
    struct virtual_uart_port *virtual = (struct virtual_uart_port *)file->private_data;

    virtual_fasync(-1, file, 0);

    down(&virtual->async_sem);    
    virtual->async_queue = NULL;
    virtual->manager_reset = 1;
    up(&virtual->async_sem);

    file->private_data = NULL;
    return 0;
}

ssize_t virtual_read(struct file *file, char __user *buf, size_t size, loff_t *offset)
{
    struct virtual_uart_port *virtual = (struct virtual_uart_port *)file->private_data;
    unsigned long tx_len = virtual->tx_len;
    int read_len = 0;

    read_len = size > tx_len ? tx_len : size;
    copy_to_user(buf, virtual->tx_fifo, read_len);
    virtual->tx_len -= read_len;

    if (virtual->tx_len == 0)
        complete(&virtual->write_ok);

    return read_len;
}

ssize_t virtual_write(struct file *file, const char __user *buf, size_t size, loff_t *offset)
{
    struct virtual_uart_port *virtual = (struct virtual_uart_port *)file->private_data;
    // struct tty_struct *tty = NULL;
    struct uart_port *port = NULL;
    unsigned char ch = 0;
    int i = 0;  

    if (virtual == NULL)
    {
        printk("virtual is nullptr\n");
        return -EIO;
    }

    port = &virtual->port;
    if (virtual->port.state == NULL)
    {
        printk("virtual->port.state is nullptr\n");
        return -EIO;    
    }   

    struct tty_port *tty = NULL;
    tty = &virtual->port.state->port;

    // tty = virtual->port.state->port.tty;
    if (tty == NULL)
    {
        printk("tty is nullptr\n");
        return -EIO;    
    }   

    copy_from_user(virtual->rx_fifo, buf, size);
    virtual->rx_len = size;


    // insert chars
    for (i = 0; i < size ; i++) {
        ch = *(virtual->rx_fifo + i);
        port->icount.rx++;

        tty_insert_flip_char(tty, ch, 0);
    }

    tty_flip_buffer_push(tty);  


    return size;
}

long virtual_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct virtual_uart_port *virtual = (struct virtual_uart_port *)file->private_data;

    switch (cmd){
    case 0xde:
        printk("virtual_ioctl baud %d\n", virtual->baud);
        copy_to_user((void *)arg, &virtual->baud, sizeof(unsigned int));
        break;

    default:
        break;
    }   

    return 0;
}

unsigned int virtual_poll(struct file *file, struct poll_table_struct *pwait)
{
    struct virtual_uart_port *virtual = (struct virtual_uart_port *)file->private_data;
    unsigned int mask = 0;

    printk("%d tx_len %d\n", virtual->index, virtual->tx_len);
    poll_wait(file, &virtual->poll_wq, pwait);

    if (virtual->tx_len)
        mask |= POLLIN | POLLRDNORM;

    return mask;
}

int virtual_fasync(int fd, struct file *filp, int mode) 
{
    struct virtual_uart_port *virtual = (struct virtual_uart_port *)filp->private_data;
    int ret = 0;

    down(&virtual->async_sem);
    ret = fasync_helper(fd, filp, mode, &virtual->async_queue);

    // 如果管理进程重置过,则需要主动通知其更新termios
    if (virtual->manager_reset) {
        virtual->manager_reset = 0;
        kill_fasync(&virtual->async_queue, SIGIO, POLL_IN);       
    }

    up(&virtual->async_sem);

    complete(&virtual->manager_activie);

    return ret;
}

int create_manager_device(struct platform_device *pdev, int index)
{
    struct virtual_uart_port *virtual = NULL;
    struct virtual_serial_data *data = NULL;
    struct device *tmp = NULL;
    int ret = 0;
    dev_t dev = 0;
    char dev_name[64] = {0};

    data = (struct virtual_serial_data *)pdev->dev.platform_data;
    if (!data) {
        printk("not platform data\n");
        return -EINVAL;
    }


    virtual = (struct virtual_uart_port *)\
                kmalloc(sizeof(struct virtual_uart_port), GFP_KERNEL);
    if (!virtual) {
        printk("malloc virtual error\n");
        return -ENOMEM;
    }

    memset(virtual, 0, sizeof(struct virtual_uart_port));
    virtual->index = index;
    virtual->is_default_termios = 1;  
    sema_init(&virtual->async_sem, 1);
    init_completion(&virtual->manager_activie);
    init_waitqueue_head(&virtual->poll_wq);

    virtual->tx_fifo = (unsigned char*)kmalloc(data->tx_fifo_size, GFP_KERNEL);
    virtual->rx_fifo = (unsigned char*)kmalloc(data->rx_fifo_size, GFP_KERNEL);
    if (!virtual->tx_fifo || !virtual->rx_fifo) {
        printk("virtual fifo kmalloc err rx=%p tx=%p\n", \
                virtual->rx_fifo, virtual->tx_fifo);
        ret = -ENOMEM;
        goto FIFO_ERR;
    }

    virtual->port_data = (struct virtual_port_data*)data;    

    virtual->port.dev = &(pdev->dev);
    virtual->port.mapbase = 0;
    virtual->port.membase = (unsigned char *)(0xdeadbeef);
    virtual->port.iotype = UPIO_MEM;
    virtual->port.irq = 0;
    virtual->port.fifosize = 16;
    virtual->port.ops = &virtual_uart_ops;
    virtual->port.flags = UPF_BOOT_AUTOCONF;
    virtual->port.line = index;
    virtual->port.type = PORT_AMBA;

    ret = uart_add_one_port(&virtual_driver, &virtual->port);
    if (ret) {
        printk("uart drv add one port err. ret = 0x%08x\n", ret);
        goto PORT_ERR;
    }

    virtual_array[index] = virtual;

    if (!virtual_serial_major) {
        sprintf(dev_name, "valueserial%d", 0);
        ret = alloc_chrdev_region(&dev, 0, virtual_SERIAL_NR, dev_name);
        if (!ret) {
            virtual_serial_major = MAJOR(dev);
            virtual_serial_minor_start = MINOR(dev);
            // printk("major %d minor_start %d\n", virtual_serial_major, virtual_serial_minor_start);
        } else {
            printk("register cdev err, ret=%d\n", ret);
            goto DEV_ERR;   
        }   
    } else {
        dev = MKDEV(virtual_serial_major, virtual_serial_minor_start + index);
        sprintf(dev_name, "valueserial%d", index);
    }   

    cdev_init(&virtual->c_dev, &virtual_fops);
    virtual->c_dev.owner = THIS_MODULE;   
    cdev_add(&virtual->c_dev, dev, virtual_SERIAL_NR);

    tmp = device_create(virtual_class, NULL, dev, NULL, dev_name);
    if (NULL == tmp) {
        printk("create device err! %d, %s\n", dev, dev_name);
    }

    printk("create virtual serial manager device /dev/%s\n", dev_name);

    // platform_set_drvdata(pdev, virtual);

    return ret;

DEV_ERR:
    uart_remove_one_port(&virtual_driver, &virtual->port);

PORT_ERR:
    if (virtual->rx_fifo)
        kfree(virtual->rx_fifo);

    if (virtual->tx_fifo)
        kfree(virtual->tx_fifo);

FIFO_ERR:
    kfree(virtual);

    return ret;
}


static int serial_virtual_probe(struct platform_device *pdev)
{
    int i = 0;
    int ret = 0;

    for(i = 0; i < virtual_serial_nr; i++) {
        ret = create_manager_device(pdev, i);
        if (ret) {
            printk("create virtual manager device err, index = %d\n", i);
        }
    }   
    return 0;
}

static int serial_virtual_remove(struct platform_device *dev)
{   
    struct virtual_uart_port *virtual =  NULL;
    int i = 0;
    dev_t dev_num = 0;

    for (i = 0; i < virtual_serial_nr; i++) {
        virtual = virtual_array[i];

        dev_num = MKDEV(virtual_serial_major, virtual_serial_minor_start + i);
        device_destroy(virtual_class, dev_num);
        cdev_del(&virtual->c_dev);

        uart_remove_one_port(&virtual_driver, &virtual->port);

        if (virtual->rx_fifo)
            kfree(virtual->rx_fifo);

        if (virtual->tx_fifo)
            kfree(virtual->tx_fifo);

        kfree(virtual);
    }

    dev_num = MKDEV(virtual_serial_major, virtual_serial_minor_start);
    unregister_chrdev_region(dev_num, virtual_SERIAL_NR);

    return 0;
}

void virtual_serial_release(struct device *dev)
{


}

static int __init serial_virtual_init(void)
{   
    int ret = 0;   

    if (virtual_serial_nr > virtual_SERIAL_NR) {
        printk("virtual serial nr(%d) is more than max(%d)\n", \
            virtual_serial_nr, virtual_SERIAL_NR);
        return -EINVAL;
    }

    // 注册平台设备
    ret = platform_device_register(&virtual_serial_device);
    virtual_class  = class_create(THIS_MODULE, "dumtty");

    ret = uart_register_driver(&virtual_driver);
    if (ret) {
        printk("virtual drv register err ret = 0x%08x\n", ret);
        return -EINVAL;
    }

    ret = platform_driver_register(&virtual_serial_driver);
    if (ret) {
        printk("virtual platform drv register err ret = %d\n", ret);
        platform_device_unregister(&virtual_serial_device);
        uart_unregister_driver(&virtual_driver);
    }

    return ret;
}

static void __exit serial_virtual_exit(void)
{   
    platform_driver_unregister(&virtual_serial_driver);
    uart_unregister_driver(&virtual_driver);
    class_destroy(virtual_class);

    platform_device_unregister(&virtual_serial_device);
}

module_init(serial_virtual_init);
module_exit(serial_virtual_exit);

MODULE_AUTHOR("Kang Lin (kl222@126.com)");
MODULE_DESCRIPTION("virtual serial driver");
MODULE_LICENSE("GPL");

