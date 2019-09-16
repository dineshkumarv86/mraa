/*
 *
 */

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <sys/file.h>
#include <unistd.h>

#include "linux/gpio.h"
#include "mraa_internal.h"

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>


#include "common.h"
#include "gpio.h"
#include "x86/intel_adlink_lec_al.h"
#include "gpio/gpio_chardev.h"

#define SYSFS_CLASS_GPIO "/sys/class/gpio"

#define PLATFORM_NAME "LEC-AL"

#define MRAA_LEC_AL_GPIOCOUNT  17
#define MAX_SIZE 64
#define POLL_TIMEOUT

static int hwid;
static long m_period;

static mraa_result_t pwm_init_raw_replace(mraa_pwm_context dev, int pin)
{
	syslog(LOG_NOTICE, "lec_al: %s\n", __func__);
	char buffer[100] = {0};
	int i, fd;
	for(i = 0; i < 100; i++)
	{
		sprintf(buffer, "/sys/class/hwmon/hwmon%d/device/driver/adl-bmc-hwmon/modalias",i);
		if((fd = open(buffer, O_RDONLY)) != -1)
		{
			hwid = i;
			close(fd);
			break;
		}
	}
	if(i >= 100)
	{
		close(fd);
		return MRAA_ERROR_NO_RESOURCES;
	}
	close(fd);
	return MRAA_SUCCESS;
}

static mraa_result_t pwm_period_replace(mraa_pwm_context dev, int period)
{
	syslog(LOG_NOTICE, "lec_al: %s %d\n", __func__, period);
	m_period = period;
	return MRAA_SUCCESS;
}

static float pwm_read_replace(mraa_pwm_context dev)
{
	syslog(LOG_NOTICE, "lec_al: %s\n", __func__);
	int fd, count = 0;
	char buffer[100] = {0};
	sprintf(buffer, "/sys/class/hwmon/hwmon%d/device/fan1_auto_point4_pwm",hwid);
	if((fd = open(buffer, O_RDONLY)) == -1)
	{
		return 0;
	}

	if((count = read(fd,buffer,3)) != 0)
	{
		buffer[count] = 0;
	}

	close(fd);
	int cycle = (atoi(buffer) * m_period)/100;
	return cycle;
}

static mraa_result_t pwm_write_replace(mraa_pwm_context dev, float duty)
{
	syslog(LOG_NOTICE, "lec_al: %s %f\n", __func__, duty);
	int fd;

	int count = 0;
	char buffer[100] = {0};
	sprintf(buffer, "/sys/class/hwmon/hwmon%d/device/fan1_auto_point4_pwm",hwid);

	if((fd = open(buffer, O_RDWR)) == -1)
	{
		return MRAA_ERROR_NO_RESOURCES;
	}

	duty = (duty/m_period) * 100;

	count = sprintf(buffer,"%d", (int)duty);
	buffer[count] = 0;

	if(write(fd, buffer, count) != count)
	{
		close(fd);
		return MRAA_ERROR_NO_RESOURCES;
	}
	close(fd);
	return MRAA_SUCCESS;
}

static mraa_result_t pwm_enable_replace(mraa_pwm_context dev, int enable)
{
	syslog(LOG_NOTICE, "lec_al: %s\n", __func__);
	char buffer[100] = {0};
	int fd, i;

	sprintf(buffer, "/sys/class/hwmon/hwmon%d/device/fan1_enable",hwid);

	if((fd = open(buffer, O_RDWR)) == -1)
	{
		return MRAA_ERROR_NO_RESOURCES;
	}

	if((i = write(fd, "2", 1)) != 1)
	{
		close(fd);
		return MRAA_ERROR_NO_RESOURCES;
	}
	close(fd);

	sprintf(buffer, "/sys/class/hwmon/hwmon%d/device/fan1_enable",hwid);

	if((fd = open(buffer, O_RDWR)) == -1)
	{
		close(fd);
		return MRAA_ERROR_NO_RESOURCES;
	}

	if(read(fd, buffer, 1) != 1)
	{
		close(fd);
		return MRAA_ERROR_NO_RESOURCES;
	}
	if(buffer[0] != '2')
	{
		close(fd);
		return MRAA_ERROR_NO_RESOURCES;
	}

	close(fd);

	for(i = 1; i < 5; i++)
	{
		sprintf(buffer, "/sys/class/hwmon/hwmon%d/device/fan1_auto_point%d_temp", hwid, i);

		if((fd = open(buffer, O_RDWR)) == -1)
		{
			return MRAA_ERROR_NO_RESOURCES;
		}

		if(write(fd, "0", 1) != 1)
		{
			close(fd);
			return MRAA_ERROR_NO_RESOURCES;
		}
		close(fd);
		sprintf(buffer, "/sys/class/hwmon/hwmon%d/device/fan1_auto_point%d_temp", hwid, i);

		if((fd = open(buffer, O_RDWR)) == -1)
		{
			return MRAA_ERROR_NO_RESOURCES;
		}


		if(read(fd, buffer, 1) != 1)
		{
			close(fd);
			return MRAA_ERROR_NO_RESOURCES;
		}
		close(fd);
		if(buffer[0] != '0')
		{
			return MRAA_ERROR_NO_RESOURCES;
		}

	}

	return MRAA_SUCCESS;
}

typedef void (*fptr_t)(void*);

static fptr_t ind_fptr[MRAA_LEC_AL_PINCOUNT];

	static void
mraa_gpio_close_event_handles_sysfs(int fds[], int num_fds)
{
	if ((fds == NULL) || (num_fds <= 0)) {
		syslog(LOG_CRIT, "failed to close and free sysfs event handles");
		return;
	}

	for (int i = 0; i < num_fds; ++i) {
		close(fds[i]);
	}

	free(fds);
}


	static mraa_timestamp_t
_mraa_gpio_get_timestamp_sysfs()
{
	struct timeval time;
	gettimeofday(&time, NULL);

	return (time.tv_sec * 1e6 + time.tv_usec);
}

	static mraa_result_t
mraa_gpio_wait_interrupt(int fds[],
		int num_fds
#ifndef HAVE_PTHREAD_CANCEL
		,
		int control_fd
#endif
		,
		mraa_gpio_events_t events
		)

{
	unsigned char c;
#ifdef HAVE_PTHREAD_CANCEL
	struct pollfd pfd[num_fds];
#else
	struct pollfd pfd[num_fds + 1];

	if (control_fd < 0) {
		return MRAA_ERROR_INVALID_PARAMETER;
	}
#endif

	if (!fds) {
		return MRAA_ERROR_INVALID_PARAMETER;
	}

	for (int i = 0; i < num_fds; ++i) {
		pfd[i].fd = fds[i];
		// setup poll on POLLPRI
		pfd[i].events = POLLPRI;

		// do an initial read to clear interrupt
		lseek(fds[i], 0, SEEK_SET);
		read(fds[i], &c, 1);
	}

#ifdef HAVE_PTHREAD_CANCEL
	// Wait for it forever or until pthread_cancel
	// poll is a cancelable point like sleep()
	poll(pfd, num_fds, -1);
#else
	// setup poll on the controling fd
	pfd[num_fds].fd = control_fd;
	pfd[num_fds].events = 0; //  POLLHUP, POLLERR, and POLLNVAL

	// Wait for it forever or until control fd is closed
	poll(pfd, num_fds + 1, -1);
#endif

	for (int i = 0; i < num_fds; ++i) {
		if (pfd[i].revents & POLLPRI) {
			read(fds[i], &c, 1);
			events[i].id = i;
			events[i].timestamp = _mraa_gpio_get_timestamp_sysfs();
		} else
			events[i].id = -1;
	}

	return MRAA_SUCCESS;
}


	static mraa_result_t
mraa_gpio_chardev_wait_interrupt(int fds[], int num_fds, mraa_gpio_events_t events)
{   
	struct pollfd pfd[num_fds];
	struct gpioevent_data event_data;

	if (!fds) {
		return MRAA_ERROR_INVALID_PARAMETER;
	}

	for (int i = 0; i < num_fds; ++i) {
		pfd[i].fd = fds[i];
		pfd[i].events = POLLIN;

		lseek(fds[i], 0, SEEK_SET);
	}

	poll(pfd, num_fds, -1);

	for (int i = 0; i < num_fds; ++i) {
		if (pfd[i].revents & POLLIN) {
			read(fds[i], &event_data, sizeof(event_data));
			events[i].id = i;
			events[i].timestamp = event_data.timestamp;
		} else
			events[i].id = -1;
	}

	return MRAA_SUCCESS;
}

	static void*
mraa_gpio_interrupt_handler(void* arg)
{
	if (arg == NULL) {
		syslog(LOG_ERR, "gpio: interrupt_handler: context is invalid");
		return NULL;
	}

	mraa_result_t ret;
	mraa_gpio_context dev = (mraa_gpio_context) arg;
	int idx = 0, count;

	if (IS_FUNC_DEFINED(dev, gpio_interrupt_handler_init_replace)) {
		if (dev->advance_func->gpio_interrupt_handler_init_replace(dev) != MRAA_SUCCESS)
			return NULL;
	}

	int *fps = malloc(dev->num_pins * sizeof(int));
	if (!fps) {
		syslog(LOG_ERR, "mraa_gpio_interrupt_handler_multiple() malloc error");
		return NULL;
	}

	/* Is this pin on a subplatform? Do nothing... */
	if (mraa_is_sub_platform_id(dev->pin)) {}
	/* Is the platform chardev_capable? */
	else if (plat->chardev_capable) {
		mraa_gpiod_group_t gpio_group;

		for_each_gpio_group(gpio_group, dev) {
			for (int i = 0; i < gpio_group->num_gpio_lines; ++i) {
				fps[idx++] = gpio_group->event_handles[i];
			}
		}
	}
	/* Else, attempt fs access */
	else {
		mraa_gpio_context it = dev;

		while (it) {
			// open gpio value with open(3)
			char bu[MAX_SIZE];
			snprintf(bu, MAX_SIZE, SYSFS_CLASS_GPIO "/gpio%d/value", it->pin);
			fps[idx] = open(bu, O_RDONLY);
			if (fps[idx] < 0) {
				syslog(LOG_ERR, "gpio%i: interrupt_handler: failed to open 'value' : %s", it->pin,
						strerror(errno));
				mraa_gpio_close_event_handles_sysfs(fps, idx);
				return NULL;
			}

			idx++;
			it = it->next;
		}
	}

#ifndef HAVE_PTHREAD_CANCEL
	if (pipe(dev->isr_control_pipe)) {
		syslog(LOG_ERR, "gpio%i: interrupt_handler: failed to create isr control pipe: %s",
				dev->pin, strerror(errno));
		mraa_gpio_close_event_handles_sysfs(fps, dev->num_pins);
		return NULL;
	}
#endif

	if (lang_func->java_attach_thread != NULL) {
		if (dev->isr == lang_func->java_isr_callback) {
			if (lang_func->java_attach_thread() != MRAA_SUCCESS) {
				mraa_gpio_close_event_handles_sysfs(fps, dev->num_pins);
				return NULL;
			}
		}
	}

	for (;;) {
		if (IS_FUNC_DEFINED(dev, gpio_wait_interrupt_replace)) {
			ret = dev->advance_func->gpio_wait_interrupt_replace(dev);
		} else {
			if (plat->chardev_capable) {
				ret = mraa_gpio_chardev_wait_interrupt(fps, idx, dev->events);
			} else {
				ret = mraa_gpio_wait_interrupt(fps, idx
#ifndef HAVE_PTHREAD_CANCEL
						,
						dev->isr_control_pipe[0]
#endif
						,
						dev->events
						);
			}
		}

		if (ret == MRAA_SUCCESS && !dev->isr_thread_terminating) {
#ifdef HAVE_PTHREAD_CANCEL
			pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
#endif

			for(count = 0; count < MRAA_LEC_AL_PINCOUNT; count++)
			{
				if(ind_fptr[count] != NULL)
				{
					if (lang_func->python_isr != NULL) {
						lang_func->python_isr(ind_fptr[count], dev->isr_args);
					} else {
						(ind_fptr[count])(dev->isr_args);
					}
				}
			}
#ifdef HAVE_PTHREAD_CANCEL
			pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
#endif
		} else {
			// we must have got an error code or exit request so die nicely
#ifdef HAVE_PTHREAD_CANCEL
			pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
#endif
			mraa_gpio_close_event_handles_sysfs(fps, dev->num_pins);

			if (lang_func->java_detach_thread != NULL && lang_func->java_delete_global_ref != NULL) {
				if (dev->isr == lang_func->java_isr_callback) {
					lang_func->java_delete_global_ref(dev->isr_args);
					lang_func->java_detach_thread();
				}
			}
			return NULL;
		}
	}
}



// utility function to setup pin mapping of boards
static mraa_result_t mraa_lec_al_set_pininfo(mraa_board_t* board, int mraa_index, char* name,
		mraa_pincapabilities_t caps, int sysfs_pin)
{
	if (mraa_index < board->phy_pin_count) {
		mraa_pininfo_t* pin_info = &board->pins[mraa_index];
		strncpy(pin_info->name, name, MRAA_PIN_NAME_SIZE);
		pin_info->capabilities = caps;
		if (caps.gpio) {
			pin_info->gpio.pinmap = sysfs_pin;
			pin_info->gpio.mux_total = 0;
		}
		if (caps.i2c) {
			pin_info->i2c.pinmap = 1;
			pin_info->i2c.mux_total = 0;
		}

		if (caps.uart) {
			pin_info->uart.mux_total = 0;
		}
		if (caps.spi) {
			pin_info->spi.mux_total = 0;
		}
		if(strncmp(pin_info->name, "INT", strlen(pin_info->name)) == 0)
		{
			pin_info->gpio.pinmap = sysfs_pin;
			pin_info->gpio.mux_total = 0;
		}
		return MRAA_SUCCESS;
	}
	return MRAA_ERROR_INVALID_RESOURCE;
}

static mraa_result_t mraa_lec_al_get_pin_index(mraa_board_t* board, char* name, int* pin_index)
{
	int i;
	for (i = 0; i < board->phy_pin_count; ++i) {
		if (strncmp(name, board->pins[i].name, MRAA_PIN_NAME_SIZE) == 0) {
			*pin_index = i;
			return MRAA_SUCCESS;
		}
	}

	syslog(LOG_CRIT, "lec_al: Failed to find pin name %s", name);

	return MRAA_ERROR_INVALID_RESOURCE;
}

static mraa_result_t gpio_isr_replace(mraa_gpio_context dev, mraa_gpio_edge_t mode, void (*fptr)(void*), void* args)
{
	mraa_result_t status;

	syslog(LOG_CRIT, "lec_al: gpio_isr_replace");

	mraa_gpio_context gpio = mraa_gpio_init(0);
	if (gpio == NULL) {
		return MRAA_ERROR_INVALID_RESOURCE;
	}

	/* set GPIO to input */
	status = mraa_gpio_dir(gpio, MRAA_GPIO_IN);
	if (status != MRAA_SUCCESS) {
		return MRAA_ERROR_INVALID_RESOURCE;
	}


	if (gpio->thread_id != 0) { 
		return MRAA_ERROR_NO_RESOURCES;
	}

	mraa_result_t ret; 

	ret = mraa_gpio_edge_mode(gpio, mode);

	if (ret != MRAA_SUCCESS) {
		return ret; 
	}

	gpio->isr = NULL;
	syslog(LOG_CRIT, "lec_al: pin %d is registered with isr\n", dev->phy_pin);
	ind_fptr[gpio->phy_pin] = fptr;


	/* Most UPM sensors use the C API, the Java global ref must be created here. */
	/* The reason for checking the callback function is internal callbacks. */
	if (lang_func->java_create_global_ref != NULL) {
		if (gpio->isr == lang_func->java_isr_callback) {
			args = lang_func->java_create_global_ref(args);
		}    
	}

	gpio->isr_args = args;

	pthread_create(&gpio->thread_id, NULL, mraa_gpio_interrupt_handler, (void*) gpio);

	return MRAA_SUCCESS;
}


mraa_board_t* mraa_lec_al_board()
{
	mraa_board_t* b = (mraa_board_t*) calloc(1, sizeof (mraa_board_t));

	if (b == NULL) {
		return NULL;
	}

	b->platform_name = PLATFORM_NAME;
	b->phy_pin_count = MRAA_LEC_AL_PINCOUNT;
	b->gpio_count = MRAA_LEC_AL_GPIOCOUNT;
	b->chardev_capable = 0;

	b->pwm_max_period = 2147483;
	b->pwm_min_period = 1;

	b->pins = (mraa_pininfo_t*) malloc(sizeof(mraa_pininfo_t) * MRAA_LEC_AL_PINCOUNT);
	if (b->pins == NULL) {
		goto error;
	}

	b->adv_func = (mraa_adv_func_t *) calloc(1, sizeof (mraa_adv_func_t));
	if (b->adv_func == NULL) {
		free(b->pins);
		goto error;
	}

	b->adv_func->gpio_isr_replace = gpio_isr_replace;

	// initializations of pwm functions
	b->adv_func->pwm_init_raw_replace = pwm_init_raw_replace;
	b->adv_func->pwm_period_replace = pwm_period_replace;
	b->adv_func->pwm_read_replace = pwm_read_replace;
	b->adv_func->pwm_write_replace = pwm_write_replace;
	b->adv_func->pwm_enable_replace = pwm_enable_replace;

	mraa_lec_al_set_pininfo(b, 0,  "INT",        (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 }, 434 + 22);
	mraa_lec_al_set_pininfo(b, 1,  "3v3",        (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 2,  "5v",         (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 3,  "I2C0_DAT",   (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 1, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 4,  "5v",         (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 5,  "I2C0_CK",    (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 1, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 6,  "GND",        (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 7,  "GPIO04",     (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 }, 255);
	mraa_lec_al_set_pininfo(b, 8,  "UART_TXD",   (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 1 }, -1);
	mraa_lec_al_set_pininfo(b, 9,  "GND",        (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 10, "UART_RXD",   (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 0, 0, 1 }, -1);
	mraa_lec_al_set_pininfo(b, 11, "GPIO06",     (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 }, 256);
	mraa_lec_al_set_pininfo(b, 12, "GPIO05",     (mraa_pincapabilities_t){ 1, 1, 1, 0, 0, 0, 0, 0 }, 257);
	mraa_lec_al_set_pininfo(b, 13, "GPIO07",     (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 }, 258);
	mraa_lec_al_set_pininfo(b, 14, "GND",        (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 15, "GPIO08",     (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 }, 259);
	mraa_lec_al_set_pininfo(b, 16, "GPIO09",     (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 }, 260);
	mraa_lec_al_set_pininfo(b, 17, "3v3",        (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 18, "GPIO10",     (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 }, 261);
	mraa_lec_al_set_pininfo(b, 19, "SPI_0_MOSI", (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 20, "GND",        (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 21, "SPI_0_MISO", (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 22, "GPIO11",     (mraa_pincapabilities_t){ 1, 1, 0, 0, 0, 0, 0, 0 }, 262);
	mraa_lec_al_set_pininfo(b, 23, "SPI_0_SCLK", (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 24, "SPI_0_CE0",  (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 25, "GND",        (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 26, "SPI_0_CE1",  (mraa_pincapabilities_t){ 1, 0, 0, 0, 1, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 27, "I2C1_DAT",   (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 1, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 28, "I2C1_CK",    (mraa_pincapabilities_t){ 1, 0, 0, 0, 0, 1, 0, 0 }, -1);

	mraa_lec_al_set_pininfo(b, 29, "GPIO1_0",    (mraa_pincapabilities_t){ 0, 1, 0, 0, 0, 0, 0, 0 }, 234);
	mraa_lec_al_set_pininfo(b, 30, "GND",        (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 31, "GPIO1_1",    (mraa_pincapabilities_t){ 0, 1, 0, 0, 0, 0, 0, 0 }, 235);
	mraa_lec_al_set_pininfo(b, 32, "GPIO1_2",    (mraa_pincapabilities_t){ 0, 1, 0, 0, 0, 0, 0, 0 }, 236);
	mraa_lec_al_set_pininfo(b, 33, "GPIO1_3",    (mraa_pincapabilities_t){ 0, 1, 0, 0, 0, 0, 0, 0 }, 237);
	mraa_lec_al_set_pininfo(b, 34, "GND",        (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 35, "GPIO1_4",    (mraa_pincapabilities_t){ 0, 1, 0, 0, 0, 0, 0, 0 }, 238);
	mraa_lec_al_set_pininfo(b, 36, "GPIO1_5",    (mraa_pincapabilities_t){ 0, 1, 0, 0, 0, 0, 0, 0 }, 239);
	mraa_lec_al_set_pininfo(b, 37, "GPIO1_6",    (mraa_pincapabilities_t){ 0, 1, 0, 0, 0, 0, 0, 0 }, 240);
	mraa_lec_al_set_pininfo(b, 38, "GPIO1_7",    (mraa_pincapabilities_t){ 0, 1, 0, 0, 0, 0, 0, 0 }, 241);
	mraa_lec_al_set_pininfo(b, 39, "GND",        (mraa_pincapabilities_t){ 0, 0, 0, 0, 0, 0, 0, 0 }, -1);
	mraa_lec_al_set_pininfo(b, 40, "GPIO2_8",    (mraa_pincapabilities_t){ 0, 1, 0, 0, 0, 0, 0, 0 }, 242);


	// Configure UART #1 (default)
	b->uart_dev_count = 1;

	mraa_lec_al_get_pin_index(b, "UART_RXD",  &(b->uart_dev[0].rx));
	mraa_lec_al_get_pin_index(b, "UART_TXD",  &(b->uart_dev[0].tx));
	b->uart_dev[0].device_path = "/dev/ttyS4";

	b->def_uart_dev = 0;

	// Configure SPI #0 CS1
	b->spi_bus_count = 0;
	b->spi_bus[b->spi_bus_count].bus_id = 1;
	b->spi_bus[b->spi_bus_count].slave_s = 0;
	mraa_lec_al_get_pin_index(b, "SPI_0_CE0",  &(b->spi_bus[b->spi_bus_count].cs));
	mraa_lec_al_get_pin_index(b, "SPI_0_MOSI", &(b->spi_bus[b->spi_bus_count].mosi));
	mraa_lec_al_get_pin_index(b, "SPI_0_MISO", &(b->spi_bus[b->spi_bus_count].miso));
	mraa_lec_al_get_pin_index(b, "SPI_0_SCLK",  &(b->spi_bus[b->spi_bus_count].sclk));
	b->spi_bus_count++;

	b->spi_bus[b->spi_bus_count].bus_id = 1;
	b->spi_bus[b->spi_bus_count].slave_s = 1;
	mraa_lec_al_get_pin_index(b, "SPI_0_CE1",  &(b->spi_bus[b->spi_bus_count].cs));
	mraa_lec_al_get_pin_index(b, "SPI_0_MOSI", &(b->spi_bus[b->spi_bus_count].mosi));
	mraa_lec_al_get_pin_index(b, "SPI_0_MISO", &(b->spi_bus[b->spi_bus_count].miso));
	mraa_lec_al_get_pin_index(b, "SPI_0_SCLK",  &(b->spi_bus[b->spi_bus_count].sclk));
	b->spi_bus_count++;


	// Set number of i2c adaptors usable from userspace
	b->i2c_bus_count = 2;
	b->def_i2c_bus = 0;
	int i2c_bus_num;


	i2c_bus_num = mraa_find_i2c_bus_pci("0000:00", "0000:00:16.1", "i2c_designware.1");
	if (i2c_bus_num != -1) {
		b->i2c_bus[0].bus_id = i2c_bus_num;
		syslog(LOG_NOTICE, "lec_al: i2c bus num %d\n", i2c_bus_num);
		mraa_lec_al_get_pin_index(b, "I2C1_DAT", (int*) &(b->i2c_bus[1].sda));
		mraa_lec_al_get_pin_index(b, "I2C1_CK", (int*) &(b->i2c_bus[1].scl));
		b->i2c_bus_count++;
	}

	i2c_bus_num = mraa_find_i2c_bus_pci("0000:00", "0000:00:1f.1", ".");
	if (i2c_bus_num != -1) {
		b->i2c_bus[1].bus_id = i2c_bus_num;
		mraa_lec_al_get_pin_index(b, "I2C0_DAT", (int*) &(b->i2c_bus[0].sda));
		mraa_lec_al_get_pin_index(b, "I2C0_CK", (int*) &(b->i2c_bus[0].scl));
		b->i2c_bus_count++;
	}

	const char* pinctrl_path = "/sys/bus/platform/drivers/broxton-pinctrl";
	int have_pinctrl = access(pinctrl_path, F_OK) != -1;
	syslog(LOG_NOTICE, "lec_al: kernel pinctrl driver %savailable", have_pinctrl ? "" : "un");

	if (have_pinctrl)
		return b;

error:
	syslog(LOG_CRIT, "lec_al: Platform failed to initialise");
	free(b);
	return NULL;
}
