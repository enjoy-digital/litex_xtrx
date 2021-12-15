// This file is Copyright (c) 2020-2021 Florent Kermarrec <florent@enjoy-digital.fr>
// License: BSD

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <irq.h>
#include <libbase/uart.h>
#include <libbase/console.h>
#include <generated/csr.h>

#include "i2c0.h"
#include "i2c1.h"

/*-----------------------------------------------------------------------*/
/* Constants                                                             */
/*-----------------------------------------------------------------------*/

#define XTRX_EXT_CLK   (1 << 0)
#define XTRX_VCTXO_CLK (0 << 0)

#define TMP108_I2C_ADDR  0x4a
#define LP8758_I2C_ADDR  0x60
#define LTC26X6_I2C_ADDR 0x62

/*-----------------------------------------------------------------------*/
/* Helpers                                                               */
/*-----------------------------------------------------------------------*/

void busy_wait(unsigned int ms)
{
	timer0_en_write(0);
	timer0_reload_write(0);
	timer0_load_write(CONFIG_CLOCK_FREQUENCY/1000*ms);
	timer0_en_write(1);
	timer0_update_value_write(1);
	while(timer0_value_read()) timer0_update_value_write(1);
}

/*-----------------------------------------------------------------------*/
/* UART                                                                  */
/*-----------------------------------------------------------------------*/

static char *readstr(void)
{
	char c[2];
	static char s[64];
	static int ptr = 0;

	if(readchar_nonblock()) {
		c[0] = getchar();
		c[1] = 0;
		switch(c[0]) {
			case 0x7f:
			case 0x08:
				if(ptr > 0) {
					ptr--;
					fputs("\x08 \x08", stdout);
				}
				break;
			case 0x07:
				break;
			case '\r':
			case '\n':
				s[ptr] = 0x00;
				fputs("\n", stdout);
				ptr = 0;
				return s;
			default:
				if(ptr >= (sizeof(s) - 1))
					break;
				fputs(c, stdout);
				s[ptr] = c[0];
				ptr++;
				break;
		}
	}

	return NULL;
}

static char *get_token(char **str)
{
	char *c, *d;

	c = (char *)strchr(*str, ' ');
	if(c == NULL) {
		d = *str;
		*str = *str+strlen(*str);
		return d;
	}
	*c = 0;
	d = *str;
	*str = c+1;
	return d;
}

static void prompt(void)
{
	printf("\e[92;1mlitex-xtrx\e[0m> ");
}

/*-----------------------------------------------------------------------*/
/* Help                                                                  */
/*-----------------------------------------------------------------------*/

static void help(void)
{
	puts("\nLiteX-XTRX firmware built "__DATE__" "__TIME__"\n");
	puts("Available commands:");
	puts("help               - Show this command");
	puts("reboot             - Reboot CPU");
	puts("i2c_test           - Test I2C Buses");
	puts("temp_test          - Test Temperature Sensor");
	puts("vctxo_test         - Test VCTXO");
	puts("rfic_test          - Test RFIC");
	puts("xtrx_init          - Initialize XTRX");
}

/*-----------------------------------------------------------------------*/
/* Commands                                                              */
/*-----------------------------------------------------------------------*/

static void reboot_cmd(void)
{
	ctrl_reset_write(1);
}

/*-----------------------------------------------------------------------*/
/* I2C                                                                   */
/*-----------------------------------------------------------------------*/

static void i2c_test(void)
{
	printf("I2C0 Scan...\n");
	i2c0_scan();

	printf("\n");

	printf("I2C1 Scan...\n");
	i2c1_scan();
}

/*-----------------------------------------------------------------------*/
/* Temperature                                                           */
/*-----------------------------------------------------------------------*/

static void temp_test(void)
{
	unsigned int temp;
	unsigned char dat[2];
	i2c1_read(TMP108_I2C_ADDR, 0x00, dat, 2, true);
	temp = (dat[0] << 4) | (dat[1] >> 4);
	temp = (62500*temp)/1000000; /* 0.0625°C/count */
	printf("Temperature: %d°C\n", temp);
}

/*-----------------------------------------------------------------------*/
/* VCTXO                                                                  */
/*-----------------------------------------------------------------------*/

/* TODO: Qualify LTC26X6 effect on VCTXO: +- XXppm */

static void vctxo_test(void)
{
	int i;
	int prev;
	int curr;
	prev = 0;
	vctxo_control_write(XTRX_VCTXO_CLK);
	for (i=0; i<2; i++) {
		vctxo_cycles_latch_write(1);
		curr = vctxo_cycles_read();
		if (i > 0)
			printf("VCTXO freq: %3d.%03dMHz\n", (curr - prev)/100000, ((curr - prev)/100)%1000);
		prev = curr;
		busy_wait(100);
	}
}

/*-----------------------------------------------------------------------*/
/* RFIC                                                                  */
/*-----------------------------------------------------------------------*/

static void rfic_test(void)
{
	int i;
	int prev;
	int curr;
	prev = 0;
	for (i=0; i<8; i++) {
		lms7002m_cycles_latch_write(1);
		curr = lms7002m_cycles_read();
		if (i > 0)
			printf("LMS7002M RX freq: %3d.%03dMHz\n", (curr - prev)/100000, ((curr - prev)/100)%1000);
		prev = curr;
		busy_wait(100);
	}
}

/*-----------------------------------------------------------------------*/
/* Init                                                                  */
/*-----------------------------------------------------------------------*/

static int xtrx_init(void)
{
	unsigned char adr;
	unsigned char dat;

	printf("PMICs Initialization...\n");
	printf("-----------------------\n");

	printf("PMIC-LMS: Check ID ");
	adr = 0x01;
	i2c0_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
	if (dat != 0xe0) {
		printf("KO, exiting.\n");
		return 0;
	} else {
		printf("OK.\n");
	}

	printf("PMIC-LMS: Enable Buck1.\n");
	adr = 0x04;
	dat = 0x88;
	i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

	printf("PMIC-LMS: Set Buck1 to 3280mV.\n");
	adr = 0x0c;
	dat = 0xfb;
	i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

	printf("PMIC-LMS: Disable Buck0.\n");
	adr = 0x02;
	dat = 0xc8;
	i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

	printf("PMIC-LMS: Disable Buck2.\n");
	adr = 0x06;
	dat = 0xc8;
	i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

	printf("PMIC-LMS: Disable Buck3.\n");
	adr = 0x08;
	dat = 0xc8;
	i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

	printf("PMIC-LMS: Set Buck0 to 1880mV.\n");
	adr = 0x0a;
	dat = 0xb5;
	i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

	printf("PMIC-LMS: Set Buck2 to 1480mV.\n");
	adr = 0x0e;
	dat = 0xa1;
	i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

	printf("PMIC-LMS: Set Buck3 to 1340mV.\n");
	adr = 0x10;
	dat = 0x92;
	i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

	busy_wait(1);

	printf("PMIC-FPGA: Check ID ");
	adr = 0x1;
	i2c1_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
	if (dat != 0xe0) {
		printf("KO, exiting.\n");
		return 0;
	} else {
		printf("OK.\n");
	}


	printf("PMIC-LMS: Enable Buck0.\n");
	adr = 0x02;
	dat = 0x88;
	i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

	printf("PMIC-LMS: Enable Buck2.\n");
	adr = 0x06;
	dat = 0x88;
	i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

	printf("PMIC-LMS: Enable Buck3.\n");
	adr = 0x08;
	dat = 0x88;
	i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

	printf("PMIC-FPGA: Set Buck1 to 1800mV.\n");
	adr = 0x0c;
	dat = 0xb1;
	i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);


#if 0
	printf("PMIC-LMS Dump...\n");
	for (adr=0; adr<32; adr++) {
		i2c0_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
		printf("0x%02x: 0x%02x\n", adr, dat);
	}
	printf("PMIC-FPGA Dump...\n");
	for (adr=0; adr<32; adr++) {
		i2c1_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
		printf("0x%02x: 0x%02x\n", adr, dat);
	}
#endif

	printf("\n");
	printf("VCTXO Initialization...\n");
	printf("----------------------\n");
	printf("Using VCTXO Clk.\n");
	vctxo_control_write(XTRX_VCTXO_CLK);

	printf("\n");
	printf("Board Tests...\n");
	printf("--------------\n");
	i2c_test();
	temp_test();
	vctxo_test();

	return 1;
}

/*-----------------------------------------------------------------------*/
/* Console service / Main                                                */
/*-----------------------------------------------------------------------*/

static void console_service(void)
{
	char *str;
	char *token;

	str = readstr();
	if(str == NULL) return;
	token = get_token(&str);
	if(strcmp(token, "help") == 0)
		help();
	else if(strcmp(token, "reboot") == 0)
		reboot_cmd();
	else if(strcmp(token, "xtrx_init") == 0)
		xtrx_init();
	else if(strcmp(token, "i2c_test") == 0)
		i2c_test();
	else if(strcmp(token, "temp_test") == 0)
		temp_test();
	else if(strcmp(token, "vctxo_test") == 0)
		vctxo_test();
	else if(strcmp(token, "rfic_test") == 0)
		rfic_test();
	prompt();
}

int main(void)
{
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setmask(0);
	irq_setie(1);
#endif
	uart_init();
	xtrx_init();

	help();
	prompt();

	while(1) {
		console_service();
	}

	return 0;
}