/*
 * INA228 - TI Current/Voltage/Power Monitor Code
 * Copyright (C) 2021 Craig Peacock
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#ifndef MAIN_INA228_H_
#define MAIN_INA228_H_


#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <byteswap.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include "ina228.h"

 /*
  * SHUNT_CAL is a conversion constant that represents the shunt resistance
  * used to calculate current value in Amps. This also sets the resolution
  * (CURRENT_LSB) for the current register.
  *
  * SHUNT_CAL is 15 bits wide (0 - 32768)
  *
  * SHUNT_CAL = 13107.2 x 10^6 x CURRENT_LSB x Rshunt
  *
  * CURRENT_LSB = Max Expected Current / 2^19
  */

  // normal
  //#define CURRENT_LSB 	0.15625
  // low
#define CURRENT_LSB 	0.0390625

#define SHUNT_CAL	1024




#define INA228_SLAVE_ADDRESS		0x4A

#define INA228_CONFIG			0x00
#define INA228_ADC_CONFIG		0x01
#define INA228_SHUNT_CAL		0x02
#define INA228_SHUNT_TEMPCO		0x03
#define INA228_VSHUNT			0x04
#define INA228_VBUS			0x05
#define INA228_DIETEMP			0x06
#define INA228_CURRENT			0x07
#define INA228_POWER			0x08
#define INA228_ENERGY			0x09
#define INA228_CHARGE			0x0A
#define INA228_DIAG_ALRT		0x0B
#define INA228_SOVL			0x0C
#define INA228_SUVL			0x0D
#define INA228_BOVL			0x0E
#define INA228_BUVL			0x0F
#define INA228_TEMP_LIMIT		0x10
#define INA228_PWR_LIMIT		0x11
#define INA228_MANUFACTURER_ID	0x3E
#define INA228_DEVICE_ID		0x3F

int i2c_init(char* devname);
int i2c_read_short(uint32_t i2c_master_port, uint8_t address, uint8_t command);
void i2c_write_short(uint32_t i2c_master_port, uint8_t address, uint8_t command, uint16_t data);
void i2c_read_buf(uint32_t i2c_master_port, uint8_t address, uint8_t command, uint8_t* buffer, uint8_t len);


void ina228_init(uint32_t i2c_master_port, uint8_t i2c_slave_addr);
float ina228_voltage(uint32_t i2c_master_port, uint8_t i2c_slave_addr);
float ina228_dietemp(uint32_t i2c_master_port, uint8_t i2c_slave_addr);
float ina228_shuntvoltage(uint32_t i2c_master_port, uint8_t i2c_slave_addr);
float ina228_current(uint32_t i2c_master_port, uint8_t i2c_slave_addr);
float ina228_power(uint32_t i2c_master_port, uint8_t i2c_slave_addr);
float ina228_energy(uint32_t i2c_master_port, uint8_t i2c_slave_addr);
float ina228_charge(uint32_t i2c_master_port, uint8_t i2c_slave_addr);



int i2c_init(char* devname)
{
	int handle;

	if ((handle = open(devname, O_RDWR)) < 0) {
		printf("Failed to open I2C port %s\r\n", devname);
		//exit(1);
	}
	printf("i2c_init %s success.\r\n", devname);
	return handle;
}

void i2c_write_short(uint32_t i2c_master_port, uint8_t address, uint8_t command, uint16_t data)
{
	uint8_t buffer[3];

	buffer[0] = command;
	buffer[1] = (data & 0xFF00) >> 8;
	buffer[2] = data & 0xFF;

	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data msgset[1];

	// Message Set 0: Write Command
	msgs[0].addr = address;
	msgs[0].flags = 0;
	msgs[0].len = 3;
	msgs[0].buf = buffer;

	// Message Set contains 1 messages
	msgset[0].msgs = msgs;
	msgset[0].nmsgs = 1;

	if (ioctl(i2c_master_port, I2C_RDWR, &msgset) < 0) {
		printf("Write I2C failed\r\n");
		//exit(1);
	}
}


int i2c_read_short(uint32_t i2c_master_port, uint8_t address, uint8_t command)
{
	uint16_t buffer;

	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data msgset[1];

	// Message Set 0: Write Command
	msgs[0].addr = address;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &command;

	// Message Set 1: Read 2 bytes
	msgs[1].addr = address;
	msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
	msgs[1].len = 2;
	msgs[1].buf = (unsigned char*)&buffer;

	// Message Set contains 2 messages
	msgset[0].msgs = msgs;
	msgset[0].nmsgs = 2;

	if (ioctl(i2c_master_port, I2C_RDWR, &msgset) < 0) {
		printf("Read I2C failed\r\n");
		//exit(1);
	}

	return(bswap_16(buffer));
}

void i2c_read_buf(uint32_t i2c_master_port, uint8_t address, uint8_t command, uint8_t* buffer, uint8_t len)
{
	//uint16_t buffer;

	struct i2c_msg msgs[2];
	struct i2c_rdwr_ioctl_data msgset[1];

	// Message Set 0: Write Command
	msgs[0].addr = address;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &command;

	// Message Set 1: Read len bytes
	msgs[1].addr = address;
	msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
	msgs[1].len = len;
	msgs[1].buf = buffer;

	// Message Set contains 2 messages
	msgset[0].msgs = msgs;
	msgset[0].nmsgs = 2;

	if (ioctl(i2c_master_port, I2C_RDWR, &msgset) < 0) {
		msgs[1].buf = 0;
		printf("Read I2C failed\r\n");
		//exit(1);
	}

}

////


void ina228_init(uint32_t i2c_master_port, uint8_t i2c_slave_addr)
{
	printf("ina228_init, i2c_master_port %d, i2c_slave_addr %d\r\n", i2c_master_port, i2c_slave_addr);
	i2c_write_short(i2c_master_port, i2c_slave_addr, INA228_CONFIG, 0x8000);	// Reset

	// normal
	i2c_write_short(i2c_master_port, i2c_slave_addr, INA228_CONFIG, 0x0000);
	// low
	//i2c_write_short(i2c_master_port, i2c_slave_addr, INA228_CONFIG, 0x0010);

	//printf("Manufacturer ID: 0x%04X\r\n",i2c_read_short(i2c_master_port, INA228_SLAVE_ADDRESS, INA228_MANUFACTURER_ID));
	//printf("Device ID:       0x%04X\r\n",i2c_read_short(i2c_master_port, INA228_SLAVE_ADDRESS, INA228_DEVICE_ID));
	printf("Manufacturer ID: 0x%04X\r\n", i2c_read_short(i2c_master_port, i2c_slave_addr, INA228_MANUFACTURER_ID));
	printf("Device ID:       0x%04X\r\n", i2c_read_short(i2c_master_port, i2c_slave_addr, INA228_DEVICE_ID));
	i2c_write_short(i2c_master_port, i2c_slave_addr, INA228_SHUNT_CAL, SHUNT_CAL);
	printf("\r\n");
}

float ina228_voltage(uint32_t i2c_master_port, uint8_t i2c_slave_addr)
{
	int32_t iBusVoltage;
	float fBusVoltage;
	bool sign;

	i2c_read_buf(i2c_master_port, i2c_slave_addr, INA228_VBUS, (uint8_t*)&iBusVoltage, 3);
	printf("iBusVoltage [%x]", iBusVoltage);
	sign = iBusVoltage & 0x80;
	//iBusVoltage = bswap_32(iBusVoltage & 0xFFFFFF) >> 12;

	iBusVoltage = (bswap_32(iBusVoltage & 0xFFFFFF));
	printf("-> [%x]", iBusVoltage);
	iBusVoltage = (iBusVoltage >> 12);


	printf("-> [%x]\n", iBusVoltage);

	if (sign)
		iBusVoltage += 0xFFF00000;
	//iBusVoltage = (iBusVoltage >> 4) & 0x0fffff;
	fBusVoltage = (iBusVoltage) * 0.0001953125;

	return (fBusVoltage);
}

float ina228_dietemp(uint32_t i2c_master_port, uint8_t i2c_slave_addr)
{
	uint16_t iDieTemp;
	float fDieTemp;

	iDieTemp = i2c_read_short(i2c_master_port, i2c_slave_addr, INA228_DIETEMP);
	fDieTemp = (iDieTemp) * 0.0078125;

	return (fDieTemp);
}

float ina228_shuntvoltage(uint32_t i2c_master_port, uint8_t i2c_slave_addr)
{
	int32_t iShuntVoltage;
	float fShuntVoltage;
	bool sign;

	i2c_read_buf(i2c_master_port, i2c_slave_addr, INA228_VSHUNT, (uint8_t*)&iShuntVoltage, 3);
	sign = iShuntVoltage & 0x80;
	iShuntVoltage = bswap_32(iShuntVoltage & 0xFFFFFF) >> 12;
	if (sign) iShuntVoltage += 0xFFF00000;

	fShuntVoltage = (iShuntVoltage) * 0.0003125;		// Output in mV when ADCRange = 0
	//fShuntVoltage = (iShuntVoltage) * 0.000078125;	// Output in mV when ADCRange = 1

	return (fShuntVoltage);
}

float ina228_current(uint32_t i2c_master_port, uint8_t i2c_slave_addr)
{
	int32_t iCurrent;
	float fCurrent;
	bool sign;

	i2c_read_buf(i2c_master_port, i2c_slave_addr, INA228_CURRENT, (uint8_t*)&iCurrent, 3);
	sign = iCurrent & 0x80;
	iCurrent = bswap_32(iCurrent & 0xFFFFFF) >> 12;
	if (sign) iCurrent += 0xFFF00000;
	fCurrent = (iCurrent)*CURRENT_LSB;

	return (fCurrent);
}

float ina228_power(uint32_t i2c_master_port, uint8_t i2c_slave_addr)
{
	uint32_t iPower;
	float fPower;

	i2c_read_buf(i2c_master_port, i2c_slave_addr, INA228_POWER, (uint8_t*)&iPower, 3);
	iPower = bswap_32(iPower & 0xFFFFFF) >> 8;
	fPower = 3.2 * CURRENT_LSB * iPower;

	return (fPower);
}

/*
 * Returns energy in Joules.
 * 1 Watt = 1 Joule per second
 * 1 W/hr = Joules / 3600
 */

float ina228_energy(uint32_t i2c_master_port, uint8_t i2c_slave_addr)
{
	uint64_t iEnergy;
	float fEnergy;

	i2c_read_buf(i2c_master_port, i2c_slave_addr, INA228_ENERGY, (uint8_t*)&iEnergy, 5);
	iEnergy = bswap_64(iEnergy & 0xFFFFFFFFFF) >> 24;

	fEnergy = 16 * 3.2 * CURRENT_LSB * iEnergy;

	return (fEnergy);
}

/*
 * Returns electric charge in Coulombs.
 * 1 Coulomb = 1 Ampere per second.
 * Hence Amp-Hours (Ah) = Coulombs / 3600
 */

float ina228_charge(uint32_t i2c_master_port, uint8_t i2c_slave_addr)
{
	int64_t iCharge;
	float fCharge;
	bool sign;

	i2c_read_buf(i2c_master_port, i2c_slave_addr, INA228_CHARGE, (uint8_t*)&iCharge, 5);
	sign = iCharge & 0x80;
	iCharge = bswap_64(iCharge & 0xFFFFFFFFFF) >> 24;
	if (sign) iCharge += 0xFFFFFF0000000000;

	fCharge = CURRENT_LSB * iCharge;

	return (fCharge);
}


#endif
