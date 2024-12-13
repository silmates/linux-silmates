// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments ADS1120 ADC
 *
 * Copyright (c) 2020 SILMATES PRIVATE LIMITED
 *   Jigar Patel <jigar@silmates.com>
 *
 * Datasheet: https://www.ti.com/lit/ds/symlink/ads1120.pdf
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <asm/unaligned.h>

/* Commands */
#define ADS1120_CMD_RESET		0x06
#define ADS1120_CMD_START		0x08
#define ADS1120_CMD_STOP		0x0A
#define ADS1120_CMD_POWERDOWN	0x02
#define ADS1120_CMD_RDATA		0x12
#define ADS1120_CMD_RREG(r,n)		(BIT(5) | ((r & GENMASK(1, 0)) << 2) | (n & GENMASK(1, 0)))
#define ADS1120_CMD_WREG(r,n)		(BIT(6) | ((r & GENMASK(1, 0)) << 2) | (n & GENMASK(1, 0)))

#define ADS1120_CFG0_REG	0x00
#define ADS1120_CFG1_REG	0x01
#define ADS1120_CFG2_REG	0x02
#define ADS1120_CFG3_REG	0x03

#define ADS1120_CFG0_MUX_MASK	GENMASK(7, 4)
#define ADS1120_CFG0_GAIN_MASK	GENMASK(3, 1)
#define ADS1120_CFG0_PGA_BYPASS_MASK	BIT(0)

#define ADS1120_CFG1_DR_MASK	GENMASK(7, 5)
#define ADS1120_CFG1_MODE_MASK	GENMASK(4, 3)
#define ADS1120_CFG1_CM_MASK	BIT(2)
#define ADS1120_CFG1_TS_MASK	BIT(1)
#define ADS1120_CFG1_BCS_MASK	BIT(0)

#define ADS1120_CFG2_VREF_MASK	GENMASK(7, 6)
#define ADS1120_CFG2_50_60_MASK	GENMASK(5, 4)
#define ADS1120_CFG2_PSW_MASK	BIT(3)
#define ADS1120_CFG2_IDAC_MASK	GENMASK(2, 0)

#define ADS1120_CFG3_I1MUX_MASK	GENMASK(7, 5)
#define ADS1120_CFG3_I2MUX_MASK	GENMASK(4, 2)
#define ADS1120_CFG3_DRDYM_MASK	BIT(1)

#define ADS1120_CFG0_MUX_SHIFT		4
#define ADS1120_CFG0_GAIN_SHIFT		1
#define ADS1120_CFG0_PGA_BYPASS_SHIFT 	0

#define ADS1120_CFG1_DR_SHIFT	5
#define ADS1120_CFG1_MODE_SHIFT	3
#define ADS1120_CFG1_CM_SHIFT	2
#define ADS1120_CFG1_TS_SHIFT	1
#define ADS1120_CFG1_BCS_SHIFT	0

#define ADS1120_CFG2_VREF_SHIFT		6
#define ADS1120_CFG2_50_60_SHIFT	4
#define ADS1120_CFG2_PSW_SHIFT		3
#define ADS1120_CFG2_IDAC_SHIFT		0

#define ADS1120_CFG3_I1MUX_SHIFT	5
#define ADS1120_CFG3_I2MUX_SHIFT	2
#define ADS1120_CFG3_DRDYM_SHIFT	1

/* Operating mode */
#define ADS1120_CFG1_MODE_NORMAL	0
#define ADS1120_CFG1_MODE_DUTY_CYCLE	1
#define ADS1120_CFG1_MODE_TURBO		2

/* PGA Bypass */
#define ADS1120_CFG0_PGA_BYPASS_ENABLE	0
#define ADS1120_CFG0_PGA_BYPASS_DISABLE	1

/* Comparator mode field */
#define ADS1120_CFG_COMP_MODE_TRAD	0
#define ADS1120_CFG_COMP_MODE_WINDOW	1

/* device operating modes */
#define ADS1120_CFG1_CM_CONTINUOUS	0
#define ADS1120_CFG1_CM_SINGLESHOT	1

/* Temperature sensor mode */
#define ADS1120_CFG1_TS_DISABLE	0
#define ADS1120_CFG1_TS_ENABLE	1

/* burn out current sources */
#define ADS1120_CFG1_BCS_OFF	0
#define ADS1120_CFG1_BCS_ON	1

/* volatage reference selection */
#define ADS1120_CFG2_VREF_INT			0
#define ADS1120_CFG2_VREF_DEDICATED_PINS	1
#define ADS1120_CFG2_VREF_SELECTIVE_PINS	2
#define ADS1120_CFG2_VREF_ANALOG		3

#define ADS1120_SLEEP_DELAY_MS		2000
#define ADS1120_DEFAULT_PGA_GAIN	1
#define ADS1120_DEFAULT_DATA_RATE	20
#define ADS1120_DEFAULT_IDAC		0
#define ADS1120_DEFAULT_IDAC_uA		0
#define ADS1120_DEFAULT_MUX			8
#define ADS1120_DEFAULT_IDAC_MUX	0


#define ADS1120_VREF_2V048_mV	2.048

#define ADS1120_MAX_CHANNELS	4

#define ADS1120_WAIT_RESET_CYCLES	18
#define ADS1120_MAX_SETTLING_TIME_MS	6

#define ADS1120_NUM_DATA_BYTES_MAX	2


enum ads1120_ids {
	ads1120,
};

enum ads1120_operating_mode {
	ads1120_mode_normal,
	ads1120_mode_duty_cycle,
	ads1120_mode_turbo,
};

enum ads1120_idac_channel {
	ads1120_idac1,
	ads1120_idac2,
};

struct ads1120_info {
	unsigned int max_channels;
	const char *name;
};

struct ads1120_channel_config {
	unsigned int raw_value;
	unsigned int data_rate;
	unsigned int idac;
	unsigned int idac_ua;
	unsigned int idac_mux;
	unsigned int pga_gain;
	unsigned int mux;
	unsigned int r_sense_val;
	bool pga_bypass;
};

struct ads1120_data_rate_desc {
	unsigned int rate;  /* data rate in kSPS */
	u8 reg;             /* reg value */
};

static const struct ads1120_data_rate_desc ads1120_data_rate_normal_tbl[] = {
	{ .rate = 20,    .reg = 0x00 },
	{ .rate = 45,    .reg = 0x01 },
	{ .rate = 90,    .reg = 0x02 },
	{ .rate = 175,   .reg = 0x03 },
	{ .rate = 330,   .reg = 0x04 },
	{ .rate = 600,   .reg = 0x05 },
	{ .rate = 1000,  .reg = 0x06 },
};
/*
static const struct ads1120_data_rate_desc ads1120_data_rate_duty_cycle_tbl[] = {
	{ .rate = 5,     .reg = 0x00 },
	{ .rate = 11.25, .reg = 0x01 },
	{ .rate = 22.5,  .reg = 0x02 },
	{ .rate = 44,    .reg = 0x03 },
	{ .rate = 82.5,  .reg = 0x04 },
	{ .rate = 150,   .reg = 0x05 },
	{ .rate = 250,   .reg = 0x06 },
};
*/
static const struct ads1120_data_rate_desc ads1120_data_rate_turbo_tbl[] = {
	{ .rate = 40,    .reg = 0x00 },
	{ .rate = 90,    .reg = 0x01 },
	{ .rate = 180,   .reg = 0x02 },
	{ .rate = 350,   .reg = 0x03 },
	{ .rate = 660,   .reg = 0x04 },
	{ .rate = 1200,  .reg = 0x05 },
	{ .rate = 2000,  .reg = 0x06 },
};

struct ads1120_pga_gain_desc {
	unsigned int gain;  /* PGA gain value */
	u8 reg;             /* field value */
};

static const struct ads1120_pga_gain_desc ads1120_pga_gain_tbl[] = {
	{ .gain = 1,   .reg = 0x00 },
	{ .gain = 2,   .reg = 0x01 },
	{ .gain = 4,   .reg = 0x02 },
	{ .gain = 8,   .reg = 0x03 },
	{ .gain = 16,  .reg = 0x04 },
	{ .gain = 32,  .reg = 0x05 },
	{ .gain = 64,  .reg = 0x06 },
	{ .gain = 128, .reg = 0x07 },
};

struct ads1120_idac_desc {
	unsigned int uA;  /* PGA gain value */
	u8 reg;             /* field value */
};

static const struct ads1120_idac_desc ads1120_idac_tbl[] = {
	{ .uA = 0,    .reg = 0x00 },
	{ .uA = 50,   .reg = 0x02 },
	{ .uA = 100,  .reg = 0x03 },
	{ .uA = 250,  .reg = 0x04 },
	{ .uA = 500,  .reg = 0x05 },
	{ .uA = 1000, .reg = 0x06 },
	{ .uA = 1500, .reg = 0x07 },
};

static const u8 ads1120_valid_channel_mux_values[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

struct ads1120_state {
	const struct ads1120_info *info;
	struct spi_device *spi;
	struct iio_trigger *trig;
	struct clk *adc_clk;
	struct regulator *vref_reg;
	struct ads1120_channel_config *channel_config;
	struct ads1120_data_rate_desc * data_rate_tble;
	unsigned int data_rate;
	unsigned int vref;
	unsigned int vref_mv;
	unsigned int sdecode_delay_us;
	unsigned int reset_delay_us;
	unsigned int readback_len;
	unsigned int operating_mode;

	struct completion completion;

	struct {
		u16 data[ADS1120_MAX_CHANNELS];
		s64 ts __aligned(8);
	} tmp_buf;

	u8 tx_buf[3] ____cacheline_aligned;
	/*
	 * Add extra one padding byte to be able to access the last channel
	 * value using u32 pointer
	 */
	u8 rx_buf[ADS1120_NUM_DATA_BYTES_MAX + 1];
};

static const struct ads1120_info ads1120_info_tbl[] = {
	[ads1120] = {
		.max_channels = 4,
		.name = "ads1120",
	},
};

static int ads1120_exec_cmd(struct ads1120_state *st, u8 cmd)
{
	int ret;

	ret = spi_write_then_read(st->spi, &cmd, 1, NULL, 0);
	if (ret)
		dev_err(&st->spi->dev, "Exec cmd(%02x) failed\n", cmd);

	return ret;
}

static int ads1120_read_reg(struct ads1120_state *st, u8 reg)
{
	int ret;
	struct spi_transfer transfer[] = {
		{
			.tx_buf = &st->tx_buf,
			.len = 1,
		}, {
			.rx_buf = &st->rx_buf,
			.len = 1,
		},
	};

	st->tx_buf[0] = ADS1120_CMD_RREG(reg,0);

	ret = spi_sync_transfer(st->spi, transfer, ARRAY_SIZE(transfer));
	if (ret) {
		dev_err(&st->spi->dev, "Read register failed\n");
		return ret;
	}

	return st->rx_buf[0];
}

static int ads1120_write_reg(struct ads1120_state *st, u8 reg, u8 value)
{
	int ret;
	struct spi_transfer transfer[] = {
		{
			.tx_buf = &st->tx_buf,
			.len = 2,
		}
	};

	st->tx_buf[0] = ADS1120_CMD_WREG(reg,0);
	st->tx_buf[1] = value;

	ret = spi_sync_transfer(st->spi, transfer, ARRAY_SIZE(transfer));
	if (ret)
		dev_err(&st->spi->dev, "Write register failed\n");

	return ret;
}

static int ads1120_read_data(struct ads1120_state *st, int rx_len)
{
	int ret;
	struct spi_transfer transfer[] = {
		{
			.tx_buf = &st->tx_buf,
			.len = 1,
		}, {
			.rx_buf = &st->rx_buf,
			.len = rx_len,
		},
	};

	st->tx_buf[0] = ADS1120_CMD_RDATA;

	ret = spi_sync_transfer(st->spi, transfer, ARRAY_SIZE(transfer));
	if (ret)
		dev_err(&st->spi->dev, "Read data failed\n");

	return ret;
}

static int ads1120_data_rate_to_field_value(struct ads1120_state *st,
	unsigned int data_rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads1120_data_rate_normal_tbl); i++) {
		if (ads1120_data_rate_normal_tbl[i].rate == data_rate)
			break;
	}

	if (i == ARRAY_SIZE(ads1120_data_rate_normal_tbl)) {
		dev_err(&st->spi->dev, "invalid datarate value\n");
		return -EINVAL;
	}

	return ads1120_data_rate_normal_tbl[i].reg;
}

static int ads1120_set_data_rate(struct ads1120_state *st, unsigned int data_rate)
{
	int i, reg, ret,read;

	for (i = 0; i < ARRAY_SIZE(ads1120_data_rate_normal_tbl); i++) {
		if (ads1120_data_rate_normal_tbl[i].rate == data_rate)
			break;
	}

	if (i == ARRAY_SIZE(ads1120_data_rate_normal_tbl)) {
		dev_err(&st->spi->dev, "invalid data rate value\n");
		return -EINVAL;
	}

	read = ads1120_read_reg(st, ADS1120_CFG1_REG);
	if (read < 0)
		return read;

	reg = read;
	reg &= ~ADS1120_CFG1_DR_MASK;
	reg |= FIELD_PREP(ADS1120_CFG1_DR_MASK,
		ads1120_data_rate_normal_tbl[i].reg);

	ret = ads1120_write_reg(st, ADS1120_CFG1_REG, reg);
	if (ret)
		return ret;

	st->data_rate = data_rate;

	return 0;
}

static int ads1120_pga_gain_to_field_value(struct ads1120_state *st,
	unsigned int pga_gain)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads1120_pga_gain_tbl); i++) {
		if (ads1120_pga_gain_tbl[i].gain == pga_gain)
			break;
	}

	if (i == ARRAY_SIZE(ads1120_pga_gain_tbl)) {
		dev_err(&st->spi->dev, "invalid PGA gain value\n");
		return -EINVAL;
	}

	return ads1120_pga_gain_tbl[i].reg;
}

static int ads1120_set_pga_gain(struct ads1120_state *st,
	unsigned int channel,unsigned int pga_gain)
{
	int field_value, reg, read;

	field_value = ads1120_pga_gain_to_field_value(st, pga_gain);
	if (field_value < 0)
		return field_value;

	read = ads1120_read_reg(st, ADS1120_CFG0_REG);
	if (read < 0)
		return read;

	reg = read;
	reg &= ~ADS1120_CFG0_GAIN_MASK;
	reg |= FIELD_PREP(ADS1120_CFG0_GAIN_MASK, field_value);

	reg &= ~ADS1120_CFG0_PGA_BYPASS_MASK;
	reg |= FIELD_PREP(ADS1120_CFG0_PGA_BYPASS_MASK, st->channel_config[channel].pga_bypass);

	return ads1120_write_reg(st, ADS1120_CFG0_REG, reg);
}

static int ads1120_set_idac_mux(struct ads1120_state *st,
	unsigned int idac_channel,unsigned int idac_mux)
{
	int reg,read;

	read = ads1120_read_reg(st, ADS1120_CFG3_REG);
	if (read < 0)
		return read;

	reg = read;
	if(idac_channel == ads1120_idac1){
		reg &= ~ADS1120_CFG3_I1MUX_MASK;
		reg |= FIELD_PREP(ADS1120_CFG3_I1MUX_MASK, idac_mux);
	} else {
		reg &= ~ADS1120_CFG3_I2MUX_MASK;
		reg |= FIELD_PREP(ADS1120_CFG3_I2MUX_MASK, idac_mux);
	}

	return ads1120_write_reg(st, ADS1120_CFG3_REG, reg);
}

static int ads1120_idac_ua_to_field_value(struct ads1120_state *st,
	unsigned int idac_ua)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads1120_idac_tbl); i++) {
		if (ads1120_idac_tbl[i].uA == idac_ua)
			break;
	}

	if (i == ARRAY_SIZE(ads1120_idac_tbl)) {
		dev_err(&st->spi->dev, "invalid IDAC uA value\n");
		return -EINVAL;
	}

	return ads1120_idac_tbl[i].reg;
}

static int ads1120_set_idac_ua(struct ads1120_state *st,
	unsigned int idac_ua)
{
	int field_value, reg, read;

	field_value = ads1120_idac_ua_to_field_value(st, idac_ua);
	if (field_value < 0)
		return field_value;

	read = ads1120_read_reg(st, ADS1120_CFG2_REG);
	if (read < 0)
		return read;

	reg = read;
	reg &= ~ADS1120_CFG2_IDAC_MASK;
	reg |= FIELD_PREP(ADS1120_CFG2_IDAC_MASK, field_value);

	return ads1120_write_reg(st, ADS1120_CFG2_REG, reg);
}

static int ads1120_validate_channel_mux(struct ads1120_state *st,
	unsigned int mux)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads1120_valid_channel_mux_values); i++) {
		if (ads1120_valid_channel_mux_values[i] == mux)
			break;
	}

	if (i == ARRAY_SIZE(ads1120_valid_channel_mux_values)) {
		dev_err(&st->spi->dev, "invalid channel mux value\n");
		return -EINVAL;
	}

	return 0;
}

static int ads1120_set_channel_mux(struct ads1120_state *st,
	unsigned int channel, unsigned int mux)
{
	int reg,read;

	read = ads1120_read_reg(st, ADS1120_CFG0_REG);
	if (read < 0)
		return read;

	reg = read;
	reg &= ~ADS1120_CFG0_MUX_MASK;
	reg |= FIELD_PREP(ADS1120_CFG0_MUX_MASK, mux);

	return ads1120_write_reg(st, ADS1120_CFG0_REG, reg);
}

static int ads1120_config_reference_voltage(struct ads1120_state *st)
{
	int reg,read;

	read = ads1120_read_reg(st, ADS1120_CFG2_REG);
	if (read < 0)
		return read;

	reg = read;
	reg &= ~ADS1120_CFG2_VREF_MASK;
	reg |= FIELD_PREP(ADS1120_CFG2_VREF_MASK,
		st->vref);

	return ads1120_write_reg(st, ADS1120_CFG2_REG, reg);
}

static int ads1120_initial_config(struct iio_dev *indio_dev)
{
	struct ads1120_state *st = iio_priv(indio_dev);
	int ret;

	ret = ads1120_exec_cmd(st, ADS1120_CMD_RESET);
	if (ret)
		return ret;

	udelay(st->reset_delay_us);

	ret = ads1120_set_data_rate(st, ADS1120_DEFAULT_DATA_RATE);
	if (ret)
		return ret;

	ret = ads1120_config_reference_voltage(st);
	if (ret)
		return ret;

	return ret;
}

static int ads1120_read_channel_data(struct ads1120_state *st,unsigned int channel, u16 * value)
{
	int ret;

	ret = ads1120_set_channel_mux(st, channel,st->channel_config[channel].mux);
	if (ret) {
		dev_err(&st->spi->dev, "Set CH : %d mux failed\n",channel);
		return ret;
	}

	ret = ads1120_set_pga_gain(st, channel, st->channel_config[channel].pga_gain);
	if (ret) {
		dev_err(&st->spi->dev, "Set CH : %d pga gain failed\n",channel);
		return ret;
	}

	ret = ads1120_set_idac_ua(st, st->channel_config[channel].idac_ua);
	if (ret) {
		dev_err(&st->spi->dev, "Set CH : %d IDAC uA failed\n",channel);
		return ret;
	}

	ret = ads1120_set_idac_mux(st, 
		st->channel_config[channel].idac, 
		st->channel_config[channel].idac_mux);
	if (ret) {
		dev_err(&st->spi->dev, "Set CH : %d IDAC mux failed\n",channel);
		return ret;
	}

	ret = ads1120_exec_cmd(st, ADS1120_CMD_START);
	if (ret)
		return ret;

	ret = ads1120_read_data(st, 2);
	if (ret)
		return ret;

	*value = get_unaligned_be16(&st->rx_buf[0]);

	return ads1120_exec_cmd(st, ADS1120_CMD_STOP);
}

static int ads1120_set_scale(struct ads1120_state *data,
			     struct iio_chan_spec const *chan,
			     int scale)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads1120_pga_gain_tbl); i++) {
		if (ads1120_pga_gain_tbl[i].gain == scale) {
			data->channel_config[chan->channel].pga_gain = scale;
			if(scale == 1){
				data->channel_config[chan->channel].pga_bypass = true;
			} else {
				data->channel_config[chan->channel].pga_bypass = false;
			}
			return 0;
		}
	}

	return -EINVAL;
}

static int ads1120_set_idac(struct ads1120_state *data,
			     struct iio_chan_spec const *chan,
			     int bias)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ads1120_idac_tbl); i++) {
		if (ads1120_idac_tbl[i].uA == bias) {
			data->channel_config[chan->channel].idac_ua = bias;
			return 0;
		}
	}

	return -EINVAL;
}


static int ads1120_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *channel, int *value,
	int *value2, long mask)
{
	struct ads1120_state *st = iio_priv(indio_dev);
	int ret;
	u16 local_value;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		// ret = iio_device_claim_direct_mode(indio_dev);
		// if (ret)
		// 	return ret;

		ret = ads1120_read_channel_data(st, channel->channel, &local_value);
		st->channel_config[channel->channel].raw_value = local_value;
		*value = local_value;
		// iio_device_release_direct_mode(indio_dev);
		// if (ret)
		// 	return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*value = st->channel_config[channel->channel].pga_gain;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_CALIBBIAS:
		*value = st->channel_config[channel->channel].idac_ua;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*value = st->data_rate;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int ads1120_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *channel, int value,
	int value2, long mask)
{
	struct ads1120_state *st = iio_priv(indio_dev);
	int ret,i;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		for(i=0 ; i<indio_dev->num_channels ; i++)
		{
			ret = ads1120_set_data_rate(st, value);
		}
		return ret;

	case IIO_CHAN_INFO_SCALE:
		for(i=0 ; i<indio_dev->num_channels ; i++)
		{
			ret = ads1120_set_scale(st, &indio_dev->channels[i], value);
		}
		return ret;

	case IIO_CHAN_INFO_CALIBBIAS:
		for(i=0 ; i<indio_dev->num_channels ; i++)
		{
			ret = ads1120_set_idac(st, &indio_dev->channels[i], value);
		}
		return ret;

	default:
		return -EINVAL;
	}
}

static int ads1120_debugfs_reg_access(struct iio_dev *indio_dev,
	unsigned int reg, unsigned int writeval, unsigned int *readval)
{
	struct ads1120_state *st = iio_priv(indio_dev);

	if (readval) {
		int ret = ads1120_read_reg(st, reg);
		*readval = ret;
		return ret;
	}

	return ads1120_write_reg(st, reg, writeval);
}

static IIO_CONST_ATTR_NAMED(ads1120_gain_available, scale_available,
	"1 2 4 8 16 32 64 128");

static IIO_CONST_ATTR_NAMED(ads1120_bias_available, bias_available,
	"0 50 100 250 500 1000 1500");

static struct attribute *ads1120_attributes[] = {
	&iio_const_attr_ads1120_gain_available.dev_attr.attr,
	&iio_const_attr_ads1120_bias_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ads1120_attribute_group = {
	.attrs = ads1120_attributes,
};

static const struct iio_info ads1120_iio_info = {
	.read_raw = ads1120_read_raw,
	.write_raw = ads1120_write_raw,
	.debugfs_reg_access = &ads1120_debugfs_reg_access,
	.attrs = &ads1120_attribute_group,
};

static int ads1120_set_trigger_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct ads1120_state *st = iio_priv(indio_dev);
	u8 cmd = state ? ADS1120_CMD_START : ADS1120_CMD_STOP;

	return ads1120_exec_cmd(st, cmd);
}

static const struct iio_trigger_ops ads1120_trigger_ops = {
	.set_trigger_state = &ads1120_set_trigger_state,
	.validate_device = &iio_trigger_validate_own_device,
};
/*
static irqreturn_t ads1120_trigger_handler(int irq, void *private)
{
	struct iio_poll_func *pf = private;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ads1120_state *st = iio_priv(indio_dev);
	unsigned int chn, i = 0;
	// u8 *src, *dest;
	int ret;
	u16 value = 0;

	for_each_set_bit(chn, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		ret = ads1120_read_channel_data(st, chn , &value);
		if (ret)
			dev_err(&st->spi->dev, "Read ADC CH - %d failed \n",chn);

		st->tmp_buf.data[chn] = value;
		i++;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, st->tmp_buf.data,
		iio_get_time_ns(indio_dev));

	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static irqreturn_t ads1120_interrupt(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct ads1120_state *st = iio_priv(indio_dev);

	if (iio_buffer_enabled(indio_dev) && iio_trigger_using_own(indio_dev))
		iio_trigger_poll(st->trig);
	else
		complete(&st->completion);

	return IRQ_HANDLED;
}
*/
#define ADS1120_CHAN(__type, index, __address) ({ \
	struct iio_chan_spec __chan = { \
		.type = __type, \
		.indexed = 1, \
		.channel = index, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) , \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_CALIBBIAS), \
		.scan_index = index,					\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 16,					\
			.storagebits = 32,				\
			.shift = 8,					\
			.endianness = IIO_BE,				\
		},							\
	}; \
	__chan; \
})

static int ads1120_alloc_channels(struct iio_dev *indio_dev)
{
	struct ads1120_state *st = iio_priv(indio_dev);
	struct ads1120_channel_config *channel_config;
	struct device *dev = &st->spi->dev;
	struct iio_chan_spec *channels;
	struct fwnode_handle *node;
	unsigned int channel, tmp;
	int num_channels, i, ret;

	ret = device_property_read_u32(dev, "ti,mode", &tmp);
	if (ret)
		tmp = 0;

	switch (tmp) {
	case 0:
		st->operating_mode = ads1120_mode_normal;
		break;
	case 1:
		st->operating_mode = ads1120_mode_duty_cycle;
		break;
	case 2:
		st->operating_mode = ads1120_mode_turbo;
		break;
	default:
		dev_err(&st->spi->dev, "invalid operating mode\n");
		return -EINVAL;
	}

	ret = device_property_read_u32(dev, "ti,vref", &tmp);
	if (ret)
		tmp = 0;

	st->vref = tmp;
	// switch (tmp) {
	// case 0:
	// 	st->vref_mv = ADS1120_VREF_2V048_mV;
	// 	break;
	// // case 1:
	// // 	st->vref_mv = ADS1120_VREF_4V_mV;
	// // 	break;
	// default:
	// 	dev_err(&st->spi->dev, "invalid internal voltage reference\n");
	// 	return -EINVAL;
	// }

	num_channels = device_get_child_node_count(dev);
	if (num_channels == 0) {
		dev_err(&st->spi->dev, "no channel children\n");
		return -ENODEV;
	}

	if (num_channels > st->info->max_channels) {
		dev_err(&st->spi->dev, "num of channel children out of range\n");
		return -EINVAL;
	}

	channels = devm_kcalloc(&st->spi->dev, num_channels,
		sizeof(*channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	channel_config = devm_kcalloc(&st->spi->dev, num_channels,
		sizeof(*channel_config), GFP_KERNEL);
	if (!channel_config)
		return -ENOMEM;

	i = 0;
	device_for_each_child_node(dev, node) {
		ret = fwnode_property_read_u32(node, "reg", &channel);
		if (ret)
			goto err_child_out;

		ret = fwnode_property_read_u32(node, "ti,gain", &tmp);
		if (ret) {
			channel_config[i].pga_gain = ADS1120_DEFAULT_PGA_GAIN;
		} else {
			ret = ads1120_pga_gain_to_field_value(st, tmp);
			if (ret < 0)
				goto err_child_out;

			channel_config[i].pga_gain = tmp;
		}

		if (fwnode_property_read_bool(node, "ti,gain-bypass"))
			channel_config[i].pga_bypass = true;

		ret = fwnode_property_read_u32(node, "ti,datarate", &tmp);
		if (ret) {
			channel_config[i].data_rate = ADS1120_DEFAULT_DATA_RATE;
		} else {
			ret = ads1120_data_rate_to_field_value(st, tmp);
			if (ret < 0)
				goto err_child_out;

			channel_config[i].data_rate = tmp;
		}

		ret = fwnode_property_read_u32(node, "ti,idac", &tmp);
		if (ret) {
			channel_config[i].idac = ADS1120_DEFAULT_IDAC;
		} else {
			channel_config[i].idac = tmp;
		}

		ret = fwnode_property_read_u32(node, "ti,idac-ua", &tmp);
		if (ret) {
			channel_config[i].idac_ua = ADS1120_DEFAULT_IDAC_uA;
		} else {
			ret = ads1120_idac_ua_to_field_value(st, tmp);
			if (ret < 0)
				goto err_child_out;

			channel_config[i].idac_ua = tmp;
		}

		ret = fwnode_property_read_u32(node, "ti,idac-mux", &tmp);
		if (ret) {
			channel_config[i].idac_mux = ADS1120_DEFAULT_IDAC_MUX;
		} else {
			channel_config[i].idac_mux = tmp;
		}

		ret = fwnode_property_read_u32(node, "ti,mux", &tmp);
		if (ret) {
			channel_config[i].mux = ADS1120_DEFAULT_MUX;
		} else {
			ret = ads1120_validate_channel_mux(st, tmp);
			if (ret)
				goto err_child_out;

			channel_config[i].mux = tmp;
		}

		ret = fwnode_property_read_u32(node, "ti,rsense-val-milli-ohms", &tmp);
		if (ret) {
			channel_config[i].r_sense_val = 1;
		} else {
			/* Times 1000 because we have milli-ohms */
			channel_config[i].r_sense_val = tmp;
		}

		channels[i] = ADS1120_CHAN(IIO_TEMP,channel,i);
		i++;
	}

	indio_dev->channels = channels;
	indio_dev->num_channels = num_channels;
	st->channel_config = channel_config;

	return 0;

err_child_out:
	fwnode_handle_put(node);
	return ret;
}

static int ads1120_probe(struct spi_device *spi)
{
	const struct ads1120_info *info;
	struct ads1120_state *st;
	struct iio_dev *indio_dev;
	int ret;

	info = device_get_match_data(&spi->dev);
	if (!info) {
		dev_err(&spi->dev, "failed to get match data\n");
		return -ENODEV;
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev) {
		dev_err(&spi->dev, "failed to allocate IIO device\n");
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);
	st->info = info;
	st->spi = spi;

	ret = ads1120_alloc_channels(indio_dev);
	if (ret)
		return ret;

	indio_dev->name = st->info->name;
	indio_dev->info = &ads1120_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	// init_completion(&st->completion);

	// if (spi->irq) {
	// 	ret = devm_request_irq(&spi->dev, spi->irq,
	// 		ads1120_interrupt,
	// 		IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	// 		spi->dev.driver->name, indio_dev);
	// 	if (ret)
	// 		return dev_err_probe(&spi->dev, ret,
	// 				     "request irq failed\n");
	// } else {
	// 	dev_err(&spi->dev, "data ready IRQ missing\n");
	// 	return -ENODEV;
	// }

	// st->trig = devm_iio_trigger_alloc(&spi->dev, "%s-dev%d",
	// 	indio_dev->name, iio_device_id(indio_dev));
	// if (!st->trig) {
	// 	dev_err(&spi->dev, "failed to allocate IIO trigger\n");
	// 	return -ENOMEM;
	// }

	// st->trig->ops = &ads1120_trigger_ops;
	// st->trig->dev.parent = &spi->dev;
	// iio_trigger_set_drvdata(st->trig, indio_dev);
	// ret = devm_iio_trigger_register(&spi->dev, st->trig);
	// if (ret) {
	// 	dev_err(&spi->dev, "failed to register IIO trigger\n");
	// 	return -ENOMEM;
	// }

	// indio_dev->trig = iio_trigger_get(st->trig);

	// ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
	// 	NULL, &ads1120_trigger_handler, NULL);
	// if (ret) {
	// 	dev_err(&spi->dev, "failed to setup IIO buffer\n");
	// 	return ret;
	// }

	st->sdecode_delay_us = 0;
	st->reset_delay_us = 50;

	ret = ads1120_initial_config(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "initial configuration failed\n");
		return ret;
	}

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ads1120_of_match[] = {
	{ .compatible = "ti,ads1120",
	  .data = &ads1120_info_tbl[ads1120], },
	{}
};
MODULE_DEVICE_TABLE(of, ads1120_of_match);

static struct spi_driver ads1120_driver = {
	.driver = {
		.name = "ads1120",
		.of_match_table = ads1120_of_match,
	},
	.probe = ads1120_probe,
};
module_spi_driver(ads1120_driver);

MODULE_AUTHOR("Jigar Patel <jigar@silmates.com>");
MODULE_DESCRIPTION("Driver for ADS1120 ADC");
MODULE_LICENSE("GPL v2");
