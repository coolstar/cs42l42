#define DESCRIPTOR_DEF
#include "cs42l42.h"
#include "registers.h"

#define bool int

void cs42l42_jack_det(PCS42L42_CONTEXT pDevice);

static ULONG Cs42l42DebugLevel = 100;
static ULONG Cs42l42DebugCatagories = DBG_INIT || DBG_PNP || DBG_IOCTL;

/* HPOUT Load Capacity */
#define CS42L42_HPOUT_LOAD_1NF		0
#define CS42L42_HPOUT_LOAD_10NF		1

/* HPOUT Clamp to GND Override */
#define CS42L42_HPOUT_CLAMP_EN		0
#define CS42L42_HPOUT_CLAMP_DIS		1

/* Tip Sense Inversion */
#define CS42L42_TS_INV_DIS			0
#define CS42L42_TS_INV_EN			1

/* Tip Sense Debounce */
#define CS42L42_TS_DBNCE_0			0
#define CS42L42_TS_DBNCE_125			1
#define CS42L42_TS_DBNCE_250			2
#define CS42L42_TS_DBNCE_500			3
#define CS42L42_TS_DBNCE_750			4
#define CS42L42_TS_DBNCE_1000			5
#define CS42L42_TS_DBNCE_1250			6
#define CS42L42_TS_DBNCE_1500			7

/* Button Press Software Debounce Times */
#define CS42L42_BTN_DET_INIT_DBNCE_MIN		0
#define CS42L42_BTN_DET_INIT_DBNCE_DEFAULT	100
#define CS42L42_BTN_DET_INIT_DBNCE_MAX		200

#define CS42L42_BTN_DET_EVENT_DBNCE_MIN		0
#define CS42L42_BTN_DET_EVENT_DBNCE_DEFAULT	10
#define CS42L42_BTN_DET_EVENT_DBNCE_MAX		20

/* Button Detect Level Sensitivities */
#define CS42L42_NUM_BIASES		4

#define CS42L42_HS_DET_LEVEL_15		0x0F
#define CS42L42_HS_DET_LEVEL_8		0x08
#define CS42L42_HS_DET_LEVEL_4		0x04
#define CS42L42_HS_DET_LEVEL_1		0x01

#define CS42L42_HS_DET_LEVEL_MIN	0
#define CS42L42_HS_DET_LEVEL_MAX	0x3F

/* HS Bias Ramp Rate */

#define CS42L42_HSBIAS_RAMP_FAST_RISE_SLOW_FALL		0
#define CS42L42_HSBIAS_RAMP_FAST			1
#define CS42L42_HSBIAS_RAMP_SLOW			2
#define CS42L42_HSBIAS_RAMP_SLOWEST			3

#define CS42L42_HSBIAS_RAMP_TIME0			10
#define CS42L42_HSBIAS_RAMP_TIME1			40
#define CS42L42_HSBIAS_RAMP_TIME2			90
#define CS42L42_HSBIAS_RAMP_TIME3			170

NTSTATUS
DriverEntry(
	__in PDRIVER_OBJECT  DriverObject,
	__in PUNICODE_STRING RegistryPath
)
{
	NTSTATUS               status = STATUS_SUCCESS;
	WDF_DRIVER_CONFIG      config;
	WDF_OBJECT_ATTRIBUTES  attributes;

	Cs42l42Print(DEBUG_LEVEL_INFO, DBG_INIT,
		"Driver Entry\n");

	WDF_DRIVER_CONFIG_INIT(&config, Cs42l42EvtDeviceAdd);

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);

	//
	// Create a framework driver object to represent our driver.
	//

	status = WdfDriverCreate(DriverObject,
		RegistryPath,
		&attributes,
		&config,
		WDF_NO_HANDLE
	);

	if (!NT_SUCCESS(status))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_INIT,
			"WdfDriverCreate failed with status 0x%x\n", status);
	}

	return status;
}

NTSTATUS cs42l42_switch_page(
	_In_ PCS42L42_CONTEXT pDevice,
	uint8_t page
) {
	uint8_t buf[2];
	buf[0] = CS42L42_PAGE_REGISTER;
	buf[1] = page;

	return SpbWriteDataSynchronously(&pDevice->I2CContext, buf, sizeof(buf));
}

NTSTATUS cs42l42_reg_read(
	_In_ PCS42L42_CONTEXT pDevice,
	uint16_t reg,
	unsigned int* data
) {
	NTSTATUS status;
	WdfWaitLockAcquire(pDevice->RegisterLock, NULL); //Lock page

	status = cs42l42_switch_page(pDevice, (reg >> 8) & 0xff);
	if (!NT_SUCCESS(status)) {
		goto exit;
	}

	uint8_t reg8 = (reg & 0xff);

	uint8_t raw_data = 0;
	status = SpbXferDataSynchronously(&pDevice->I2CContext, &reg8, sizeof(uint8_t), &raw_data, sizeof(uint8_t));
	*data = raw_data;

exit:
	WdfWaitLockRelease(pDevice->RegisterLock);
	return status;
}

NTSTATUS cs42l42_reg_write(
	_In_ PCS42L42_CONTEXT pDevice,
	uint16_t reg,
	unsigned int data
) {
	NTSTATUS status;
	WdfWaitLockAcquire(pDevice->RegisterLock, NULL); //Lock page

	status = cs42l42_switch_page(pDevice, (reg >> 8) & 0xff);
	if (!NT_SUCCESS(status)) {
		goto exit;
	}

	uint8_t buf[2];
	buf[0] = (reg & 0xff);
	buf[1] = data;
	status = SpbWriteDataSynchronously(&pDevice->I2CContext, buf, sizeof(buf));

exit:
	WdfWaitLockRelease(pDevice->RegisterLock);
	return status;
}

NTSTATUS cs42l42_reg_update(
	_In_ PCS42L42_CONTEXT pDevice,
	uint16_t reg,
	unsigned int mask,
	unsigned int val
) {
	unsigned int tmp = 0, orig = 0;

	NTSTATUS status = cs42l42_reg_read(pDevice, reg, &orig);
	if (!NT_SUCCESS(status)) {
		return status;
	}

	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig) {
		status = cs42l42_reg_write(pDevice, reg, tmp);
	}
	return status;
}

void udelay(ULONG usec) {
	LARGE_INTEGER Interval;
	Interval.QuadPart = -10 * (LONGLONG)usec;
	KeDelayExecutionThread(KernelMode, false, &Interval);
}

void msleep(ULONG msec) {
	udelay(msec * 1000);
}

NTSTATUS cs42l42_read_reg_poll_timeout(PCS42L42_CONTEXT pDevice, UINT16 reg, UINT8 val, UINT32 mask, ULONG sleep_us, ULONG timeout_us) {
	UINT8 regval;
	LARGE_INTEGER StartTime;
	KeQuerySystemTimePrecise(&StartTime);
	for (;;) {
		cs42l42_reg_read(pDevice, reg, &regval);
		LARGE_INTEGER CurrentTime;
		KeQuerySystemTimePrecise(&CurrentTime);
		if ((regval & mask) == val || ((CurrentTime.QuadPart - StartTime.QuadPart) / 10) > timeout_us)
			break;
		if (sleep_us)
			udelay(sleep_us);
	}
	return (regval & mask) == val ? STATUS_SUCCESS : STATUS_IO_TIMEOUT;
}

NTSTATUS
OnPrepareHardware(
	_In_  WDFDEVICE     FxDevice,
	_In_  WDFCMRESLIST  FxResourcesRaw,
	_In_  WDFCMRESLIST  FxResourcesTranslated
)
/*++

Routine Description:

This routine caches the SPB resource connection ID.

Arguments:

FxDevice - a handle to the framework device object
FxResourcesRaw - list of translated hardware resources that
the PnP manager has assigned to the device
FxResourcesTranslated - list of raw hardware resources that
the PnP manager has assigned to the device

Return Value:

Status

--*/
{
	PCS42L42_CONTEXT pDevice = GetDeviceContext(FxDevice);
	BOOLEAN fSpbResourceFound = FALSE;
	NTSTATUS status = STATUS_INSUFFICIENT_RESOURCES;

	UNREFERENCED_PARAMETER(FxResourcesRaw);

	//
	// Parse the peripheral's resources.
	//

	ULONG resourceCount = WdfCmResourceListGetCount(FxResourcesTranslated);

	for (ULONG i = 0; i < resourceCount; i++)
	{
		PCM_PARTIAL_RESOURCE_DESCRIPTOR pDescriptor;
		UCHAR Class;
		UCHAR Type;

		pDescriptor = WdfCmResourceListGetDescriptor(
			FxResourcesTranslated, i);

		switch (pDescriptor->Type)
		{
		case CmResourceTypeConnection:
			//
			// Look for I2C or SPI resource and save connection ID.
			//
			Class = pDescriptor->u.Connection.Class;
			Type = pDescriptor->u.Connection.Type;
			if (Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL &&
				Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C)
			{
				if (fSpbResourceFound == FALSE)
				{
					status = STATUS_SUCCESS;
					pDevice->I2CContext.I2cResHubId.LowPart = pDescriptor->u.Connection.IdLowPart;
					pDevice->I2CContext.I2cResHubId.HighPart = pDescriptor->u.Connection.IdHighPart;
					fSpbResourceFound = TRUE;
				}
				else
				{
				}
			}
			break;
		default:
			//
			// Ignoring all other resource types.
			//
			break;
		}
	}

	//
	// An SPB resource is required.
	//

	if (fSpbResourceFound == FALSE)
	{
		status = STATUS_NOT_FOUND;
	}

	status = SpbTargetInitialize(FxDevice, &pDevice->I2CContext);

	if (!NT_SUCCESS(status))
	{
		return status;
	}

	return status;
}

NTSTATUS
OnReleaseHardware(
	_In_  WDFDEVICE     FxDevice,
	_In_  WDFCMRESLIST  FxResourcesTranslated
)
/*++

Routine Description:

Arguments:

FxDevice - a handle to the framework device object
FxResourcesTranslated - list of raw hardware resources that
the PnP manager has assigned to the device

Return Value:

Status

--*/
{
	PCS42L42_CONTEXT pDevice = GetDeviceContext(FxDevice);
	NTSTATUS status = STATUS_SUCCESS;

	UNREFERENCED_PARAMETER(FxResourcesTranslated);

	SpbTargetDeinitialize(FxDevice, &pDevice->I2CContext);

	return status;
}

static inline int cirrus_read_device_id(PCS42L42_CONTEXT pDevice, unsigned int reg) {
	UINT8 devid[3];
	NTSTATUS status;

	for (int i = 0; i < 3; i++) {
		status = cs42l42_reg_read(pDevice, reg + i, &devid[i]);
		if (!NT_SUCCESS(status)) {
			return -1;
		}
	}

	return ((devid[0] & 0xFF) << 12) |
		((devid[1] & 0xFF) << 4) |
		((devid[2] & 0xF0) >> 4);
}

static void cs42l42_set_interrupt_masks(PCS42L42_CONTEXT pDevice)
{
	cs42l42_reg_update(pDevice, CS42L42_ADC_OVFL_INT_MASK,
		CS42L42_ADC_OVFL_MASK,
		(1 << CS42L42_ADC_OVFL_SHIFT));

	cs42l42_reg_update(pDevice, CS42L42_MIXER_INT_MASK,
		CS42L42_MIX_CHB_OVFL_MASK |
		CS42L42_MIX_CHA_OVFL_MASK |
		CS42L42_EQ_OVFL_MASK |
		CS42L42_EQ_BIQUAD_OVFL_MASK,
		(1 << CS42L42_MIX_CHB_OVFL_SHIFT) |
		(1 << CS42L42_MIX_CHA_OVFL_SHIFT) |
		(1 << CS42L42_EQ_OVFL_SHIFT) |
		(1 << CS42L42_EQ_BIQUAD_OVFL_SHIFT));

	cs42l42_reg_update(pDevice, CS42L42_SRC_INT_MASK,
		CS42L42_SRC_ILK_MASK |
		CS42L42_SRC_OLK_MASK |
		CS42L42_SRC_IUNLK_MASK |
		CS42L42_SRC_OUNLK_MASK,
		(1 << CS42L42_SRC_ILK_SHIFT) |
		(1 << CS42L42_SRC_OLK_SHIFT) |
		(1 << CS42L42_SRC_IUNLK_SHIFT) |
		(1 << CS42L42_SRC_OUNLK_SHIFT));

	cs42l42_reg_update(pDevice, CS42L42_ASP_RX_INT_MASK,
		CS42L42_ASPRX_NOLRCK_MASK |
		CS42L42_ASPRX_EARLY_MASK |
		CS42L42_ASPRX_LATE_MASK |
		CS42L42_ASPRX_ERROR_MASK |
		CS42L42_ASPRX_OVLD_MASK,
		(1 << CS42L42_ASPRX_NOLRCK_SHIFT) |
		(1 << CS42L42_ASPRX_EARLY_SHIFT) |
		(1 << CS42L42_ASPRX_LATE_SHIFT) |
		(1 << CS42L42_ASPRX_ERROR_SHIFT) |
		(1 << CS42L42_ASPRX_OVLD_SHIFT));

	cs42l42_reg_update(pDevice, CS42L42_ASP_TX_INT_MASK,
		CS42L42_ASPTX_NOLRCK_MASK |
		CS42L42_ASPTX_EARLY_MASK |
		CS42L42_ASPTX_LATE_MASK |
		CS42L42_ASPTX_SMERROR_MASK,
		(1 << CS42L42_ASPTX_NOLRCK_SHIFT) |
		(1 << CS42L42_ASPTX_EARLY_SHIFT) |
		(1 << CS42L42_ASPTX_LATE_SHIFT) |
		(1 << CS42L42_ASPTX_SMERROR_SHIFT));

	cs42l42_reg_update(pDevice, CS42L42_CODEC_INT_MASK,
		CS42L42_PDN_DONE_MASK |
		CS42L42_HSDET_AUTO_DONE_MASK,
		(1 << CS42L42_PDN_DONE_SHIFT) |
		(1 << CS42L42_HSDET_AUTO_DONE_SHIFT));

	cs42l42_reg_update(pDevice, CS42L42_SRCPL_INT_MASK,
		CS42L42_SRCPL_ADC_LK_MASK |
		CS42L42_SRCPL_DAC_LK_MASK |
		CS42L42_SRCPL_ADC_UNLK_MASK |
		CS42L42_SRCPL_DAC_UNLK_MASK,
		(1 << CS42L42_SRCPL_ADC_LK_SHIFT) |
		(1 << CS42L42_SRCPL_DAC_LK_SHIFT) |
		(1 << CS42L42_SRCPL_ADC_UNLK_SHIFT) |
		(1 << CS42L42_SRCPL_DAC_UNLK_SHIFT));

	cs42l42_reg_update(pDevice, CS42L42_DET_INT1_MASK,
		CS42L42_TIP_SENSE_UNPLUG_MASK |
		CS42L42_TIP_SENSE_PLUG_MASK |
		CS42L42_HSBIAS_SENSE_MASK,
		(1 << CS42L42_TIP_SENSE_UNPLUG_SHIFT) |
		(1 << CS42L42_TIP_SENSE_PLUG_SHIFT) |
		(1 << CS42L42_HSBIAS_SENSE_SHIFT));

	cs42l42_reg_update(pDevice, CS42L42_DET_INT2_MASK,
		CS42L42_M_DETECT_TF_MASK |
		CS42L42_M_DETECT_FT_MASK |
		CS42L42_M_HSBIAS_HIZ_MASK |
		CS42L42_M_SHORT_RLS_MASK |
		CS42L42_M_SHORT_DET_MASK,
		(1 << CS42L42_M_DETECT_TF_SHIFT) |
		(1 << CS42L42_M_DETECT_FT_SHIFT) |
		(1 << CS42L42_M_HSBIAS_HIZ_SHIFT) |
		(1 << CS42L42_M_SHORT_RLS_SHIFT) |
		(1 << CS42L42_M_SHORT_DET_SHIFT));

	cs42l42_reg_update(pDevice, CS42L42_VPMON_INT_MASK,
		CS42L42_VPMON_MASK,
		(1 << CS42L42_VPMON_SHIFT));

	cs42l42_reg_update(pDevice, CS42L42_PLL_LOCK_INT_MASK,
		CS42L42_PLL_LOCK_MASK,
		(1 << CS42L42_PLL_LOCK_SHIFT));

	cs42l42_reg_update(pDevice, CS42L42_TSRS_PLUG_INT_MASK,
		CS42L42_RS_PLUG_MASK |
		CS42L42_RS_UNPLUG_MASK |
		CS42L42_TS_PLUG_MASK |
		CS42L42_TS_UNPLUG_MASK,
		(1 << CS42L42_RS_PLUG_SHIFT) |
		(1 << CS42L42_RS_UNPLUG_SHIFT) |
		(0 << CS42L42_TS_PLUG_SHIFT) |
		(0 << CS42L42_TS_UNPLUG_SHIFT));
}

static void cs42l42_setup_hs_type_detect(PCS42L42_CONTEXT pDevice)
{
	unsigned int reg = 0;

	pDevice->hs_type = CS42L42_PLUG_INVALID;

	/*
	 * DETECT_MODE must always be 0 with ADC and HP both off otherwise the
	 * FILT+ supply will not charge properly.
	 */
	cs42l42_reg_update(pDevice, CS42L42_MISC_DET_CTL,
		CS42L42_DETECT_MODE_MASK, 0);

	/* Latch analog controls to VP power domain */
	cs42l42_reg_update(pDevice, CS42L42_MIC_DET_CTL1,
		CS42L42_LATCH_TO_VP_MASK |
		CS42L42_EVENT_STAT_SEL_MASK |
		CS42L42_HS_DET_LEVEL_MASK,
		(1 << CS42L42_LATCH_TO_VP_SHIFT) |
		(0 << CS42L42_EVENT_STAT_SEL_SHIFT) |
		(pDevice->bias_thresholds[0] <<
			CS42L42_HS_DET_LEVEL_SHIFT));

	/* Remove ground noise-suppression clamps */
	cs42l42_reg_update(pDevice,
		CS42L42_HS_CLAMP_DISABLE,
		CS42L42_HS_CLAMP_DISABLE_MASK,
		(1 << CS42L42_HS_CLAMP_DISABLE_SHIFT));

	/* Enable the tip sense circuit */
	cs42l42_reg_update(pDevice, CS42L42_TSENSE_CTL,
		CS42L42_TS_INV_MASK, CS42L42_TS_INV_MASK);

	cs42l42_reg_update(pDevice, CS42L42_TIPSENSE_CTL,
		CS42L42_TIP_SENSE_CTRL_MASK |
		CS42L42_TIP_SENSE_INV_MASK |
		CS42L42_TIP_SENSE_DEBOUNCE_MASK,
		(3 << CS42L42_TIP_SENSE_CTRL_SHIFT) |
		(!pDevice->ts_inv << CS42L42_TIP_SENSE_INV_SHIFT) |
		(2 << CS42L42_TIP_SENSE_DEBOUNCE_SHIFT));

	/* Save the initial status of the tip sense */
	cs42l42_reg_read(pDevice,
		CS42L42_TSRS_PLUG_STATUS,
		&reg);
	pDevice->plug_state = (((char)reg) &
		(CS42L42_TS_PLUG_MASK | CS42L42_TS_UNPLUG_MASK)) >>
		CS42L42_TS_PLUG_SHIFT;
}

struct cs42l42_pll_params {
	uint32_t sclk;
	uint8_t mclk_div;
	uint8_t mclk_src_sel;
	uint8_t sclk_prediv;
	uint8_t pll_div_int;
	uint32_t pll_div_frac;
	uint8_t pll_mode;
	uint8_t pll_divout;
	uint32_t mclk_int;
	uint8_t pll_cal_ratio;
	uint8_t n;
};

/*
 * Common PLL Settings for given SCLK
 * Table 4-5 from the Datasheet
 */
static const struct cs42l42_pll_params pll_ratio_table[] = {
	{ 1536000, 0, 1, 0x00, 0x7D, 0x000000, 0x03, 0x10, 12000000, 125, 2},
	{ 2304000, 0, 1, 0x00, 0x55, 0xC00000, 0x02, 0x10, 12288000,  85, 2},
	{ 2400000, 0, 1, 0x00, 0x50, 0x000000, 0x03, 0x10, 12000000,  80, 2},
	{ 2822400, 0, 1, 0x00, 0x40, 0x000000, 0x03, 0x10, 11289600, 128, 1},
	{ 3000000, 0, 1, 0x00, 0x40, 0x000000, 0x03, 0x10, 12000000, 128, 1},
	{ 3072000, 0, 1, 0x00, 0x3E, 0x800000, 0x03, 0x10, 12000000, 125, 1},
	{ 4000000, 0, 1, 0x00, 0x30, 0x800000, 0x03, 0x10, 12000000,  96, 1},
	{ 4096000, 0, 1, 0x00, 0x2E, 0xE00000, 0x03, 0x10, 12000000,  94, 1},
	{ 5644800, 0, 1, 0x01, 0x40, 0x000000, 0x03, 0x10, 11289600, 128, 1},
	{ 6000000, 0, 1, 0x01, 0x40, 0x000000, 0x03, 0x10, 12000000, 128, 1},
	{ 6144000, 0, 1, 0x01, 0x3E, 0x800000, 0x03, 0x10, 12000000, 125, 1},
	{ 11289600, 0, 0, 0, 0, 0, 0, 0, 11289600, 0, 1},
	{ 12000000, 0, 0, 0, 0, 0, 0, 0, 12000000, 0, 1},
	{ 12288000, 0, 0, 0, 0, 0, 0, 0, 12288000, 0, 1},
	{ 22579200, 1, 0, 0, 0, 0, 0, 0, 22579200, 0, 1},
	{ 24000000, 1, 0, 0, 0, 0, 0, 0, 24000000, 0, 1},
	{ 24576000, 1, 0, 0, 0, 0, 0, 0, 24576000, 0, 1}
};

void cs42l42_set_pll(PCS42L42_CONTEXT pDevice, unsigned int bclk) {

	uint32_t fsync;
	uint32_t srate = 48000;

	for (int i = 0; i < ARRAYSIZE(pll_ratio_table); i++) {
		if (pll_ratio_table[i].sclk == bclk) {
			pDevice->pll_config = i;

			/* Configure the internal sample rate */
			cs42l42_reg_update(pDevice,  CS42L42_MCLK_CTL,
				CS42L42_INTERNAL_FS_MASK,
				((pll_ratio_table[i].mclk_int !=
					12000000) &&
					(pll_ratio_table[i].mclk_int !=
						24000000)) <<
				CS42L42_INTERNAL_FS_SHIFT);
			cs42l42_reg_update(pDevice,  CS42L42_MCLK_SRC_SEL,
				CS42L42_MCLKDIV_MASK,
				(pll_ratio_table[i].mclk_div <<
					CS42L42_MCLKDIV_SHIFT));
			/* Set up the LRCLK */
			fsync = bclk / srate;
			if (((fsync * srate) != bclk)
				|| ((fsync % 2) != 0)) {
				DbgPrint("Unsupported sclk %d/sample rate %d\n",
					bclk,
					srate);
				return;
			}
			/* Set the LRCLK period */
			cs42l42_reg_update(pDevice, 
				CS42L42_FSYNC_P_LOWER,
				CS42L42_FSYNC_PERIOD_MASK,
				CS42L42_FRAC0_VAL(fsync - 1) <<
				CS42L42_FSYNC_PERIOD_SHIFT);
			cs42l42_reg_update(pDevice, 
				CS42L42_FSYNC_P_UPPER,
				CS42L42_FSYNC_PERIOD_MASK,
				CS42L42_FRAC1_VAL(fsync - 1) <<
				CS42L42_FSYNC_PERIOD_SHIFT);
			/* Set the LRCLK to 50% duty cycle */
			fsync = fsync / 2;
			cs42l42_reg_update(pDevice, 
				CS42L42_FSYNC_PW_LOWER,
				CS42L42_FSYNC_PULSE_WIDTH_MASK,
				CS42L42_FRAC0_VAL(fsync - 1) <<
				CS42L42_FSYNC_PULSE_WIDTH_SHIFT);
			cs42l42_reg_update(pDevice, 
				CS42L42_FSYNC_PW_UPPER,
				CS42L42_FSYNC_PULSE_WIDTH_MASK,
				CS42L42_FRAC1_VAL(fsync - 1) <<
				CS42L42_FSYNC_PULSE_WIDTH_SHIFT);
			cs42l42_reg_update(pDevice, 
				CS42L42_ASP_FRM_CFG,
				CS42L42_ASP_5050_MASK,
				CS42L42_ASP_5050_MASK);
			/* Set the frame delay to 1.0 SCLK clocks */
			cs42l42_reg_update(pDevice,  CS42L42_ASP_FRM_CFG,
				CS42L42_ASP_FSD_MASK,
				CS42L42_ASP_FSD_1_0 <<
				CS42L42_ASP_FSD_SHIFT);
			/* Set the sample rates (96k or lower) */
			cs42l42_reg_update(pDevice,  CS42L42_FS_RATE_EN,
				CS42L42_FS_EN_MASK,
				(CS42L42_FS_EN_IASRC_96K |
					CS42L42_FS_EN_OASRC_96K) <<
				CS42L42_FS_EN_SHIFT);
			/* Set the input/output internal MCLK clock ~12 MHz */
			cs42l42_reg_update(pDevice,  CS42L42_IN_ASRC_CLK,
				CS42L42_CLK_IASRC_SEL_MASK,
				CS42L42_CLK_IASRC_SEL_12 <<
				CS42L42_CLK_IASRC_SEL_SHIFT);
			cs42l42_reg_update(pDevice, 
				CS42L42_OUT_ASRC_CLK,
				CS42L42_CLK_OASRC_SEL_MASK,
				CS42L42_CLK_OASRC_SEL_12 <<
				CS42L42_CLK_OASRC_SEL_SHIFT);
			if (pll_ratio_table[i].mclk_src_sel == 0) {
				/* Pass the clock straight through */
				cs42l42_reg_update(pDevice, 
					CS42L42_PLL_CTL1,
					CS42L42_PLL_START_MASK, 0);
			}
			else {
				/* Configure PLL per table 4-5 */
				cs42l42_reg_update(pDevice, 
					CS42L42_PLL_DIV_CFG1,
					CS42L42_SCLK_PREDIV_MASK,
					pll_ratio_table[i].sclk_prediv
					<< CS42L42_SCLK_PREDIV_SHIFT);
				cs42l42_reg_update(pDevice, 
					CS42L42_PLL_DIV_INT,
					CS42L42_PLL_DIV_INT_MASK,
					pll_ratio_table[i].pll_div_int
					<< CS42L42_PLL_DIV_INT_SHIFT);
				cs42l42_reg_update(pDevice, 
					CS42L42_PLL_DIV_FRAC0,
					CS42L42_PLL_DIV_FRAC_MASK,
					CS42L42_FRAC0_VAL(
						pll_ratio_table[i].pll_div_frac)
					<< CS42L42_PLL_DIV_FRAC_SHIFT);
				cs42l42_reg_update(pDevice, 
					CS42L42_PLL_DIV_FRAC1,
					CS42L42_PLL_DIV_FRAC_MASK,
					CS42L42_FRAC1_VAL(
						pll_ratio_table[i].pll_div_frac)
					<< CS42L42_PLL_DIV_FRAC_SHIFT);
				cs42l42_reg_update(pDevice, 
					CS42L42_PLL_DIV_FRAC2,
					CS42L42_PLL_DIV_FRAC_MASK,
					CS42L42_FRAC2_VAL(
						pll_ratio_table[i].pll_div_frac)
					<< CS42L42_PLL_DIV_FRAC_SHIFT);
				cs42l42_reg_update(pDevice, 
					CS42L42_PLL_CTL4,
					CS42L42_PLL_MODE_MASK,
					pll_ratio_table[i].pll_mode
					<< CS42L42_PLL_MODE_SHIFT);
				cs42l42_reg_update(pDevice, 
					CS42L42_PLL_CTL3,
					CS42L42_PLL_DIVOUT_MASK,
					(pll_ratio_table[i].pll_divout * pll_ratio_table[i].n)
					<< CS42L42_PLL_DIVOUT_SHIFT);
				cs42l42_reg_update(pDevice, 
					CS42L42_PLL_CAL_RATIO,
					CS42L42_PLL_CAL_RATIO_MASK,
					pll_ratio_table[i].pll_cal_ratio
					<< CS42L42_PLL_CAL_RATIO_SHIFT);
			}
			return;
		}
	}
}

void cs42l42_hw_params(PCS42L42_CONTEXT pDevice) {
	unsigned int bclk = 24576000;
	unsigned int width = (24 / 8) - 1;
	unsigned int channels = 2;
	unsigned int sample_rate = 48000;

	{ //enable capture
		unsigned int val = 0;
		val = CS42L42_ASP_TX_CH2_AP_MASK |
			(width << CS42L42_ASP_TX_CH2_RES_SHIFT) |
			(width << CS42L42_ASP_TX_CH1_RES_SHIFT);

		cs42l42_reg_update(pDevice, CS42L42_ASP_TX_CH_AP_RES,
			CS42L42_ASP_TX_CH1_AP_MASK | CS42L42_ASP_TX_CH2_AP_MASK |
			CS42L42_ASP_TX_CH2_RES_MASK | CS42L42_ASP_TX_CH1_RES_MASK, val);
	}

	{ //enable playback
		unsigned int val = 0;
		val |= width << CS42L42_ASP_RX_CH_RES_SHIFT;
		/* channel 1 on low LRCLK */
		cs42l42_reg_update(pDevice, CS42L42_ASP_RX_DAI0_CH1_AP_RES,
			CS42L42_ASP_RX_CH_AP_MASK |
			CS42L42_ASP_RX_CH_RES_MASK, val);
		/* Channel 2 on high LRCLK */
		val |= CS42L42_ASP_RX_CH_AP_HI << CS42L42_ASP_RX_CH_AP_SHIFT;
		cs42l42_reg_update(pDevice, CS42L42_ASP_RX_DAI0_CH2_AP_RES,
			CS42L42_ASP_RX_CH_AP_MASK |
			CS42L42_ASP_RX_CH_RES_MASK, val);

		/* Channel B comes from the last active channel */
		cs42l42_reg_update(pDevice, CS42L42_SP_RX_CH_SEL,
			CS42L42_SP_RX_CHB_SEL_MASK,
			(channels - 1) << CS42L42_SP_RX_CHB_SEL_SHIFT);

		/* Both LRCLK slots must be enabled */
		cs42l42_reg_update(pDevice, CS42L42_ASP_RX_DAI0_EN,
			CS42L42_ASP_RX0_CH_EN_MASK,
			BIT(CS42L42_ASP_RX0_CH1_SHIFT) |
			BIT(CS42L42_ASP_RX0_CH2_SHIFT));
	}

	cs42l42_set_pll(pDevice, 2400000);
}

VOID
CS42L42BootWorkItem(
	IN WDFWORKITEM  WorkItem
)
{
	WDFDEVICE Device = (WDFDEVICE)WdfWorkItemGetParentObject(WorkItem);
	PCS42L42_CONTEXT pDevice = GetDeviceContext(Device);

	//Power up the codec

	cs42l42_reg_update(pDevice, CS42L42_PWR_CTL1,
		CS42L42_ASP_DAO_PDN_MASK |
		CS42L42_ASP_DAI_PDN_MASK |
		CS42L42_MIXER_PDN_MASK |
		CS42L42_EQ_PDN_MASK |
		CS42L42_HP_PDN_MASK |
		CS42L42_ADC_PDN_MASK |
		CS42L42_PDN_ALL_MASK,
		(1 << CS42L42_ASP_DAO_PDN_SHIFT) |
		(1 << CS42L42_ASP_DAI_PDN_SHIFT) |
		(1 << CS42L42_MIXER_PDN_SHIFT) |
		(1 << CS42L42_EQ_PDN_SHIFT) |
		(1 << CS42L42_HP_PDN_SHIFT) |
		(1 << CS42L42_ADC_PDN_SHIFT) |
		(0 << CS42L42_PDN_ALL_SHIFT));

	{
		pDevice->ts_inv = CS42L42_TS_INV_EN;
		pDevice->ts_dbnc_rise = CS42L42_TS_DBNCE_1000;
		pDevice->ts_dbnc_fall = CS42L42_TS_DBNCE_0;
		pDevice->btn_det_init_dbnce = CS42L42_BTN_DET_INIT_DBNCE_DEFAULT;
		pDevice->btn_det_event_dbnce = CS42L42_BTN_DET_EVENT_DBNCE_DEFAULT;
		static const unsigned int threshold_defaults[] = {
			CS42L42_HS_DET_LEVEL_15,
			CS42L42_HS_DET_LEVEL_8,
			CS42L42_HS_DET_LEVEL_4,
			CS42L42_HS_DET_LEVEL_1
		};
		for (int i = 0; i < 4; i++) {
			pDevice->bias_thresholds[i] = threshold_defaults[i];
		}

		pDevice->hs_bias_ramp_rate = CS42L42_HSBIAS_RAMP_SLOW;
		pDevice->hs_bias_ramp_time = CS42L42_HSBIAS_RAMP_TIME2;
		pDevice->hs_bias_sense_en = 0;

		cs42l42_reg_update(pDevice, CS42L42_TSENSE_CTL,
			CS42L42_TS_RISE_DBNCE_TIME_MASK,
			(pDevice->ts_dbnc_rise <<
				CS42L42_TS_RISE_DBNCE_TIME_SHIFT));

		cs42l42_reg_update(pDevice, CS42L42_TSENSE_CTL,
			CS42L42_TS_FALL_DBNCE_TIME_MASK,
			(pDevice->ts_dbnc_fall <<
				CS42L42_TS_FALL_DBNCE_TIME_SHIFT));

		cs42l42_reg_update(pDevice, CS42L42_HS_BIAS_CTL,
			CS42L42_HSBIAS_RAMP_MASK,
			(pDevice->hs_bias_ramp_rate <<
				CS42L42_HSBIAS_RAMP_SHIFT));
	}

	/* Setup headset detection */
	cs42l42_setup_hs_type_detect(pDevice);

	/* Mask/Unmask Interrupts */
	cs42l42_set_interrupt_masks(pDevice);

	//Setup hw params
	cs42l42_hw_params(pDevice);

	//Set dai fmt
	{
		uint32_t asp_cfg_val = 0;

		asp_cfg_val |= CS42L42_ASP_SLAVE_MODE  <<
			CS42L42_ASP_MODE_SHIFT;

		/*
		 * 5050 mode, frame starts on falling edge of LRCLK,
		 * frame delayed by 1.0 SCLKs
		 */
		cs42l42_reg_update(pDevice,
			CS42L42_ASP_FRM_CFG,
			CS42L42_ASP_STP_MASK |
			CS42L42_ASP_5050_MASK |
			CS42L42_ASP_FSD_MASK,
			CS42L42_ASP_5050_MASK |
			(CS42L42_ASP_FSD_1_0 <<
				CS42L42_ASP_FSD_SHIFT));

		asp_cfg_val |= CS42L42_ASP_SCPOL_NOR << CS42L42_ASP_SCPOL_SHIFT;

		cs42l42_reg_update(pDevice, CS42L42_ASP_CLK_CFG, CS42L42_ASP_MODE_MASK |
			CS42L42_ASP_SCPOL_MASK |
			CS42L42_ASP_LCPOL_MASK,
			asp_cfg_val);
	}

	//Enable speakers
	cs42l42_reg_update(pDevice, CS42L42_PWR_CTL1, CS42L42_HP_PDN_MASK | CS42L42_MIXER_PDN_MASK | CS42L42_ASP_DAI_PDN_MASK, 0);
	//Enable microphone
	cs42l42_reg_update(pDevice, CS42L42_PWR_CTL1, CS42L42_ADC_PDN_MASK | CS42L42_ASP_DAO_PDN_MASK, 0);
	cs42l42_reg_update(pDevice, CS42L42_ASP_TX_CH_EN,
		(1 << CS42L42_ASP_TX0_CH2_SHIFT) | (1 << CS42L42_ASP_TX0_CH1_SHIFT),
		(1 << CS42L42_ASP_TX0_CH2_SHIFT) | (1 << CS42L42_ASP_TX0_CH1_SHIFT));
	cs42l42_reg_update(pDevice, CS42L42_ASP_TX_SZ_EN, 1, 1);

	//Enable sclk
	cs42l42_reg_update(pDevice, CS42L42_ASP_CLK_CFG, CS42L42_ASP_SCLK_EN_MASK, CS42L42_ASP_SCLK_EN_MASK);

	//Set volume
	cs42l42_reg_write(pDevice, CS42L42_MIXER_CHA_VOL, 0);
	cs42l42_reg_write(pDevice, CS42L42_MIXER_CHB_VOL, 0);

	cs42l42_reg_write(pDevice, CS42L42_ADC_VOLUME, 0xb);

	{ // move to csaudio callback
		unsigned int regval;

		/* SCLK must be running before codec unmute.
			 *
			 * PLL must not be started with ADC and HP both off
			 * otherwise the FILT+ supply will not charge properly.
			 * DAPM widgets power-up before stream unmute so at least
			 * one of the "DAC" or "ADC" widgets will already have
			 * powered-up.
			 */
		if (pll_ratio_table[pDevice->pll_config].mclk_src_sel) {
			cs42l42_reg_update(pDevice, CS42L42_PLL_CTL1,
				CS42L42_PLL_START_MASK, 1);

			if (pll_ratio_table[pDevice->pll_config].n > 1) {
				udelay(CS42L42_PLL_DIVOUT_TIME_US);
				regval = pll_ratio_table[pDevice->pll_config].pll_divout;
				cs42l42_reg_update(pDevice, CS42L42_PLL_CTL3,
					CS42L42_PLL_DIVOUT_MASK,
					regval <<
					CS42L42_PLL_DIVOUT_SHIFT);
			}

			NTSTATUS status = cs42l42_read_reg_poll_timeout(pDevice,
				CS42L42_PLL_LOCK_STATUS,
				1,
				1,
				CS42L42_PLL_LOCK_POLL_US,
				CS42L42_PLL_LOCK_TIMEOUT_US);
			if (!NT_SUCCESS(status))
				DbgPrint("PLL failed to lock: %x\n", status);

			/* PLL must be running to drive glitchless switch logic */
			cs42l42_reg_update(pDevice,
				CS42L42_MCLK_SRC_SEL,
				CS42L42_MCLK_SRC_SEL_MASK,
				CS42L42_MCLK_SRC_SEL_MASK);
		}
	}

	//Unmute headphones

	cs42l42_reg_update(pDevice, CS42L42_HP_CTL,
		CS42L42_HP_ANA_AMUTE_MASK |
		CS42L42_HP_ANA_BMUTE_MASK |
		CS42L42_HP_FULL_SCALE_VOL_MASK,
		CS42L42_HP_FULL_SCALE_VOL_MASK);

	//unmute microphone
	cs42l42_reg_update(pDevice, CS42L42_ADC_CTL,
		(1 << CS42L42_ADC_DIG_BOOST_SHIFT),
		(1 << CS42L42_ADC_DIG_BOOST_SHIFT));

	/* Manually call jack det handler for the first time */

	cs42l42_jack_det(pDevice);

	pDevice->DevicePoweredOn = TRUE;
end:
	WdfObjectDelete(WorkItem);
}

NTSTATUS
OnD0Entry(
	_In_  WDFDEVICE               FxDevice,
	_In_  WDF_POWER_DEVICE_STATE  FxPreviousState
)
/*++

Routine Description:

This routine allocates objects needed by the driver.

Arguments:

FxDevice - a handle to the framework device object
FxPreviousState - previous power state

Return Value:

Status

--*/
{
	UNREFERENCED_PARAMETER(FxPreviousState);

	PCS42L42_CONTEXT pDevice = GetDeviceContext(FxDevice);
	NTSTATUS status = STATUS_SUCCESS;

	int devid = cirrus_read_device_id(pDevice, CS42L42_DEVID_AB);
	if (devid < 0) {
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_PNP,
			"Failed to read device ID\n");
		status = STATUS_IO_DEVICE_ERROR;
		return status;
	}

	if (devid != CS42L42_CHIP_ID) {
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_PNP,
			"Invalid device id (0x%x), expected 0x%x\n", devid, CS42L42_CHIP_ID);
		status = STATUS_IO_DEVICE_ERROR;
		return status;
	}

	unsigned int reg = 0;
	status = cs42l42_reg_read(pDevice, CS42L42_REVID, &reg);
	if (!NT_SUCCESS(status)) {
		return status;
	}

	Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_PNP,
		"CS42L42 revision 0x%02x\n", reg & 0xFF);

	if (pDevice->plug_state != CS42L42_TS_UNPLUG)
		pDevice->plug_state = CS42L42_TS_TRANS;

	WDF_OBJECT_ATTRIBUTES attributes;
	WDF_WORKITEM_CONFIG workitemConfig;
	WDFWORKITEM hWorkItem;

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
	WDF_OBJECT_ATTRIBUTES_SET_CONTEXT_TYPE(&attributes, CS42L42_CONTEXT);
	attributes.ParentObject = pDevice->FxDevice;
	WDF_WORKITEM_CONFIG_INIT(&workitemConfig, CS42L42BootWorkItem);

	WdfWorkItemCreate(&workitemConfig,
		&attributes,
		&hWorkItem);

	WdfWorkItemEnqueue(hWorkItem);

	return status;
}

NTSTATUS
OnD0Exit(
	_In_  WDFDEVICE               FxDevice,
	_In_  WDF_POWER_DEVICE_STATE  FxPreviousState
)
/*++

Routine Description:

This routine destroys objects needed by the driver.

Arguments:

FxDevice - a handle to the framework device object
FxPreviousState - previous power state

Return Value:

Status

--*/
{
	UNREFERENCED_PARAMETER(FxPreviousState);

	PCS42L42_CONTEXT pDevice = GetDeviceContext(FxDevice);
	NTSTATUS status = STATUS_SUCCESS;

	pDevice->DevicePoweredOn = FALSE;

	return STATUS_SUCCESS;
}

static void cs42l42_manual_hs_type_detect(PCS42L42_CONTEXT pDevice)
{
	unsigned int hs_det_status;
	unsigned int hs_det_comp1;
	unsigned int hs_det_comp2;
	unsigned int hs_det_sw;

	/* Set hs detect to manual, active mode */
	cs42l42_reg_update(pDevice, 
		CS42L42_HSDET_CTL2,
		CS42L42_HSDET_CTRL_MASK |
		CS42L42_HSDET_SET_MASK |
		CS42L42_HSBIAS_REF_MASK |
		CS42L42_HSDET_AUTO_TIME_MASK,
		(1 << CS42L42_HSDET_CTRL_SHIFT) |
		(0 << CS42L42_HSDET_SET_SHIFT) |
		(0 << CS42L42_HSBIAS_REF_SHIFT) |
		(0 << CS42L42_HSDET_AUTO_TIME_SHIFT));

	/* Configure HS DET comparator reference levels. */
	cs42l42_reg_update(pDevice, 
		CS42L42_HSDET_CTL1,
		CS42L42_HSDET_COMP1_LVL_MASK |
		CS42L42_HSDET_COMP2_LVL_MASK,
		(CS42L42_HSDET_COMP1_LVL_VAL << CS42L42_HSDET_COMP1_LVL_SHIFT) |
		(CS42L42_HSDET_COMP2_LVL_VAL << CS42L42_HSDET_COMP2_LVL_SHIFT));

	/* Open the SW_HSB_HS3 switch and close SW_HSB_HS4 for a Type 1 headset. */
	cs42l42_reg_write(pDevice, CS42L42_HS_SWITCH_CTL, CS42L42_HSDET_SW_COMP1);

	msleep(100);

	cs42l42_reg_read(pDevice, CS42L42_HS_DET_STATUS, &hs_det_status);

	hs_det_comp1 = (hs_det_status & CS42L42_HSDET_COMP1_OUT_MASK) >>
		CS42L42_HSDET_COMP1_OUT_SHIFT;
	hs_det_comp2 = (hs_det_status & CS42L42_HSDET_COMP2_OUT_MASK) >>
		CS42L42_HSDET_COMP2_OUT_SHIFT;

	/* Close the SW_HSB_HS3 switch for a Type 2 headset. */
	cs42l42_reg_write(pDevice, CS42L42_HS_SWITCH_CTL, CS42L42_HSDET_SW_COMP2);

	msleep(100);

	cs42l42_reg_read(pDevice, CS42L42_HS_DET_STATUS, &hs_det_status);

	hs_det_comp1 |= ((hs_det_status & CS42L42_HSDET_COMP1_OUT_MASK) >>
		CS42L42_HSDET_COMP1_OUT_SHIFT) << 1;
	hs_det_comp2 |= ((hs_det_status & CS42L42_HSDET_COMP2_OUT_MASK) >>
		CS42L42_HSDET_COMP2_OUT_SHIFT) << 1;

	/* Use Comparator 1 with 1.25V Threshold. */
	switch (hs_det_comp1) {
	case CS42L42_HSDET_COMP_TYPE1:
		pDevice->hs_type = CS42L42_PLUG_CTIA;
		hs_det_sw = CS42L42_HSDET_SW_TYPE1;
		break;
	case CS42L42_HSDET_COMP_TYPE2:
		pDevice->hs_type = CS42L42_PLUG_OMTP;
		hs_det_sw = CS42L42_HSDET_SW_TYPE2;
		break;
	default:
		/* Fallback to Comparator 2 with 1.75V Threshold. */
		switch (hs_det_comp2) {
		case CS42L42_HSDET_COMP_TYPE1:
			pDevice->hs_type = CS42L42_PLUG_CTIA;
			hs_det_sw = CS42L42_HSDET_SW_TYPE1;
			break;
		case CS42L42_HSDET_COMP_TYPE2:
			pDevice->hs_type = CS42L42_PLUG_OMTP;
			hs_det_sw = CS42L42_HSDET_SW_TYPE2;
			break;
			/* Detect Type 3 and Type 4 Headsets as Headphones */
		default:
			pDevice->hs_type = CS42L42_PLUG_HEADPHONE;
			hs_det_sw = CS42L42_HSDET_SW_TYPE3;
			break;
		}
	}

	/* Set Switches */
	cs42l42_reg_write(pDevice, CS42L42_HS_SWITCH_CTL, hs_det_sw);

	/* Set HSDET mode to Manual—Disabled */
	cs42l42_reg_update(pDevice, 
		CS42L42_HSDET_CTL2,
		CS42L42_HSDET_CTRL_MASK |
		CS42L42_HSDET_SET_MASK |
		CS42L42_HSBIAS_REF_MASK |
		CS42L42_HSDET_AUTO_TIME_MASK,
		(0 << CS42L42_HSDET_CTRL_SHIFT) |
		(0 << CS42L42_HSDET_SET_SHIFT) |
		(0 << CS42L42_HSBIAS_REF_SHIFT) |
		(0 << CS42L42_HSDET_AUTO_TIME_SHIFT));

	/* Configure HS DET comparator reference levels. */
	cs42l42_reg_update(pDevice, 
		CS42L42_HSDET_CTL1,
		CS42L42_HSDET_COMP1_LVL_MASK |
		CS42L42_HSDET_COMP2_LVL_MASK,
		(CS42L42_HSDET_COMP1_LVL_DEFAULT << CS42L42_HSDET_COMP1_LVL_SHIFT) |
		(CS42L42_HSDET_COMP2_LVL_DEFAULT << CS42L42_HSDET_COMP2_LVL_SHIFT));
}


static void cs42l42_process_hs_type_detect(PCS42L42_CONTEXT pDevice)
{
	unsigned int hs_det_status;
	unsigned int int_status;

	/* Read and save the hs detection result */
	cs42l42_reg_read(pDevice, CS42L42_HS_DET_STATUS, &hs_det_status);

	/* Mask the auto detect interrupt */
	cs42l42_reg_update(pDevice, 
		CS42L42_CODEC_INT_MASK,
		CS42L42_PDN_DONE_MASK |
		CS42L42_HSDET_AUTO_DONE_MASK,
		(1 << CS42L42_PDN_DONE_SHIFT) |
		(1 << CS42L42_HSDET_AUTO_DONE_SHIFT));


	pDevice->hs_type = (hs_det_status & CS42L42_HSDET_TYPE_MASK) >>
		CS42L42_HSDET_TYPE_SHIFT;

	/* Set hs detect to automatic, disabled mode */
	cs42l42_reg_update(pDevice, 
		CS42L42_HSDET_CTL2,
		CS42L42_HSDET_CTRL_MASK |
		CS42L42_HSDET_SET_MASK |
		CS42L42_HSBIAS_REF_MASK |
		CS42L42_HSDET_AUTO_TIME_MASK,
		(2 << CS42L42_HSDET_CTRL_SHIFT) |
		(2 << CS42L42_HSDET_SET_SHIFT) |
		(0 << CS42L42_HSBIAS_REF_SHIFT) |
		(3 << CS42L42_HSDET_AUTO_TIME_SHIFT));

	/* Run Manual detection if auto detect has not found a headset.
	 * We Re-Run with Manual Detection if the original detection was invalid or headphones,
	 * to ensure that a headset mic is detected in all cases.
	 */
	if (pDevice->hs_type == CS42L42_PLUG_INVALID ||
		pDevice->hs_type == CS42L42_PLUG_HEADPHONE) {
		DbgPrint("Running Manual Detection Fallback\n");
		cs42l42_manual_hs_type_detect(pDevice);
	}

	/* Set up button detection */
	if ((pDevice->hs_type == CS42L42_PLUG_CTIA) ||
		(pDevice->hs_type == CS42L42_PLUG_OMTP)) {
		/* Set auto HS bias settings to default */
		cs42l42_reg_update(pDevice, 
			CS42L42_HSBIAS_SC_AUTOCTL,
			CS42L42_HSBIAS_SENSE_EN_MASK |
			CS42L42_AUTO_HSBIAS_HIZ_MASK |
			CS42L42_TIP_SENSE_EN_MASK |
			CS42L42_HSBIAS_SENSE_TRIP_MASK,
			(0 << CS42L42_HSBIAS_SENSE_EN_SHIFT) |
			(0 << CS42L42_AUTO_HSBIAS_HIZ_SHIFT) |
			(0 << CS42L42_TIP_SENSE_EN_SHIFT) |
			(3 << CS42L42_HSBIAS_SENSE_TRIP_SHIFT));

		/* Set up hs detect level sensitivity */
		cs42l42_reg_update(pDevice, 
			CS42L42_MIC_DET_CTL1,
			CS42L42_LATCH_TO_VP_MASK |
			CS42L42_EVENT_STAT_SEL_MASK |
			CS42L42_HS_DET_LEVEL_MASK,
			(1 << CS42L42_LATCH_TO_VP_SHIFT) |
			(0 << CS42L42_EVENT_STAT_SEL_SHIFT) |
			(pDevice->bias_thresholds[0] <<
				CS42L42_HS_DET_LEVEL_SHIFT));

		/* Set auto HS bias settings to default */
		cs42l42_reg_update(pDevice, 
			CS42L42_HSBIAS_SC_AUTOCTL,
			CS42L42_HSBIAS_SENSE_EN_MASK |
			CS42L42_AUTO_HSBIAS_HIZ_MASK |
			CS42L42_TIP_SENSE_EN_MASK |
			CS42L42_HSBIAS_SENSE_TRIP_MASK,
			(pDevice->hs_bias_sense_en << CS42L42_HSBIAS_SENSE_EN_SHIFT) |
			(1 << CS42L42_AUTO_HSBIAS_HIZ_SHIFT) |
			(0 << CS42L42_TIP_SENSE_EN_SHIFT) |
			(3 << CS42L42_HSBIAS_SENSE_TRIP_SHIFT));

		/* Turn on level detect circuitry */
		cs42l42_reg_update(pDevice, 
			CS42L42_MISC_DET_CTL,
			CS42L42_HSBIAS_CTL_MASK |
			CS42L42_PDN_MIC_LVL_DET_MASK,
			(3 << CS42L42_HSBIAS_CTL_SHIFT) |
			(0 << CS42L42_PDN_MIC_LVL_DET_SHIFT));

		msleep(pDevice->btn_det_init_dbnce);

		/* Clear any button interrupts before unmasking them */
		cs42l42_reg_read(pDevice, CS42L42_DET_INT_STATUS2,
			&int_status);

		/* Unmask button detect interrupts */
		cs42l42_reg_update(pDevice, 
			CS42L42_DET_INT2_MASK,
			CS42L42_M_DETECT_TF_MASK |
			CS42L42_M_DETECT_FT_MASK |
			CS42L42_M_HSBIAS_HIZ_MASK |
			CS42L42_M_SHORT_RLS_MASK |
			CS42L42_M_SHORT_DET_MASK,
			(0 << CS42L42_M_DETECT_TF_SHIFT) |
			(0 << CS42L42_M_DETECT_FT_SHIFT) |
			(0 << CS42L42_M_HSBIAS_HIZ_SHIFT) |
			(1 << CS42L42_M_SHORT_RLS_SHIFT) |
			(1 << CS42L42_M_SHORT_DET_SHIFT));
	}
	else {
		/* Make sure button detect and HS bias circuits are off */
		cs42l42_reg_update(pDevice, 
			CS42L42_MISC_DET_CTL,
			CS42L42_HSBIAS_CTL_MASK |
			CS42L42_PDN_MIC_LVL_DET_MASK,
			(1 << CS42L42_HSBIAS_CTL_SHIFT) |
			(1 << CS42L42_PDN_MIC_LVL_DET_SHIFT));
	}

	cs42l42_reg_update(pDevice, 
		CS42L42_DAC_CTL2,
		CS42L42_HPOUT_PULLDOWN_MASK |
		CS42L42_HPOUT_LOAD_MASK |
		CS42L42_HPOUT_CLAMP_MASK |
		CS42L42_DAC_HPF_EN_MASK |
		CS42L42_DAC_MON_EN_MASK,
		(0 << CS42L42_HPOUT_PULLDOWN_SHIFT) |
		(0 << CS42L42_HPOUT_LOAD_SHIFT) |
		(0 << CS42L42_HPOUT_CLAMP_SHIFT) |
		(1 << CS42L42_DAC_HPF_EN_SHIFT) |
		(0 << CS42L42_DAC_MON_EN_SHIFT));

	/* Unmask tip sense interrupts */
	cs42l42_reg_update(pDevice, 
		CS42L42_TSRS_PLUG_INT_MASK,
		CS42L42_TS_PLUG_MASK |
		CS42L42_TS_UNPLUG_MASK,
		(0 << CS42L42_TS_PLUG_SHIFT) |
		(0 << CS42L42_TS_UNPLUG_SHIFT));
}

static void cs42l42_init_hs_type_detect(PCS42L42_CONTEXT pDevice)
{
	/* Mask tip sense interrupts */
	cs42l42_reg_update(pDevice, 
		CS42L42_TSRS_PLUG_INT_MASK,
		CS42L42_TS_PLUG_MASK |
		CS42L42_TS_UNPLUG_MASK,
		(1 << CS42L42_TS_PLUG_SHIFT) |
		(1 << CS42L42_TS_UNPLUG_SHIFT));

	/* Make sure button detect and HS bias circuits are off */
	cs42l42_reg_update(pDevice, 
		CS42L42_MISC_DET_CTL,
		CS42L42_HSBIAS_CTL_MASK |
		CS42L42_PDN_MIC_LVL_DET_MASK,
		(1 << CS42L42_HSBIAS_CTL_SHIFT) |
		(1 << CS42L42_PDN_MIC_LVL_DET_SHIFT));

	/* Set auto HS bias settings to default */
	cs42l42_reg_update(pDevice, 
		CS42L42_HSBIAS_SC_AUTOCTL,
		CS42L42_HSBIAS_SENSE_EN_MASK |
		CS42L42_AUTO_HSBIAS_HIZ_MASK |
		CS42L42_TIP_SENSE_EN_MASK |
		CS42L42_HSBIAS_SENSE_TRIP_MASK,
		(0 << CS42L42_HSBIAS_SENSE_EN_SHIFT) |
		(0 << CS42L42_AUTO_HSBIAS_HIZ_SHIFT) |
		(0 << CS42L42_TIP_SENSE_EN_SHIFT) |
		(3 << CS42L42_HSBIAS_SENSE_TRIP_SHIFT));

	/* Set hs detect to manual, disabled mode */
	cs42l42_reg_update(pDevice, 
		CS42L42_HSDET_CTL2,
		CS42L42_HSDET_CTRL_MASK |
		CS42L42_HSDET_SET_MASK |
		CS42L42_HSBIAS_REF_MASK |
		CS42L42_HSDET_AUTO_TIME_MASK,
		(0 << CS42L42_HSDET_CTRL_SHIFT) |
		(2 << CS42L42_HSDET_SET_SHIFT) |
		(0 << CS42L42_HSBIAS_REF_SHIFT) |
		(3 << CS42L42_HSDET_AUTO_TIME_SHIFT));

	cs42l42_reg_update(pDevice, 
		CS42L42_DAC_CTL2,
		CS42L42_HPOUT_PULLDOWN_MASK |
		CS42L42_HPOUT_LOAD_MASK |
		CS42L42_HPOUT_CLAMP_MASK |
		CS42L42_DAC_HPF_EN_MASK |
		CS42L42_DAC_MON_EN_MASK,
		(8 << CS42L42_HPOUT_PULLDOWN_SHIFT) |
		(0 << CS42L42_HPOUT_LOAD_SHIFT) |
		(1 << CS42L42_HPOUT_CLAMP_SHIFT) |
		(1 << CS42L42_DAC_HPF_EN_SHIFT) |
		(1 << CS42L42_DAC_MON_EN_SHIFT));

	/* Power up HS bias to 2.7V */
	cs42l42_reg_update(pDevice, 
		CS42L42_MISC_DET_CTL,
		CS42L42_HSBIAS_CTL_MASK |
		CS42L42_PDN_MIC_LVL_DET_MASK,
		(3 << CS42L42_HSBIAS_CTL_SHIFT) |
		(1 << CS42L42_PDN_MIC_LVL_DET_SHIFT));

	/* Wait for HS bias to ramp up */
	msleep(pDevice->hs_bias_ramp_time);

	/* Unmask auto detect interrupt */
	cs42l42_reg_update(pDevice, 
		CS42L42_CODEC_INT_MASK,
		CS42L42_PDN_DONE_MASK |
		CS42L42_HSDET_AUTO_DONE_MASK,
		(1 << CS42L42_PDN_DONE_SHIFT) |
		(0 << CS42L42_HSDET_AUTO_DONE_SHIFT));

	/* Set hs detect to automatic, enabled mode */
	cs42l42_reg_update(pDevice, 
		CS42L42_HSDET_CTL2,
		CS42L42_HSDET_CTRL_MASK |
		CS42L42_HSDET_SET_MASK |
		CS42L42_HSBIAS_REF_MASK |
		CS42L42_HSDET_AUTO_TIME_MASK,
		(3 << CS42L42_HSDET_CTRL_SHIFT) |
		(2 << CS42L42_HSDET_SET_SHIFT) |
		(0 << CS42L42_HSBIAS_REF_SHIFT) |
		(3 << CS42L42_HSDET_AUTO_TIME_SHIFT));
}

static void cs42l42_cancel_hs_type_detect(PCS42L42_CONTEXT pDevice)
{
	/* Mask button detect interrupts */
	cs42l42_reg_update(pDevice, 
		CS42L42_DET_INT2_MASK,
		CS42L42_M_DETECT_TF_MASK |
		CS42L42_M_DETECT_FT_MASK |
		CS42L42_M_HSBIAS_HIZ_MASK |
		CS42L42_M_SHORT_RLS_MASK |
		CS42L42_M_SHORT_DET_MASK,
		(1 << CS42L42_M_DETECT_TF_SHIFT) |
		(1 << CS42L42_M_DETECT_FT_SHIFT) |
		(1 << CS42L42_M_HSBIAS_HIZ_SHIFT) |
		(1 << CS42L42_M_SHORT_RLS_SHIFT) |
		(1 << CS42L42_M_SHORT_DET_SHIFT));

	/* Ground HS bias */
	cs42l42_reg_update(pDevice, 
		CS42L42_MISC_DET_CTL,
		CS42L42_HSBIAS_CTL_MASK |
		CS42L42_PDN_MIC_LVL_DET_MASK,
		(1 << CS42L42_HSBIAS_CTL_SHIFT) |
		(1 << CS42L42_PDN_MIC_LVL_DET_SHIFT));

	/* Set auto HS bias settings to default */
	cs42l42_reg_update(pDevice, 
		CS42L42_HSBIAS_SC_AUTOCTL,
		CS42L42_HSBIAS_SENSE_EN_MASK |
		CS42L42_AUTO_HSBIAS_HIZ_MASK |
		CS42L42_TIP_SENSE_EN_MASK |
		CS42L42_HSBIAS_SENSE_TRIP_MASK,
		(0 << CS42L42_HSBIAS_SENSE_EN_SHIFT) |
		(0 << CS42L42_AUTO_HSBIAS_HIZ_SHIFT) |
		(0 << CS42L42_TIP_SENSE_EN_SHIFT) |
		(3 << CS42L42_HSBIAS_SENSE_TRIP_SHIFT));

	/* Set hs detect to manual, disabled mode */
	cs42l42_reg_update(pDevice, 
		CS42L42_HSDET_CTL2,
		CS42L42_HSDET_CTRL_MASK |
		CS42L42_HSDET_SET_MASK |
		CS42L42_HSBIAS_REF_MASK |
		CS42L42_HSDET_AUTO_TIME_MASK,
		(0 << CS42L42_HSDET_CTRL_SHIFT) |
		(2 << CS42L42_HSDET_SET_SHIFT) |
		(0 << CS42L42_HSBIAS_REF_SHIFT) |
		(3 << CS42L42_HSDET_AUTO_TIME_SHIFT));
}

static int cs42l42_handle_button_press(PCS42L42_CONTEXT pDevice)
{
	int bias_level;
	unsigned int detect_status;

	/* Mask button detect interrupts */
	cs42l42_reg_update(pDevice, 
		CS42L42_DET_INT2_MASK,
		CS42L42_M_DETECT_TF_MASK |
		CS42L42_M_DETECT_FT_MASK |
		CS42L42_M_HSBIAS_HIZ_MASK |
		CS42L42_M_SHORT_RLS_MASK |
		CS42L42_M_SHORT_DET_MASK,
		(1 << CS42L42_M_DETECT_TF_SHIFT) |
		(1 << CS42L42_M_DETECT_FT_SHIFT) |
		(1 << CS42L42_M_HSBIAS_HIZ_SHIFT) |
		(1 << CS42L42_M_SHORT_RLS_SHIFT) |
		(1 << CS42L42_M_SHORT_DET_SHIFT));

	udelay(pDevice->btn_det_event_dbnce * 1000);

	/* Test all 4 level detect biases */
	bias_level = 1;
	do {
		/* Adjust button detect level sensitivity */
		cs42l42_reg_update(pDevice, 
			CS42L42_MIC_DET_CTL1,
			CS42L42_LATCH_TO_VP_MASK |
			CS42L42_EVENT_STAT_SEL_MASK |
			CS42L42_HS_DET_LEVEL_MASK,
			(1 << CS42L42_LATCH_TO_VP_SHIFT) |
			(0 << CS42L42_EVENT_STAT_SEL_SHIFT) |
			(pDevice->bias_thresholds[bias_level] <<
				CS42L42_HS_DET_LEVEL_SHIFT));

		cs42l42_reg_read(pDevice, CS42L42_DET_STATUS2,
			&detect_status);
	} while ((detect_status & CS42L42_HS_TRUE_MASK) &&
		(++bias_level < CS42L42_NUM_BIASES));

	switch (bias_level) {
	case 1: /* Function C button press */
		bias_level = 3;
		break;
	case 2: /* Function B button press */
		bias_level = 2;
		break;
	case 3: /* Function D button press */
		bias_level = 0;
		break;
	case 4: /* Function A button press */
		bias_level = 1;
		break;
	default:
		bias_level = 0;
		break;
	}

	/* Set button detect level sensitivity back to default */
	cs42l42_reg_update(pDevice, 
		CS42L42_MIC_DET_CTL1,
		CS42L42_LATCH_TO_VP_MASK |
		CS42L42_EVENT_STAT_SEL_MASK |
		CS42L42_HS_DET_LEVEL_MASK,
		(1 << CS42L42_LATCH_TO_VP_SHIFT) |
		(0 << CS42L42_EVENT_STAT_SEL_SHIFT) |
		(pDevice->bias_thresholds[0] << CS42L42_HS_DET_LEVEL_SHIFT));

	/* Clear any button interrupts before unmasking them */
	cs42l42_reg_read(pDevice, CS42L42_DET_INT_STATUS2,
		&detect_status);

	/* Unmask button detect interrupts */
	cs42l42_reg_update(pDevice, 
		CS42L42_DET_INT2_MASK,
		CS42L42_M_DETECT_TF_MASK |
		CS42L42_M_DETECT_FT_MASK |
		CS42L42_M_HSBIAS_HIZ_MASK |
		CS42L42_M_SHORT_RLS_MASK |
		CS42L42_M_SHORT_DET_MASK,
		(0 << CS42L42_M_DETECT_TF_SHIFT) |
		(0 << CS42L42_M_DETECT_FT_SHIFT) |
		(0 << CS42L42_M_HSBIAS_HIZ_SHIFT) |
		(1 << CS42L42_M_SHORT_RLS_SHIFT) |
		(1 << CS42L42_M_SHORT_DET_SHIFT));

	return bias_level;
}

struct cs42l42_irq_params {
	uint16_t status_addr;
	uint16_t mask_addr;
	uint8_t mask;
};

static const struct cs42l42_irq_params irq_params_table[] = {
	{CS42L42_ADC_OVFL_STATUS, CS42L42_ADC_OVFL_INT_MASK,
		CS42L42_ADC_OVFL_VAL_MASK},
	{CS42L42_MIXER_STATUS, CS42L42_MIXER_INT_MASK,
		CS42L42_MIXER_VAL_MASK},
	{CS42L42_SRC_STATUS, CS42L42_SRC_INT_MASK,
		CS42L42_SRC_VAL_MASK},
	{CS42L42_ASP_RX_STATUS, CS42L42_ASP_RX_INT_MASK,
		CS42L42_ASP_RX_VAL_MASK},
	{CS42L42_ASP_TX_STATUS, CS42L42_ASP_TX_INT_MASK,
		CS42L42_ASP_TX_VAL_MASK},
	{CS42L42_CODEC_STATUS, CS42L42_CODEC_INT_MASK,
		CS42L42_CODEC_VAL_MASK},
	{CS42L42_DET_INT_STATUS1, CS42L42_DET_INT1_MASK,
		CS42L42_DET_INT_VAL1_MASK},
	{CS42L42_DET_INT_STATUS2, CS42L42_DET_INT2_MASK,
		CS42L42_DET_INT_VAL2_MASK},
	{CS42L42_SRCPL_INT_STATUS, CS42L42_SRCPL_INT_MASK,
		CS42L42_SRCPL_VAL_MASK},
	{CS42L42_VPMON_STATUS, CS42L42_VPMON_INT_MASK,
		CS42L42_VPMON_VAL_MASK},
	{CS42L42_PLL_LOCK_STATUS, CS42L42_PLL_LOCK_INT_MASK,
		CS42L42_PLL_LOCK_VAL_MASK},
	{CS42L42_TSRS_PLUG_STATUS, CS42L42_TSRS_PLUG_INT_MASK,
		CS42L42_TSRS_PLUG_VAL_MASK}
};

void cs42l42_jack_det(PCS42L42_CONTEXT pDevice) {
	unsigned int stickies[12] = { 0 };
	unsigned int masks[12] = { 0 };
	unsigned int current_plug_status;
	unsigned int current_button_status;

	/* Read sticky registers to clear interrupt */
	for (int i = 0; i < ARRAYSIZE(stickies); i++) {
		cs42l42_reg_read(pDevice, irq_params_table[i].status_addr,
			&(stickies[i]));
		cs42l42_reg_read(pDevice, irq_params_table[i].mask_addr,
			&(masks[i]));
		stickies[i] = stickies[i] & (~masks[i]) &
			irq_params_table[i].mask;
	}

	/* Read tip sense status before handling type detect */
	current_plug_status = (stickies[11] &
		(CS42L42_TS_PLUG_MASK | CS42L42_TS_UNPLUG_MASK)) >>
		CS42L42_TS_PLUG_SHIFT;

	/* Read button sense status */
	current_button_status = stickies[7] &
		(CS42L42_M_DETECT_TF_MASK |
			CS42L42_M_DETECT_FT_MASK |
			CS42L42_M_HSBIAS_HIZ_MASK);

	/*
	 * Check auto-detect status. Don't assume a previous unplug event has
	 * cleared the flags. If the jack is unplugged and plugged during
	 * system suspend there won't have been an unplug event.
	 */
	if ((~masks[5]) & irq_params_table[5].mask) {
		if (stickies[5] & CS42L42_HSDET_AUTO_DONE_MASK) {
			cs42l42_process_hs_type_detect(pDevice);

			CsAudioSpecialKeyReport report;
			report.ReportID = REPORTID_SPECKEYS;
			report.ControlCode = CONTROL_CODE_JACK_TYPE;

			switch (pDevice->hs_type) {
			case CS42L42_PLUG_CTIA:
			case CS42L42_PLUG_OMTP:
				report.ControlValue = SND_JACK_HEADSET;

				break;
			case CS42L42_PLUG_HEADPHONE:
				report.ControlValue = SND_JACK_HEADPHONE;

				size_t bytesWritten;
				Cs42l42ProcessVendorReport(pDevice, &report, sizeof(report), &bytesWritten);

				break;
			default:
				report.ControlValue = SND_JACK_HEADPHONE;
				break;
			}

			size_t bytesWritten;
			Cs42l42ProcessVendorReport(pDevice, &report, sizeof(report), &bytesWritten);
		}
	}

	/* Check tip sense status */
	if ((~masks[11]) & irq_params_table[11].mask) {
		switch (current_plug_status) {
		case CS42L42_TS_PLUG:
			if (pDevice->plug_state != CS42L42_TS_PLUG) {
				pDevice->plug_state = CS42L42_TS_PLUG;
				cs42l42_init_hs_type_detect(pDevice);
			}
			break;

		case CS42L42_TS_UNPLUG:
			if (pDevice->plug_state != CS42L42_TS_UNPLUG) {
				pDevice->plug_state = CS42L42_TS_UNPLUG;
				cs42l42_cancel_hs_type_detect(pDevice);

				CsAudioSpecialKeyReport report;
				report.ReportID = REPORTID_SPECKEYS;
				report.ControlCode = CONTROL_CODE_JACK_TYPE;
				report.ControlValue = 0;

				size_t bytesWritten;
				Cs42l42ProcessVendorReport(pDevice, &report, sizeof(report), &bytesWritten);
			}
			break;

		default:
			pDevice->plug_state = CS42L42_TS_TRANS;
		}
	}

	/* Check button detect status */
	if (pDevice->plug_state == CS42L42_TS_PLUG && ((~masks[7]) & irq_params_table[7].mask)) {
		if (!(current_button_status &
			CS42L42_M_HSBIAS_HIZ_MASK)) {

			if (current_button_status & CS42L42_M_DETECT_TF_MASK) {
				Cs42l42MediaReport report;
				report.ReportID = REPORTID_MEDIA;
				report.ControlCode = pDevice->button_state;

				size_t bytesWritten;
				Cs42l42ProcessVendorReport(pDevice, &report, sizeof(report), &bytesWritten);
			}
			else if (current_button_status & CS42L42_M_DETECT_FT_MASK) {
				pDevice->button_state = cs42l42_handle_button_press(pDevice);
			}
		}
	}
}

BOOLEAN OnInterruptIsr(
	WDFINTERRUPT Interrupt,
	ULONG MessageID) {
	UNREFERENCED_PARAMETER(MessageID);

	WDFDEVICE Device = WdfInterruptGetDevice(Interrupt);
	PCS42L42_CONTEXT pDevice = GetDeviceContext(Device);

	if (!pDevice->DevicePoweredOn)
		return true;

	NTSTATUS status = STATUS_SUCCESS;

	cs42l42_jack_det(pDevice);

	return true;
}

NTSTATUS
Cs42l42EvtDeviceAdd(
	IN WDFDRIVER       Driver,
	IN PWDFDEVICE_INIT DeviceInit
)
{
	NTSTATUS                      status = STATUS_SUCCESS;
	WDF_IO_QUEUE_CONFIG           queueConfig;
	WDF_OBJECT_ATTRIBUTES         attributes;
	WDFDEVICE                     device;
	WDF_INTERRUPT_CONFIG interruptConfig;
	WDFQUEUE                      queue;
	UCHAR                         minorFunction;
	PCS42L42_CONTEXT               devContext;

	UNREFERENCED_PARAMETER(Driver);

	PAGED_CODE();

	Cs42l42Print(DEBUG_LEVEL_INFO, DBG_PNP,
		"Cs42l42EvtDeviceAdd called\n");

	//
	// Tell framework this is a filter driver. Filter drivers by default are  
	// not power policy owners. This works well for this driver because
	// HIDclass driver is the power policy owner for HID minidrivers.
	//

	WdfFdoInitSetFilter(DeviceInit);

	{
		WDF_PNPPOWER_EVENT_CALLBACKS pnpCallbacks;
		WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);

		pnpCallbacks.EvtDevicePrepareHardware = OnPrepareHardware;
		pnpCallbacks.EvtDeviceReleaseHardware = OnReleaseHardware;
		pnpCallbacks.EvtDeviceD0Entry = OnD0Entry;
		pnpCallbacks.EvtDeviceD0Exit = OnD0Exit;

		WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpCallbacks);
	}

	//
	// Setup the device context
	//

	WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, CS42L42_CONTEXT);

	//
	// Create a framework device object.This call will in turn create
	// a WDM device object, attach to the lower stack, and set the
	// appropriate flags and attributes.
	//

	status = WdfDeviceCreate(&DeviceInit, &attributes, &device);

	if (!NT_SUCCESS(status))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfDeviceCreate failed with status code 0x%x\n", status);

		return status;
	}

	WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&queueConfig, WdfIoQueueDispatchParallel);

	queueConfig.EvtIoInternalDeviceControl = Cs42l42EvtInternalDeviceControl;

	status = WdfIoQueueCreate(device,
		&queueConfig,
		WDF_NO_OBJECT_ATTRIBUTES,
		&queue
	);

	if (!NT_SUCCESS(status))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfIoQueueCreate failed 0x%x\n", status);

		return status;
	}

	//
	// Create manual I/O queue to take care of hid report read requests
	//

	devContext = GetDeviceContext(device);

	devContext->FxDevice = device;

	WDF_IO_QUEUE_CONFIG_INIT(&queueConfig, WdfIoQueueDispatchManual);

	queueConfig.PowerManaged = WdfFalse;

	status = WdfIoQueueCreate(device,
		&queueConfig,
		WDF_NO_OBJECT_ATTRIBUTES,
		&devContext->ReportQueue
	);

	if (!NT_SUCCESS(status))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfIoQueueCreate failed 0x%x\n", status);

		return status;
	}

	WDF_OBJECT_ATTRIBUTES lockAttributes;
	WDF_OBJECT_ATTRIBUTES_INIT(&lockAttributes);
	lockAttributes.ParentObject = device;
	status = WdfWaitLockCreate(&lockAttributes, &devContext->RegisterLock);
	if (!NT_SUCCESS(status))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_PNP,
			"Error creating WDF register lock - %!STATUS!",
			status);

		return status;
	}

	//
	// Create an interrupt object for hardware notifications
	//
	WDF_INTERRUPT_CONFIG_INIT(
		&interruptConfig,
		OnInterruptIsr,
		NULL);
	interruptConfig.PassiveHandling = TRUE;

	status = WdfInterruptCreate(
		device,
		&interruptConfig,
		WDF_NO_OBJECT_ATTRIBUTES,
		&devContext->Interrupt);

	if (!NT_SUCCESS(status))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_PNP,
			"Error creating WDF interrupt object - %!STATUS!",
			status);

		return status;
	}

	return status;
}

VOID
Cs42l42EvtInternalDeviceControl(
	IN WDFQUEUE     Queue,
	IN WDFREQUEST   Request,
	IN size_t       OutputBufferLength,
	IN size_t       InputBufferLength,
	IN ULONG        IoControlCode
)
{
	NTSTATUS            status = STATUS_SUCCESS;
	WDFDEVICE           device;
	PCS42L42_CONTEXT     devContext;
	BOOLEAN             completeRequest = TRUE;

	UNREFERENCED_PARAMETER(OutputBufferLength);
	UNREFERENCED_PARAMETER(InputBufferLength);

	device = WdfIoQueueGetDevice(Queue);
	devContext = GetDeviceContext(device);

	Cs42l42Print(DEBUG_LEVEL_INFO, DBG_IOCTL,
		"%s, Queue:0x%p, Request:0x%p\n",
		DbgHidInternalIoctlString(IoControlCode),
		Queue,
		Request
	);

	//
	// Please note that HIDCLASS provides the buffer in the Irp->UserBuffer
	// field irrespective of the ioctl buffer type. However, framework is very
	// strict about type checking. You cannot get Irp->UserBuffer by using
	// WdfRequestRetrieveOutputMemory if the ioctl is not a METHOD_NEITHER
	// internal ioctl. So depending on the ioctl code, we will either
	// use retreive function or escape to WDM to get the UserBuffer.
	//

	switch (IoControlCode)
	{

	case IOCTL_HID_GET_DEVICE_DESCRIPTOR:
		//
		// Retrieves the device's HID descriptor.
		//
		status = Cs42l42GetHidDescriptor(device, Request);
		break;

	case IOCTL_HID_GET_DEVICE_ATTRIBUTES:
		//
		//Retrieves a device's attributes in a HID_DEVICE_ATTRIBUTES structure.
		//
		status = Cs42l42GetDeviceAttributes(Request);
		break;

	case IOCTL_HID_GET_REPORT_DESCRIPTOR:
		//
		//Obtains the report descriptor for the HID device.
		//
		status = Cs42l42GetReportDescriptor(device, Request);
		break;

	case IOCTL_HID_GET_STRING:
		//
		// Requests that the HID minidriver retrieve a human-readable string
		// for either the manufacturer ID, the product ID, or the serial number
		// from the string descriptor of the device. The minidriver must send
		// a Get String Descriptor request to the device, in order to retrieve
		// the string descriptor, then it must extract the string at the
		// appropriate index from the string descriptor and return it in the
		// output buffer indicated by the IRP. Before sending the Get String
		// Descriptor request, the minidriver must retrieve the appropriate
		// index for the manufacturer ID, the product ID or the serial number
		// from the device extension of a top level collection associated with
		// the device.
		//
		status = Cs42l42GetString(Request);
		break;

	case IOCTL_HID_WRITE_REPORT:
	case IOCTL_HID_SET_OUTPUT_REPORT:
		//
		//Transmits a class driver-supplied report to the device.
		//
		status = Cs42l42WriteReport(devContext, Request);
		break;

	case IOCTL_HID_READ_REPORT:
	case IOCTL_HID_GET_INPUT_REPORT:
		//
		// Returns a report from the device into a class driver-supplied buffer.
		// 
		status = Cs42l42ReadReport(devContext, Request, &completeRequest);
		break;

	case IOCTL_HID_SET_FEATURE:
		//
		// This sends a HID class feature report to a top-level collection of
		// a HID class device.
		//
		status = Cs42l42SetFeature(devContext, Request, &completeRequest);
		break;

	case IOCTL_HID_GET_FEATURE:
		//
		// returns a feature report associated with a top-level collection
		status = Cs42l42GetFeature(devContext, Request, &completeRequest);
		break;

	case IOCTL_HID_ACTIVATE_DEVICE:
		//
		// Makes the device ready for I/O operations.
		//
	case IOCTL_HID_DEACTIVATE_DEVICE:
		//
		// Causes the device to cease operations and terminate all outstanding
		// I/O requests.
		//
	default:
		status = STATUS_NOT_SUPPORTED;
		break;
	}

	if (completeRequest)
	{
		WdfRequestComplete(Request, status);

		Cs42l42Print(DEBUG_LEVEL_INFO, DBG_IOCTL,
			"%s completed, Queue:0x%p, Request:0x%p\n",
			DbgHidInternalIoctlString(IoControlCode),
			Queue,
			Request
		);
	}
	else
	{
		Cs42l42Print(DEBUG_LEVEL_INFO, DBG_IOCTL,
			"%s deferred, Queue:0x%p, Request:0x%p\n",
			DbgHidInternalIoctlString(IoControlCode),
			Queue,
			Request
		);
	}

	return;
}

NTSTATUS
Cs42l42GetHidDescriptor(
	IN WDFDEVICE Device,
	IN WDFREQUEST Request
)
{
	NTSTATUS            status = STATUS_SUCCESS;
	size_t              bytesToCopy = 0;
	WDFMEMORY           memory;

	UNREFERENCED_PARAMETER(Device);

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42GetHidDescriptor Entry\n");

	//
	// This IOCTL is METHOD_NEITHER so WdfRequestRetrieveOutputMemory
	// will correctly retrieve buffer from Irp->UserBuffer. 
	// Remember that HIDCLASS provides the buffer in the Irp->UserBuffer
	// field irrespective of the ioctl buffer type. However, framework is very
	// strict about type checking. You cannot get Irp->UserBuffer by using
	// WdfRequestRetrieveOutputMemory if the ioctl is not a METHOD_NEITHER
	// internal ioctl.
	//
	status = WdfRequestRetrieveOutputMemory(Request, &memory);

	if (!NT_SUCCESS(status))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"WdfRequestRetrieveOutputMemory failed 0x%x\n", status);

		return status;
	}

	//
	// Use hardcoded "HID Descriptor" 
	//
	bytesToCopy = DefaultHidDescriptor.bLength;

	if (bytesToCopy == 0)
	{
		status = STATUS_INVALID_DEVICE_STATE;

		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"DefaultHidDescriptor is zero, 0x%x\n", status);

		return status;
	}

	status = WdfMemoryCopyFromBuffer(memory,
		0, // Offset
		(PVOID)&DefaultHidDescriptor,
		bytesToCopy);

	if (!NT_SUCCESS(status))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"WdfMemoryCopyFromBuffer failed 0x%x\n", status);

		return status;
	}

	//
	// Report how many bytes were copied
	//
	WdfRequestSetInformation(Request, bytesToCopy);

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42GetHidDescriptor Exit = 0x%x\n", status);

	return status;
}

NTSTATUS
Cs42l42GetReportDescriptor(
	IN WDFDEVICE Device,
	IN WDFREQUEST Request
)
{
	NTSTATUS            status = STATUS_SUCCESS;
	ULONG_PTR           bytesToCopy;
	WDFMEMORY           memory;

	PCS42L42_CONTEXT devContext = GetDeviceContext(Device);

	UNREFERENCED_PARAMETER(Device);

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42GetReportDescriptor Entry\n");

	//
	// This IOCTL is METHOD_NEITHER so WdfRequestRetrieveOutputMemory
	// will correctly retrieve buffer from Irp->UserBuffer. 
	// Remember that HIDCLASS provides the buffer in the Irp->UserBuffer
	// field irrespective of the ioctl buffer type. However, framework is very
	// strict about type checking. You cannot get Irp->UserBuffer by using
	// WdfRequestRetrieveOutputMemory if the ioctl is not a METHOD_NEITHER
	// internal ioctl.
	//
	status = WdfRequestRetrieveOutputMemory(Request, &memory);
	if (!NT_SUCCESS(status))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"WdfRequestRetrieveOutputMemory failed 0x%x\n", status);

		return status;
	}

	//
	// Use hardcoded Report descriptor
	//
	bytesToCopy = DefaultHidDescriptor.DescriptorList[0].wReportLength;

	if (bytesToCopy == 0)
	{
		status = STATUS_INVALID_DEVICE_STATE;

		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"DefaultHidDescriptor's reportLength is zero, 0x%x\n", status);

		return status;
	}

	status = WdfMemoryCopyFromBuffer(memory,
		0,
		(PVOID)DefaultReportDescriptor,
		bytesToCopy);
	if (!NT_SUCCESS(status))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"WdfMemoryCopyFromBuffer failed 0x%x\n", status);

		return status;
	}

	//
	// Report how many bytes were copied
	//
	WdfRequestSetInformation(Request, bytesToCopy);

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42GetReportDescriptor Exit = 0x%x\n", status);

	return status;
}


NTSTATUS
Cs42l42GetDeviceAttributes(
	IN WDFREQUEST Request
)
{
	NTSTATUS                 status = STATUS_SUCCESS;
	PHID_DEVICE_ATTRIBUTES   deviceAttributes = NULL;

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42GetDeviceAttributes Entry\n");

	//
	// This IOCTL is METHOD_NEITHER so WdfRequestRetrieveOutputMemory
	// will correctly retrieve buffer from Irp->UserBuffer. 
	// Remember that HIDCLASS provides the buffer in the Irp->UserBuffer
	// field irrespective of the ioctl buffer type. However, framework is very
	// strict about type checking. You cannot get Irp->UserBuffer by using
	// WdfRequestRetrieveOutputMemory if the ioctl is not a METHOD_NEITHER
	// internal ioctl.
	//
	status = WdfRequestRetrieveOutputBuffer(Request,
		sizeof(HID_DEVICE_ATTRIBUTES),
		(PVOID*)&deviceAttributes,
		NULL);
	if (!NT_SUCCESS(status))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"WdfRequestRetrieveOutputBuffer failed 0x%x\n", status);

		return status;
	}

	//
	// Set USB device descriptor
	//

	deviceAttributes->Size = sizeof(HID_DEVICE_ATTRIBUTES);
	deviceAttributes->VendorID = CS42L42_VID;
	deviceAttributes->ProductID = CS42L42_PID;
	deviceAttributes->VersionNumber = CS42L42_VERSION;

	//
	// Report how many bytes were copied
	//
	WdfRequestSetInformation(Request, sizeof(HID_DEVICE_ATTRIBUTES));

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42GetDeviceAttributes Exit = 0x%x\n", status);

	return status;
}

NTSTATUS
Cs42l42GetString(
	IN WDFREQUEST Request
)
{

	NTSTATUS status = STATUS_SUCCESS;
	PWSTR pwstrID;
	size_t lenID;
	WDF_REQUEST_PARAMETERS params;
	void* pStringBuffer = NULL;

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42GetString Entry\n");

	WDF_REQUEST_PARAMETERS_INIT(&params);
	WdfRequestGetParameters(Request, &params);

	switch ((ULONG_PTR)params.Parameters.DeviceIoControl.Type3InputBuffer & 0xFFFF)
	{
	case HID_STRING_ID_IMANUFACTURER:
		pwstrID = L"Cs42l42.\0";
		break;

	case HID_STRING_ID_IPRODUCT:
		pwstrID = L"MaxTouch Touch Screen\0";
		break;

	case HID_STRING_ID_ISERIALNUMBER:
		pwstrID = L"123123123\0";
		break;

	default:
		pwstrID = NULL;
		break;
	}

	lenID = pwstrID ? wcslen(pwstrID) * sizeof(WCHAR) + sizeof(UNICODE_NULL) : 0;

	if (pwstrID == NULL)
	{

		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"Cs42l42GetString Invalid request type\n");

		status = STATUS_INVALID_PARAMETER;

		return status;
	}

	status = WdfRequestRetrieveOutputBuffer(Request,
		lenID,
		&pStringBuffer,
		&lenID);

	if (!NT_SUCCESS(status))
	{

		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"Cs42l42GetString WdfRequestRetrieveOutputBuffer failed Status 0x%x\n", status);

		return status;
	}

	RtlCopyMemory(pStringBuffer, pwstrID, lenID);

	WdfRequestSetInformation(Request, lenID);

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42GetString Exit = 0x%x\n", status);

	return status;
}

NTSTATUS
Cs42l42WriteReport(
	IN PCS42L42_CONTEXT DevContext,
	IN WDFREQUEST Request
)
{
	NTSTATUS status = STATUS_SUCCESS;
	WDF_REQUEST_PARAMETERS params;
	PHID_XFER_PACKET transferPacket = NULL;
	size_t bytesWritten = 0;

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42WriteReport Entry\n");

	WDF_REQUEST_PARAMETERS_INIT(&params);
	WdfRequestGetParameters(Request, &params);

	if (params.Parameters.DeviceIoControl.InputBufferLength < sizeof(HID_XFER_PACKET))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"Cs42l42WriteReport Xfer packet too small\n");

		status = STATUS_BUFFER_TOO_SMALL;
	}
	else
	{

		transferPacket = (PHID_XFER_PACKET)WdfRequestWdmGetIrp(Request)->UserBuffer;

		if (transferPacket == NULL)
		{
			Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
				"Cs42l42WriteReport No xfer packet\n");

			status = STATUS_INVALID_DEVICE_REQUEST;
		}
		else
		{
			//
			// switch on the report id
			//

			switch (transferPacket->reportId)
			{
			default:

				Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
					"Cs42l42WriteReport Unhandled report type %d\n", transferPacket->reportId);

				status = STATUS_INVALID_PARAMETER;

				break;
			}
		}
	}

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42WriteReport Exit = 0x%x\n", status);

	return status;

}

NTSTATUS
Cs42l42ProcessVendorReport(
	IN PCS42L42_CONTEXT DevContext,
	IN PVOID ReportBuffer,
	IN ULONG ReportBufferLen,
	OUT size_t* BytesWritten
)
{
	NTSTATUS status = STATUS_SUCCESS;
	WDFREQUEST reqRead;
	PVOID pReadReport = NULL;
	size_t bytesReturned = 0;

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42ProcessVendorReport Entry\n");

	status = WdfIoQueueRetrieveNextRequest(DevContext->ReportQueue,
		&reqRead);

	if (NT_SUCCESS(status))
	{
		status = WdfRequestRetrieveOutputBuffer(reqRead,
			ReportBufferLen,
			&pReadReport,
			&bytesReturned);

		if (NT_SUCCESS(status))
		{
			//
			// Copy ReportBuffer into read request
			//

			if (bytesReturned > ReportBufferLen)
			{
				bytesReturned = ReportBufferLen;
			}

			RtlCopyMemory(pReadReport,
				ReportBuffer,
				bytesReturned);

			//
			// Complete read with the number of bytes returned as info
			//

			WdfRequestCompleteWithInformation(reqRead,
				status,
				bytesReturned);

			Cs42l42Print(DEBUG_LEVEL_INFO, DBG_IOCTL,
				"Cs42l42ProcessVendorReport %d bytes returned\n", bytesReturned);

			//
			// Return the number of bytes written for the write request completion
			//

			*BytesWritten = bytesReturned;

			Cs42l42Print(DEBUG_LEVEL_INFO, DBG_IOCTL,
				"%s completed, Queue:0x%p, Request:0x%p\n",
				DbgHidInternalIoctlString(IOCTL_HID_READ_REPORT),
				DevContext->ReportQueue,
				reqRead);
		}
		else
		{
			Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
				"WdfRequestRetrieveOutputBuffer failed Status 0x%x\n", status);
		}
	}
	else
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"WdfIoQueueRetrieveNextRequest failed Status 0x%x\n", status);
	}

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42ProcessVendorReport Exit = 0x%x\n", status);

	return status;
}

NTSTATUS
Cs42l42ReadReport(
	IN PCS42L42_CONTEXT DevContext,
	IN WDFREQUEST Request,
	OUT BOOLEAN* CompleteRequest
)
{
	NTSTATUS status = STATUS_SUCCESS;

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42ReadReport Entry\n");

	//
	// Forward this read request to our manual queue
	// (in other words, we are going to defer this request
	// until we have a corresponding write request to
	// match it with)
	//

	status = WdfRequestForwardToIoQueue(Request, DevContext->ReportQueue);

	if (!NT_SUCCESS(status))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"WdfRequestForwardToIoQueue failed Status 0x%x\n", status);
	}
	else
	{
		*CompleteRequest = FALSE;
	}

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42ReadReport Exit = 0x%x\n", status);

	return status;
}

NTSTATUS
Cs42l42SetFeature(
	IN PCS42L42_CONTEXT DevContext,
	IN WDFREQUEST Request,
	OUT BOOLEAN* CompleteRequest
)
{
	NTSTATUS status = STATUS_SUCCESS;
	WDF_REQUEST_PARAMETERS params;
	PHID_XFER_PACKET transferPacket = NULL;

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42SetFeature Entry\n");

	WDF_REQUEST_PARAMETERS_INIT(&params);
	WdfRequestGetParameters(Request, &params);

	if (params.Parameters.DeviceIoControl.InputBufferLength < sizeof(HID_XFER_PACKET))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"Cs42l42SetFeature Xfer packet too small\n");

		status = STATUS_BUFFER_TOO_SMALL;
	}
	else
	{

		transferPacket = (PHID_XFER_PACKET)WdfRequestWdmGetIrp(Request)->UserBuffer;

		if (transferPacket == NULL)
		{
			Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
				"Cs42l42WriteReport No xfer packet\n");

			status = STATUS_INVALID_DEVICE_REQUEST;
		}
		else
		{
			//
			// switch on the report id
			//

			switch (transferPacket->reportId)
			{
			default:

				Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
					"Cs42l42SetFeature Unhandled report type %d\n", transferPacket->reportId);

				status = STATUS_INVALID_PARAMETER;

				break;
			}
		}
	}

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42SetFeature Exit = 0x%x\n", status);

	return status;
}

NTSTATUS
Cs42l42GetFeature(
	IN PCS42L42_CONTEXT DevContext,
	IN WDFREQUEST Request,
	OUT BOOLEAN* CompleteRequest
)
{
	NTSTATUS status = STATUS_SUCCESS;
	WDF_REQUEST_PARAMETERS params;
	PHID_XFER_PACKET transferPacket = NULL;

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42GetFeature Entry\n");

	WDF_REQUEST_PARAMETERS_INIT(&params);
	WdfRequestGetParameters(Request, &params);

	if (params.Parameters.DeviceIoControl.OutputBufferLength < sizeof(HID_XFER_PACKET))
	{
		Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"Cs42l42GetFeature Xfer packet too small\n");

		status = STATUS_BUFFER_TOO_SMALL;
	}
	else
	{

		transferPacket = (PHID_XFER_PACKET)WdfRequestWdmGetIrp(Request)->UserBuffer;

		if (transferPacket == NULL)
		{
			Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
				"Cs42l42GetFeature No xfer packet\n");

			status = STATUS_INVALID_DEVICE_REQUEST;
		}
		else
		{
			//
			// switch on the report id
			//

			switch (transferPacket->reportId)
			{
			default:

				Cs42l42Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
					"Cs42l42GetFeature Unhandled report type %d\n", transferPacket->reportId);

				status = STATUS_INVALID_PARAMETER;

				break;
			}
		}
	}

	Cs42l42Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Cs42l42GetFeature Exit = 0x%x\n", status);

	return status;
}

PCHAR
DbgHidInternalIoctlString(
	IN ULONG IoControlCode
)
{
	switch (IoControlCode)
	{
	case IOCTL_HID_GET_DEVICE_DESCRIPTOR:
		return "IOCTL_HID_GET_DEVICE_DESCRIPTOR";
	case IOCTL_HID_GET_REPORT_DESCRIPTOR:
		return "IOCTL_HID_GET_REPORT_DESCRIPTOR";
	case IOCTL_HID_READ_REPORT:
		return "IOCTL_HID_READ_REPORT";
	case IOCTL_HID_GET_DEVICE_ATTRIBUTES:
		return "IOCTL_HID_GET_DEVICE_ATTRIBUTES";
	case IOCTL_HID_WRITE_REPORT:
		return "IOCTL_HID_WRITE_REPORT";
	case IOCTL_HID_SET_FEATURE:
		return "IOCTL_HID_SET_FEATURE";
	case IOCTL_HID_GET_FEATURE:
		return "IOCTL_HID_GET_FEATURE";
	case IOCTL_HID_GET_STRING:
		return "IOCTL_HID_GET_STRING";
	case IOCTL_HID_ACTIVATE_DEVICE:
		return "IOCTL_HID_ACTIVATE_DEVICE";
	case IOCTL_HID_DEACTIVATE_DEVICE:
		return "IOCTL_HID_DEACTIVATE_DEVICE";
	case IOCTL_HID_SEND_IDLE_NOTIFICATION_REQUEST:
		return "IOCTL_HID_SEND_IDLE_NOTIFICATION_REQUEST";
	case IOCTL_HID_SET_OUTPUT_REPORT:
		return "IOCTL_HID_SET_OUTPUT_REPORT";
	case IOCTL_HID_GET_INPUT_REPORT:
		return "IOCTL_HID_GET_INPUT_REPORT";
	default:
		return "Unknown IOCTL";
	}
}