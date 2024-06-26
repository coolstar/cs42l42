#if !defined(_CS42L42_H_)
#define _CS42L42_H_

#pragma warning(disable:4200)  // suppress nameless struct/union warning
#pragma warning(disable:4201)  // suppress nameless struct/union warning
#pragma warning(disable:4214)  // suppress bit field types other than int warning
#include <initguid.h>
#include <wdm.h>

#pragma warning(default:4200)
#pragma warning(default:4201)
#pragma warning(default:4214)
#include <wdf.h>

#pragma warning(disable:4201)  // suppress nameless struct/union warning
#pragma warning(disable:4214)  // suppress bit field types other than int warning
#include <hidport.h>

#include "hidcommon.h"

#include <stdint.h>

#include "spb.h"

enum snd_jack_types {
	SND_JACK_HEADPHONE = 0x0001,
	SND_JACK_MICROPHONE = 0x0002,
	SND_JACK_HEADSET = SND_JACK_HEADPHONE | SND_JACK_MICROPHONE,
	SND_JACK_LINEOUT = 0x0004,
	SND_JACK_MECHANICAL = 0x0008, /* If detected separately */
	SND_JACK_VIDEOOUT = 0x0010,
	SND_JACK_AVOUT = SND_JACK_LINEOUT | SND_JACK_VIDEOOUT,
	SND_JACK_LINEIN = 0x0020,

	/* Kept separate from switches to facilitate implementation */
	SND_JACK_BTN_0 = 0x4000,
	SND_JACK_BTN_1 = 0x2000,
	SND_JACK_BTN_2 = 0x1000,
	SND_JACK_BTN_3 = 0x0800,
	SND_JACK_BTN_4 = 0x0400,
	SND_JACK_BTN_5 = 0x0200,
};

//
// String definitions
//

#define DRIVERNAME                 "da7219.sys: "

#define CS42L42_POOL_TAG            (ULONG) '24SC'

#define true 1
#define false 0

typedef UCHAR HID_REPORT_DESCRIPTOR, * PHID_REPORT_DESCRIPTOR;

#ifdef DESCRIPTOR_DEF
HID_REPORT_DESCRIPTOR DefaultReportDescriptor[] = {
	//
	// Consumer Control starts here
	//
	0x05, 0x0C, /*		Usage Page (Consumer Devices)		*/
	0x09, 0x01, /*		Usage (Consumer Control)			*/
	0xA1, 0x01, /*		Collection (Application)			*/
	0x85, REPORTID_MEDIA,	/*		Report ID=1							*/
	0x05, 0x0C, /*		Usage Page (Consumer Devices)		*/
	0x15, 0x00, /*		Logical Minimum (0)					*/
	0x25, 0x01, /*		Logical Maximum (1)					*/
	0x75, 0x01, /*		Report Size (1)						*/
	0x95, 0x04, /*		Report Count (4)					*/
	0x09, 0xCD, /*		Usage (Play / Pause)				*/
	0x09, 0xCF, /*		Usage (Voice Command)				*/
	0x09, 0xE9, /*		Usage (Volume Up)					*/
	0x09, 0xEA, /*		Usage (Volume Down)					*/
	0x81, 0x02, /*		Input (Data, Variable, Absolute)	*/
	0x95, 0x04, /*		Report Count (4)					*/
	0x81, 0x01, /*		Input (Constant)					*/
	0xC0,        /*        End Collection                        */

	0x06, 0x00, 0xff,                    // USAGE_PAGE (Vendor Defined Page 1)
	0x09, 0x04,                          // USAGE (Vendor Usage 4)
	0xa1, 0x01,                          // COLLECTION (Application)
	0x85, REPORTID_SPECKEYS,             //   REPORT_ID (Special Keys)
	0x15, 0x00,                          //   LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x00,                    //   LOGICAL_MAXIMUM (256)
	0x75, 0x08,                          //   REPORT_SIZE  (8)   - bits
	0x95, 0x01,                          //   REPORT_COUNT (1)  - Bytes
	0x09, 0x02,                          //   USAGE (Vendor Usage 1)
	0x81, 0x02,                          //   INPUT (Data,Var,Abs)
	0x09, 0x03,                          //   USAGE (Vendor Usage 2)
	0x81, 0x02,                          //   INPUT (Data,Var,Abs)
	0xc0,                                // END_COLLECTION
};


//
// This is the default HID descriptor returned by the mini driver
// in response to IOCTL_HID_GET_DEVICE_DESCRIPTOR. The size
// of report descriptor is currently the size of DefaultReportDescriptor.
//

CONST HID_DESCRIPTOR DefaultHidDescriptor = {
	0x09,   // length of HID descriptor
	0x21,   // descriptor type == HID  0x21
	0x0100, // hid spec release
	0x00,   // country code == Not Specified
	0x01,   // number of HID class descriptors
	{ 0x22,   // descriptor type 
	sizeof(DefaultReportDescriptor) }  // total length of report descriptor
};
#endif

struct reg {
	uint16_t reg;
	uint8_t val;
};

typedef struct _CS42L42_CONTEXT
{

	WDFDEVICE FxDevice;

	WDFQUEUE ReportQueue;

	WDFQUEUE IdleQueue;

	SPB_CONTEXT I2CContext;

	WDFWAITLOCK RegisterLock;

	BOOLEAN DevicePoweredOn;

	WDFINTERRUPT Interrupt;

	INT JackType;

	//device data
	UINT8 ts_inv;
	UINT8 ts_dbnc_rise;
	UINT8 ts_dbnc_fall;
	UINT8 btn_det_init_dbnce;
	UINT8 btn_det_event_dbnce;

	UINT32 bias_thresholds[4];

	UINT8 hs_bias_ramp_rate;
	UINT8 hs_bias_ramp_time;
	UINT8 hs_bias_sense_en;

	//device variables
	UINT8 plug_state;
	UINT8 hs_type;

	int pll_config;

	int button_state;
} CS42L42_CONTEXT, *PCS42L42_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(CS42L42_CONTEXT, GetDeviceContext)

//
// Power Idle Workitem context
// 
typedef struct _IDLE_WORKITEM_CONTEXT
{
	// Handle to a WDF device object
	WDFDEVICE FxDevice;

	// Handle to a WDF request object
	WDFREQUEST FxRequest;

} IDLE_WORKITEM_CONTEXT, * PIDLE_WORKITEM_CONTEXT;
WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(IDLE_WORKITEM_CONTEXT, GetIdleWorkItemContext)

//
// Function definitions
//

DRIVER_INITIALIZE DriverEntry;

EVT_WDF_DRIVER_UNLOAD Cs42l42DriverUnload;

EVT_WDF_DRIVER_DEVICE_ADD Cs42l42EvtDeviceAdd;

EVT_WDFDEVICE_WDM_IRP_PREPROCESS Cs42l42EvtWdmPreprocessMnQueryId;

EVT_WDF_IO_QUEUE_IO_INTERNAL_DEVICE_CONTROL Cs42l42EvtInternalDeviceControl;

NTSTATUS
Cs42l42GetHidDescriptor(
	IN WDFDEVICE Device,
	IN WDFREQUEST Request
);

NTSTATUS
Cs42l42GetReportDescriptor(
	IN WDFDEVICE Device,
	IN WDFREQUEST Request
);

NTSTATUS
Cs42l42GetDeviceAttributes(
	IN WDFREQUEST Request
);

NTSTATUS
Cs42l42GetString(
	IN WDFREQUEST Request
);

NTSTATUS
Cs42l42WriteReport(
	IN PCS42L42_CONTEXT DevContext,
	IN WDFREQUEST Request
);

NTSTATUS
Cs42l42ProcessVendorReport(
	IN PCS42L42_CONTEXT DevContext,
	IN PVOID ReportBuffer,
	IN ULONG ReportBufferLen,
	OUT size_t* BytesWritten
);

NTSTATUS
Cs42l42ReadReport(
	IN PCS42L42_CONTEXT DevContext,
	IN WDFREQUEST Request,
	OUT BOOLEAN* CompleteRequest
);

NTSTATUS
Cs42l42SetFeature(
	IN PCS42L42_CONTEXT DevContext,
	IN WDFREQUEST Request,
	OUT BOOLEAN* CompleteRequest
);

NTSTATUS
Cs42l42GetFeature(
	IN PCS42L42_CONTEXT DevContext,
	IN WDFREQUEST Request,
	OUT BOOLEAN* CompleteRequest
);

PCHAR
DbgHidInternalIoctlString(
	IN ULONG        IoControlCode
);

VOID
Cs42l42CompleteIdleIrp(
	IN PCS42L42_CONTEXT FxDeviceContext
);

//
// Helper macros
//

#define DEBUG_LEVEL_ERROR   1
#define DEBUG_LEVEL_INFO    2
#define DEBUG_LEVEL_VERBOSE 3

#define DBG_INIT  1
#define DBG_PNP   2
#define DBG_IOCTL 4

#if 0
#define Cs42l42Print(dbglevel, dbgcatagory, fmt, ...) {          \
    if (Cs42l42DebugLevel >= dbglevel &&                         \
        (Cs42l42DebugCatagories && dbgcatagory))                 \
	    {                                                           \
        DbgPrint(DRIVERNAME);                                   \
        DbgPrint(fmt, __VA_ARGS__);                             \
	    }                                                           \
}
#else
#define Cs42l42Print(dbglevel, fmt, ...) {                       \
}
#endif

#endif