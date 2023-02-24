#if !defined(_CS42L42_COMMON_H_)
#define _CS42L42_COMMON_H_

//
//These are the device attributes returned by vmulti in response
// to IOCTL_HID_GET_DEVICE_ATTRIBUTES.
//

#define CS42L42_PID              0x4242
#define CS42L42_VID              0x1013
#define CS42L42_VERSION          0x0001

//
// These are the report ids
//

#define REPORTID_MEDIA	0x01
#define REPORTID_SPECKEYS		0x02

#pragma pack(1)
typedef struct _CS42L42_MEDIA_REPORT
{

	BYTE      ReportID;

	BYTE	  ControlCode;

} Cs42l42MediaReport;
#pragma pack()

#define CONTROL_CODE_JACK_TYPE 0x1

#pragma pack(1)
typedef struct _CSAUDIO_SPECKEY_REPORT
{

	BYTE      ReportID;

	BYTE	  ControlCode;

	BYTE	  ControlValue;

} CsAudioSpecialKeyReport;

#pragma pack()

#pragma pack(1)
typedef struct _CSAUDIO_SPECKEYREQ_REPORT
{

	BYTE      ReportID;

	BYTE	  AnyCode;

} CsAudioSpecialKeyRequestReport;
#pragma pack()

#endif
