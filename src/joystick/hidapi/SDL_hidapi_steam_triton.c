/*
  Simple DirectMedia Layer
  Copyright (C) 1997-2026 Sam Lantinga <slouken@libsdl.org>

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/
#include "../../SDL_internal.h"

#ifdef SDL_JOYSTICK_HIDAPI

#include "../SDL_sysjoystick.h"
#include "SDL_gamecontroller.h"
#include "SDL_timer.h"
#include "SDL_hidapijoystick_c.h"

#ifdef SDL_JOYSTICK_HIDAPI_STEAM_TRITON

#define bool SDL_bool
#define true SDL_TRUE
#define false SDL_FALSE

/*****************************************************************************************************/

#include "steam/controller_constants.h"
#include "steam/controller_structs.h"

// Always 1kHz according to USB descriptor, but actually about 4 ms.
#define TRITON_SENSOR_UPDATE_INTERVAL_US 4032
// Steam Controller hardware safety timeout is around 50ms, so resend rumble every 40ms.
#define TRITON_RUMBLE_RESEND_INTERVAL_MS 40
/*
 * SDL3 sends gamepad trigger axes as signed full-range values. SDL2's
 * GameController layer remaps joystick trigger axes to 0..32767, so this
 * backport must feed SDL2 a real rest value (SDL_JOYSTICK_AXIS_MIN), not the
 * SDL3 raw math result. The SC2 receiver reports a small non-zero floor on
 * webOS, observed around 10.5k, so keep that inside rest/deadzone.
 */
#define TRITON_TRIGGER_DEADZONE 12000

typedef enum
{

    TRITON_LBUTTON_A            = 0x00000001,
    TRITON_LBUTTON_B            = 0x00000002,
    TRITON_LBUTTON_X            = 0x00000004,
    TRITON_LBUTTON_Y            = 0x00000008,

    TRITON_HBUTTON_QAM          = 0x00000010,
    TRITON_LBUTTON_R3           = 0x00000020,
    TRITON_LBUTTON_VIEW         = 0x00000040,
    TRITON_HBUTTON_R4           = 0x00000080,

    TRITON_LBUTTON_R5           = 0x00000100,
    TRITON_LBUTTON_R            = 0x00000200,
    TRITON_LBUTTON_DPAD_DOWN    = 0x00000400,
    TRITON_LBUTTON_DPAD_RIGHT   = 0x00000800,

    TRITON_LBUTTON_DPAD_LEFT    = 0x00001000,
    TRITON_LBUTTON_DPAD_UP      = 0x00002000,
    TRITON_LBUTTON_MENU         = 0x00004000,
    TRITON_LBUTTON_L3           = 0x00008000,

    TRITON_LBUTTON_STEAM        = 0x00010000,
    TRITON_HBUTTON_L4           = 0x00020000,
    TRITON_LBUTTON_L5           = 0x00040000,
    TRITON_LBUTTON_L            = 0x00080000,

    /*
	STEAM_RIGHTSTICK_FINGERDOWN_MASK,   // Right Stick Touch    0x00100000
	STEAM_RIGHTPAD_FINGERDOWN_MASK,     // Right Pad Touch      0x00200000
	STEAM_BUTTON_RIGHTPAD_CLICKED_MASK, // Right Pressure Click 0x00400000
	STEAM_RIGHT_TRIGGER_MASK,           // Right Trigger Click  0x00800000

	STEAM_LEFTSTICK_FINGERDOWN_MASK,    // Left Stick Touch     0x01000000
	STEAM_LEFTPAD_FINGERDOWN_MASK,      // Left Pad Touch       0x02000000
	STEAM_BUTTON_LEFTPAD_CLICKED_MASK,  // Left Pressure Click  0x04000000
	STEAM_LEFT_TRIGGER_MASK,            // Left Trigger Click   0x08000000
    STEAM_RIGHT_AUX_MASK,               // Right Pinky Touch   0x10000000
	STEAM_LEFT_AUX_MASK,                // Left Pinky Touch    0x20000000
    */
} TritonButtons;

typedef struct SDL_DriverSteamTriton_Context
{
    bool connected;
    bool report_sensors;
    SDL_JoystickID joystick_id;
    Uint8 last_input_report[64];
    int last_input_report_len;
    struct SDL_DriverSteamTriton_Context *next_context;
    Uint32 last_sensor_tick;
    Uint64 sensor_timestamp_us;
    Uint64 last_button_state;
    Uint64 last_lizard_update;
    Uint32 last_raw_log_tick;
    Uint32 last_raw_buttons;
    Sint16 last_raw_left_trigger;
    Sint16 last_raw_right_trigger;
    Sint16 last_raw_left_stick_x;
    Sint16 last_raw_left_stick_y;
    Sint16 last_raw_right_stick_x;
    Sint16 last_raw_right_stick_y;
    Uint16 low_frequency_rumble;
    Uint16 high_frequency_rumble;
    Uint64 last_rumble_time;
} SDL_DriverSteamTriton_Context;

static SDL_DriverSteamTriton_Context *SDL_SteamTriton_contexts;

DECLSPEC int SDLCALL SDL_SteamTritonGetLastInputReport(SDL_Joystick *joystick, Uint8 *data, int size)
{
    SDL_DriverSteamTriton_Context *ctx;
    SDL_JoystickID joystick_id;
    int copy_len;

    if (!joystick || !data || size <= 0) {
        return -1;
    }

    joystick_id = SDL_JoystickInstanceID(joystick);
    for (ctx = SDL_SteamTriton_contexts; ctx; ctx = ctx->next_context) {
        if (ctx->joystick_id != joystick_id || ctx->last_input_report_len <= 0) {
            continue;
        }
        copy_len = ctx->last_input_report_len;
        if (copy_len > size) {
            copy_len = size;
        }
        SDL_memcpy(data, ctx->last_input_report, copy_len);
        return copy_len;
    }
    return -1;
}

static bool IsProteusDongle(Uint16 product_id)
{
    return (product_id == USB_PRODUCT_VALVE_STEAM_PROTEUS_DONGLE ||
            product_id == USB_PRODUCT_VALVE_STEAM_NEREID_DONGLE);
}

static bool DisableSteamTritonLizardMode(SDL_hid_device *dev)
{
    int rc;
    Uint8 buffer[HID_FEATURE_REPORT_BYTES] = { 1 };
    FeatureReportMsg *msg = (FeatureReportMsg *)(buffer + 1);

    msg->header.type = ID_SET_SETTINGS_VALUES;
    msg->header.length = 1 * sizeof(ControllerSetting);
    msg->payload.setSettingsValues.settings[0].settingNum = SETTING_LIZARD_MODE;
    msg->payload.setSettingsValues.settings[0].settingValue = LIZARD_MODE_OFF;

    rc = SDL_hid_send_feature_report(dev, buffer, sizeof(buffer));
    if (rc != sizeof(buffer)) {
        return false;
    }

    return true;
}

static int TritonAbsInt(int value)
{
    return value < 0 ? -value : value;
}

static Sint16 HIDAPI_DriverSteamTriton_SDL2TriggerAxis(Sint16 raw)
{
    int value;
    if (raw <= TRITON_TRIGGER_DEADZONE) {
        return SDL_JOYSTICK_AXIS_MIN;
    }
    if (raw >= SDL_JOYSTICK_AXIS_MAX) {
        return SDL_JOYSTICK_AXIS_MAX;
    }
    value = SDL_JOYSTICK_AXIS_MIN +
            ((int)(raw - TRITON_TRIGGER_DEADZONE) *
             (SDL_JOYSTICK_AXIS_MAX - SDL_JOYSTICK_AXIS_MIN)) /
                    (SDL_JOYSTICK_AXIS_MAX - TRITON_TRIGGER_DEADZONE);
    if (value < SDL_JOYSTICK_AXIS_MIN) {
        return SDL_JOYSTICK_AXIS_MIN;
    }
    if (value > SDL_JOYSTICK_AXIS_MAX) {
        return SDL_JOYSTICK_AXIS_MAX;
    }
    return (Sint16)value;
}

static void HIDAPI_DriverSteamTriton_FormatHex(const Uint8 *data, int len, char *hex, size_t hex_len)
{
    static const char digits[] = "0123456789abcdef";
    size_t off = 0;
    int i;

    if (hex_len == 0) {
        return;
    }
    hex[0] = '\0';
    for (i = 0; i < len && i < 64 && off + 3 < hex_len; ++i) {
        if (i > 0) {
            hex[off++] = ' ';
        }
        hex[off++] = digits[(data[i] >> 4) & 0x0f];
        hex[off++] = digits[data[i] & 0x0f];
    }
    hex[off] = '\0';
}

static void HIDAPI_DriverSteamTriton_LogRawState(SDL_HIDAPI_Device *device,
                                                 const Uint8 *data,
                                                 int len,
                                                 const TritonMTUNoQuat_t *report)
{
    SDL_DriverSteamTriton_Context *ctx = (SDL_DriverSteamTriton_Context *)device->context;
    Uint32 now = SDL_GetTicks();
    char hex[3 * 64];
    bool buttons_changed = (report->buttons != ctx->last_raw_buttons);
    bool axes_changed =
        TritonAbsInt((int)report->sTriggerLeft - (int)ctx->last_raw_left_trigger) > 2048 ||
        TritonAbsInt((int)report->sTriggerRight - (int)ctx->last_raw_right_trigger) > 2048 ||
        TritonAbsInt((int)report->sLeftStickX - (int)ctx->last_raw_left_stick_x) > 4096 ||
        TritonAbsInt((int)report->sLeftStickY - (int)ctx->last_raw_left_stick_y) > 4096 ||
        TritonAbsInt((int)report->sRightStickX - (int)ctx->last_raw_right_stick_x) > 4096 ||
        TritonAbsInt((int)report->sRightStickY - (int)ctx->last_raw_right_stick_y) > 4096;

    if (!buttons_changed && (!axes_changed || (now - ctx->last_raw_log_tick) < 120)) {
        return;
    }

    HIDAPI_DriverSteamTriton_FormatHex(data, len, hex, sizeof(hex));
    SDL_LogInfo(SDL_LOG_CATEGORY_INPUT,
                "SC2.RAW path=%s product=0x%04x report=0x%02x len=%d seq=%u buttons=0x%08x lt=%d rt=%d lx=%d ly=%d rx=%d ry=%d lpx=%d lpy=%d lp=%u rpx=%d rpy=%d rp=%u hex=%s",
                device->path ? device->path : "-", device->product_id, data[0], len, report->seq_num,
                report->buttons, report->sTriggerLeft, report->sTriggerRight,
                report->sLeftStickX, report->sLeftStickY, report->sRightStickX, report->sRightStickY,
                report->sLeftPadX, report->sLeftPadY, report->ucPressureLeft,
                report->sRightPadX, report->sRightPadY, report->ucPressureRight, hex);

    ctx->last_raw_log_tick = now;
    ctx->last_raw_buttons = report->buttons;
    ctx->last_raw_left_trigger = report->sTriggerLeft;
    ctx->last_raw_right_trigger = report->sTriggerRight;
    ctx->last_raw_left_stick_x = report->sLeftStickX;
    ctx->last_raw_left_stick_y = report->sLeftStickY;
    ctx->last_raw_right_stick_x = report->sRightStickX;
    ctx->last_raw_right_stick_y = report->sRightStickY;
}

static void HIDAPI_DriverSteamTriton_HandleState(SDL_HIDAPI_Device *device,
                                               SDL_Joystick *joystick,
                                               TritonMTUNoQuat_t *pTritonReport)
{
    float values[3];
    SDL_DriverSteamTriton_Context *ctx = (SDL_DriverSteamTriton_Context *)device->context;

    if (pTritonReport->buttons != ctx->last_button_state) {
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_A,
                               ((pTritonReport->buttons & TRITON_LBUTTON_A) != 0));
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_B,
                               ((pTritonReport->buttons & TRITON_LBUTTON_B) != 0));
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_X,
                               ((pTritonReport->buttons & TRITON_LBUTTON_X) != 0));
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_Y,
                               ((pTritonReport->buttons & TRITON_LBUTTON_Y) != 0));

        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_LEFTSHOULDER,
                               ((pTritonReport->buttons & TRITON_LBUTTON_L) != 0));
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER,
                               ((pTritonReport->buttons & TRITON_LBUTTON_R) != 0));

        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_BACK,
                               ((pTritonReport->buttons & TRITON_LBUTTON_MENU) != 0));
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_START,
                               ((pTritonReport->buttons & TRITON_LBUTTON_VIEW) != 0));
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_GUIDE,
                               ((pTritonReport->buttons & TRITON_LBUTTON_STEAM) != 0));
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_MISC1,
                               ((pTritonReport->buttons & TRITON_HBUTTON_QAM) != 0));

        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_LEFTSTICK,
                               ((pTritonReport->buttons & TRITON_LBUTTON_L3) != 0));
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_RIGHTSTICK,
                               ((pTritonReport->buttons & TRITON_LBUTTON_R3) != 0));

        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_PADDLE1,
                               ((pTritonReport->buttons & TRITON_HBUTTON_R4) != 0));
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_PADDLE2,
                               ((pTritonReport->buttons & TRITON_HBUTTON_L4) != 0));
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_PADDLE3,
                               ((pTritonReport->buttons & TRITON_LBUTTON_R5) != 0));
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_PADDLE4,
                               ((pTritonReport->buttons & TRITON_LBUTTON_L5) != 0));

        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_DPAD_UP,
                               ((pTritonReport->buttons & TRITON_LBUTTON_DPAD_UP) != 0));
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_DPAD_DOWN,
                               ((pTritonReport->buttons & TRITON_LBUTTON_DPAD_DOWN) != 0));
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_DPAD_LEFT,
                               ((pTritonReport->buttons & TRITON_LBUTTON_DPAD_LEFT) != 0));
        SDL_PrivateJoystickButton(joystick, SDL_CONTROLLER_BUTTON_DPAD_RIGHT,
                               ((pTritonReport->buttons & TRITON_LBUTTON_DPAD_RIGHT) != 0));

        ctx->last_button_state = pTritonReport->buttons;
    }

    // RKRK There're button bits for this if you so choose.
    SDL_PrivateJoystickAxis(joystick, SDL_CONTROLLER_AXIS_TRIGGERLEFT,
                         HIDAPI_DriverSteamTriton_SDL2TriggerAxis(pTritonReport->sTriggerLeft));
    SDL_PrivateJoystickAxis(joystick, SDL_CONTROLLER_AXIS_TRIGGERRIGHT,
                         HIDAPI_DriverSteamTriton_SDL2TriggerAxis(pTritonReport->sTriggerRight));

    SDL_PrivateJoystickAxis(joystick, SDL_CONTROLLER_AXIS_LEFTX,
                         pTritonReport->sLeftStickX);
    SDL_PrivateJoystickAxis(joystick, SDL_CONTROLLER_AXIS_LEFTY,
                         -pTritonReport->sLeftStickY);
    SDL_PrivateJoystickAxis(joystick, SDL_CONTROLLER_AXIS_RIGHTX,
                         pTritonReport->sRightStickX);
    SDL_PrivateJoystickAxis(joystick, SDL_CONTROLLER_AXIS_RIGHTY,
                         -pTritonReport->sRightStickY);

    if (ctx->report_sensors && pTritonReport->imu.timestamp != ctx->last_sensor_tick) {
        Uint32 delta_us = (pTritonReport->imu.timestamp - ctx->last_sensor_tick);

        ctx->sensor_timestamp_us += delta_us;

        values[0] = (pTritonReport->imu.sGyroX / 32768.0f) * (2000.0f * (((float)M_PI) / 180.0f));
        values[1] = (pTritonReport->imu.sGyroZ / 32768.0f) * (2000.0f * (((float)M_PI) / 180.0f));
        values[2] = (-pTritonReport->imu.sGyroY / 32768.0f) * (2000.0f * (((float)M_PI) / 180.0f));
        SDL_PrivateJoystickSensor(joystick, SDL_SENSOR_GYRO, ctx->sensor_timestamp_us, values, 3);

        values[0] = (pTritonReport->imu.sAccelX / 32768.0f) * 2.0f * SDL_STANDARD_GRAVITY;
        values[1] = (pTritonReport->imu.sAccelZ / 32768.0f) * 2.0f * SDL_STANDARD_GRAVITY;
        values[2] = (-pTritonReport->imu.sAccelY / 32768.0f) * 2.0f * SDL_STANDARD_GRAVITY;
        SDL_PrivateJoystickSensor(joystick, SDL_SENSOR_ACCEL, ctx->sensor_timestamp_us, values, 3);

        ctx->last_sensor_tick = pTritonReport->imu.timestamp;
    }
}

static void HIDAPI_DriverSteamTriton_HandleBatteryStatus(SDL_HIDAPI_Device *device,
                                                         SDL_Joystick *joystick,
                                                         TritonBatteryStatus_t *pTritonBatteryStatus)
{
    switch (pTritonBatteryStatus->ucChargeState) {
    case k_EChargeStateDischarging:
        break;
    case k_EChargeStateCharging:
        SDL_PrivateJoystickBatteryLevel(joystick, SDL_JOYSTICK_POWER_WIRED);
        return;
    case k_EChargeStateChargingDone:
        SDL_PrivateJoystickBatteryLevel(joystick, SDL_JOYSTICK_POWER_FULL);
        return;
    default:
        // Error state?
        SDL_PrivateJoystickBatteryLevel(joystick, SDL_JOYSTICK_POWER_UNKNOWN);
        return;
    }

    if (pTritonBatteryStatus->ucBatteryLevel <= 5) {
        SDL_PrivateJoystickBatteryLevel(joystick, SDL_JOYSTICK_POWER_EMPTY);
    } else if (pTritonBatteryStatus->ucBatteryLevel <= 20) {
        SDL_PrivateJoystickBatteryLevel(joystick, SDL_JOYSTICK_POWER_LOW);
    } else if (pTritonBatteryStatus->ucBatteryLevel <= 70) {
        SDL_PrivateJoystickBatteryLevel(joystick, SDL_JOYSTICK_POWER_MEDIUM);
    } else {
        SDL_PrivateJoystickBatteryLevel(joystick, SDL_JOYSTICK_POWER_FULL);
    }
}

static bool HIDAPI_DriverSteamTriton_SetControllerConnected(SDL_HIDAPI_Device *device, bool connected)
{
    SDL_DriverSteamTriton_Context *ctx = (SDL_DriverSteamTriton_Context *)device->context;

    if (ctx->connected != connected) {
        ctx->connected = connected;

        if (connected) {
            SDL_JoystickID joystickID;
            if (!HIDAPI_JoystickConnected(device, &joystickID)) {
                return false;
            }
        } else {
            if (device->num_joysticks > 0) {
                HIDAPI_JoystickDisconnected(device, device->joysticks[0]);
            }
        }
    }
    return true;
}

static void HIDAPI_DriverSteamTriton_HandleWirelessStatus(SDL_HIDAPI_Device *device,
                                                          TritonWirelessStatus_t *pTritonWirelessStatus)
{
    switch (pTritonWirelessStatus->state) {
    case k_ETritonWirelessStateConnect:
        HIDAPI_DriverSteamTriton_SetControllerConnected(device, true);
        break;
    case k_ETritonWirelessStateDisconnect:
        HIDAPI_DriverSteamTriton_SetControllerConnected(device, false);
        break;
    default:
        break;
    }
}

/*****************************************************************************************************/

static void HIDAPI_DriverSteamTriton_RegisterHints(SDL_HintCallback callback, void *userdata)
{
    SDL_AddHintCallback(SDL_HINT_JOYSTICK_HIDAPI_STEAM, callback, userdata);
}

static void HIDAPI_DriverSteamTriton_UnregisterHints(SDL_HintCallback callback, void *userdata)
{
    SDL_DelHintCallback(SDL_HINT_JOYSTICK_HIDAPI_STEAM, callback, userdata);
}

static bool HIDAPI_DriverSteamTriton_IsEnabled(void)
{
    return SDL_GetHintBoolean(SDL_HINT_JOYSTICK_HIDAPI_STEAM,
                              SDL_GetHintBoolean(SDL_HINT_JOYSTICK_HIDAPI, SDL_HIDAPI_DEFAULT));
}

static bool HIDAPI_DriverSteamTriton_IsSupportedDevice(
    SDL_HIDAPI_Device *device,
    const char *name,
    SDL_GameControllerType type,
    Uint16 vendor_id,
    Uint16 product_id,
    Uint16 version,
    int interface_number,
    int interface_class,
    int interface_subclass,
    int interface_protocol)
{

    if (IsProteusDongle(product_id)) {
        if (interface_number >= 2 && interface_number <= 5) {
            // The set of controller interfaces for Proteus & Nereid...currently
            return true;
        }
    } else if (SDL_IsJoystickSteamTriton(vendor_id, product_id)) {
		return true;
	}
    return false;
}

static bool HIDAPI_DriverSteamTriton_InitDevice(SDL_HIDAPI_Device *device)
{
    SDL_DriverSteamTriton_Context *ctx;

    ctx = (SDL_DriverSteamTriton_Context *)SDL_calloc(1, sizeof(*ctx));
    if (ctx == NULL) {
        return false;
    }

    device->context = ctx;
    ctx->joystick_id = -1;
    ctx->next_context = SDL_SteamTriton_contexts;
    SDL_SteamTriton_contexts = ctx;

    HIDAPI_SetDeviceName(device, "Steam Controller");

    if (IsProteusDongle(device->product_id)) {
        return true;
    }

    // Wired controller, connected!
    return HIDAPI_DriverSteamTriton_SetControllerConnected(device, true);
}

static int HIDAPI_DriverSteamTriton_GetDevicePlayerIndex(SDL_HIDAPI_Device *device, SDL_JoystickID instance_id)
{
    return -1;
}

static void HIDAPI_DriverSteamTriton_SetDevicePlayerIndex(SDL_HIDAPI_Device *device, SDL_JoystickID instance_id, int player_index)
{
}

static int HIDAPI_DriverSteamTriton_RumbleJoystick(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, Uint16 low_frequency_rumble, Uint16 high_frequency_rumble);

static bool HIDAPI_DriverSteamTriton_UpdateDevice(SDL_HIDAPI_Device *device)
{
    SDL_DriverSteamTriton_Context *ctx = (SDL_DriverSteamTriton_Context *)device->context;
    SDL_Joystick *joystick = NULL;

    if (device->num_joysticks > 0) {
        joystick = SDL_JoystickFromInstanceID(device->joysticks[0]);
    }

    if (ctx->connected && joystick) {
        Uint64 now = SDL_GetTicks64();
        if (!ctx->last_lizard_update || (now - ctx->last_lizard_update) >= 3000) {
            DisableSteamTritonLizardMode(device->dev);
            ctx->last_lizard_update = now;
        }
        if (ctx->low_frequency_rumble || ctx->high_frequency_rumble) {
            if ((now - ctx->last_rumble_time) >= TRITON_RUMBLE_RESEND_INTERVAL_MS) {
                HIDAPI_DriverSteamTriton_RumbleJoystick(device, joystick,
                                                        ctx->low_frequency_rumble,
                                                        ctx->high_frequency_rumble);
            }
        }
    }

    for (;;) {
        uint8_t data[64];
        int r = SDL_hid_read(device->dev, data, sizeof(data));

        if (r == 0) {
            return true;
        }
        if (r < 0) {
            // Failed to read from controller
            HIDAPI_DriverSteamTriton_SetControllerConnected(device, false);
            return false;
        }

        switch (data[0]) {
        case ID_TRITON_CONTROLLER_STATE:
        case ID_TRITON_CONTROLLER_STATE_BLE:
            if (!joystick) {
                HIDAPI_DriverSteamTriton_SetControllerConnected(device, true);
                if (device->num_joysticks > 0) {
                    joystick = SDL_JoystickFromInstanceID(device->joysticks[0]);
                }
            }
            if (joystick && r >= (1 + sizeof(TritonMTUNoQuat_t))) {
                TritonMTUNoQuat_t *pTritonReport = (TritonMTUNoQuat_t *)&data[1];
                int report_len = r;
                if (report_len > (int)sizeof(ctx->last_input_report)) {
                    report_len = (int)sizeof(ctx->last_input_report);
                }
                ctx->joystick_id = SDL_JoystickInstanceID(joystick);
                SDL_memcpy(ctx->last_input_report, data, report_len);
                ctx->last_input_report_len = report_len;
                HIDAPI_DriverSteamTriton_LogRawState(device, data, r, pTritonReport);
                HIDAPI_DriverSteamTriton_HandleState(device, joystick, pTritonReport);
            }
            break;
        case ID_TRITON_BATTERY_STATUS:
            if (joystick && r >= (1 + sizeof(TritonBatteryStatus_t))) {
                TritonBatteryStatus_t *pTritonBatteryStatus = (TritonBatteryStatus_t *)&data[1];
                HIDAPI_DriverSteamTriton_HandleBatteryStatus(device, joystick, pTritonBatteryStatus);
            }
            break;
        case ID_TRITON_WIRELESS_STATUS_X:
        case ID_TRITON_WIRELESS_STATUS:
            if (r >= (1 + sizeof(TritonWirelessStatus_t))) {
                TritonWirelessStatus_t *pTritonWirelessStatus = (TritonWirelessStatus_t *)&data[1];
                HIDAPI_DriverSteamTriton_HandleWirelessStatus(device, pTritonWirelessStatus);
            }
            break;
        default:
            break;
        }
    }
}

static bool HIDAPI_DriverSteamTriton_OpenJoystick(SDL_HIDAPI_Device *device, SDL_Joystick *joystick)
{
    SDL_DriverSteamTriton_Context *ctx = (SDL_DriverSteamTriton_Context *)device->context;
    float update_rate_in_hz = 1000000.0f / TRITON_SENSOR_UPDATE_INTERVAL_US;

    SDL_AssertJoysticksLocked();

    ctx->joystick_id = SDL_JoystickInstanceID(joystick);

    // Initialize the joystick capabilities
    joystick->nbuttons = SDL_CONTROLLER_BUTTON_MAX;
    joystick->naxes = SDL_CONTROLLER_AXIS_MAX;

    SDL_PrivateJoystickAddSensor(joystick, SDL_SENSOR_GYRO, update_rate_in_hz);
    SDL_PrivateJoystickAddSensor(joystick, SDL_SENSOR_ACCEL, update_rate_in_hz);

    return true;
}

static int HIDAPI_DriverSteamTriton_RumbleJoystick(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, Uint16 low_frequency_rumble, Uint16 high_frequency_rumble)
{
    SDL_DriverSteamTriton_Context *ctx = (SDL_DriverSteamTriton_Context *)device->context;
    Uint8 buffer[HID_RUMBLE_OUTPUT_REPORT_BYTES] = { 0 };
    OutputReportMsg *msg = (OutputReportMsg *)(buffer);
    int rc;

    ctx->low_frequency_rumble = low_frequency_rumble;
    ctx->high_frequency_rumble = high_frequency_rumble;
    ctx->last_rumble_time = SDL_GetTicks64();

	msg->report_id = ID_OUT_REPORT_HAPTIC_RUMBLE;
    msg->payload.hapticRumble.type = 0;
    msg->payload.hapticRumble.intensity = 0;
    msg->payload.hapticRumble.left.speed = low_frequency_rumble;
    msg->payload.hapticRumble.left.gain = 0;
    msg->payload.hapticRumble.right.speed = high_frequency_rumble;
    msg->payload.hapticRumble.right.gain = 0;


    rc = SDL_hid_write(device->dev, buffer, sizeof(buffer));
    if (rc < 0) {
        SDL_LogError(SDL_LOG_CATEGORY_INPUT,
                     "Steam Controller HID Write FAILED! rc: %d. SDL_Error: %s",
                     rc, SDL_GetError());
        return -1;
    }
    return 0;
}

static int HIDAPI_DriverSteamTriton_RumbleJoystickTriggers(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, Uint16 left_rumble, Uint16 right_rumble)
{
    return SDL_Unsupported();
}

static Uint32 HIDAPI_DriverSteamTriton_GetJoystickCapabilities(SDL_HIDAPI_Device *device, SDL_Joystick *joystick)
{
    return SDL_JOYCAP_RUMBLE;
}

static int HIDAPI_DriverSteamTriton_SetJoystickLED(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, Uint8 red, Uint8 green, Uint8 blue)
{
    return SDL_Unsupported();
}

static int HIDAPI_DriverSteamTriton_SendJoystickEffect(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, const void *data, int size)
{
    if (size == HID_FEATURE_REPORT_BYTES) {
        int rc = SDL_hid_send_feature_report(device->dev, data, size);
        if (rc != size) {
            return -1;
        }
        return 0;
    }
    return SDL_Unsupported();
}

static int HIDAPI_DriverSteamTriton_SetSensorsEnabled(SDL_HIDAPI_Device *device, SDL_Joystick *joystick, SDL_bool enabled)
{
    SDL_DriverSteamTriton_Context *ctx = (SDL_DriverSteamTriton_Context *)device->context;
    int rc;
    Uint8 buffer[HID_FEATURE_REPORT_BYTES] = { 1 };
    FeatureReportMsg *msg = (FeatureReportMsg *)(buffer + 1);

    msg->header.type = ID_SET_SETTINGS_VALUES;
    msg->header.length = 1 * sizeof(ControllerSetting);
    msg->payload.setSettingsValues.settings[0].settingNum = SETTING_IMU_MODE;
    if (enabled) {
        msg->payload.setSettingsValues.settings[0].settingValue = (SETTING_GYRO_MODE_SEND_RAW_ACCEL | SETTING_GYRO_MODE_SEND_RAW_GYRO);
    } else {
        msg->payload.setSettingsValues.settings[0].settingValue = SETTING_GYRO_MODE_OFF;
    }

    rc = SDL_hid_send_feature_report(device->dev, buffer, sizeof(buffer));
    if (rc != sizeof(buffer)) {
        return -1;
    }

    ctx->report_sensors = enabled;

    return 0;
}

static void HIDAPI_DriverSteamTriton_CloseJoystick(SDL_HIDAPI_Device *device, SDL_Joystick *joystick)
{
    // Lizard mode id automatically re-enabled by watchdog. Nothing to do here.
}

static void HIDAPI_DriverSteamTriton_FreeDevice(SDL_HIDAPI_Device *device)
{
    SDL_DriverSteamTriton_Context *ctx = (SDL_DriverSteamTriton_Context *)device->context;
    SDL_DriverSteamTriton_Context **iter = &SDL_SteamTriton_contexts;

    while (*iter) {
        if (*iter == ctx) {
            *iter = ctx->next_context;
            break;
        }
        iter = &(*iter)->next_context;
    }
}

SDL_HIDAPI_DeviceDriver SDL_HIDAPI_DriverSteamTriton = {
    SDL_HINT_JOYSTICK_HIDAPI_STEAM,
    SDL_TRUE,
    HIDAPI_DriverSteamTriton_RegisterHints,
    HIDAPI_DriverSteamTriton_UnregisterHints,
    HIDAPI_DriverSteamTriton_IsEnabled,
    HIDAPI_DriverSteamTriton_IsSupportedDevice,
    HIDAPI_DriverSteamTriton_InitDevice,
    HIDAPI_DriverSteamTriton_GetDevicePlayerIndex,
    HIDAPI_DriverSteamTriton_SetDevicePlayerIndex,
    HIDAPI_DriverSteamTriton_UpdateDevice,
    HIDAPI_DriverSteamTriton_OpenJoystick,
    HIDAPI_DriverSteamTriton_RumbleJoystick,
    HIDAPI_DriverSteamTriton_RumbleJoystickTriggers,
    HIDAPI_DriverSteamTriton_GetJoystickCapabilities,
    HIDAPI_DriverSteamTriton_SetJoystickLED,
    HIDAPI_DriverSteamTriton_SendJoystickEffect,
    HIDAPI_DriverSteamTriton_SetSensorsEnabled,
    HIDAPI_DriverSteamTriton_CloseJoystick,
    HIDAPI_DriverSteamTriton_FreeDevice,
};


#undef bool
#undef true
#undef false

#endif /* SDL_JOYSTICK_HIDAPI_STEAM_TRITON */

#endif /* SDL_JOYSTICK_HIDAPI */
