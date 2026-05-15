/**
 * Joystick test suite
 */

#include "SDL.h"
#include "SDL_test.h"
#include "../src/joystick/usb_ids.h"
#include "../src/joystick/hidapi/steam/controller_structs.h"
#include <stddef.h>

/* ================= Test Case Implementation ================== */

/* Test case functions */

/**
 * @brief Check virtual joystick creation
 *
 * @sa SDL_JoystickAttachVirtualEx
 */
static int
TestVirtualJoystick(void *arg)
{
    SDL_VirtualJoystickDesc desc;
    SDL_Joystick *joystick = NULL;
    int device_index;

    SDLTest_AssertCheck(SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER) == 0, "SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER)");

    SDL_zero(desc);
    desc.version = SDL_VIRTUAL_JOYSTICK_DESC_VERSION;
    desc.type = SDL_JOYSTICK_TYPE_GAMECONTROLLER;
    desc.naxes = SDL_CONTROLLER_AXIS_MAX;
    desc.nbuttons = SDL_CONTROLLER_BUTTON_MAX;
    desc.vendor_id = USB_VENDOR_NVIDIA;
    desc.product_id = USB_PRODUCT_NVIDIA_SHIELD_CONTROLLER_V104;
    desc.name = "Virtual NVIDIA SHIELD Controller";
    device_index = SDL_JoystickAttachVirtualEx(&desc);
    SDLTest_AssertCheck(device_index >= 0, "SDL_JoystickAttachVirtualEx()");
    SDLTest_AssertCheck(SDL_JoystickIsVirtual(device_index), "SDL_JoystickIsVirtual()");
    if (device_index >= 0) {
        joystick = SDL_JoystickOpen(device_index);
        SDLTest_AssertCheck(joystick != NULL, "SDL_JoystickOpen()");
        if (joystick) {
            SDLTest_AssertCheck(SDL_strcmp(SDL_JoystickName(joystick), desc.name) == 0, "SDL_JoystickName()");
            SDLTest_AssertCheck(SDL_JoystickGetVendor(joystick) == desc.vendor_id, "SDL_JoystickGetVendor()");
            SDLTest_AssertCheck(SDL_JoystickGetProduct(joystick) == desc.product_id, "SDL_JoystickGetProduct()");
            SDLTest_AssertCheck(SDL_JoystickGetProductVersion(joystick) == 0, "SDL_JoystickGetProductVersion()");
            SDLTest_AssertCheck(SDL_JoystickGetFirmwareVersion(joystick) == 0, "SDL_JoystickGetFirmwareVersion()");
            SDLTest_AssertCheck(SDL_JoystickGetSerial(joystick) == NULL, "SDL_JoystickGetSerial()");
            SDLTest_AssertCheck(SDL_JoystickGetType(joystick) == desc.type, "SDL_JoystickGetType()");
            SDLTest_AssertCheck(SDL_JoystickNumAxes(joystick) == desc.naxes, "SDL_JoystickNumAxes()");
            SDLTest_AssertCheck(SDL_JoystickNumBalls(joystick) == 0, "SDL_JoystickNumBalls()");
            SDLTest_AssertCheck(SDL_JoystickNumHats(joystick) == desc.nhats, "SDL_JoystickNumHats()");
            SDLTest_AssertCheck(SDL_JoystickNumButtons(joystick) == desc.nbuttons, "SDL_JoystickNumButtons()");

            SDLTest_AssertCheck(SDL_JoystickSetVirtualButton(joystick, SDL_CONTROLLER_BUTTON_A, SDL_PRESSED) == 0, "SDL_JoystickSetVirtualButton(SDL_CONTROLLER_BUTTON_A, SDL_PRESSED)");
            SDL_JoystickUpdate();
            SDLTest_AssertCheck(SDL_JoystickGetButton(joystick, SDL_CONTROLLER_BUTTON_A) == SDL_PRESSED, "SDL_JoystickGetButton(SDL_CONTROLLER_BUTTON_A) == SDL_PRESSED");
            SDLTest_AssertCheck(SDL_JoystickSetVirtualButton(joystick, SDL_CONTROLLER_BUTTON_A, SDL_RELEASED) == 0, "SDL_JoystickSetVirtualButton(SDL_CONTROLLER_BUTTON_A, SDL_RELEASED)");
            SDL_JoystickUpdate();
            SDLTest_AssertCheck(SDL_JoystickGetButton(joystick, SDL_CONTROLLER_BUTTON_A) == SDL_RELEASED, "SDL_JoystickGetButton(SDL_CONTROLLER_BUTTON_A) == SDL_RELEASED");

            SDL_JoystickClose(joystick);
        }
        SDLTest_AssertCheck(SDL_JoystickDetachVirtual(device_index) == 0, "SDL_JoystickDetachVirtual()");
    }
    SDLTest_AssertCheck(!SDL_JoystickIsVirtual(device_index), "!SDL_JoystickIsVirtual()");

    SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);

    return TEST_COMPLETED;
}

/**
 * @brief Check Steam Triton controller report layouts imported from SDL main
 *
 * @sa SDL_hidapi_steam_triton.c
 */
static int
TestSteamTritonReportLayout(void *arg)
{
    (void)arg;

    SDLTest_AssertCheck(sizeof(TritonMTUIMUNoQuat_t) == 16,
                        "TritonMTUIMUNoQuat_t has expected 16-byte BLE IMU layout");
    SDLTest_AssertCheck(sizeof(TritonMTUNoQuat_t) == 45,
                        "TritonMTUNoQuat_t has expected 45-byte controller payload layout");
    SDLTest_AssertCheck(sizeof(TritonMTUFull_t) == 53,
                        "TritonMTUFull_t keeps expected 53-byte USB/dongle payload layout");
    SDLTest_AssertCheck(offsetof(TritonMTUNoQuat_t, imu) == 29,
                        "TritonMTUNoQuat_t IMU offset matches SDL main report parser");
    SDLTest_AssertCheck(offsetof(TritonMTUNoQuat_t, imu.sGyroZ) == 43,
                        "TritonMTUNoQuat_t gyro Z offset matches SDL main report parser");
    SDLTest_AssertCheck(ID_TRITON_CONTROLLER_STATE == 0x42,
                        "Steam Triton USB/dongle state report id is 0x42");
    SDLTest_AssertCheck(ID_TRITON_CONTROLLER_STATE_BLE == 0x45,
                        "Steam Triton BLE state report id is 0x45");
    SDLTest_AssertCheck(ID_TRITON_WIRELESS_STATUS_X == 0x46,
                        "Steam Triton wireless status report id is 0x46");

    return TEST_COMPLETED;
}

/* ================= Test References ================== */

/* Joystick routine test cases */
static const SDLTest_TestCaseReference joystickTest1 = {
    (SDLTest_TestCaseFp)TestVirtualJoystick, "TestVirtualJoystick", "Test virtual joystick functionality", TEST_ENABLED
};

static const SDLTest_TestCaseReference joystickTest2 = {
    (SDLTest_TestCaseFp)TestSteamTritonReportLayout, "TestSteamTritonReportLayout", "Test Steam Triton HID report layout", TEST_ENABLED
};

/* Sequence of Joystick routine test cases */
static const SDLTest_TestCaseReference *joystickTests[] = {
    &joystickTest1,
    &joystickTest2,
    NULL
};

/* Joystick routine test suite (global) */
SDLTest_TestSuiteReference joystickTestSuite = {
    "Joystick",
    NULL,
    joystickTests,
    NULL
};
