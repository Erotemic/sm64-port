#if !defined(_WIN32) && !defined(_WIN64)

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <SDL2/SDL.h>

#include <ultra64.h>

#include "controller_api.h"

#define DEADZONE 4960

static bool init_ok;
static SDL_GameController *sdl_cntrl;

static void controller_sdl_init(void) {
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
        fprintf(stderr, "SDL init error: %s\n", SDL_GetError());
        return;
    }

    init_ok = true;
}

static bool needs_controller_input_update = true;


OSContPad prev_pad;
void sdl_controller_printf(OSContPad *pad) {
    if (prev_pad.button != pad->button ||
        prev_pad.stick_x != pad->stick_x ||
        prev_pad.stick_y != pad->stick_y ||
        prev_pad.errnum != pad->errnum)
    {
        // Write controller information to stdout
        printf("SDL pad: stick_x=%d stick_y=%d button=%d errnum=%d\n",
                pad->stick_x, pad->stick_y, pad->button, pad->errnum);
    }
    prev_pad = *pad;
}

static void controller_sdl_read(OSContPad *pad) {
    if (!init_ok) {
        return;
    }

    SDL_GameControllerUpdate();

    if (sdl_cntrl != NULL && !SDL_GameControllerGetAttached(sdl_cntrl)) {
        printf("Close SDL Game Controller: %p\n", sdl_cntrl);
        SDL_GameControllerClose(sdl_cntrl);
        sdl_cntrl = NULL;
        needs_controller_input_update = true;
    }
    if (sdl_cntrl == NULL) {
        int num_joysticks = SDL_NumJoysticks();
        if (needs_controller_input_update){
            printf("Open SDL Game Controller\n");
            printf("num_joysticks = %d\n", num_joysticks);
        }
        for (int i = 0; i < SDL_NumJoysticks(); i++) {
            const char* ctrl_name = SDL_GameControllerNameForIndex(i);
            if (needs_controller_input_update){
                printf("Checking SDL Game Controller %d: %s\n", i, ctrl_name);
            }
            if (SDL_IsGameController(i)) {
                sdl_cntrl = SDL_GameControllerOpen(i);
                printf("Found SDL Game Controller = %p\n", sdl_cntrl);
                if (sdl_cntrl != NULL) {
                    break;
                }
            }
        }
        needs_controller_input_update = false;
        if (sdl_cntrl == NULL) {
            if (needs_controller_input_update){
                printf("No SDL Game Controller detected\n");
            }
            return;
        }
    }

    if (SDL_GameControllerGetButton(sdl_cntrl, SDL_CONTROLLER_BUTTON_START)) pad->button |= START_BUTTON;
    if (SDL_GameControllerGetButton(sdl_cntrl, SDL_CONTROLLER_BUTTON_LEFTSHOULDER)) pad->button |= Z_TRIG;
    if (SDL_GameControllerGetButton(sdl_cntrl, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER)) pad->button |= R_TRIG;
    if (SDL_GameControllerGetButton(sdl_cntrl, SDL_CONTROLLER_BUTTON_A)) pad->button |= A_BUTTON;
    if (SDL_GameControllerGetButton(sdl_cntrl, SDL_CONTROLLER_BUTTON_X)) pad->button |= B_BUTTON;

    int16_t leftx = SDL_GameControllerGetAxis(sdl_cntrl, SDL_CONTROLLER_AXIS_LEFTX);
    int16_t lefty = SDL_GameControllerGetAxis(sdl_cntrl, SDL_CONTROLLER_AXIS_LEFTY);
    int16_t rightx = SDL_GameControllerGetAxis(sdl_cntrl, SDL_CONTROLLER_AXIS_RIGHTX);
    int16_t righty = SDL_GameControllerGetAxis(sdl_cntrl, SDL_CONTROLLER_AXIS_RIGHTY);

    int16_t ltrig = SDL_GameControllerGetAxis(sdl_cntrl, SDL_CONTROLLER_AXIS_TRIGGERLEFT);
    int16_t rtrig = SDL_GameControllerGetAxis(sdl_cntrl, SDL_CONTROLLER_AXIS_TRIGGERRIGHT);

#ifdef TARGET_WEB
    // Firefox has a bug: https://bugzilla.mozilla.org/show_bug.cgi?id=1606562
    // It sets down y to 32768.0f / 32767.0f, which is greater than the allowed 1.0f,
    // which SDL then converts to a int16_t by multiplying by 32767.0f, which overflows into -32768.
    // Maximum up will hence never become -32768 with the current version of SDL2,
    // so this workaround should be safe in compliant browsers.
    if (lefty == -32768) {
        lefty = 32767;
    }
    if (righty == -32768) {
        righty = 32767;
    }
#endif

    if (rightx < -0x4000) pad->button |= L_CBUTTONS;
    if (rightx > 0x4000) pad->button |= R_CBUTTONS;
    if (righty < -0x4000) pad->button |= U_CBUTTONS;
    if (righty > 0x4000) pad->button |= D_CBUTTONS;

    if (ltrig > 30 * 256) pad->button |= Z_TRIG;
    if (rtrig > 30 * 256) pad->button |= R_TRIG;

    uint32_t magnitude_sq = (uint32_t)(leftx * leftx) + (uint32_t)(lefty * lefty);
    if (magnitude_sq > (uint32_t)(DEADZONE * DEADZONE)) {
        // Game expects stick coordinates within -80..80
        // 32768 / 409 = ~80
        pad->stick_x = leftx / 409;
        pad->stick_y = -lefty / 409;
    }
    sdl_controller_printf(pad);
}

struct ControllerAPI controller_sdl = {
    controller_sdl_init,
    controller_sdl_read
};

#endif
