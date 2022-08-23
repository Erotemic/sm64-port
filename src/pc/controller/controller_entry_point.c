#include "macros.h"
#include <stdio.h>

#include "lib/src/libultra_internal.h"
#include "lib/src/osContInternal.h"

#include "controller_recorded_tas.h"
#include "controller_keyboard.h"

#if defined(_WIN32) || defined(_WIN64)
#include "controller_xinput.h"
#else
#include "controller_sdl.h"
#endif

#ifdef __linux__
#include "controller_wup.h"
#endif

static struct ControllerAPI *controller_implementations[] = {
    &controller_recorded_tas,
#if defined(_WIN32) || defined(_WIN64)
    &controller_xinput,
#else
    &controller_sdl,
#endif
#ifdef __linux__
    &controller_wup,
#endif
    &controller_keyboard,
};

s32 osContInit(UNUSED OSMesgQueue *mq, u8 *controllerBits, UNUSED OSContStatus *status) {
    size_t num_controller_impls = sizeof(controller_implementations) / sizeof(struct ControllerAPI *);
    printf("Initialize controllers\n");
    printf("num_controller_impls = %d\n", num_controller_impls);

    for (size_t i = 0; i < num_controller_impls; i++) {
        controller_implementations[i]->init();
    }
    *controllerBits = 1;
    return 0;
}

s32 osContStartReadData(UNUSED OSMesgQueue *mesg) {
    return 0;
}

void osContGetReadData(OSContPad *pad) {
    pad->button = 0;
    pad->stick_x = 0;
    pad->stick_y = 0;
    pad->errnum = 0;

    size_t num_controller_impls = sizeof(controller_implementations) / sizeof(struct ControllerAPI *);

    for (size_t i = 0; i < num_controller_impls; i++) {
        controller_implementations[i]->read(pad);
    }
}
