#ifndef WND_H
#define WND_H

#if !defined(USE_GLFW)

#include "SDL.h"
#include "SDL_opengl.h"

#define WND_QUIT                    SDL_QUIT

#define WND_KEY_1                   SDLK_1
#define WND_KEY_9                   SDLK_9
#define WND_KEY_0                   SDLK_0
#define WND_KEY_W                   SDLK_w
#define WND_KEY_S                   SDLK_s
#define WND_KEY_A                   SDLK_a
#define WND_KEY_D                   SDLK_d
#define WND_KEY_T                   SDLK_t
#define WND_KEY_TAB                 SDLK_TAB
#define WND_KEY_RIGHT               SDLK_RIGHT
#define WND_KEY_LEFT                SDLK_LEFT
#define WND_KEY_SPACE               SDLK_SPACE
#define WND_KEY_ESCAPE              SDLK_ESCAPE

#define WND_KMOD_SHIFT              KMOD_SHIFT
#define WND_KMOD_CTRL               KMOD_CTRL

#define WND_MOUSE_BUTTON_LEFT       SDL_BUTTON_LEFT
#define WND_MOUSE_BUTTON_RIGHT      SDL_BUTTON_RIGHT
#define WND_MOUSE_WHEEL_UP          SDL_BUTTON_WHEELUP
#define WND_MOUSE_WHEEL_DOWN        SDL_BUTTON_WHEELDOWN

#define WND_KEY_PRESSED             SDL_KEYDOWN
#define WND_KEY_RELEASED            SDL_KEYUP
#define WND_MOUSE_BUTTON_PRESSED    SDL_MOUSEBUTTONDOWN
#define WND_MOUSE_BUTTON_RELEASED   SDL_MOUSEBUTTONUP
#define WND_MOUSE_MOVED             SDL_MOUSEMOTION

#else // USE_GLFW

#include <GL/gl.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>

#define WND_KEY_1                   GLFW_KEY_1
#define WND_KEY_9                   GLFW_KEY_9
#define WND_KEY_0                   GLFW_KEY_0
#define WND_KEY_W                   GLFW_KEY_W
#define WND_KEY_S                   GLFW_KEY_S
#define WND_KEY_A                   GLFW_KEY_A
#define WND_KEY_D                   GLFW_KEY_D
#define WND_KEY_T                   GLFW_KEY_T
#define WND_KEY_TAB                 GLFW_KEY_TAB
#define WND_KEY_RIGHT               GLFW_KEY_RIGHT
#define WND_KEY_LEFT                GLFW_KEY_LEFT
#define WND_KEY_SPACE               GLFW_KEY_SPACE
#define WND_KEY_ESCAPE              GLFW_KEY_ESCAPE

#define WND_KMOD_SHIFT              GLFW_MOD_SHIFT
#define WND_KMOD_CTRL               GLFW_MOD_CONTROL

#define WND_MOUSE_BUTTON_LEFT       GLFW_MOUSE_BUTTON_LEFT
#define WND_MOUSE_BUTTON_RIGHT      GLFW_MOUSE_BUTTON_RIGHT
#define WND_MOUSE_WHEEL_UP          (GLFW_MOUSE_BUTTON_LAST + 1)
#define WND_MOUSE_WHEEL_DOWN        (GLFW_MOUSE_BUTTON_LAST + 2)

#define WND_KEY_PRESSED             1
#define WND_KEY_RELEASED            2
#define WND_MOUSE_BUTTON_PRESSED    3
#define WND_MOUSE_BUTTON_RELEASED   4
#define WND_MOUSE_MOVED             5
#define WND_QUIT                    6

#endif

struct wnd_event_key
{
    int type;
    int code;
};

struct wnd_event_mouse
{
    int type;
    int button;
    int x;
    int y;
};

typedef union wnd_event
{
    int type;
    wnd_event_key   key;
    wnd_event_mouse mouse;
} wnd_event;

bool wnd_init(int* width, int* height, bool presentationMode);
void wnd_quit();
double wnd_ticks();
bool wnd_poll_event(wnd_event& event);
void wnd_sleep(int ms);
void wnd_swap();
bool wnd_mod_state(int kmod);
bool wnd_get_key(int key);
bool wnd_get_mouse(int button);

#endif // WND_H

