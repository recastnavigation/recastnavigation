#include "wnd.h"

#if defined(USE_GLFW)

#include <queue>
#include <algorithm>
#include <stdio.h>

static bool _should_quit = false;
static GLFWwindow* _window = 0;
static int _current_mods = 0;
static std::queue<wnd_event> _events;

void key_cb(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    _current_mods = mods;

    wnd_event evt;
    evt.type = action == GLFW_PRESS ? WND_KEY_PRESSED : WND_KEY_RELEASED;
    evt.key.code = key;
    _events.push(evt);
}

void mouse_move_cb(GLFWwindow* window, double x, double y)
{
    wnd_event evt;
    evt.type = WND_MOUSE_MOVED;
    evt.mouse.x = int(x);
    evt.mouse.y = int(y);
    _events.push(evt);
}

void mouse_button_cb(GLFWwindow* window, int button, int action, int mods)
{
    wnd_event evt;
    evt.type = action == GLFW_PRESS ? WND_MOUSE_BUTTON_PRESSED : WND_MOUSE_BUTTON_RELEASED;
    evt.mouse.button = button;
    _events.push(evt);
}

void mouse_scroll_cb(GLFWwindow* window, double x, double y)
{
    wnd_event evt;
    evt.type = WND_MOUSE_BUTTON_PRESSED;
    evt.mouse.button = y > 0 ? WND_MOUSE_WHEEL_UP : WND_MOUSE_WHEEL_DOWN;
    _events.push(evt);
}

bool wnd_init(int* width, int* height, bool presentationMode)
{
    if (!glfwInit())
    {
		printf("Could not initialise GLFW\n");
        return false;
    }

    GLFWmonitor* fullscreen = 0;
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    if (presentationMode)
    {
        *width = mode->width;
        *height = mode->height;
        fullscreen = glfwGetPrimaryMonitor();
    }
    else
    {
        *width = std::min(mode->width, (int)(mode->height * 16.0 / 9.0));
        *width = *width - 80;
        *height = mode->height - 80;
    }

    _window = glfwCreateWindow(*width, *height, "Recast Demo", fullscreen, NULL);
    if (!_window)
    {
		printf("Could not initialise GLFW window\n");
		return false;
    }

    glfwSetKeyCallback(_window, key_cb);
    glfwSetCursorPosCallback(_window, mouse_move_cb);
    glfwSetScrollCallback(_window, mouse_scroll_cb);
    glfwSetMouseButtonCallback(_window, mouse_button_cb);

    glfwMakeContextCurrent(_window);
    glfwSwapInterval(1);

    return true;
}

void wnd_quit()
{
    glfwTerminate();
}

double wnd_ticks()
{
    return glfwGetTime();
}

bool wnd_poll_event(wnd_event& event)
{
    if (_events.empty())
        return false;

    event = _events.front();
    _events.pop();

    return true;
}

void wnd_sleep(int ms)
{
    // sleep
}

void wnd_swap()
{
    glfwSwapBuffers(_window);
    glfwPollEvents();

    if (glfwWindowShouldClose(_window) && !_should_quit)
    {
        _should_quit = true;
        wnd_event evt;
        evt.type = WND_QUIT;
        _events.push(evt);
    }
}

bool wnd_mod_state(int kmod)
{
    return _current_mods & kmod;
}

bool wnd_get_key(int key)
{
    return glfwGetKey(_window, key) == GLFW_PRESS;
}

bool wnd_get_mouse(int button)
{
    return glfwGetMouseButton(_window, button) == GLFW_PRESS;
}

#endif // USE_GLFW

