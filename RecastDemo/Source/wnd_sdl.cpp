#include "wnd.h"

#if !defined(USE_GLFW)

#include <algorithm>

bool wnd_init(int* width, int* height, bool presentationMode)
{
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
    {
		printf("Could not initialise SDL\n");
        return false;
    }

	// Center window
	char env[] = "SDL_VIDEO_CENTERED=1";
	putenv(env);

	// Init OpenGL
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
//#ifndef WIN32
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
	SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
//#endif

	const SDL_VideoInfo* vi = SDL_GetVideoInfo();

	SDL_Surface* screen = 0;
	if (presentationMode)
	{
		*width = vi->current_w;
		*height = vi->current_h;
		screen = SDL_SetVideoMode(*width, *height, 0, SDL_OPENGL|SDL_FULLSCREEN);
	}
	else
	{
		*width = std::min(vi->current_w, (int)(vi->current_h * 16.0 / 9.0));
		*width = *width - 80;
		*height = vi->current_h - 80;
		screen = SDL_SetVideoMode(*width, *height, 0, SDL_OPENGL);
	}

	if (!screen)
	{
		printf("Could not initialise SDL opengl\n");
		return false;
	}

	SDL_WM_SetCaption("Recast Demo", 0);

    return true;
}

void wnd_quit()
{
    SDL_Quit();
}

double wnd_ticks()
{
    return (double)SDL_GetTicks();
}

bool wnd_poll_event(wnd_event& event)
{
    SDL_Event e;
    if (SDL_PollEvent(&e))
    {
        event.type = e.type;
        switch (e.type)
        {
            case SDL_KEYDOWN:
            case SDL_KEYUP:
                event.key.code = e.key.keysym.sym;
                break;
            case SDL_MOUSEBUTTONDOWN:
            case SDL_MOUSEBUTTONUP:
                event.mouse.button = e.button.button;
                break;
            case SDL_MOUSEMOTION:
                event.mouse.x = e.motion.x;
                event.mouse.y = e.motion.y;
                break;
        }

        return true;
    }

    return false;
}

void wnd_sleep(int ms)
{
    SDL_Delay(ms);
}

void wnd_swap()
{
    SDL_GL_SwapBuffers();
}

bool wnd_mod_state(int kmod)
{
    return SDL_GetModState() & kmod;
}

bool wnd_get_key(int key)
{
    return SDL_GetKeyState(NULL)[key] == 1;
}

bool wnd_get_mouse(int button)
{
    int mask = (button == WND_MOUSE_BUTTON_LEFT) ? SDL_BUTTON_LMASK : SDL_BUTTON_RMASK;
    return (SDL_GetMouseState(0, 0) & mask);
}

#endif // !USE_GLFW

