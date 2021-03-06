/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of copyright holder the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef AMINO_RX_SCENE_SDL_H
#define AMINO_RX_SCENE_SDL_H

/**
 * @file scene_sdl.h
 * @brief SDL (Simple DirectMedia Layer) support
 */

#include <SDL.h>

struct aa_gl_globals;

/**
 * Initialize SDL library and options.
 *
 * The following environement variables may be used to set the
 * corresponding SDL attributes:
 *
 *   - SDL_GL_DOUBLEBUFFER
 *   - SDL_GL_MULTISAMPLEBUFFERS
 *   - SDL_GL_MULTISAMPLESAMPLES
 *   - SDL_GL_DEPTH_SIZE
 *
 * The following environement variables may be used to set the
 * corresponding SDL hints:
 *   - SDL_HINT_VIDEO_MINIMIZE_ON_FOCUS_LOSS
 *   - SDL_HINT_RENDER_DRIVER
 *   - SDL_HINT_VIDEO_X11_XRANDR
 *   - SDL_HINT_VIDEO_X11_XVIDMODE
 *   - SDL_HINT_VIDEO_X11_XINERAMA
 *   - SDL_HINT_VIDEO_ALLOW_SCREENSAVER
 *   - SDL_HINT_RENDER_SCALE_QUALITY
 *
 */
AA_API void aa_sdl_init( void );

/**
 * Run the display loop function.
 */
AA_API void aa_sdl_display_loop(
    SDL_Window* window,
    struct aa_gl_globals * globals,
    aa_sdl_display_fun display,
    void *context );


/**
 * Create an SDL window with an OpenGL context.
 */
AA_API void aa_sdl_gl_window(
    const char* title,
    int x_pos,
    int y_pos,
    int width,
    int height,
    Uint32 flags,
    SDL_Window **pwindow,
    SDL_GLContext *p_glcontext );

/**
 * Return the current time step.
 */
AA_API const struct timespec *
aa_sdl_display_params_get_time_now( struct aa_sdl_display_params *params );

/**
 * Return the previous time step.
 */
AA_API const struct timespec *
aa_sdl_display_params_get_time_last( struct aa_sdl_display_params *params );

/**
 * Return the initial time step.
 */
AA_API const struct timespec *
aa_sdl_display_params_get_time_initial( struct aa_sdl_display_params *params );

/**
 * Return whether to update the display.
 */
AA_API int
aa_sdl_display_params_get_update( struct aa_sdl_display_params *params );

/**
 * Indicate a quit (window close) request.
 */
AA_API void
aa_sdl_display_params_set_quit( struct aa_sdl_display_params *params );

/**
 * Indicate a display update is needed.
 */
AA_API void
aa_sdl_display_params_set_update( struct aa_sdl_display_params *params );

/**
 * Check if this is the fist display call.
 */
AA_API int
aa_sdl_display_params_is_first( struct aa_sdl_display_params *params );


/**
 * Retrieve the most recent SDL event.
 */
const SDL_Event *
aa_sdl_display_params_get_event( struct aa_sdl_display_params *params );

/** An event handler.
 *
 * @return 0 when event is handled, non-zero otherwise.
 */
typedef int
(*aa_sdl_handler_function) ( void *cx,
                             struct aa_sdl_display_params *params);

/**
 * Bind a handler function for an SDL event.
 */
AA_API void
aa_sdl_bind_event( SDL_EventType event_type,
                   aa_sdl_handler_function handler,
                   void *cx );

/**
 * Bind a handler function for an SDL key press.
 */
AA_API void
aa_sdl_bind_key( SDL_Keycode key,
                 aa_sdl_handler_function handler,
                 void *cx );

#endif /*AMINO_RX_SCENE_SDL_H*/
