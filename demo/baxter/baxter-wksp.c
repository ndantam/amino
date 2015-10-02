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
 *   AND ON ANY HEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */
#define GL_GLEXT_PROTOTYPES

#include <error.h>
#include <stdio.h>
#include <math.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <SDL.h>

#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/amino_gl.h"
#include "amino/rx/amino_sdl.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_kin.h"

#include "baxter-demo.h"



struct display_cx {
    const struct aa_gl_globals *globals;
    const struct aa_rx_sg *scenegraph;
    const struct aa_rx_sg_sub *ssg;
    double *q;
    struct timespec first;
    struct timespec last;
};

int display( void *cx_, int updated, const struct timespec *now )
{
    struct display_cx *cx = (struct display_cx *)cx_;
    const struct aa_gl_globals *globals = cx->globals;
    const struct aa_rx_sg *scenegraph = cx->scenegraph;

    if( 0 == cx->first.tv_sec && 0 == cx->first.tv_nsec ) {
        memcpy( &cx->first, now, sizeof(*now) );
        memcpy( &cx->last, now, sizeof(*now) );
    }

    double t = aa_tm_timespec2sec( aa_tm_sub(*now, cx->first) );
    double dt = aa_tm_timespec2sec( aa_tm_sub(*now, cx->last) );


    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    baxter_demo_check_error("glClearColor");

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    baxter_demo_check_error("glClear");

    aa_rx_frame_id n = aa_rx_sg_frame_count(scenegraph);
    aa_rx_frame_id m = aa_rx_sg_config_count(scenegraph);
    aa_rx_frame_id n_c = aa_rx_sg_sub_config_count(cx->ssg);
    aa_rx_frame_id n_f = aa_rx_sg_sub_frame_count(cx->ssg);

    double TF_rel[7*n];
    double TF_abs[7*n];
    aa_rx_sg_tf(scenegraph, m, cx->q,
                n,
                TF_rel, 7,
                TF_abs, 7 );
    aa_rx_sg_render( scenegraph, globals,
                     (size_t)n, TF_abs, 7 );


    double dx_r[6] = {0};
    size_t rows,cols;
    aa_rx_sg_sub_jacobian_size( cx->ssg, &rows, &cols );
    double dq_subset[n_c];
    /* Feed forward velocity only, so some drift */
    dx_r[AA_TF_DX_V+2] = .5*cos(t*2*M_PI);

    double J[rows*cols], J_star[rows*cols];
    aa_rx_sg_sub_jacobian( cx->ssg, n, TF_abs, 7,
                           J, rows );
    // dx = J * dq
    aa_la_dls( rows, n_c, 1e-6, J, dx_r, dq_subset );

    /* Alternative damped-least squares computation */
    /* aa_la_dzdpinv( 6, n_c, 1e-2, J, J_star ); */
    /* aa_la_dpinv( 6, n_c, 5e-3, J, J_star ); */
    /* double dq_r[cx->n_ch_config]; */
    /* AA_MEM_ZERO(dq_r,cx->n_ch_config); */
    /* aa_la_xlsnp( 6, cx->n_ch_config, J, J_star, */
    /*              dx_r, dq_r, dq_subset ); */

    aa_rx_frame_id f_ee = aa_rx_sg_sub_frame(cx->ssg,n_f-1);
    double *E_ee = &TF_abs[7 * f_ee];

    // check
    double dx_u[6];
    cblas_dgemv( CblasColMajor, CblasNoTrans,
                 6, n_c,
                 1.0, J, 6,
                 dq_subset, 1,
                 0.0, dx_u, 1 );
    //aa_dump_vec( stdout, dx_u, 6 );



    // integrate
    double q_subset[n_c];
    aa_rx_sg_config_get( scenegraph, m, n_c,
                         aa_rx_sg_sub_configs(cx->ssg), cx->q, q_subset );
    for( size_t i = 0; i < n_c; i ++ ) {
        q_subset[i] += dt*dq_subset[i];
    }
    aa_rx_sg_config_set( scenegraph, m, n_c,
                         aa_rx_sg_sub_configs(cx->ssg),
                         q_subset, cx->q );



    memcpy( &cx->last, now, sizeof(*now) );
    return 1;
}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    SDL_Window* window = NULL;
    SDL_GLContext gContext = NULL;
    struct aa_gl_globals *globals;

    // Initialize scene graph
    struct aa_rx_sg *scenegraph = generate_scenegraph(NULL);
    aa_rx_sg_index(scenegraph);

    // setup window
    baxter_demo_setup_window( scenegraph,
                              &window, &gContext, &globals );
    aa_gl_globals_set_show_visual(globals, 1);
    aa_gl_globals_set_show_collision(globals, 0);

    // setup control context
    size_t n_q = aa_rx_sg_config_count(scenegraph);
    struct display_cx cx = {0};
    cx.globals = globals;
    cx.scenegraph = scenegraph;
    cx.q = AA_NEW0_AR(double, n_q );

    aa_rx_frame_id tip_id = aa_rx_sg_frame_id(scenegraph, "right_w2");
    cx.ssg = aa_rx_sg_chain_create( scenegraph, AA_RX_FRAME_ROOT, tip_id);

    // set start and goal states
    const char *names[] = {"right_s0",
                           "right_s1",
                           "right_e0",
                           "right_e1",
                           "right_w0",
                           "right_w1",
                           "right_w2"};
    aa_rx_config_id ids[7];
    aa_rx_sg_config_indices( scenegraph, 7,
                             names, ids );
    double q1[7] = {-.25 * M_PI, // s0
                    0 * M_PI, // s1
                    M_PI, // e0
                    .5*M_PI, // e1
                    0, // w0
                    0*M_PI, // w1
                    0 // w2
    };
    aa_rx_sg_config_set( scenegraph, n_q, 7,
                         ids, q1, cx.q );


    aa_sdl_display_loop( window, globals,
                         display,
                         &cx );

    SDL_GL_DeleteContext(gContext);
    SDL_DestroyWindow( window );
    SDL_Quit();
    return 0;
}
