/*
 * $Id$
 *
 * Copyright (C) 2011 Alexandre Mendes
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file gumstix_camera_interface.h
 *
 * made from the demo module with blinking LEDs
 */

#ifndef _GUMSTIX_CAMERA_INTERFACE_H_
#define _GUMSTIX_CAMERA_INTERFACE_H_

extern int gumstix_camera_interface_dummy_var;
extern int x_gumstix_simulated;
extern int y_gumstix_simulated;

#ifndef DEMO_MODULE_LED
#define DEMO_MODULE_LED 4
#endif

void init_gumstix_camera_interface(void);
void periodic_gumstix_camera_interface(void);
//void periodic_10Hz_demo(void);
void start_gumstix_camera_interface(void);
void stop_gumstix_camera_interface(void);

#endif
