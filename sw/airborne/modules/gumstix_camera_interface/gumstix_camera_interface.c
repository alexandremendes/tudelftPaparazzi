/*
 * $Id$
 *
 * Copyright (C) 2011  Alexandre Mendes
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

#include "gumstix_camera_interface.h"
#include "generated/flight_plan.h"
#include "led.h"
#include "subsystems/nav.h"

#include "arch/stm32/modules/overo_stm_spi_duplex/overo_stm_spi_duplex.h"
#include "subsystems/ahrs.h"
#include "subsystems/imu.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int gumstix_camera_interface_dummy_var;
int x_gumstix_simulated;
int y_gumstix_simulated;

struct Int32Eulers euler_angles_test;
//struct FloatEulers euler_angles_test;
struct Int32Vect3 accel_body;


void init_gumstix_camera_interface(void) {
  // this part is already done by led_init in fact
  LED_INIT(DEMO_MODULE_LED);
  LED_OFF(DEMO_MODULE_LED);
}

void periodic_gumstix_camera_interface(void) {

  if (gumstix_camera_interface_dummy_var == 1)
  {
    //LED_TOGGLE(DEMO_MODULE_LED);
    //WaypointX(WP_CAM) = x_gumstix_simulated;
    //WaypointY(WP_CAM) = y_gumstix_simulated;
    //WaypointAlt(WP_CAM) = 1;
    if (overo_msg_rx.z > 100)
    {
      //LED_TOGGLE(DEMO_MODULE_LED);
    }
    euler_angles_test = ahrs.ltp_to_body_euler;
    if (euler_angles_test.phi < 10)
    {
      //LED_TOGGLE(DEMO_MODULE_LED);
    }
    /* untilt accels */

    INT32_RMAT_TRANSP_VMULT(accel_body, imu.body_to_imu_rmat, imu.accel);
    //struct Int32Vect3 accel_ltp;
    //INT32_RMAT_TRANSP_VMULT(accel_ltp, ahrs.ltp_to_body_rmat, accel_body);
    float z_accel_float = ACCEL_FLOAT_OF_BFP(accel_body.z);
    if (z_accel_float > -9)
    {
	//LED_TOGGLE(DEMO_MODULE_LED);
    }
    if (overo_msg_available == TRUE)
    {
       LED_TOGGLE(DEMO_MODULE_LED);
    }
  }
  else
  {
  }
}

void start_gumstix_camera_interface(void) {
  LED_ON(DEMO_MODULE_LED);
}

void stop_gumstix_camera_interface(void) {
  LED_OFF(DEMO_MODULE_LED);
}

