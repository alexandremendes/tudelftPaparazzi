/*
 * $Id: booz_imu_crista.h 3732 2009-07-20 17:46:54Z poine $
 *  
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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
 */

#ifndef BOOZ_IMU_AEROVINCI_H
#define BOOZ_IMU_AEROVINCI_H

#include "booz_imu.h"


#define BoozImuEvent(_gyro_accel_handler, _mag_handler) {		\
      booz_imu.gyro_unscaled.p = 0;					\
      booz_imu.gyro_unscaled.q = 1;					\
      booz_imu.gyro_unscaled.r = 2;					\
      booz_imu.accel_unscaled.x = 3;					\
      booz_imu.accel_unscaled.y = 4;					\
      booz_imu.accel_unscaled.z = 5;					\
      _gyro_accel_handler();						\
      _mag_handler();					\
  }



#endif /* BOOZ_IMU_AEROVINCI_H */

