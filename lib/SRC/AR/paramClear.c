/*
 *  paramClear.c
 *  ARToolKit5
 *
 *  This file is part of ARToolKit.
 *
 *  ARToolKit is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  ARToolKit is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with ARToolKit.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of this library give you
 *  permission to link this library with independent modules to produce an
 *  executable, regardless of the license terms of these independent modules, and to
 *  copy and distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the terms and
 *  conditions of the license of that module. An independent module is a module
 *  which is neither derived from nor based on this library. If you modify this
 *  library, you may extend this exception to your version of the library, but you
 *  are not obligated to do so. If you do not wish to do so, delete this exception
 *  statement from your version.
 *
 *  Copyright 2015 Daqri, LLC.
 *  Copyright 2002-2015 ARToolworks, Inc.
 *
 *  Author(s): Hirokazu Kato, Philip Lamb
 *
 */
/*******************************************************
 *
 * Author: Hirokazu Kato
 *
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Revision: 2.1
 * Date: 99/07/16
 *
 *******************************************************/

#include <stdio.h>
#include <math.h>
#include <AR/ar.h>

int arParamClear( ARParam *param, int xsize, int ysize, int dist_function_version )
{
    if (!param) return (-1);
    
    param->xsize = xsize;
    param->ysize = ysize;
	param->dist_function_version = dist_function_version;

    param->mat[0][0] =   1.0;
    param->mat[0][1] =   0.0;
    param->mat[0][2] = xsize/2.0;
    param->mat[0][3] =   0.0;
    param->mat[1][0] =   0.0;
    param->mat[1][1] =   1.0;
    param->mat[1][2] = ysize/2.0;
    param->mat[1][3] =   0.0;
    param->mat[2][0] =   0.0;
    param->mat[2][1] =   0.0;
    param->mat[2][2] =   1.0;
    param->mat[2][3] =   0.0;

    return arParamDistFactorClear( param->dist_factor, xsize, ysize, param->dist_function_version );
}


int arParamDistFactorClear( ARdouble dist_factor[AR_DIST_FACTOR_NUM_MAX], int xsize, int ysize, int dist_function_version )
{
    if (!dist_factor) return (-1);

	if (dist_function_version == 4) {
		dist_factor[0] = 0.0;           /*  k1  */
		dist_factor[1] = 0.0;           /*  k2  */
		dist_factor[2] = 0.0;           /*  p1  */
		dist_factor[3] = 0.0;           /*  p2  */
		dist_factor[4] = 1.0;           /*  fx  */
		dist_factor[5] = 1.0;           /*  fx  */
		dist_factor[6] = xsize / 2.0;   /*  x0  */
		dist_factor[7] = ysize / 2.0;   /*  y0  */
		dist_factor[8] = 1.0;           /*  Size adjust */
		return 0;
	} else if (dist_function_version == 3) {
		dist_factor[0] = xsize / 2.0;
		dist_factor[1] = ysize / 2.0;
		dist_factor[2] = 1.0;
		dist_factor[3] = 1.0;
		dist_factor[4] = 0.0;
		dist_factor[5] = 0.0;
		return 0;
	} else if (dist_function_version == 2) {
		dist_factor[0] = xsize / 2.0;
		dist_factor[1] = ysize / 2.0;
		dist_factor[2] = 1.0;
		dist_factor[3] = 0.0;
		dist_factor[4] = 0.0;
		return 0;		
	} else if (dist_function_version == 1) {
		dist_factor[0] = xsize / 2.0;
		dist_factor[1] = ysize / 2.0;
		dist_factor[2] = 1.0;
		dist_factor[3] = 0.0;
		return 0;		
	} else {
		return -1;
	}
}
