/*
 *  paramDisp.c
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
 *  Author(s): Takeshi Mita, Shinsaku Hiura, Hirokazu Kato, Philip Lamb
 *
 */
/*******************************************************
 *
 * Author: Takeshi Mita, Shinsaku Hiura, Hirokazu Kato
 *
 *         tmita@inolab.sys.es.osaka-u.ac.jp
 *         shinsaku@sys.es.osaka-u.ac.jp
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Revision: 4.1
 * Date: 01/12/07
 *
 *******************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <AR/ar.h>


int arParamDisp( const ARParam *param )
{
    int     i, j;

    ARLOG("--------------------------------------\n");
    ARLOG("SIZE = %d, %d\n", param->xsize, param->ysize);
	if (param->dist_function_version == 4) {
		ARLOG("Distortion factor: k1=%1.10f, k2=%1.10f, p1=%1.10f, p2=%1.10f\n", param->dist_factor[0],
				param->dist_factor[1], param->dist_factor[2], param->dist_factor[3]);
		ARLOG("                  fx=%f, fy=%f, x0=%f, y0=%f, s=%f\n", param->dist_factor[4],
            param->dist_factor[5], param->dist_factor[6], param->dist_factor[7], param->dist_factor[8]);
	} else if (param->dist_function_version == 3) {
		ARLOG("Distortion factor = %f %f %f %f %f %f\n", param->dist_factor[0],
			   param->dist_factor[1], param->dist_factor[2], param->dist_factor[3],
			   param->dist_factor[4], param->dist_factor[5] );
	} else if (param->dist_function_version == 2) {
		ARLOG("Distortion factor = %f %f %f %f %f\n", param->dist_factor[0],
			   param->dist_factor[1], param->dist_factor[2],
			   param->dist_factor[3], param->dist_factor[4] );
	} else if (param->dist_function_version == 1) {
		ARLOG("Distortion factor = %f %f %f %f\n", param->dist_factor[0],
			   param->dist_factor[1], param->dist_factor[2], param->dist_factor[3] );
	} else {
		ARLOG("Distortion factor = INVALID or UNKNOWN format\n");
	}
    for( j = 0; j < 3; j++ ) {
        for( i = 0; i < 4; i++ ) ARLOG("%7.5f ", param->mat[j][i]);
        ARLOG("\n");
    }
    ARLOG("--------------------------------------\n");

    return 0;
}

int arParamDispExt( ARdouble trans[3][4] )
{
    int     i, j;
                                                                                
    ARLOG("--------------------------------------\n");
    for( j = 0; j < 3; j++ ) {
        for( i = 0; i < 4; i++ ) ARLOG("%7.5f ", trans[j][i]);
        ARLOG("\n");
    }
    ARLOG("--------------------------------------\n");
                                                                                
    return 0;
}

int arParamDispOptical(const ARdouble fovy, const ARdouble aspect, const ARdouble m[16])
{
    int     i;
	
    ARLOG("--------------------------------------\n");
	ARLOG("Field-of-view vertical = %.1f, horizontal = %.1f degrees, aspect ratio = %.3f\n", fovy, fovy*aspect, aspect);
	ARLOG("Transformation to camera, in eye coordinate system = \n");
	for (i = 0; i < 4; i++) ARLOG("[% .3f % .3f % .3f] [% 6.1f]\n", m[i], m[i + 4], m[i + 8], m[i + 12]);
    ARLOG("--------------------------------------\n");
	
    return 0;
}


