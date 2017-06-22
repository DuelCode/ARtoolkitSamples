/*
 *  arPattCreateHandle.c
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
 *  Copyright 2003-2015 ARToolworks, Inc.
 *
 *  Author(s): Hirokazu Kato
 *
 */
/*******************************************************
 *
 * Author: Hirokazu Kato
 *
 *         kato@sys.im.hiroshima-cu.ac.jp
 *
 * Revision: 5.1
 * Date: 03/08/13
 *
 *******************************************************/

#include <AR/ar.h>
#include <stdio.h>
#include <math.h>

ARPattHandle *arPattCreateHandle(void)
{
    return (arPattCreateHandle2(AR_PATT_SIZE1, AR_PATT_NUM_MAX));
}

ARPattHandle *arPattCreateHandle2(const int pattSize, const int patternCountMax)
{
    ARPattHandle  *pattHandle;
    int            i, j;
    
    if (pattSize < 16 || pattSize > AR_PATT_SIZE1_MAX || patternCountMax <= 0) return NULL;

    arMalloc( pattHandle, ARPattHandle, 1 );

    pattHandle->patt_num = 0;
    pattHandle->patt_num_max = patternCountMax;
    //pattHandle->pattRatio = AR_PATT_RATIO;
    pattHandle->pattSize = pattSize;
    
    arMalloc(pattHandle->pattf, int, patternCountMax);
    arMalloc(pattHandle->patt, int *, patternCountMax*4)
    arMalloc(pattHandle->pattBW, int *, patternCountMax*4)
    arMalloc(pattHandle->pattpow, ARdouble, patternCountMax*4)
    arMalloc(pattHandle->pattpowBW, ARdouble, patternCountMax*4)
    for (i = 0; i < patternCountMax; i++) {
        pattHandle->pattf[i] = 0;
        for (j = 0; j < 4; j++) {
            arMalloc(pattHandle->patt[i*4 + j], int, pattSize*pattSize*3);
            arMalloc(pattHandle->pattBW[i*4 + j], int, pattSize*pattSize);
        }
    }

    return pattHandle;
}

int arPattDeleteHandle(ARPattHandle *pattHandle)
{
	int i, j;
	
	if (pattHandle == NULL) return (-1);
	
    for (i = 0; i < pattHandle->patt_num_max; i++) {
		if (pattHandle->pattf[i] != 0) arPattFree(pattHandle, i);
        for (j = 0; j < 4; j++) {
            free(pattHandle->patt[i*4 + j]);
            free(pattHandle->pattBW[i*4 + j]);
        }
	}
	free(pattHandle);
	pattHandle = NULL;
	
	return (0);
}

/*
int arPattGetPattRatio( ARPattHandle *pattHandle, float *ratio )
{
    if( pattHandle == NULL ) return -1;
    *ratio = pattHandle->pattRatio;
    return 0;
}

int arPattSetPattRatio( ARPattHandle *pattHandle, float  ratio )
{
    if( pattHandle == NULL ) return -1;
    if( ratio <= 0.0 || ratio >= 1.0 ) return -1;
    pattHandle->pattRatio = ratio;
    return 0;
}
*/