/********************************************************************
* Description: dualbipod, derived from trivkins.c
*   Kinematics for a dual bipod machine
*   1st bipod x-y plane, 2nd u-v plane
*
*   Derived from a work by Fred Proctor & Will Shackleford
*   20.05.2009: added x00, y00 to enable offsets at reference point
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
* $Revision: 5 $
* $Author: GerhardGleixner@web.de $
* $Date: 2010-02-21 22:39:46 +0100 (So, 21. Feb 2010) $
********************************************************************/

#include "kinematics.h"		/* these decls */
#include "rtapi_math.h"

#include "hal.h"

struct hal_dat {
    hal_float_t dist, hJ0, hJ1, x0, y0, x00, y00;
};

struct hal_dat * halXY = 0;
struct hal_dat * halUV = 0;

int bipodForward(double j0, double j1, double* x, double* y, struct hal_dat* par) 
{
 // input data, shall be passed by parameter
    double x0, y0, AD2, BD2, x1, y2;
    double Bx = par->dist ; 	// distance of motors
    double x00 = par->x00 ;
    double y00 = par->y00 ;

    // offsets
    x0 = (par->dist * (0.5 +(par->hJ0*par->hJ0 - par->hJ1*par->hJ1)/(2*par->dist*par->dist)));
    y0 = sqrt(par->hJ0*par->hJ0 - x0*x0);
    //x0 = par->x0;// = x0;
    //y0 = par->y0;// = y0;
   
    AD2 = j0 * j0;
    BD2 = j1 * j1;
    x1 = (AD2 - BD2 + Bx * Bx) / (2 * Bx);
    y2 = AD2 - x1 * x1;
    if(y2 < 0) return -1;
    *x = x1 - x0 + x00;
    *y = y0 - sqrt(y2) + y00;
    return 0;
}

int bipodInverse(double* j0, double* j1, double x, double y, struct hal_dat* par) 
{
    double x0, y0, x2, x3, y2, yy;
    double Bx = par->dist ; 	// distance of motors
    double x00 = par->x00 ;	// x of ref point
    double y00 = par->y00 ;	// y of ref point

    // offsets
    x0 = (par->dist * (0.5 +(par->hJ0*par->hJ0 - par->hJ1*par->hJ1)/(2*par->dist*par->dist)));
    y0 = sqrt(par->hJ0*par->hJ0 - x0*x0);
    //x0 = par->x0;
    //y0 = par->y0;

    x3 = x0 + x - x00;
    x2 = x3 * x3;
    yy = y0 - y + y00;
    y2 = (yy) * (yy);
    *j0 = sqrt(x2 + y2);
    *j1 = sqrt((Bx - x3)*(Bx - x3) + y2);
    return 0;
}

int kinematicsForward(const double *joints,
		      EmcPose * pos,
		      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags)
{
   int rv;
    rv = bipodForward (joints[0], joints[1], &pos->tran.x, &pos->tran.y, halXY);
    if(rv < 0) return rv;
    rv = bipodForward (joints[6], joints[7], &pos->u, &pos->v, halUV);
    if(rv < 0) return rv;

    //pos->tran.x = joints[0];
    //pos->tran.y = joints[1];
    pos->tran.z = joints[2];
    pos->a = joints[3];
    pos->b = joints[4];
    pos->c = joints[5];
    //pos->u = joints[6];
    //pos->v = joints[7];
    pos->w = joints[8];

    return 0;
}

int kinematicsInverse(const EmcPose * pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS * iflags,
		      KINEMATICS_FORWARD_FLAGS * fflags)
{
    bipodInverse (&joints[0], &joints[1], pos->tran.x, pos->tran.y, halXY);
    bipodInverse (&joints[6], &joints[7], pos->u, pos->v, halUV);

    //joints[0] = pos->tran.x;
    //joints[1] = pos->tran.y;
    joints[2] = pos->tran.z;
    joints[3] = pos->a;
    joints[4] = pos->b;
    joints[5] = pos->c;
    //joints[6] = pos->u;
    //joints[7] = pos->v;
    joints[8] = pos->w;

    return 0;
}

/* implemented for these kinematics as giving joints preference */
/* when is this fct called ???? seems never */
int kinematicsHome(EmcPose * world,
		   double *joint,
		   KINEMATICS_FORWARD_FLAGS * fflags,
		   KINEMATICS_INVERSE_FLAGS * iflags)
{
  struct hal_dat* par;
    *fflags = 0;
    *iflags = 0;
    // offsets
    par = halXY;
    par->x0 = par->dist * (0.5 +(par->hJ0*par->hJ0 - par->hJ1*par->hJ1)/(2*par->dist*par->dist));
    par->y0 = sqrt(par->hJ0*par->hJ0 - par->x0*par->x0);
    par = halUV;
    par->x0 = par->dist * (0.5 +(par->hJ0*par->hJ0 - par->hJ1*par->hJ1)/(2*par->dist*par->dist));
    par->y0 = sqrt(par->hJ0*par->hJ0 - par->x0*par->x0);

    return kinematicsForward(joint, world, fflags, iflags);
}

KINEMATICS_TYPE kinematicsType()
{
    return KINEMATICS_BOTH;
}

#ifdef RTAPI
#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */
#include "hal.h"

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsForward);
EXPORT_SYMBOL(kinematicsInverse);
EXPORT_SYMBOL(kinematicsHome);
MODULE_LICENSE("GPL");

int comp_id;
int rtapi_app_main(void) 
{
  //struct hal_dat* par;
  int res = 0;

   comp_id = hal_init("dualbipod");
   if(comp_id < 0) return comp_id;

   halXY = hal_malloc(sizeof(*halXY));
   halUV = hal_malloc(sizeof(*halUV));
   if(!halXY) goto error;
   if(!halUV) goto error;

   if((res = hal_param_float_new("dualbipod.D01", HAL_RW, &halXY->dist, comp_id)) != HAL_SUCCESS) goto error;
   if((res = hal_param_float_new("dualbipod.D67", HAL_RW, &halUV->dist, comp_id)) != HAL_SUCCESS) goto error;
   if((res = hal_param_float_new("dualbipod.L0",  HAL_RW, &halXY->hJ0, comp_id)) != HAL_SUCCESS) goto error;
   if((res = hal_param_float_new("dualbipod.L1",  HAL_RW, &halXY->hJ1, comp_id)) != HAL_SUCCESS) goto error;
   if((res = hal_param_float_new("dualbipod.L6",  HAL_RW, &halUV->hJ0, comp_id)) != HAL_SUCCESS) goto error;
   if((res = hal_param_float_new("dualbipod.L7",  HAL_RW, &halUV->hJ1, comp_id)) != HAL_SUCCESS) goto error;
   if((res = hal_param_float_new("dualbipod.X0",  HAL_RW, &halXY->x0, comp_id)) != HAL_SUCCESS) goto error;
   if((res = hal_param_float_new("dualbipod.Y0",  HAL_RW, &halXY->y0, comp_id)) != HAL_SUCCESS) goto error;
   if((res = hal_param_float_new("dualbipod.U0",  HAL_RW, &halUV->x0, comp_id)) != HAL_SUCCESS) goto error;
   if((res = hal_param_float_new("dualbipod.V0",  HAL_RW, &halUV->y0, comp_id)) != HAL_SUCCESS) goto error;
   if((res = hal_param_float_new("dualbipod.X00", HAL_RW, &halXY->x00, comp_id)) != HAL_SUCCESS) goto error;
   if((res = hal_param_float_new("dualbipod.Y00", HAL_RW, &halXY->y00, comp_id)) != HAL_SUCCESS) goto error;
   if((res = hal_param_float_new("dualbipod.U00", HAL_RW, &halUV->x00, comp_id)) != HAL_SUCCESS) goto error;
   if((res = hal_param_float_new("dualbipod.V00", HAL_RW, &halUV->y00, comp_id)) != HAL_SUCCESS) goto error;
   
   hal_ready(comp_id);
   return 0;

error:
    hal_exit(comp_id);
    return res;
}

void rtapi_app_exit(void) { hal_exit(comp_id); }
#endif
