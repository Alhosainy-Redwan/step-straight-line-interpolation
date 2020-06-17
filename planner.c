/*
 * File:   planner.c
 * Author: Alhosainy
 *
 * Created on 25 November 2019, 21:46
 */


#include "planner.h"
//  out_line function used to out the appropriate steps for X ,Y and z axes  based on Bresenham algorithm
// inputs for the function are deltaX(X_steps) ,deltaY(Y_steps),deltaZ ,sign of X, sign of Y and sign of Z (right or left)
void out_line(uint24_t dX, uint24_t dY, uint24_t dZ, bool XS, bool YS, bool ZS)
{
    InitLine();
    
    // first we suppose that deltaX greater than deltaY so the slope < 1
    //and deltaX greater than deltaZ so the slope < 1
    // so we step x incrementally or decrementally along the line
    // next we set the direction of x_steps ,Y_steps and Z_steps along the line 
    DirX = XS;
    DirY = YS;
    DirZ = ZS;
    // there are some variables used to predict  where is the next step
    uint24_t twoDx = dX<<1; 
    uint24_t twoDy = dY<<1;  
    uint24_t twoDz = dZ<<1;   

    if((dX >= dY)&&(dX >= dZ))
    {
        int24_t ey = twoDy -dX; //  initially y error = 2*deltaY  - deltaX   
        int24_t ez = twoDz -dX; //  initially z error = 2*deltaZ  - deltaX  
        int24_t twoDy_twoDx = twoDy - twoDx;  // = 2*deltaY  - 2*deltaX 
        int24_t twoDz_twoDx = twoDz - twoDx;  // = 2*deltaZ  - 2*deltaX 
     while(dX)  // loop for the greatest steps (X_Steps in this case)
     {
        if(ey >= 0)  // in this case we should step in y 
        {
            StepY = 1;  // rise the step high
            ey = ey + twoDy_twoDx;  // accumulative error(this is from the proof of Bresenham in this case )
        }
        else
             ey = ey + twoDy;  // accumulative error(this is from the proof of Bresenham in this case )
        
         if(ez >= 0)   // in this case we should step in z
        { 
            StepZ = 1;  // rise the step high
            ez = ez + twoDz_twoDx;// accumulative error(this is from the proof of Bresenham in this case )

        }
         else
             ez = ez + twoDz;
        
        dX--; // decrement number of steps by 1
        StepX = 1; // as dx are the greatest steps we should step x in every step 
        while(busy);  //wait if the previous step had not be sent 
        SendPulse(); // send the pulse 
        
     }  
    }
    // similarly we write the same code in the other two cases if dz are the greatest or dy are the greatest
    else if((dY >= dX)&&(dY >= dZ))
    {
        int24_t ex = twoDx -dY;   
        int24_t ez = twoDz -dY; 
        int24_t twoDx_twoDy = twoDx - twoDy;  
        int24_t twoDz_twoDy = twoDz - twoDy; 
      while(dY)  
       {
        if(ex >= 0)  
        {
            StepX = 1; 
            ex = ex + twoDx_twoDy;  
        }
        else
              ex = ex + twoDx;  
        if(ez >= 0)  
        { 
            StepZ = 1;  
            ez = ez + twoDz_twoDy;
        }
        else
             ez = ez + twoDz;
        dY--; 
        StepY = 1; 
        while(busy);  
        SendPulse(); 
        
      }   
    }
     else
    {
        int24_t ex = twoDx -dZ;   
        int24_t ey = twoDy -dZ; 
        int24_t twoDx_twoDz = twoDx - twoDz;  
        int24_t twoDy_twoDz = twoDy - twoDz; 
      while(dZ)  
       {
        if(ex >= 0)  
        {
            StepX = 1; 
            ex = ex + twoDx_twoDz;  
        }
         else
             ex = ex + twoDx;  
        if(ey >= 0)  
        { 
            StepY = 1;  
            ey = ey + twoDy_twoDz;
        }
        else
            ey = ey + twoDy;
        dZ--; 
        StepZ = 1; 
        while(busy);  
        SendPulse(); 
        
      }   
    }
    finish_command = 0; // set the the finish_command flag to 0 to receive another line   
    
}

/**********************************************************************************************/

                                        // TODO: reset cur_ind in setup() => done in header file
void SendPulse(void)
{
    busy = 1;                           // prevent further pulses while sending current one
    
    if(!acc_done_f)                     // acceleration period
    {
        TMR0 = timing[cur_ind];
        if(cur_ind == acc_ind)          // BUG: expected(need testing)
        {                               // no overflow but out-of-bounds may occur
            acc_done_f = 1;
            cur_ind--;                  // FIX: prevent out-of-bounds
        }
        cur_ind++;
        TMR0ON = 1 ;
        return;
    }
    
    if(!const_done_f && acc_done_f)     // constant speed period
    {
        TMR0 = timing[cur_ind];
        cur_const++;
        if (cur_const > const_steps)
            const_done_f = 1;
        TMR0ON = 1 ;
        return;
    }
    
    if(!deacc_done_f && const_done_f)   // de-acceleration period
    {
        TMR0 = timing[cur_ind];
        if (cur_ind == deacc_ind)       // BUG: missing pulse => FIXED
        {
            deacc_done_f = 1;
            cur_ind++;                  // FIX: prevent underflow (var = 0xffff)
        }
        cur_ind--;
        TMR0ON = 1 ;
        return;
    }
}

void InitLine(void)
{
    acc_done_f = 0;                             // clearing flags
    const_done_f = 0;
    deacc_done_f = 0;
    
    cur_const = 1;

    if(xSteps > ySteps)                         // getting longest axis
        const_steps = xSteps;
    else 
        const_steps = ySteps;
    if(zSteps > const_steps) const_steps = zSteps;
    
    //uint24_t acc_ind24 = acc_ind;               // from 16bit var to 24bit var(dummy way)
    //uint24_t deacc_ind24 = deacc_ind;           // as compiler wasn't producing any code for next lines 

    const_steps = const_steps - acc_ind + cur_ind;      // BUG: calculate the right offset => approved
    const_steps = const_steps - acc_ind + deacc_ind;    // BUG: offset -2
   /*
    *
    *                           /-----------------------------\
    *            acc end ->    /-------------------------------\    <- deacc start
    *                         /                                 \
    *                        /                                   \  <- deacc end
    *   acc start(cur) ->   /                                     \
    *                      /                                       \
    * 
    */
}
//void SendPulse(void)
//{
//        busy  = 1;  
//        __delay_ms(50);
//        StepX = 0;                  // pull pulse pins high again
//        StepY = 0;
//        StepZ = 0;
//        busy  = 0;  
//       
//}