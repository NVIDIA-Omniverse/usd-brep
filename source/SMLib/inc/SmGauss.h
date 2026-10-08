// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/***********************************************************************
** SmGauss.h: Gauss integration support                               **
***********************************************************************/

// file     : SmGauss.h                                                                                                       
// created  : April 20, 2010                                                                                             
// author   : GWCelniker                                                                                                 
// contains : tables   : SM_gauss_wt                 // tables of daWeight values                                         
//                       SM_gauss_pt                 // tables of sample point location values                            
//                                                                                                                       
//            MACROS   : SM_ASSERT_NTGRL_DEGREE      // check iNtgrlDegree is supported in table                          
//                       SM_SCALE_GPT_LOC            // scale a gauss point location to desired domain                    
//                       SM_GPT_SCALE                // calc scale  of desired_loc = scale*SM_gauss_pt[iCnt][i] + offset  
//                       SM_GPT_OFFSET               // calc offset of desired_loc = scale*SM_gauss_pt[iCnt][i] + offset  
//                                                                                                                       
//            functions: SM_LinearGaussPtCount()     // eff: solve for n      in Degree=2*n-1                             
//                       SM_LinearNtgrlDegree()      // eff: solve for Degree in Degree=2*n-1                             
//                                                                                                                       
// Summary  : This file declares tables of gauss_point and gauss_weight                                                  
//            values used to numerically integrate 1d functions over                                                     
//            the domain of [-1 +1].  These tables can be used                                                           
//            to integrate functions over aribtrary domains, [a b],                                                      
//            of any rectilinear domain_dimension [1, 2, 3, ...] as                                                      
//            described in the Use note below.                                                                           
//            Gauss integration approximates a finite integral                                                           
//            as the weighted sum of function sample points, as                                                          
//                                                                                                                       
//               b                                                                                                       
//            Integral(func(s)) ~= Sum_i(wt_i * func(s_i)).                                                              
//               a                                                                                                       
//                                                                                                                       
//            Gauss Integration maximizes the integration accuracy                                                       
//            while minimizing the number of function sample points                                                      
//            by specifying both the daWeight and sample_point locations.                                                
//            A sum of n sample points will exactly integrate a polynomial                                               
//            function of 2n-1,                                                                                          
//               iNtgrlDegree = 2*n-1.                                                                                   
//                                                                                                                       
// Use in 1D: To use gauss integration in 1d for domains other than [-1, +1]:                                            
//                                                                                                                       
//                  b                       +1                                                                           
//               Integral(func(s) dDs) =  Integral(func(s(dX)) dDs/dx dx                                                 
//                  a                       -1                                                                           
//                      where a = domain_min                                                                             
//                            b = domain_max                                                                             
//                      and gauss_pts are scaled to desired domain [a, b] with                                           
//                           s   = (b-a)*(dX+1)/2 + a, then                                                              
//                         dDs/dx = (b-a)/2.0                                                                            
//                                                                                                                       
//                  a                                                                                                    
//               Integral(func(s) dDs) = for(gpt=0,intgrl=0.0;gpt<gpt_count;gpt++)                                       
//                  b                     {                                                                              
//                                          intgrl +=   (b-a)/2.0                                                        
//                                                    * SM_gauss_wt[gpt_count][gpt]                                      
//                                                    * func(SM_SCALE_GPT_LOC(gpt,gpt_count,a,b))                        
//                                        }                                                                              
//                                                                                                                       
// Use in nD: To use gauss integration in 2d or higher dimensions for rectilinear domains:                               
//                                                                                                                       
//                  b0        b1                           +1        +1                                                  
//               Integral (Integral(func(dU,dV) daDu dv)) = Integral (Integral(func(dU(dX),dV(dY)) daDu/dx dv/dy dx dy)) 
//                  a0        a1                           -1        -1                                                  
//                                                                                                                       
//                  b0        b1                                                                                         
//               Integral (Integral(func(dU,dV) daDu dv)) = for(intgrl=0.0,gpti=0;gpti<gpti_count;gpti++)                
//                  a0        a1                          {                                                              
//                                                          for(gptj=0;gptj<gptj_count;gptj++)                           
//                                                            {                                                          
//                                                              intgrl +=   (b0-a0)/2.0 * (b1-a1)/2.0                    
//                                                                        * SM_gauss_wt[gpti_count][gpti]                
//                                                                        * SM_gauss_wt[gptj_count][gptj]                
//                                                                        * func(SM_SCALE_GPT_LOC(gpti,gpt_count,a0,b0), 
//                                                                               SM_SCALE_GPT_LOC(gptj,gpt_count,a1,b1)) 
//                                                            }                                                          
//                                                        }                                                              
//                                                                                                                       
//                NOTE: gpti_count does not have to equal gptj_count                                                     

#ifndef __SM_GAUSS_H__
#define __SM_GAUSS_H__

#include <SmTypes.h>                  // SM_EXPORT    

#define SM_GAUSS_COUNT        40         // maximum number of gauss points  
#define SM_MAX_NTGRL_DEGREE   79         // max iNtgrlDegree for SM_GAUSS_COUNT  
#define SM_GAUSS_ERROR      1.0e-12      // expected 32 bit double machine roundoff errors,  
                                         // constants are good to 64 bit if run on a longer word machine.  

// a standard iNtgrlDegree validty check  
#define SM_ASSERT_NTGRL_DEGREE(iNtgrlDegree)                                    \
  ( SM_ASSERT_ERR((iNtgrlDegree) > 0 && (iNtgrlDegree) <= SM_MAX_NTGRL_DEGREE,  \
                  SM_BAD_NTGRL_DEGREE))
// convenience functions  

SM_EXPORT long                           // rtn: gauss pt iCount for given polynomial degree   
 SM_LinearGaussPtCount                   // eff: solve for n in degree=2*n-1  
 (long iNtgrlDegree) ;                   // in : integration accuracy, degree of polynomial   
                                         //      exactly integrated.  
                                         
SM_EXPORT long                           // rtn: iNtgrlDegree for given sample iCount  
 SM_LinearNtgrlDegree                    // eff: solve for degree=2*n-1  
 (long iGptCount) ;                      // in : number of sample points  

// MACROS for scaling gauss_pt locations  
//                                        
//  desired_pt = SM_GPT_SCALE * SM_gauss_pt[gpt_count][gpt] + SM_GPC_OFFSET, or  
//  desired_pt = SM_SCALE_GPT_LOC(i,gpt_count,domain_min,domain_max)  

#define SM_GPT_SCALE(domain_min, domain_max)   (((domain_max) - (domain_min))/2.0)
#define SM_GPT_OFFSET(domain_min, domain_max)  (((domain_max) - (domain_min))/2.0 + (domain_min))
#define SM_SCALE_GPT_LOC(gpt_index, gpt_count, domain_min, domain_max) \
           (  (((domain_max) - (domain_min))/2.0)                     \
            * (SM_gauss_pt[(gpt_count)][(gpt_index)] + 1.0)           \
            + (domain_min) )

// The gauss point daWeight table for n = 1 to 40               
// note: SM_gauss_wt[n] = array of n daWeights for n gauss pts  
//       SM_gauss_wt[0] has no entries                          

SM_EXPORT extern double SM_gauss_wt[SM_GAUSS_COUNT+1][SM_GAUSS_COUNT] ;

// The gauss point location table for n = 1 to 40  

SM_EXPORT extern double SM_gauss_pt[SM_GAUSS_COUNT+1][SM_GAUSS_COUNT] ;


#endif // __SM_GAUSS_H__  
