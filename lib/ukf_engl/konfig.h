/*************************************************************************************************************
 * This file contains configuration parameters
 * 
 * 
 * See https://github.com/pronenewbits for more!
 ************************************************************************************************************/
#ifndef KONFIG_H
#define KONFIG_H

#include <stdlib.h>
#include <stdint.h>
#include <math.h>



/* State Space dimension */
#define SS_X_LEN    (6)
#define SS_Z_LEN    (7)
#define SS_U_LEN    (6)


/* Change this size based on the biggest matrix you will use */
#define MATRIX_MAXIMUM_SIZE     (13)

/* Define this to enable matrix bound checking */
#define MATRIX_USE_BOUNDS_CHECKING

/* Set this define to choose math precision of the system */
#define PRECISION_SINGLE    1
#define PRECISION_DOUBLE    2
#define FPU_PRECISION       (PRECISION_SINGLE)

#if (FPU_PRECISION == PRECISION_SINGLE)
    #define float_prec          float
    #define float_prec_ZERO     (1e-7)
    #define float_prec_ZERO_ECO (1e-5)      /* 'Economical' zero, for noisy calculation where 'somewhat zero' is good enough */
#elif (FPU_PRECISION == PRECISION_DOUBLE)
    #define float_prec          double
    #define float_prec_ZERO     (1e-13)
    #define float_prec_ZERO_ECO (1e-8)      /* 'Economical' zero, for noisy calculation where 'somewhat zero' is good enough */
#else
    #error("FPU_PRECISION has not been defined!");
#endif



/* Set this define to choose system implementation (mainly used to define how you print the matrix via the Matrix::vPrint() & Matrix::vPrintFull() function) */
#define SYSTEM_IMPLEMENTATION_PC                    1
#define SYSTEM_IMPLEMENTATION_EMBEDDED_CUSTOM       2
#define SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO      3

#define SYSTEM_IMPLEMENTATION                       (SYSTEM_IMPLEMENTATION_EMBEDDED_CUSTOM)



/* ASSERT is evaluated locally (without function call) to lower the computation cost */
void SPEW_THE_ERROR(char const * str);
#define ASSERT(truth, str) { if (!(truth)) SPEW_THE_ERROR(str); }


#endif // KONFIG_H
