/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems
*/

/* Wraps S-Function around filter.h */
#include "filter.h"
#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  filter_wrapper
/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

/*
* Checks parameters as they are entered in Simulink
*/
#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
void mdlCheckParameters(SimStruct *S) {
  /* Check only 2 dimensions */
  if ((mxGetNumberOfDimensions(ssGetSFcnParam(S, 0)) != 2) || mxGetNumberOfDimensions(ssGetSFcnParam(S, 1)) != 2) {
    ssSetErrorStatus(S,"Parameters must be vectors of filter coefficients");
  }
  /* Check that parameters are either a row or column vector */
  const mwSize *pVal0Dims = mxGetDimensions(ssGetSFcnParam(S, 0));
  const mwSize *pVal1Dims = mxGetDimensions(ssGetSFcnParam(S, 1));
  if ((pVal0Dims[0] != 1) && (pVal0Dims[1]!= 1)) {
    ssSetErrorStatus(S,"Parameters must be vectors of filter coefficients");
  }
  if ((pVal1Dims[0] != 1) && (pVal1Dims[1]!= 1)) {
    ssSetErrorStatus(S,"Parameters must be vectors of filter coefficients");
  }
  /* Check minimum sizes */
  if ((mxGetNumberOfElements(ssGetSFcnParam(S, 0)) == 0) || (mxGetNumberOfElements(ssGetSFcnParam(S, 0)) == 0)) {
    ssSetErrorStatus(S,"At least one numerator and denomenator coefficient must be provided");
  }
}
#endif /* MDL_CHECK_PARAMETERS */
/* 
* Callback used by Simulink to determine block characteristics 
* i.e. I/O ports, sizes, states, etc.
*/
static void mdlInitializeSizes(SimStruct *S) {
  /* Number of expected parameters: 2, the a and b coefficient vectors */
  ssSetNumSFcnParams(S, 2);
  /* Check the parameter validity */
#if defined(MATLAB_MEX_FILE)
  if(ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
    mdlCheckParameters(S);
    if(ssGetErrorStatus(S) != NULL) return;
  } else {
    return; /* The Simulink engine reports a mismatch error. */
  }
#endif
  /* No tunable parameters */
  ssSetSFcnParamTunable(S, 0, 0);
  /* No continuous states */
  ssSetNumContStates(S, 0);
  /* No discrete states */
  ssSetNumDiscStates(S, 0);
  /* Number of input ports */
  if (!ssSetNumInputPorts(S, 1)) return;
  /* Input port width 1 */
  ssSetInputPortWidth(S, 0, 1);
  /* Input port type */
  ssSetInputPortDataType(S, 0, SS_SINGLE);
  /* Input port feedthrough */
  ssSetInputPortDirectFeedThrough(S, 0, 1);
  /* Number of output ports */
  if (!ssSetNumOutputPorts(S, 1)) return;
  /* Output port width 1 */
  ssSetOutputPortWidth(S, 0, 1);
  /* Output port type */
  ssSetOutputPortDataType(S, 0, SS_SINGLE);
  /* One sample time */
  ssSetNumSampleTimes(S, 1);
  ssSetNumRWork(S, 0);
  ssSetNumIWork(S, 0);
  /* 
  * Reserve elements in the pointers vector to store our objects
  */
  ssSetNumPWork(S, 1);
  ssSetNumModes(S, 0);
  ssSetNumNonsampledZCs(S, 0);
  ssSetOperatingPointCompliance(S, USE_CUSTOM_OPERATING_POINT);
  /* Set this S-function as runtime thread-safe for multicore execution */
  ssSetRuntimeThreadSafetyCompliance(S, RUNTIME_THREAD_SAFETY_COMPLIANCE_TRUE);
  ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}
/*
* Specifies sample times for the class
*/
static void mdlInitializeSampleTimes(SimStruct *S) {
  ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
  ssSetOffsetTime(S, 0, 0.0);
  ssSetModelReferenceSampleTimeDefaultInheritance(S);
}
/*
* Called once at the start of model execution, used for initialization
*/
#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
static void mdlStart(SimStruct *S) {
  /* Create an instance of the filter class and store in the pointers vector */
  const mxArray *pVal0 = ssGetSFcnParam(S, 0);
  const mxArray *pVal1 = ssGetSFcnParam(S, 1);
  std::vector<float> b;
  std::vector<float> a;
  b.resize(mxGetNumberOfElements(pVal0));
  a.resize(mxGetNumberOfElements(pVal1));
  for (std::size_t i = 0; i < b.size(); i++) {
    b[i] = (float) mxGetDoubles(ssGetSFcnParam(S, 0))[i];
  }
  for (std::size_t i = 0; i < a.size(); i++) {
    a[i] = (float) mxGetDoubles(ssGetSFcnParam(S, 1))[i];
  }
  ssGetPWork(S)[0] = (void *) new Filter(b, a);
}
#endif   /* MDL_START */
/*
* Called every frame to compute the class outputs
*/
static void mdlOutputs(SimStruct *S, int_T tid) {
  /* Retrieve the object */
  Filter *f = (Filter *) ssGetPWork(S)[0];
  /* Input port signal */
  const real_T  *u = ssGetInputPortRealSignal(S, 0);
  /* Output port signal */
  real_T  *y = ssGetOutputPortRealSignal(S, 0);
  /* Compute the outputs */
  y[0] = f->Filt((float)u[0]);
}
/* Define to indicate that this S-Function has the mdlG[S]etOperatingPoint mothods */
#define MDL_OPERATING_POINT

/* Function: mdlGetOperatingPoint =====================================================
 * Abstract:
 *
 */
static mxArray* mdlGetOperatingPoint(SimStruct* S)
{
    // counter* c = (counter*) ssGetPWork(S)[0];
    mxArray* outSS = mxCreateDoubleMatrix(1,1,mxREAL);
    // mxGetPr(outSS)[0] = c->getX();
    return outSS;
}
/* Function: mdlGetOperatingPoint =====================================================
 * Abstract:
 *
 */
static void mdlSetOperatingPoint(SimStruct* S, const mxArray* ma)
{
    // counter* c = (counter*) ssGetPWork(S)[0];
    // c->setX(mxGetPr(ma)[0]);
}

/*
* Called to perform any actions necessary for cleaning up, such as 
* freeing memory
*/
static void mdlTerminate(SimStruct *S) {
  /* Free memory used by our object */
  Filter *f = (Filter *) ssGetPWork(S)[0];
  delete f; 
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
