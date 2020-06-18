/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState x1;
    DifferentialState x2;
    DifferentialState x3;
    DifferentialState x4;
    DifferentialState x5;
    DifferentialState x6;
    DifferentialState x7;
    DifferentialState x8;
    DifferentialState x9;
    DifferentialState x10;
    DifferentialState x11;
    DifferentialState x12;
    DifferentialState x13;
    DifferentialState x14;
    DifferentialState x15;
    DifferentialState x16;
    DifferentialState x17;
    DifferentialState x18;
    DifferentialState x19;
    DifferentialState x20;
    DifferentialState x21;
    DifferentialState x22;
    DifferentialState x23;
    DifferentialState x24;
    DifferentialState x25;
    DifferentialState x26;
    Control u1;
    Control u2;
    Control u3;
    Control u4;
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(x1) == x12;
    acadodata_f1 << dot(x2) == x13;
    acadodata_f1 << dot(x3) == x14;
    acadodata_f1 << dot(x4) == x15;
    acadodata_f1 << dot(x5) == x16;
    acadodata_f1 << dot(x6) == x17;
    acadodata_f1 << dot(x7) == x18;
    acadodata_f1 << dot(x8) == x19;
    acadodata_f1 << dot(x9) == x20;
    acadodata_f1 << dot(x10) == x21;
    acadodata_f1 << dot(x11) == x22;
    acadodata_f1 << dot(x12) == 0;
    acadodata_f1 << dot(x13) == 0;
    acadodata_f1 << dot(x14) == (pow(x23,2.00000000000000000000e+00)+pow(x24,2.00000000000000000000e+00)+pow(x25,2.00000000000000000000e+00)+pow(x26,2.00000000000000000000e+00))/1.30000000000000004441e+00*3.83047103757247131756e-06;
    acadodata_f1 << dot(x15) == (pow(x24,2.00000000000000000000e+00)-pow(x26,2.00000000000000000000e+00))*3.83047103757247131756e-06/8.10000000000000025535e-02;
    acadodata_f1 << dot(x16) == (-pow(x23,2.00000000000000000000e+00)+pow(x25,2.00000000000000000000e+00))*3.83047103757247131756e-06/8.10000000000000025535e-02;
    acadodata_f1 << dot(x17) == (pow(x23,2.00000000000000000000e+00)-pow(x24,2.00000000000000000000e+00)+pow(x25,2.00000000000000000000e+00)-pow(x26,2.00000000000000000000e+00))/1.41999999999999987343e-01*2.25183816121291638110e-08;
    acadodata_f1 << dot(x18) == (-(pow(x24,2.00000000000000000000e+00)-pow(x26,2.00000000000000000000e+00))*4.72897658959564319201e-05);
    acadodata_f1 << dot(x19) == (-pow(x23,2.00000000000000000000e+00)+pow(x25,2.00000000000000000000e+00))*4.72897658959564319201e-05;
    acadodata_f1 << dot(x20) == (pow(x23,2.00000000000000000000e+00)+pow(x24,2.00000000000000000000e+00)+pow(x25,2.00000000000000000000e+00)+pow(x26,2.00000000000000000000e+00))/1.30000000000000004441e+00*3.83047103757247131756e-06;
    acadodata_f1 << dot(x21) == (pow(x24,2.00000000000000000000e+00)-pow(x26,2.00000000000000000000e+00))/1.05300000000000004707e-01*1.30000000000000004441e+00*3.83047103757247131756e-06;
    acadodata_f1 << dot(x22) == (-pow(x23,2.00000000000000000000e+00)+pow(x25,2.00000000000000000000e+00))/1.05300000000000004707e-01*1.30000000000000004441e+00*3.83047103757247131756e-06;
    acadodata_f1 << dot(x23) == u1;
    acadodata_f1 << dot(x24) == u2;
    acadodata_f1 << dot(x25) == u3;
    acadodata_f1 << dot(x26) == u4;

    OCP ocp1(0, 20);
    ocp1.minimizeLagrangeTerm(((pow(x10,2.00000000000000000000e+00)+pow(x11,2.00000000000000000000e+00))+pow(x7,2.00000000000000000000e+00)+pow(x8,2.00000000000000000000e+00)+pow(x9,2.00000000000000000000e+00)));
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(AT_START, x1 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x2 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x3 == 2.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x4 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x5 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x6 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x7 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x8 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x9 == 1.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x10 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x11 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x12 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x13 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x14 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x15 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x16 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x17 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x18 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x19 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x20 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x21 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x22 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x23 == 9.12109000000000037289e+02);
    ocp1.subjectTo(AT_START, x24 == 9.12109000000000037289e+02);
    ocp1.subjectTo(AT_START, x25 == 9.12109000000000037289e+02);
    ocp1.subjectTo(AT_START, x26 == 9.12109000000000037289e+02);
    ocp1.subjectTo(AT_END, x1 == 4.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, x2 == 5.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, x3 == 6.00000000000000000000e+00);
    ocp1.subjectTo(1.00000000000000002082e-02 <= x23 <= 1.04719699999999988904e+03);
    ocp1.subjectTo(1.00000000000000002082e-02 <= x24 <= 1.04719699999999988904e+03);
    ocp1.subjectTo(1.00000000000000002082e-02 <= x25 <= 1.04719699999999988904e+03);
    ocp1.subjectTo(1.00000000000000002082e-02 <= x26 <= 1.04719699999999988904e+03);


    OptimizationAlgorithm algo1(ocp1);
    algo1.set( KKT_TOLERANCE, 1.000000E-03 );
    algo1.set( MAX_NUM_ITERATIONS, 20 );
    returnValue returnvalue = algo1.solve();

    VariablesGrid out_states; 
    VariablesGrid out_parameters; 
    VariablesGrid out_controls; 
    VariablesGrid out_disturbances; 
    VariablesGrid out_algstates; 
    algo1.getDifferentialStates(out_states);
    algo1.getControls(out_controls);
    const char* outputFieldNames[] = {"STATES", "CONTROLS", "PARAMETERS", "DISTURBANCES", "ALGEBRAICSTATES", "CONVERGENCE_ACHIEVED"}; 
    plhs[0] = mxCreateStructMatrix( 1,1,6,outputFieldNames ); 
    mxArray *OutS = NULL;
    double  *outS = NULL;
    OutS = mxCreateDoubleMatrix( out_states.getNumPoints(),1+out_states.getNumValues(),mxREAL ); 
    outS = mxGetPr( OutS );
    for( int i=0; i<out_states.getNumPoints(); ++i ){ 
      outS[0*out_states.getNumPoints() + i] = out_states.getTime(i); 
      for( int j=0; j<out_states.getNumValues(); ++j ){ 
        outS[(1+j)*out_states.getNumPoints() + i] = out_states(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"STATES",OutS );
    mxArray *OutC = NULL;
    double  *outC = NULL;
    OutC = mxCreateDoubleMatrix( out_controls.getNumPoints(),1+out_controls.getNumValues(),mxREAL ); 
    outC = mxGetPr( OutC );
    for( int i=0; i<out_controls.getNumPoints(); ++i ){ 
      outC[0*out_controls.getNumPoints() + i] = out_controls.getTime(i); 
      for( int j=0; j<out_controls.getNumValues(); ++j ){ 
        outC[(1+j)*out_controls.getNumPoints() + i] = out_controls(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"CONTROLS",OutC );
    mxArray *OutP = NULL;
    double  *outP = NULL;
    OutP = mxCreateDoubleMatrix( out_parameters.getNumPoints(),1+out_parameters.getNumValues(),mxREAL ); 
    outP = mxGetPr( OutP );
    for( int i=0; i<out_parameters.getNumPoints(); ++i ){ 
      outP[0*out_parameters.getNumPoints() + i] = out_parameters.getTime(i); 
      for( int j=0; j<out_parameters.getNumValues(); ++j ){ 
        outP[(1+j)*out_parameters.getNumPoints() + i] = out_parameters(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"PARAMETERS",OutP );
    mxArray *OutW = NULL;
    double  *outW = NULL;
    OutW = mxCreateDoubleMatrix( out_disturbances.getNumPoints(),1+out_disturbances.getNumValues(),mxREAL ); 
    outW = mxGetPr( OutW );
    for( int i=0; i<out_disturbances.getNumPoints(); ++i ){ 
      outW[0*out_disturbances.getNumPoints() + i] = out_disturbances.getTime(i); 
      for( int j=0; j<out_disturbances.getNumValues(); ++j ){ 
        outW[(1+j)*out_disturbances.getNumPoints() + i] = out_disturbances(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"DISTURBANCES",OutW );
    mxArray *OutZ = NULL;
    double  *outZ = NULL;
    OutZ = mxCreateDoubleMatrix( out_algstates.getNumPoints(),1+out_algstates.getNumValues(),mxREAL ); 
    outZ = mxGetPr( OutZ );
    for( int i=0; i<out_algstates.getNumPoints(); ++i ){ 
      outZ[0*out_algstates.getNumPoints() + i] = out_algstates.getTime(i); 
      for( int j=0; j<out_algstates.getNumValues(); ++j ){ 
        outZ[(1+j)*out_algstates.getNumPoints() + i] = out_algstates(i, j); 
       } 
    } 

    mxSetField( plhs[0],0,"ALGEBRAICSTATES",OutZ );
    mxArray *OutConv = NULL;
    if ( returnvalue == SUCCESSFUL_RETURN ) { OutConv = mxCreateDoubleScalar( 1 ); }else{ OutConv = mxCreateDoubleScalar( 0 ); } 
    mxSetField( plhs[0],0,"CONVERGENCE_ACHIEVED",OutConv );


    clearAllStaticCounters( ); 
 
} 

