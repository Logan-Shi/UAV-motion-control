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
    Control u1;
    Control u2;
    Control u3;
    Control u4;
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(x1) == x2;
    acadodata_f1 << dot(x2) == (cos(x11)*x9+sin(x11)*x7)*(pow(x13,2.00000000000000000000e+00)+pow(x14,2.00000000000000000000e+00)+pow(x15,2.00000000000000000000e+00)+pow(x16,2.00000000000000000000e+00))*2.94651618274805472935e-06;
    acadodata_f1 << dot(x3) == x4;
    acadodata_f1 << dot(x4) == (-cos(x11)*x7+sin(x11)*x9)*(pow(x13,2.00000000000000000000e+00)+pow(x14,2.00000000000000000000e+00)+pow(x15,2.00000000000000000000e+00)+pow(x16,2.00000000000000000000e+00))*2.94651618274805472935e-06;
    acadodata_f1 << dot(x5) == x6;
    acadodata_f1 << dot(x6) == ((pow(x13,2.00000000000000000000e+00)+pow(x14,2.00000000000000000000e+00)+pow(x15,2.00000000000000000000e+00)+pow(x16,2.00000000000000000000e+00))*2.94651618274805472935e-06-9.80659999999999953957e+00);
    acadodata_f1 << dot(x7) == x8;
    acadodata_f1 << dot(x8) == ((-7.53086419753086211415e-01)*x10*x12+(pow(x14,2.00000000000000000000e+00)-pow(x16,2.00000000000000000000e+00))*8.27570903179237609424e-06-(x13-x14+x15-x16)*5.17333333333333186821e-04*x10);
    acadodata_f1 << dot(x9) == x10;
    acadodata_f1 << dot(x10) == ((-pow(x13,2.00000000000000000000e+00)+pow(x15,2.00000000000000000000e+00))*8.27570903179237609424e-06+(x13-x14+x15-x16)*5.17333333333333186821e-04*x8+7.53086419753086211415e-01*x12*x8);
    acadodata_f1 << dot(x11) == x12;
    acadodata_f1 << dot(x12) == (pow(x13,2.00000000000000000000e+00)-pow(x14,2.00000000000000000000e+00)+pow(x15,2.00000000000000000000e+00)-pow(x16,2.00000000000000000000e+00))*1.58580152198092713520e-07;
    acadodata_f1 << dot(x13) == u1;
    acadodata_f1 << dot(x14) == u2;
    acadodata_f1 << dot(x15) == u3;
    acadodata_f1 << dot(x16) == u4;

    OCP ocp1(0, 20, 101);
    ocp1.minimizeLagrangeTerm(((pow(u1,2.00000000000000000000e+00)+pow(u2,2.00000000000000000000e+00)+pow(u3,2.00000000000000000000e+00)+pow(u4,2.00000000000000000000e+00))*3.25966926723878102934e-06+2.77598644631446361212e-04*pow(x13,2.00000000000000000000e+00)+2.77598644631446361212e-04*pow(x14,2.00000000000000000000e+00)+2.77598644631446361212e-04*pow(x15,2.00000000000000000000e+00)+2.77598644631446361212e-04*pow(x16,2.00000000000000000000e+00)+2.97017855685883347405e+00+2.97017855685883347405e+00+2.97017855685883347405e+00+2.97017855685883347405e+00+3.92392851620067306585e-08*pow(x13,3.00000000000000000000e+00)+3.92392851620067306585e-08*pow(x14,3.00000000000000000000e+00)+3.92392851620067306585e-08*pow(x15,3.00000000000000000000e+00)+3.92392851620067306585e-08*pow(x16,3.00000000000000000000e+00)+6.97017855685883314099e-02*x13+6.97017855685883314099e-02*x14+6.97017855685883314099e-02*x15+6.97017855685883314099e-02*x16+9.41319217589370912105e-13*pow(x13,4.00000000000000000000e+00)+9.41319217589370912105e-13*pow(x14,4.00000000000000000000e+00)+9.41319217589370912105e-13*pow(x15,4.00000000000000000000e+00)+9.41319217589370912105e-13*pow(x16,4.00000000000000000000e+00)));
    ocp1.subjectTo(acadodata_f1);
    ocp1.subjectTo(AT_START, x1 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x2 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x3 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x4 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x5 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x6 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x7 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x8 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x9 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x10 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x11 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x12 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_START, x13 == 9.12109000000000037289e+02);
    ocp1.subjectTo(AT_START, x14 == 9.12109000000000037289e+02);
    ocp1.subjectTo(AT_START, x15 == 9.12109000000000037289e+02);
    ocp1.subjectTo(AT_START, x16 == 9.12109000000000037289e+02);
    ocp1.subjectTo(AT_END, x1 == 4.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, x2 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, x3 == 5.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, x4 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, x5 == 6.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, x6 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, x7 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, x8 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, x9 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, x10 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, x11 == 7.85398163397448278999e-01);
    ocp1.subjectTo(AT_END, x12 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, x13 == 9.12109000000000037289e+02);
    ocp1.subjectTo(AT_END, x14 == 9.12109000000000037289e+02);
    ocp1.subjectTo(AT_END, x15 == 9.12109000000000037289e+02);
    ocp1.subjectTo(AT_END, x16 == 9.12109000000000037289e+02);
    ocp1.subjectTo(1.00000000000000002082e-02 <= x13 <= 1.04719699999999988904e+03);
    ocp1.subjectTo(1.00000000000000002082e-02 <= x14 <= 1.04719699999999988904e+03);
    ocp1.subjectTo(1.00000000000000002082e-02 <= x15 <= 1.04719699999999988904e+03);
    ocp1.subjectTo(1.00000000000000002082e-02 <= x16 <= 1.04719699999999988904e+03);


    OptimizationAlgorithm algo1(ocp1);
    algo1.set( KKT_TOLERANCE, 1.000000E-05 );
    algo1.set( MAX_NUM_ITERATIONS, 40 );
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

