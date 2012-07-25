// $Id: nonlinearanalyticconditionalgaussianmobile.cpp 5823 2005-10-27 13:43:02Z TDeLaet $
// Copyright (C) 2006  Tinne De Laet <first dot last at mech dot kuleuven dot be>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//

// modified by Brice Rebsamen in July 2012


#include "nonlinearanalyticconditionalgaussianmobile.h"
#include "EKF_cts.h"

// #include <bfl/wrappers/rng/rng.h> // Wrapper around several rng libraries


#define NUMCONDARGUMENTS_MOBILE 2   //1 means that the system has no inputs.
                                    //2 means there is one input (the dt)

NonLinearAnalyticConditionalGaussianMobile::NonLinearAnalyticConditionalGaussianMobile(const BFL::Gaussian& additiveNoise)
: BFL::AnalyticConditionalGaussianAdditiveNoise(additiveNoise, NUMCONDARGUMENTS_MOBILE)
{

}


NonLinearAnalyticConditionalGaussianMobile::~NonLinearAnalyticConditionalGaussianMobile()
{

}

MatrixWrapper::ColumnVector NonLinearAnalyticConditionalGaussianMobile::ExpectedValueGet() const
{
    MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
    MatrixWrapper::ColumnVector input = ConditionalArgumentGet(1);
    double dt = input(1);
    state(STATE::X) += cos(state(STATE::T)) * state(STATE::V) * dt;
    state(STATE::Y) += sin(state(STATE::T)) * state(STATE::V) * dt;
    state(STATE::T) += state(STATE::W) * dt;
    // state(STATE::V) and state(STATE::W) remain the same
    return state + AdditiveNoiseMuGet();
}

MatrixWrapper::Matrix NonLinearAnalyticConditionalGaussianMobile::dfGet(unsigned int i) const
{
    if (i==0)//derivative to the first conditional argument (x)
    {
        MatrixWrapper::ColumnVector state = ConditionalArgumentGet(0);
        MatrixWrapper::ColumnVector input = ConditionalArgumentGet(1);
        double dt = input(1);

        MatrixWrapper::Matrix df(STATE::SIZE, STATE::SIZE);

        // Init as identity matrix
        for(unsigned i=1; i<=STATE::SIZE; i++)
            for(unsigned j=1; j<=STATE::SIZE; j++)
                df(i,j) = (i==j ? 1.0 : 0.0);

        df(STATE::X, STATE::T) = -state(STATE::V)*sin(state(STATE::T))*dt;
        df(STATE::X, STATE::V) = cos(state(STATE::T))*dt;
        df(STATE::Y, STATE::T) = state(STATE::V)*cos(state(STATE::T))*dt;
        df(STATE::Y, STATE::V) = sin(state(STATE::T))*dt;
        df(STATE::T, STATE::W) = dt;
        return df;
    }
    else
    {
        if (i >= NumConditionalArgumentsGet())
        {
            std::cerr << "This pdf Only has " << NumConditionalArgumentsGet() << " conditional arguments\n";
            ::exit(-BFL_ERRMISUSE);
        }
        else
        {
            std::cerr << "The df is not implemented for the" <<i << "th conditional argument\n";
            ::exit(-BFL_ERRMISUSE);
        }
    }
}
