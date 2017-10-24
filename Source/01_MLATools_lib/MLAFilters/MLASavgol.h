#ifndef MLA_SAVGOL_H
#define MLA_SAVGOL_H

// Behold the horror of code wrote by progamming-illiterate people
// I'm currently rewriting the whole thing, to feel less like a
// Fortran/MATLAB code, and more like a C++ one.

#include <math.h>
#include <vector>
#include <algorithm> // std::min
#include <iostream>

namespace savgol {
	std::vector<double> Savgol(std::vector<double>&, unsigned int, unsigned int);

	const int NMAX = 2048;	//Maximum number of input data ordinates
	const int NP = 50;		//Maximum number of filter coefficients
	const int MMAX = 6;		//Maximum order of smoothing polynomial

	typedef double MAT[MMAX + 2][MMAX + 2];
};

#endif //MLA_SAVGOL_H