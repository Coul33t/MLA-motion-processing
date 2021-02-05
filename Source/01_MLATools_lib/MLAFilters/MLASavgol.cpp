#include "MLASavgol.h"
namespace Mla {
	namespace Filters {

		/**************************************************************
		* Given an N x N matrix A, this routine replaces it by the LU *
		* decomposition of a rowwise permutation of itself. A and N   *
		* are input. INDX is an output vector which records the row   *
		* permutation effected by the partial pivoting; D is output   *
		* as -1 or 1, depending on whether the number of row inter-   *
		* changes was even or odd, respectively. This routine is used *
		* in combination with LUBKSB to solve linear equations or to  *
		* invert a matrix. Return code is 1, if matrix is singular.   *
		**************************************************************/
		void LUDCMP(MAT A, int N, int *INDX, int *D, int *CODE) {

			const int nmx = 100;

			double AMAX, DUM, SUM, TINY;
			double VV[nmx];
			int i_max = 0;

			TINY = (double)1e-12;

			*D = 1; 
			*CODE = 0;

			for (int i = 1; i <= N; i++) {
				AMAX = 0.0;
				for (int j = 1; j <= N; j++)
					if (fabs(A[i][j]) > AMAX) AMAX = (double)fabs(A[i][j]);
				if (AMAX < TINY) {
					*CODE = 1;
					return;
				}
				VV[i] = (double)1.0 / AMAX;
			}

			for (int j = 1; j <= N; j++) {
				for (int i = 1; i < j; i++) {
					SUM = A[i][j];
					for (int k = 1; k < i; k++)
						SUM -= A[i][k] * A[k][j];
					A[i][j] = SUM;
				}
				AMAX = 0.0;
				for (int i = j; i <= N; i++) {
					SUM = A[i][j];
					for (int k = 1; k < j; k++)
						SUM -= A[i][k] * A[k][j];
					A[i][j] = SUM;
					DUM = VV[i] * (double)fabs(SUM);
					if (DUM >= AMAX) {
						i_max = i;
						AMAX = DUM;
					}
				}

				if (j != i_max) {
					for (int k = 1; k <= N; k++) {
						DUM = A[i_max][k];
						A[i_max][k] = A[j][k];
						A[j][k] = DUM;
					}
					*D = -(*D);
					VV[i_max] = VV[j];
				}

				INDX[j] = i_max;
				if ((double)fabs(A[j][j]) < TINY)  A[j][j] = TINY;

				if (j != N) {
					DUM = (double)1.0 / A[j][j];
					for (int i = j + 1; i <= N; i++)  A[i][j] *= DUM;
				}
			} //j loop

		} //LUDCMP()


		/*****************************************************************
		* Solves the set of N linear equations A . X = B.  Here A is    *
		* input, not as the matrix A but rather as its LU decomposition, *
		* determined by the routine LUDCMP. INDX is input as the permuta-*
		* tion vector returned by LUDCMP. B is input as the right-hand   *
		* side vector B, and returns with the solution vector X. A, N and*
		* INDX are not modified by this routine and can be used for suc- *
		* cessive calls with different right-hand sides. This routine is *
		* also efficient for plain matrix inversion.                     *
		*****************************************************************/
		void LUBKSB(MAT A, int N, int *INDX, double *B)  {

			double SUM;
			int I, II, J, LL;

			II = 0;

			for (I = 1; I <= N; I++) {
				LL = INDX[I];
				SUM = B[LL];
				B[LL] = B[I];
				if (II != 0)
					for (J = II; J < I; J++)
						SUM -= A[I][J] * B[J];
				else if (SUM != 0.0)
					II = I;
				B[I] = SUM;
			}

			for (I = N; I > 0; I--) {
				SUM = B[I];
				if (I < N)
					for (J = I + 1; J <= N; J++)
						SUM -= A[I][J] * B[J];
				B[I] = SUM / A[I][I];
			}

		}


		void SavgolCoeffs(std::vector<double>& coefs, int nl, int nr, int ld, int m)  {
			/*-------------------------------------------------------------------------------------------
			USES lubksb,ludcmp given below.
			Returns in c(np), in wrap-around order (see reference) consistent with the argument respns
			in routine convlv, a set of Savitzky-Golay filter coefficients. nl is the number of leftward
			(past) data points used, while nr is the number of rightward (future) data points, making
			the total number of data points used nl +nr+1. ld is the order of the derivative desired
			(e.g., ld = 0 for smoothed function). m is the order of the smoothing polynomial, also
			equal to the highest conserved moment; usual values are m = 2 or m = 4.
			-------------------------------------------------------------------------------------------*/
			int d, icode, imj, ipj, j, k, mm;
			int indx[MMAX + 2];
			double fac, sum;
			MAT a;
			double b[MMAX + 2];

			for (size_t i = 1; i <= MMAX + 1; i++) {
				for (j = 1; j <= MMAX + 1; j++) a[i][j] = 0.0;
				b[i] = 0.0;
				indx[i] = 0;
			}

			for (ipj = 0; ipj <= 2 * m; ipj++) { //Set up the normal equations of the desired leastsquares fit.
				sum = 0.0;
				if (ipj == 0) sum = 1.0;
				for (k = 1; k <= nr; k++) sum += (double)pow(k, ipj);
				for (k = 1; k <= nl; k++) sum += (double)pow(-k, ipj);
				mm = std::min(ipj, 2 * m - ipj);
				imj = -mm;
				do {
					a[1 + (ipj + imj) / 2][1 + (ipj - imj) / 2] = sum;
					imj += 2;
				} while (imj <= mm);
			}

			LUDCMP(a, m + 1, indx, &d, &icode);    //Solve them: LU decomposition

			for (j = 1; j <= m + 1; j++) b[j] = 0.0;
			b[ld + 1] = 1.0;    //Right-hand side vector is unit vector, depending on which derivative we want.

			LUBKSB(a, m + 1, indx, b);      //Backsubstitute, giving one row of the inverse matrix.

			for (k = -nl; k <= nr; k++) {         //Each Savitzky-Golay coefficient is the dot product
				sum = b[1];                       //of powers of an integer with the inverse matrix row.
				fac = 1.0;
				for (mm = 1; mm <= m; mm++) {
					fac *= k;
					sum += b[mm + 1] * fac;
				}        //Store in wrap-around order}
				coefs.push_back(sum);
			}

			std::reverse(coefs.begin(), coefs.end()); // reverse the order of the coefficients
		}

		void Savgol(std::vector<double>& output_data, const std::vector<double>& data, unsigned int polynom_order, unsigned int window_size) {

			if (window_size % 2 == 0) {
				std::cout << "ERROR: window size must be odd" << std::endl;
				return;
			}

			if (polynom_order >= window_size) {
				std::cout << "ERROR: polynom order must be inferior than window size" << std::endl;
				return;
			}

			output_data.clear();

			// The windows length for the left and right of the signal
			// (The window is centered on the current point)
			const unsigned int half_window = (window_size - 1) / 2;

			std::vector<double> padded_data = data;

			// Mirroring signal to avoid boundaries effects.
			// We have multiple options for the boundaries of the signal:
			//  - we can simply put all values to 0 while we don't have enough points (either to the left or the right)
			//  - we can copy the initial values while we don't have enough points (either to the left or the right)
			//  - we can do some zero-padding to the left and the right
			//  - we can mirror the nth first values and the nth last values
			// The last option is used there. It gives a pretty smooth beginning and ending, compared to the zero-padding
			for (size_t i = 0; i < half_window; i++) {
				padded_data.insert(padded_data.begin(), data[i]);
				padded_data.insert(padded_data.end(), data[data.size() - 1 - i]);
			}



			// Compute the Savitzky-Golay filter coefficients.
			// Window_size, left-window size, right window-size, derivate order (0 = smooth), smoothing polynom order
			// I've yet to see a shifting-window algorithm using different values for left and right, so it'll stay like this.
			// Furthermore, the Python implementation explicitly states that the window_size MUST be odd, for this exact reason.
			std::vector<double> coefs;
			SavgolCoeffs(coefs, half_window, half_window, 0, polynom_order);

			// Apply filter to input data.
			for (size_t i = 0; i < data.size(); i++) {
				output_data.push_back(0.0);
				for (size_t j = 0; j < coefs.size(); j++)
					output_data[i] += coefs[j] * padded_data[j + i];
			}
		}
	}
}