/////////////////////////////////////////////////////////////////////////////////
//                          ariphmetic
/////////////////////////////////////////////////////////////////////////////////

#include "main.h"
#include "math.h"
#include "arithmetic.h"


static const int cordic_ctab[] = {  0x20000000, 0x12E4051E, 0x09FB385B, 0x051111D4,
		    0x028B0D43, 0x0145D7E1, 0x00A2F61E, 0x00517C55, 0x0028BE53, 0x00145F2F,
		    0x000A2F98, 0x000517CC, 0x00028BE6, 0x000145F3, 0x0000A2FA, 0x0000517D,
		    0x000028BE, 0x0000145F, 0x00000A30, 0x00000518, 0x0000028C, 0x00000146,
		    0x000000A3, 0x00000051, 0x00000029, 0x00000014, 0x0000000A, 0x00000005,
		    0x00000003, 0x00000001, 0x00000001, 0x00000000 };

/**
 * @brief:  cplxDiv()
 *          This function does complex division:
 *          z = (a + i*b) / (c + i*d)
 *
 * @param:  *res, *div
 * @return: *res, *div
 */

void cplxDiv(cplx * res, cplx * div) {
	cplx tmp;
	float mod2 = div->Re * div->Re + div->Im * div->Im;

	tmp.Re = (res->Re * div->Re + res->Im * div->Im) / mod2; //(a*c+b*d)/(c*c+d*d);
	tmp.Im = (res->Im * div->Re - res->Re * div->Im) / mod2; //(b*c-a*d)/(c*c+d*d);
	res->Re = tmp.Re;
	res->Im = tmp.Im;
}

/**
 * @brief:  cplxMul()
 *          This function does complex multiplication:
 *          z = (a + i*b) * (c + i*d)
 *
 * @param:  *res, *mul
 * @return: *res, *mul
 */

void cplxMul(cplx * res, cplx * mul) {
	cplx tmp;
	tmp.Re = (res->Re * mul->Re - res->Im * mul->Im); // a*c - b*d
	tmp.Im = (res->Re * mul->Im + res->Im * mul->Re); // a*d + b*c
	res->Re = tmp.Re;
	res->Im = tmp.Im;
}

/**
 * @brief:  cordic()
 *          This function represents cordic algorithm
 *          for sin and cos calculation
 *
 * @param:  theta
 * @return: y
 */

int cordic(int theta) {
	int k, tx, ty;
	int x = cordic_1K, y = 0, z = theta;

	if ((z >= half_pi) || (z < -half_pi))
		z = (half_pi << 1) - z;
	for (k = 0; k < 32; ++k) // 32bit
			{
		if (z >= 0) {
			tx = x - (y >> k);
			ty = y + (x >> k);
			z = z - cordic_ctab[k];
			x = tx;
			y = ty;
		} else {
			tx = x + (y >> k);
			ty = y - (x >> k);
			z = z + cordic_ctab[k];
			x = tx;
			y = ty;
		}
	}
	return (y);
	//*c = x; *s = y;
}


/**
 * @brief:  absolute()
 *          This function returns absolute value of x
 *
 * @param:  x
 * @return: x
 */

float absolute(float x) {
	if (x < 0)
		return (-x);
	return (x);
}

/**
 * @brief:  square()
 *          This function calculates guess value while circle "while" is true
 *
 * @param:  x
 * @return: guess
 */

float square(float x) {
	float guess = 1;
	int lim = 40;

	while ((absolute(guess * guess - x) >= 1e-40) && (lim-- > 0))
		guess = ((x / guess) + guess) * 0.5;

	return (guess);
}

/**
 * @brief:  getSigma()
 *          This function calculates variable sum
 *
 * @param:  * data, n
 * @return: the call of "square(sum / (float) n)" function
 */

float getSigma(float * data, int n) {
	float mean = getMean(data, n);
	float sum = 0.0;

	for (int i = 0; i < n; i++) {
		sum += (data[i] - mean) * (data[i] - mean);
	}

	return square(sum / (float) n);
}

/**
 * @brief:  getMean()
 *          This function calculates variable sum
 *
 * @param:  * data, n
 * @return: sum / (float) n, * data
 */

float getMean(float * data, int n) {
	float sum = 0.0;

	for (int i = 0; i < n; i++) {
		sum += data[i];
	}
	return (sum / (float) n);
}


