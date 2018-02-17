// DeltaKinematics - kinematics computation for delta parallel robot arm
// Copyright(C) 2018  Szymon Szantula
//
// This program is free software : you can redistribute it and / or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.If not, see <http://www.gnu.org/licenses/.> 

#include "DeltaKinematics.h"

template <typename RealDataType>
const RealDataType DeltaKinematics<RealDataType>::_SQRT3 = std::sqrt(3.0);

template <typename RealDataType>
const RealDataType DeltaKinematics<RealDataType>::_HSQRT3 = std::sqrt(3.0)/ 2.0;

template <typename RealDataType>
const RealDataType DeltaKinematics<RealDataType>::_DEG2RAD_FACTOR = M_PI / 180.0;

template <typename RealDataType>
const RealDataType DeltaKinematics<RealDataType>::_RAD2DEG_FACTOR = 180.0 / M_PI;

template <typename RealDataType>
const RealDataType DeltaKinematics<RealDataType>::_MROTZ120[3][3] = {{-0.5, 0.866025403784439, 0.0},
																	 {-0.866025403784439, -0.5, 0.0},
																	 {0.0, 0.0, 1.0}};

template <typename RealDataType>
const RealDataType DeltaKinematics<RealDataType>::_ROTZ120[3][3] = {{-0.5, -0.866025403784439, 0.0},
																	{0.866025403784439, -0.5, 0.0},
																	{0.0, 0.0, 1.0}};

template <typename RealDataType>
DeltaKinematics<RealDataType>::DeltaKinematics(DeltaGeometricDim dim)
{
	_sb = dim.sb;
	_sp = dim.sp;
	_Ll = dim.L;
	_l = dim.l;
	_h = dim.h;
	_max_neg_angle = dim.max_neg_angle;
	_min_parallelogram_angle = dim.min_parallelogram_angle;
	Initialise();
}

template <typename RealDataType>
void DeltaKinematics<RealDataType>::Initialise()
{
	_wb = _SQRT3 / 6 * _sb;
	_ub = _SQRT3 / 3 * _sb;
	_wp = _SQRT3 / 6 * _sp;
	_up = _SQRT3 / 3 * _sp;
	_B1[0] = 0;
	_B1[1] = -_wb;
	_B1[2] = 0;
	_B2[0] = _HSQRT3 * _wb;
	_B2[1] = _wb / 2;
	_B2[2] = 0;
	_B3[0] = -_HSQRT3 * _wb;
	_B3[1] = _wb / 2;
	_B3[2] = 0;
	_Pp1[0] = 0;
	_Pp1[1] = -_up;
	_Pp1[2] = 0;
	_Pp2[0] = _sp / 2;
	_Pp2[1] = -_wp;
	_Pp2[2] = 0;
	_Pp3[0] = -_sp / 2;
	_Pp3[1] = -_wp;
	_Pp3[2] = 0;
	_b1[0] = _sb / 2;
	_b1[1] = -_wb;
	_b1[2] = 0;
	_b2[0] = 0;
	_b2[1] = -_ub;
	_b2[2] = 0;
	_b3[0] = -_sb / 2;
	_b3[1] = -_wb;
	_b3[2] = 0;
}

template <typename RealDataType>
void DeltaKinematics<RealDataType>::RotateByMatrix(RealDataType *point, const RealDataType (*matrix)[3])
{
	RealDataType temp[3];
	for (int i = 0; i < 3; ++i)
	{
		temp[i] = matrix[i][0] * point[0] + matrix[i][1] * point[1] + matrix[i][2] * point[2];
	}
	point[0] = temp[0];
	point[1] = temp[1];
	point[2] = temp[2];
}

template <typename RealDataType>
int DeltaKinematics<RealDataType>::CalculateIpk(DeltaVector *v, int num)
{
	RealDataType temp_p[3];
	RealDataType temp_phi;
	int flag;

	for (int i = 0; i < num; ++i)
	{
		/* compute p1*/
		temp_p[0] = _Pp1[0] + v[i].x;
		temp_p[1] = _Pp1[1] + v[i].y;
		temp_p[2] = _Pp1[2] + v[i].z;
		/* calculate angle */
		flag = CalculateAngle(_b1, temp_p, &temp_phi);
		if (flag)
		{
			return 1;
		}
		else
		{
			v[i].phi1 = temp_phi;
		}

		/* compute p2 and change reference system */
		temp_p[0] = v[i].x;
		temp_p[1] = v[i].y;
		temp_p[2] = v[i].z;
		RotateByMatrix(temp_p, _MROTZ120);
		temp_p[0] += _Pp1[0];
		temp_p[1] += _Pp1[1];
		temp_p[2] += _Pp1[2];
		/* check all conditions */
		flag = CalculateAngle(_b1, temp_p, &temp_phi);
		if (flag)
		{
			return 1;
		}
		else
		{
			v[i].phi2 = temp_phi;
		}

		/* compute p3 change reference system */
		temp_p[0] = v[i].x;
		temp_p[1] = v[i].y;
		temp_p[2] = v[i].z;
		RotateByMatrix(temp_p, _ROTZ120);
		temp_p[0] += _Pp1[0];
		temp_p[1] += _Pp1[1];
		temp_p[2] += _Pp1[2];
		/* check all conditions */
		flag = CalculateAngle(_b1, temp_p, &temp_phi);
		if (flag)
		{
			return 1;
		}
		else
		{
			v[i].phi3 = temp_phi;
		}
	}

	return 0;
}

template <typename RealDataType>
int DeltaKinematics<RealDataType>::CalculateFpk(DeltaVector *v, int num)
{
	_A1[0] = 0.0;
	RealDataType hsp = _sp / 2.0;
	int flag;
	for (int i = 0; i < num; ++i)
	{
		_A1[1] = -_wb - _Ll * std::cos(_DEG2RAD_FACTOR * v[i].phi1) + _up;
		_A1[2] = -_Ll * std::sin(_DEG2RAD_FACTOR * v[i].phi1);
		_A2[0] = _HSQRT3 * (_wb + _Ll * std::cos(_DEG2RAD_FACTOR * v[i].phi2)) - hsp;
		_A2[1] = 0.5 * (_wb + _Ll * std::cos(_DEG2RAD_FACTOR * v[i].phi2)) - _wp;
		_A2[2] = -_Ll * std::sin(_DEG2RAD_FACTOR * v[i].phi2);
		_A3[0] = -_HSQRT3 * (_wb + _Ll * std::cos(_DEG2RAD_FACTOR * v[i].phi3)) + hsp;
		_A3[1] = 0.5 * (_wb + _Ll * std::cos(_DEG2RAD_FACTOR * v[i].phi3)) - _wp;
		_A3[2] = -_Ll * std::sin(_DEG2RAD_FACTOR * v[i].phi3);
		if (_A1[2] == _A2[2] && _A2[2] == _A3[2])
		{
			flag = ThreeSpheresIntersectionB(&v[i]);
		}
		else
		{
			flag = ThreeSpheresIntersectionA(&v[i]);
		}
		if (flag)
		{
			return flag;
		}
	}
	return 0;
}

template <typename RealDataType>
int DeltaKinematics<RealDataType>::ThreeSpheresIntersectionA(DeltaVector *v)
{
	RealDataType a11, a12, a13, a21, a22, a23, b1, b2, a1, a2, a3, a4, a5, a6, a7, a, b, c, delta;
	a11 = 2 * (_A3[0] - _A1[0]);
	a12 = 2 * (_A3[1] - _A1[1]);
	a13 = 2 * (_A3[2] - _A1[2]);
	if (a13 == 0.0)
		return 1;
	a21 = 2 * (_A3[0] - _A2[0]);
	a22 = 2 * (_A3[1] - _A2[1]);
	a23 = 2 * (_A3[2] - _A2[2]);
	if (a23 == 0.0)
		return 1;
	RealDataType A3_pow2 = _A3[0] * _A3[0] + _A3[1] * _A3[1] + _A3[2] * _A3[2];
	b1 = -_A1[0] * _A1[0] - _A1[1] * _A1[1] - _A1[2] * _A1[2] + A3_pow2;
	b2 = -_A2[0] * _A2[0] - _A2[1] * _A2[1] - _A2[2] * _A2[2] + A3_pow2;
	a1 = a11 / a13 - a21 / a23;
	a2 = a12 / a13 - a22 / a23;
	a3 = b2 / a23 - b1 / a13;
	if (a1 == 0.0)
		return 1;
	a4 = -a2 / a1;
	a5 = -a3 / a1;
	a6 = (-a21 * a4 - a22) / a23;
	a7 = (b2 - a21 * a5) / a23;
	a = a4 * a4 + 1 + a6 * a6;
	if (a == 0.0)
		return 1;
	b = 2 * a4 * (a5 - _A1[0]) - 2 * _A1[1] + 2 * a6 * (a7 - _A1[2]);
	c = a5 * (a5 - 2 * _A1[0]) + a7 * (a7 - 2 * _A1[2]) + _A1[0] * _A1[0] + _A1[1] * _A1[1] + _A1[2] * _A1[2] - _l * _l;
	if ((delta = b * b - 4 * a * c) < 0.0)
		return 1;
	DeltaVector pp1, pp2;
	pp1.y = (-b + std::sqrt(delta)) / (2 * a);
	pp1.x = a4 * pp1.y + a5;
	pp1.z = a6 * pp1.y + a7;
	pp2.y = (-b - std::sqrt(delta)) / (2 * a);
	pp2.x = a4 * pp2.y + a5;
	pp2.z = a6 * pp2.y + a7;
	if (pp1.z < 0.0 && CalculateIpk(&pp1, 1) == 0)
	{
		v->x = pp1.x;
		v->y = pp1.y;
		v->z = pp1.z;
		return 0;
	}
	else if (pp2.z < 0.0 && CalculateIpk(&pp2, 1) == 0)
	{
		v->x = pp2.x;
		v->y = pp2.y;
		v->z = pp2.z;
		return 0;
	}
	else
	{
		return 1;
	}
}

template <typename RealDataType>
int DeltaKinematics<RealDataType>::ThreeSpheresIntersectionB(DeltaVector *v)
{
	RealDataType a, b, c, d, e, f, zn, B, C, delta;
	zn = _A1[2];
	a = 2 * (_A3[0] - _A1[0]);
	b = 2 * (_A3[1] - _A1[1]);
	d = 2 * (_A3[0] - _A2[0]);
	e = 2 * (_A3[1] - _A2[1]);
	RealDataType A3_pow2 = _A3[0] * _A3[0] + _A3[1] * _A3[1];
	c = -_A1[0] * _A1[0] - _A1[1] * _A1[1] + A3_pow2;
	f = -_A2[0] * _A2[0] - _A2[1] * _A2[1] + A3_pow2;
	DeltaVector pp;
	RealDataType denom = a * e - b * d;
	if (denom == 0.0)
		return 1;
	pp.x = (c * e - b * f) / denom;
	pp.y = (a * f - c * d) / denom;
	B = -2*zn;
	C = zn * zn - _l * _l + (pp.x - _A1[0]) * (pp.x - _A1[0]) + (pp.y - _A1[1]) * (pp.y - _A1[1]);
	delta = B * B - 4 * C;
	if (delta < 0.0)
		return 1;
	RealDataType z1, z2;
	z1 = (-B + std::sqrt(delta)) / 2;
	z2 = (-B - std::sqrt(delta)) / 2;
	pp.z = z1;
	if (pp.z < 0.0 && CalculateIpk(&pp, 1) == 0)
	{
		v->x = pp.x;
		v->y = pp.y;
		v->z = pp.z;
		return 0;
	}
	pp.z = z2;
	if (pp.z < 0.0 && CalculateIpk(&pp, 1) == 0)
	{
		v->x = pp.x;
		v->y = pp.y;
		v->z = pp.z;
		return 0;
	}
	return 1;
}

template <typename RealDataType>
int DeltaKinematics<RealDataType>::CalculateAngle(const RealDataType *B, const RealDataType *P, RealDataType *phi)
{
	/* length of projection of vector AP on yz plane */
	RealDataType lyz = _l * _l - P[0] * P[0];
	if (lyz <= 0.0)
		return 1;
	lyz = std::sqrt(lyz);

	/* check if gamma angle is correct */
	if (P[0] != 0.0)
	{
		if (_RAD2DEG_FACTOR * std::atan(lyz / std::abs(P[0])) < _min_parallelogram_angle)
			return 1;
	}

	/* compute vector BP -> both points B and P should be reduced to 2D points (yz) */
	RealDataType vector_bp[2];
	vector_bp[0] = P[1] - B[1];
	vector_bp[1] = P[2] - B[2];

	/* check if vector points in right direction */
	if (vector_bp[1] >= 0)
		return 1;

	/* check if the vector have right size */
	RealDataType d = std::sqrt(vector_bp[1] * vector_bp[1] + vector_bp[0] * vector_bp[0]);

	if (d >= lyz + _Ll || d <= std::abs(lyz - _Ll))
		return 1;
	/* calculate alpha angle */
	RealDataType alpha = 180.0 + std::atan2(vector_bp[1], vector_bp[0]) * _RAD2DEG_FACTOR;

	/* calculate beta angle - cosine theorem */

	RealDataType beta = _RAD2DEG_FACTOR * std::acos((_Ll * _Ll + d * d - lyz * lyz) / (2 * _Ll * d));

	/* calculate phi angle and check max_neg_angle condition */
	*phi = alpha - beta;

	if (*phi < _max_neg_angle)
	{
		return 1;
	}

	/* if everything is OK */
	return 0;
}

template <typename RealDataType>
void DeltaKinematics<RealDataType>::DeltaVector::Print()
{
	#ifdef LIB_TEST_MODE
		std::cout << "x = " << x << " "
		<< "y = " << y << " "
		<< "z = " << z << "\n"
		<< "phi1 = " << phi1 << " "
		<< "phi2 = " << phi2 << " "
		<< "phi3 = " << phi3 << "\n";
	#endif
}
template <typename RealDataType>
void DeltaKinematics<RealDataType>::DeltaVector::Clear()
{
	x = y = z = phi1 = phi2 = phi3 = 0.0;
}	

/* Explicit instantations */

template class DeltaKinematics<float>;
template class DeltaKinematics<double>;
