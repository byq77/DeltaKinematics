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

#ifndef __DELTAKINEMATICS_H__
#define __DELTAKINEMATICS_H__

/**
 * @brief For Visual Sudio only
 */
#define _USE_MATH_DEFINES

#include <cmath>

#ifdef LIB_TEST_MODE
#include <iostream>
#endif

/**
 * @brief PI value
 */
#define M_PI 3.14159265358979323846

/** 
 * @brief Delta Robot computation class.
 *
 * Performs kinematics computation for delta parallel robot with revolute inputs.
 */

template <typename RealDataType>
class DeltaKinematics
{
  public:
	/** 
	* @brief Structure that represents basic vector.
	*
	* This structure might be used to describe tcp position, velocity and acceleration.
	* The below description of structure members refer to tcp position.
	* @sa DeltaTrajectory
	*/
	struct DeltaVector
	{
		RealDataType x;	///< cartesian position in base reference frame
		RealDataType y;	///< cartesian position in base reference frame
		RealDataType z;	///< cartesian position in base reference frame
		RealDataType phi1; ///< joint 1 angle [deg] ( negative above the base platform ! )
		RealDataType phi2; ///< joint 2 angle [deg] ( negative above the base platform ! )
		RealDataType phi3; ///< joint 3 angle [deg] ( negative above the base platform ! )

		/**
		 * @brief Prints values of all the position variables.
		 */
		void Print();

		/**
		 * @brief Sets all position parameters to zero;
		 */
		void Clear();
	};

	/** 
	* @brief Struct that represents element of trajectory.
	*
	* Trajectory is a time history of position, velocity and acceleration for each DOF.
	*/
	struct DeltaTrajectory
	{
		DeltaVector pos;   ///< tcp position
		DeltaVector vel;   ///< tcp velocity
		DeltaVector accel; ///< tcp acceleration
	};

	/** 
	 * @brief Delta Robot geometric dimmensions and constraints struct.
	 *
	 * The required input for the class. See the description of individual members.
	 */
	struct DeltaGeometricDim
	{
		RealDataType sb;					  ///< base equilateral triangle side [ mm ]
		RealDataType sp;					  ///< platform equilateral triangle side [ mm ]
		RealDataType L;						  ///< upper legs length [ mm ]
		RealDataType l;						  ///< lower legs parallelogram length [ mm ]
		RealDataType h;						  ///< lower legs prallelogram width [ mm ]
		RealDataType max_neg_angle;			  ///< max negative angle that each arm can achive ( knee above the fixed-base plane ) [ deg ]
		RealDataType min_parallelogram_angle; ///< the limitation introduced by universal joints [ deg ]
	};

	/** 
	 * @brief DeltaKinematics constructor.
	 *
	 * @param dim a 7 element struct of delta geometric features and constraints.
	 */
	DeltaKinematics(DeltaGeometricDim dim);

	/**
	 * @brief Calculates inverse position kinematics for delta parallel manipulator.
	 *
	 * @param v a pointer to the matrix of position vectors, only joint coordinates are changed
	 * @param num a number of vectors in the matrix
	 * @return 1 if 1 if there was unreachable position, 0 if success
	 * @sa CalculateFpk
	 */
	int CalculateIpk(DeltaVector *v, int num);

	/**
	 * @brief Calculates forward position kinematics for delta parallel manipulator.
	 *
	 * @param v a pointer to the matrix of position vectors, only cartesian coordinates are changed
	 * @param num a number of vectors in the matrix
	 * @return 1 if there was unreachable position or if singularity happend, 0 if success
	 * @sa CalculateIpk
	 */
	int CalculateFpk(DeltaVector *v, int num);

	// /**
	//  * @brief Calculates
	//  *
	//  *
	//  */
	// int CalculateCartesianVelocity( DeltaVector *v, int num);
	// int CalculateJointVelocity( DeltaVector *v, int num);

  private:
	RealDataType _sb;						   ///< base equilateral triangle side [ mm ]
	RealDataType _sp;						   ///< platform equilateral triangle side [ mm ]
	RealDataType _Ll;						   ///< upper legs length [ mm ]
	RealDataType _l;						   ///< lower legs parallelogram length [ mm ]
	RealDataType _h;						   ///< lower legs prallelogram width [ mm ]
	RealDataType _wb;						   ///< planar distance from base reference frame to near base side [ mm ]
	RealDataType _ub;						   ///< planar distance from base reference frame to a base vertex [ mm ]
	RealDataType _wp;						   ///< planar distance from platform reference frame to near platform side [ mm ]
	RealDataType _up;						   ///< planar distance from platform reference frame to a platform vertex [ mm ]
	RealDataType _Pp1[3];					   ///< platorm-fixed U-joint virtual connection in the local platform frame
	RealDataType _Pp2[3];					   ///< platorm-fixed U-joint virtual connection in the local platform frame
	RealDataType _Pp3[3];					   ///< platorm-fixed U-joint virtual connection in the local platform frame
	RealDataType _B1[3];					   ///< fixed-base revolute joint point
	RealDataType _B2[3];					   ///< fixed-base revolute joint point
	RealDataType _B3[3];					   ///< fixed-base revolute joint point
	RealDataType _b1[3];					   ///< fixed-base vertex
	RealDataType _b2[3];					   ///< fixed-base vertex
	RealDataType _b3[3];					   ///< fixed-base vertex
	RealDataType _A1[3];					   ///< first knee point
	RealDataType _A2[3];					   ///< second knee point
	RealDataType _A3[3];					   ///< third knee point
	RealDataType _max_neg_angle;			   ///< max negative angle that each arm can achive ( knee above the fixed-base plane ) [ deg ]
	RealDataType _min_parallelogram_angle;	 ///< the limitation introduced by universal joints [ deg ]
	static const RealDataType _SQRT3;		   ///< sqrt(3)
	static const RealDataType _HSQRT3;		   ///< sqrt(3)/2
	static const RealDataType _ROTZ120[3][3];  ///< basic rotation matrix for z axis and 120 degree
	static const RealDataType _MROTZ120[3][3]; ///< basic roation matrix for z axis and -120 degree
	static const RealDataType _DEG2RAD_FACTOR; ///< basicly equals M_PI/180
	static const RealDataType _RAD2DEG_FACTOR; ///< basilcy equals 180/M_PI

	/**
	 * @brief Initialise the class members e.g. _Pp1.
	 */
	void Initialise();

	/**
	 * @brief Performs vector rotation by given rotation matrix.
	 * 
	 * @param point a position vector
	 * @param matrix a rotation matrix
	 */
	void RotateByMatrix(RealDataType *point, const RealDataType (*matrix)[3]);

	/**
	 * @brief Calculates a joint angle .
	 *
	 * @param B a rotated base joint point
	 * @param P a rotated platform joint point
	 * @param phi an angle that will be calculated
	 * @return 0 if success 1 if error
	 */
	int CalculateAngle(const RealDataType *B, const RealDataType *P, RealDataType *phi);

	/**
	 * @brief Three spheres intersection algorithm ( different z hights ).
	 *
	 * @param v a position vector
	 * @return 0 if success 1 if error
	 */
	int ThreeSpheresIntersectionA(DeltaVector *v);

	/**
	 * @brief Three spheres intersection algorithm ( the same z hights ).
	 *
	 * @param v a position vector
	 * @return 0 if success 1 if error
	 */
	int ThreeSpheresIntersectionB(DeltaVector *v);
};
#endif /* __DELTAKINEMATICS_H__ */
