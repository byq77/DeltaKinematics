/*
 * DeltaKinematics - kinematics computation for delta parallel robot arm
 * Copyright(C) 2017  Szymon Szantula
 *
 * This program is free software : you can redistribute it and / or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.If not, see <http://www.gnu.org/licenses/.>
 */

#ifndef __DELTAKINEMATICS_H__
#define __DELTAKINEMATICS_H__

/**
 * @brief For Visual Sudio only
 */
#define _USE_MATH_DEFINES
#include <cmath>

 /**
 * @brief PI value
 */
#define M_PI 3.14159265358979323846

/** 
 * @brief Structure that represents basic vector.
 *
 * This structure might be used to describe tcp position, velocity and acceleration.
 * The below description of structure members refer to tcp position.
 * @sa DeltaTrajectory
 */
struct DeltaVector
{
	float x; /*!< cartesian position in base reference frame */
	float y; /*!< cartesian position in base reference frame */
	float z; /*!< cartesian position in base reference frame */
	float phi1; /*!< joint 1 angle [deg] ( negative above the base platform ! ) */
	float phi2; /*!< joint 2 angle [deg] ( negative above the base platform ! ) */
	float phi3; /*!< joint 3 angle [deg] ( negative above the base platform ! ) */
};

/** 
 * @brief Struct that represents element of trajectory.
 *
 * Trajectory is a time history of position, velocity and acceleration for each DOF.
 */
struct DeltaTrajectory
{
	DeltaVector pos; /*!< tcp position */
	DeltaVector vel; /*!< tcp velocity */
	DeltaVector accel; /*!< tcp acceleration */
};

/** 
 * @brief Delta Robot computation class.
 *
 * This class performs IPK and FPK computations for delta parallel robot with revolute inputs.
 */
class DeltaKinematics
{
	public:
	
		/** 
		 * @brief Delta Robot geometric dimmensions and constraints struct.
		 *
		 * The required input for the class. See the description of individual members.
		 */	
		struct DeltaGeometricDim
		{
			float sb; //!< base equilateral triangle side [ mm ]
			float sp; //!< platform equilateral triangle side [ mm ]
			float L; //!< upper legs length [ mm ]
			float l; //!< lower legs parallelogram length [ mm ]
			float h; //!< lower legs prallelogram width [ mm ]
			float max_neg_angle; //!< max negative angle that each arm can achive ( knee above the fixed-base plane ) [ deg ]
			float min_parallelogram_angle; //!< the limitation introduced by universal joints [ deg ]
		};

		/** 
		 * @brief DeltaKinematics constructor.
		 *
		 * @param dim a 7 element struct of delta geometric features and constraints.
		 */	
		DeltaKinematics(DeltaGeometricDim dim);
		
		/**
		* @brief Inverse position kinematics function.
		*
		* This function calculates ipk for delta parallel manipulator.
		*
		* @param v a pointer to the matrix of position vectors, only joint coordinates are changed
		* @param num a number of vectors in the matrix
		* @return 1 if 1 if there was unreachable position, 0 if success
		* @sa CalculateFpk
		*/		
		int CalculateIpk(DeltaVector * v, int num);

		/**
		* @brief Forward position kinematics function.
		*
		* This function calculates fpk for delta parallel manipulator. 
		*
		* @param v a pointer to the matrix of position vectors, only cartesian coordinates are changed
		* @param num a number of vectors in the matrix
		* @return 1 if there was unreachable position or if singularity happend, 0 if success
		* @sa CalculateIpk
		*/
		int CalculateFpk(DeltaVector * v, int num);

	private:
		enum
		{
			_DIAGONAL_MATRIX,
			_STANDART_MATRIX
		};

		float _sb; //!< base equilateral triangle side [ mm ]
		float _sp; //!< platform equilateral triangle side [ mm ]
		float _Ll;  //!< upper legs length [ mm ]
		float _l;  //!< lower legs parallelogram length [ mm ]
		float _h;  //!< lower legs prallelogram width [ mm ]
		float _wb; //!< planar distance from base reference frame to near base side [ mm ]
		float _ub; //!< planar distance from base reference frame to a base vertex [ mm ]
		float _wp; //!< planar distance from platform reference frame to near platform side [ mm ]
		float _up; //!< planar distance from platform reference frame to a platform vertex [ mm ]
		float _Pp1[3]; //!< platorm-fixed U-joint virtual connection in the local platform frame 
		float _Pp2[3]; //!< platorm-fixed U-joint virtual connection in the local platform frame 
		float _Pp3[3]; //!< platorm-fixed U-joint virtual connection in the local platform frame 
		float _B1[3]; //!< fixed-base revolute joint point 
		float _B2[3]; //< fixed-base revolute joint point 
		float _B3[3]; //!< fixed-base revolute joint point 
		float _b1[3]; //!< fixed-base vertex 
		float _b2[3]; //!< fixed-base vertex 
		float _b3[3]; //!< fixed-base vertex 
		float _A1[3]; //!< first knee point
		float _A2[3]; //!< second knee point
		float _A3[3]; //!< third knee point
		float _max_neg_angle; //!< max negative angle that each arm can achive ( knee above the fixed-base plane ) [ deg ]
		float _min_parallelogram_angle; //!< the limitation introduced by universal joints [ deg ]
		static const float _SQRT3; //!< sqrt(3)
		static const float _HSQRT3; //!< sqrt(3)/2;
		static const float _ROTZ120[3][3]; //<! basic rotation matrix for z axis and 120 degree
		static const float _MROTZ120[3][3]; //<! basic roation matrix for z axis and -120 degree
		static const float _DEG2RAD_FACTOR; //<! basicly equals M_PI/180
		static const float _RAD2DEG_FACTOR; //<! basilcy equals 180/M_PI
		void Initialise(); //<! internal function that initialise all above 3 element vectors
		void RotateByMatrix(float * point, const float(*matrix)[3]); //<! internal function that performs vector rotation by given rot matrix
		int CalculateAngle(const float * B, const float * P, float * phi); //<! internal function that calculates the joint angle
		int ThreeSpheresIntersectionA(DeltaVector * v); //<! internal function
		int ThreeSpheresIntersectionB(DeltaVector * v); //<! internal function -> all z hights the same
		/**
		 * @brief Internal function that calculates matrix derivative using Sarus method.
		 */
		// float CalculateDetSarus(int type = _STANDART_MATRIX);
};

#endif /* __DELTAKINEMATICS_H__ */