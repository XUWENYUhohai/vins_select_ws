#pragma once
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

template <typename T> inline
void QuaternionInverse(const T q[4], T q_inverse[4])
{
	q_inverse[0] = q[0];
	q_inverse[1] = -q[1];
	q_inverse[2] = -q[2];
	q_inverse[3] = -q[3];
};


/**
 * 同一时刻vins与里程计位置对齐，约束
*/
struct OdomError
{
	OdomError(double t_x, double t_y, double t_z, double q_w, double q_x, double q_y, double q_z)
				  :t_x(t_x), t_y(t_y), t_z(t_z), q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z){}

	template <typename T>
	bool operator()(const T* tj , T* residuals) const//todo(改了)
	{
		residuals[0] = T(1) * (tj[0] - T(t_x));//! 0.01  0.1  1  10 0.001  1000000   0.000000001
		residuals[1] = T(1) * (tj[1] - T(t_y));
		residuals[2] = T(1) * (tj[2] - T(t_z));

        T w_q_odom[4];
		w_q_odom[0] = T(q_w);
		w_q_odom[1] = T(q_x);
		w_q_odom[2] = T(q_y);
		w_q_odom[3] = T(q_z);

        T odom_q_w[4];
        QuaternionInverse(w_q_odom,odom_q_w);

		T q_w_odom[4];
		q_w_odom[0] = tj[6];
		q_w_odom[1] = tj[3];
		q_w_odom[2] = tj[4];
		q_w_odom[3] = tj[5];

        T error[4];
        ceres::QuaternionProduct(odom_q_w, q_w_odom, error); 

        residuals[3] = T(1) * error[1];
        residuals[4] = T(1) * error[2];
        residuals[5] = T(1) * error[3];

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z, const double q_w, const double q_x, const double q_y, const double q_z) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          OdomError, 6, 7>(
	          	new OdomError(t_x, t_y, t_z, q_w, q_x, q_y, q_z)));
	}

	double t_x, t_y, t_z, q_w, q_x, q_y, q_z;

};



/**
 * 约束相邻里程计之间的相对变换
*/
struct RelativeOdomError
{
	RelativeOdomError(double t_x, double t_y, double t_z, 
					double q_w, double q_x, double q_y, double q_z,
					double t_var, double q_var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), 
				   q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
				   t_var(t_var), q_var(q_var){}

	template <typename T>
	bool operator()(const T* ti, const T* tj, T* residuals) const
	{// 参数：i帧旋转，i帧平移，j帧旋转，j帧平移，残差 //todo(改了)
		T t_w_ij[3];
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		T w_q_i[4];
		w_q_i[0] = ti[6];
		w_q_i[1] = ti[3];
		w_q_i[2] = ti[4];
		w_q_i[3] = ti[5];


		T i_q_w[4];
		QuaternionInverse(w_q_i, i_q_w);

		// T t_i_ij[3];
		// ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);// 平移增量转换到i系

// 当前平移量与给定参考平移量之间的残差
		// residuals[0] = (t_w_ij[0] - T(t_x)) / T(t_var);
		// residuals[1] = (t_w_ij[1] - T(t_y)) / T(t_var);
		// residuals[2] = (t_w_ij[2] - T(t_z)) / T(t_var);



		T relative_q[4];
		relative_q[0] = T(q_w);
		relative_q[1] = T(q_x);
		relative_q[2] = T(q_y);
		relative_q[3] = T(q_z);

		T w_q_j[4];
		w_q_j[0] = tj[6];
		w_q_j[1] = tj[3];
		w_q_j[2] = tj[4];
		w_q_j[3] = tj[5];

		//todo 11.17
		//first relative
		T j_q_w[4];
		QuaternionInverse(w_q_j, j_q_w);
		T q_tmp[4], t_tmp[3];
		ceres::QuaternionProduct(w_q_i,j_q_w,q_tmp);
		// ceres::QuaternionRotatePoint(q_tmp, t_w_ij, t_tmp);

		//second relative
		ceres::QuaternionRotatePoint(q_tmp, tj, t_tmp);
		//TODO


		T q_i_j[4];
		ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);

		T relative_q_inv[4];
		QuaternionInverse(relative_q, relative_q_inv);

		T error_q[4];
		ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q); 


		//todo
		//first
		// residuals[0] = (t_tmp[0] - T(t_x)) / T(t_var);
		// residuals[1] = (t_tmp[1] - T(t_y)) / T(t_var);
		// residuals[2] = (t_tmp[2] - T(t_z)) / T(t_var);
		//second
		residuals[0] = (t_tmp[0] - ti[0] - T(t_x)) / T(t_var);
		residuals[1] = (t_tmp[1] - ti[1] - T(t_y)) / T(t_var);
		residuals[2] = (t_tmp[2] - ti[2] - T(t_z)) / T(t_var);
		//todo
		
		residuals[3] = error_q[1] / T(q_var);
		residuals[4] = error_q[2] / T(q_var);
		residuals[5] = error_q[3] / T(q_var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double q_w, const double q_x, const double q_y, const double q_z,
									   const double t_var, const double q_var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          RelativeOdomError, 6, 7, 7>(
	          	new RelativeOdomError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
	}

	double t_x, t_y, t_z, t_norm;
	double q_w, q_x, q_y, q_z;
	double t_var, q_var;

};
