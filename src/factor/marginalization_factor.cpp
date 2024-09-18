/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "marginalization_factor.h"

void ResidualBlockInfo::Evaluate()
{
    residuals.resize(cost_function->num_residuals());

    std::vector<int> block_sizes = cost_function->parameter_block_sizes();  // 每个变量块中变量个数
    raw_jacobians = new double *[block_sizes.size()];
    jacobians.resize(block_sizes.size());

    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
    {
        jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);//resize(行，列)
        raw_jacobians[i] = jacobians[i].data();//    // 这里就是把jacobians每个matrix地址赋给raw_jacobians，然后把raw_jacobians传递给ceres的接口，这样计算结果直接放进了这个matrix
        //dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
    }
      // 执行一次优化，得到残差、Jacobian。注：因为Marg操作是在优化结束之后执行的，当前变量基本上已经是最优的了，这里再执行一次，残差、Jacobian都很小
    cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);//在c++11中，vector 增加了data()的用法，它返回内置vecotr所指的数组内存的第一个元素的指针.

    //std::vector<int> tmp_idx(block_sizes.size());
    //Eigen::MatrixXd tmp(dim, dim);
    //for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
    //{
    //    int size_i = localSize(block_sizes[i]);
    //    Eigen::MatrixXd jacobian_i = jacobians[i].leftCols(size_i);
    //    for (int j = 0, sub_idx = 0; j < static_cast<int>(parameter_blocks.size()); sub_idx += block_sizes[j] == 7 ? 6 : block_sizes[j], j++)
    //    {
    //        int size_j = localSize(block_sizes[j]);
    //        Eigen::MatrixXd jacobian_j = jacobians[j].leftCols(size_j);
    //        tmp_idx[j] = sub_idx;
    //        tmp.block(tmp_idx[i], tmp_idx[j], size_i, size_j) = jacobian_i.transpose() * jacobian_j;
    //    }
    //}
    //Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(tmp);
    //std::cout << saes.eigenvalues() << std::endl;
    //ROS_ASSERT(saes.eigenvalues().minCoeff() >= -1e-6);



    // 如果有核函数，那么就对残差进行相关调整
    if (loss_function)
    {
        double residual_scaling_, alpha_sq_norm_;

        double sq_norm, rho[3];

        sq_norm = residuals.squaredNorm();
        loss_function->Evaluate(sq_norm, rho);  // rho[0]:核函数这个点的值 rho[1]这个点的导数 rho[2]这个点的二阶导数
        //printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm, rho[0], rho[1], rho[2]);

        double sqrt_rho1_ = sqrt(rho[1]);

        if ((sq_norm == 0.0) || (rho[2] <= 0.0))// 柯西核p = log(s+1),rho[2]<＝0始终成立，一般核函数二阶导数都是小于0
        {
            residual_scaling_ = sqrt_rho1_;
            alpha_sq_norm_ = 0.0;
        }
        else
        {
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_ = alpha / sq_norm;
        }

        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
        { // 这里就相当于残差雅克比都乘上sqrt_rho1_，及核函数所在的点的一阶导，基本都是小于1
            jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
        }

        residuals *= residual_scaling_;
    }
}

MarginalizationInfo::~MarginalizationInfo()
{
    //ROS_WARN("release marginlizationinfo");
    
    for (auto it = parameter_block_data.begin(); it != parameter_block_data.end(); ++it)
        delete it->second;

    for (int i = 0; i < (int)factors.size(); i++)
    {

        delete[] factors[i]->raw_jacobians;
        
        delete factors[i]->cost_function;

        delete factors[i];
    }
}

/**
 * 添加与当前Marg帧有关联的残差块信息，后面用于计算Jacobian、残差、变量值
 * 1、factors 所有残差块
 *    1) 前一帧Marg留下的先验
 *    2) 当前Marg帧与后一帧IMU残差
 *    3) 当前Marg帧与滑窗内其他帧的视觉残差
 * 2、parameter_block_size <变量块起始地址, 变量块尺寸>
 * 3、parameter_block_idx <将要被丢弃的变量块起始地址，0>
*/
void MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo *residual_block_info)
{
    factors.emplace_back(residual_block_info);

    std::vector<double *> &parameter_blocks = residual_block_info->parameter_blocks;
    std::vector<int> parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();

    //reinterpret_cast<type-id> (expression)
    //type-id 必须是一个指针、引用、算术类型、函数指针或者成员指针。它可以把一个指针转换成一个整数，也可以把一个整数转换成一个指针（先把一个指针转换成一个整数，再把该整数转换成原类型的指针，还可以得到原先的指针值）。
    for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); i++)//即reinterpret_cast运算符允许将任意指针转换到其他指针类型，也允许做任意整数类型和任意指针类型之间的转换
    {
        double *addr = parameter_blocks[i];// 变量块起始地址
        int size = parameter_block_sizes[i]; // 变量块中参数的个数，记为尺寸
        parameter_block_size[reinterpret_cast<long>(addr)] = size;//将指针强转为数据的地址
    }

    for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); i++)
    {
        double *addr = parameter_blocks[residual_block_info->drop_set[i]];
        parameter_block_idx[reinterpret_cast<long>(addr)] = 0;
    }
}


// 之前通过调用 addResidualBlockInfo() 已经确定marg变量的数量、存储位置、长度以及待优化变量的数量以及存储位置，下面就需要调用 preMarginalize() 进行预处理
void MarginalizationInfo::preMarginalize()
{
    for (auto it : factors)
    {
        it->Evaluate();// 调用这个接口计算各个残差块的残差和雅克比矩阵

        std::vector<int> block_sizes = it->cost_function->parameter_block_sizes();// 得到每个残差块的参数块大小
        for (int i = 0; i < static_cast<int>(block_sizes.size()); i++)
        {
            long addr = reinterpret_cast<long>(it->parameter_blocks[i]); // 得到该参数块的地址
            int size = block_sizes[i];
            if (parameter_block_data.find(addr) == parameter_block_data.end())
            {
                double *data = new double[size];
                 // 深拷贝
                memcpy(data, it->parameter_blocks[i], sizeof(double) * size);//重新开辟一块内存,函数的功能是从源内存地址的起始位置开始拷贝若干个字节到目标内存地址中，即从源source中拷贝n个字节到目标destin中。
                parameter_block_data[addr] = data;//通过之前的优化变量的数据的地址和新开辟的内存数据进行关联
            }
        }
    }
}

int MarginalizationInfo::localSize(int size) const
{
    return size == 7 ? 6 : size;
}

int MarginalizationInfo::globalSize(int size) const
{
    return size == 6 ? 7 : size;
}

//分线程构造Ax = b
void* ThreadsConstructA(void* threadsstruct)
{
    ThreadsStruct* p = ((ThreadsStruct*)threadsstruct);
    for (auto it : p->sub_factors)
    {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
        {
            int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
            int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
            // 确保是local size
            if (size_i == 7)
                size_i = 6;
                       // 之前pre margin 已经算好了各个残差和雅克比，这里取出来
            Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
            {
                int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
                if (size_j == 7)
                    size_j = 6;
                Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                if (i == j)
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                else
                {
                    p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
              // ? : 为什么不是-JTb
            p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    return threadsstruct;
}

/**
 * 边缘化操作
 * 1、需要Marg的变量放前面，计算相关索引
 * 2、从Jacobian和Residual构造信息矩阵A = J^T·J， b = J^T·r，这个A、b没有保存，每次都是从Jacobian和Residual计算
 * 3、舒尔补边缘化
 *    A = Arr - Arm * Amm_inv * Amr;
 *    b = brr - Arm * Amm_inv * bmm;
 * 4、从A、b恢复Jacobian和residual，保存起来
*/
void MarginalizationInfo::marginalize()
{
    int pos = 0;//pos为所有变量维度
    for (auto &it : parameter_block_idx)
    {
        it.second = pos;// 这就是在所有参数中排序的idx，待边缘化的排在前面
        pos += localSize(parameter_block_size[it.first]); // 因为要进行求导，因此大小是local size，具体一点就是使用李代数
    }

    m = pos;// 总共待边缘化的参数块总大小（不是个数）

    // 其他参数块
    for (const auto &it : parameter_block_size)
    {
        if (parameter_block_idx.find(it.first) == parameter_block_idx.end())//如果这个变量不是是待marg的优化变量
        {
            parameter_block_idx[it.first] = pos;// 这样每个参数块的大小都能被正确找到
            pos += localSize(it.second);
        }
    }

    n = pos - m;
    //ROS_INFO("marginalization, pos: %d, m: %d, n: %d, size: %d", pos, m, n, (int)parameter_block_idx.size());
    if(m == 0)
    {
        valid = false;
        printf("unstable tracking...\n");
        return;
    }

    TicToc t_summing;
    Eigen::MatrixXd A(pos, pos);
    Eigen::VectorXd b(pos);
    A.setZero();
    b.setZero();
    /*
     //单线程构建A和b
    for (auto it : factors)
    {
        for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
        {
            int idx_i = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
            int size_i = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])]);
            Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
            for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++)
            {
                int idx_j = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
                int size_j = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])]);
                Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
                if (i == j)
                    A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                else
                {
                    A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
                    A.block(idx_j, idx_i, size_j, size_i) = A.block(idx_i, idx_j, size_i, size_j).transpose();
                }
            }
            b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
        }
    }
    ROS_INFO("summing up costs %f ms", t_summing.toc());
    */
    //multi thread

    // 往A矩阵和b矩阵中填东西，利用多线程加速
    //---------------函数会通过多线程快速构造各个残差对应的各个优化变量的信息矩阵（雅克比和残差前面都已经求出来了），然后在加起来.
    //---------------因为这里构造信息矩阵时采用的正是parameter_block_idx作为构造顺序，因此，就会自然而然地将待边缘化的变量构造在矩阵的左上方。
    TicToc t_thread_summing;
    pthread_t tids[NUM_THREADS];
    ThreadsStruct threadsstruct[NUM_THREADS];
    int i = 0;
    for (auto it : factors) // 每个线程均匀分配任务
    {
        threadsstruct[i].sub_factors.push_back(it);
        i++;
        i = i % NUM_THREADS;
    }
    for (int i = 0; i < NUM_THREADS; i++)
    {
        TicToc zero_matrix;
        threadsstruct[i].A = Eigen::MatrixXd::Zero(pos,pos);
        threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
          // 多线程访问会带来冲突，因此每个线程备份一下要查询的map
        threadsstruct[i].parameter_block_size = parameter_block_size;
        threadsstruct[i].parameter_block_idx = parameter_block_idx;
        //pthread_create是类Unix操作系统（Unix、Linux、Mac OS X等）的创建线程的函数。它的功能是创建线程（实际上就是确定调用该线程函数的入口点），在线程创建以后，就开始运行相关的线程函数。
        int ret = pthread_create( &tids[i], NULL, ThreadsConstructA ,(void*)&(threadsstruct[i]));//成功返回0
        if (ret != 0)
        {
            ROS_WARN("pthread_create error");
            ROS_BREAK();
        }
    }
    for( int i = NUM_THREADS - 1; i >= 0; i--)  
    {
        //如果你的主线程，也就是main函数执行的那个线程，在你其他线程退出之前就已经退出，那么带来的bug则不可估量。通过pthread_join函数会让主线程阻塞，直到所有线程都已经退出。
        //主线程等待子线程的终止。也就是在子线程调用了pthread_join()方法后面的代码，只有等到子线程结束了才能执行。
        pthread_join( tids[i], NULL ); 
        A += threadsstruct[i].A;
        b += threadsstruct[i].b;
    }
    //ROS_DEBUG("thread summing up costs %f ms", t_thread_summing.toc());
    //ROS_INFO("A diff %f , b diff %f ", (A - tmp_A).sum(), (b - tmp_b).sum());


    //TODO
    /**
     * |Amm  Amr|   |delta_xm|   |bmm|
     * |        | * |        | = |   |
     * |Arm  Arr|   |delta_xr|   |brr|
     * Marg之后的A：Arr - Arm * Amm_inv * Amr
     * Marg之后的b：brr - Arm * Amm_inv * bmm
     * Marg之后：A * delta_xr = b，将delta_xm对应的信息（约束）施加到A、b上了，下次优化的时候，加上这个约束项，就是先验餐残差
    */

    // Amm矩阵的构建是为了保证其正定性
    Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);//计算自伴矩阵的特征值和特征向量 .eigenvectors() :https://blog.csdn.net/qq_42679415/article/details/101939806

    //ROS_ASSERT_MSG(saes.eigenvalues().minCoeff() >= -1e-4, "min eigenvalue %f", saes.eigenvalues().minCoeff());
    // 一个逆矩阵的特征值是原矩阵的倒数，特征向量相同　select类似c++中 ? :运算符-> // (R < s ? P : Q)具体：http://eigen.tuxfamily.org/dox/classEigen_1_1DenseBase.html#a65e78cfcbc9852e6923bebff4323ddca
    // 利用特征值取逆来构造其逆矩阵
    //asDiagonal() 通过x中元素作为对角元素（构建对角阵）这里x必须为VectorXX类型不能为矩阵
    //对称阵不同的特征值对应的特征向量是相互正交的。
    //https://zhuanlan.zhihu.com/p/542970967
    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() * saes.eigenvectors().transpose();
    //printf("error1: %f\n", (Amm * Amm_inv - Eigen::MatrixXd::Identity(m, m)).sum());

    Eigen::VectorXd bmm = b.segment(0, m);
    Eigen::MatrixXd Amr = A.block(0, m, m, n);
    Eigen::MatrixXd Arm = A.block(m, 0, n, m);
    Eigen::MatrixXd Arr = A.block(m, m, n, n);
    Eigen::VectorXd brr = b.segment(m, n);
    A = Arr - Arm * Amm_inv * Amr;
    b = brr - Arm * Amm_inv * bmm;

    // 这个地方根据Ax = b => JT*J = - JT * e
    // 对A做特征值分解 A = V * S * VT,其中Ｓ是特征值构成的对角矩阵
    // 所以J = S^(1/2) * VT , 这样JT * J = (S^(1/2) * VT)T * S^(1/2) * VT = V * S^(1/2)T *  S^(1/2) * VT = V * S * VT(对角矩阵的转置等于其本身)
    // e = -(JT)-1 * b = - (S^-(1/2) * V^-1) * b = - (S^-(1/2) * VT) * b
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
    Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
    Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

    Eigen::VectorXd S_sqrt = S.cwiseSqrt();//求各个系数的开方,// 这个求得就是 S^(1/2)，不过这里是向量还不是矩阵
    Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();
 // 边缘化为了实现对剩下参数块的约束，为了便于一起优化，就抽象成了残差和雅克比的形式，这样也形成了一种残差约束
 // ceres优化处理的是Jacobian和Residual，所以这里不保存A、b，而是恢复出J、r保存，方便下一次ceres优化
    linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
    linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
    //std::cout << A << std::endl
    //          << std::endl;
    //std::cout << linearized_jacobians << std::endl;
    //printf("error2: %f %f\n", (linearized_jacobians.transpose() * linearized_jacobians - A).sum(),
    //      (linearized_jacobians.transpose() * linearized_residuals - b).sum());
}

//Marg最早帧p0 [<p1,p0>,<p2,p1>,...,<pn,pn-1>], Marg倒数第二帧pn-1 [<p0,p0>, <p1,p1>,...,<pn-1,pn-1>,<pn,pn-1>]
std::vector<double *> MarginalizationInfo::getParameterBlocks(std::unordered_map<long, double *> &addr_shift)
{
    std::vector<double *> keep_block_addr;
    keep_block_size.clear();
    keep_block_idx.clear();
    keep_block_data.clear();

    for (const auto &it : parameter_block_idx)
    {
        if (it.second >= m)// 如果是留下来的，说明后续会对其形成约束
        {
            keep_block_size.push_back(parameter_block_size[it.first]);// 留下来的参数块大小 global size
            keep_block_idx.push_back(parameter_block_idx[it.first]);
            keep_block_data.push_back(parameter_block_data[it.first]);
            keep_block_addr.push_back(addr_shift[it.first]);// 对应的新地址
        }
    }
      // 留下来的边缘化后的参数块总大小 //
    // 保留下来的变量块数量，13个，10个pose，1个外参，1个td，1个速度偏置
    sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);

    return keep_block_addr;
}

// 先验残差的损失函数
MarginalizationFactor::MarginalizationFactor(MarginalizationInfo* _marginalization_info):marginalization_info(_marginalization_info)
{
    int cnt = 0;
    for (auto it : marginalization_info->keep_block_size)
    {
        mutable_parameter_block_sizes()->push_back(it);// 调用ceres接口，添加参数块大小信息,//设定待优化变量各自的维度， 使用global size
        cnt += it;
    }
    //printf("residual size: %d, %d\n", cnt, n);
    set_num_residuals(marginalization_info->n);// 残差维数就是所有剩余状态量的维数和，这里时local size
};

//边缘化结果残差和雅克比的计算
bool MarginalizationFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    //printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(), num_residuals());
    //for (int i = 0; i < static_cast<int>(keep_block_size.size()); i++)
    //{
    //    //printf("unsigned %x\n", reinterpret_cast<unsigned long>(parameters[i]));
    //    //printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
    //printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
    //printf("residual %x\n", reinterpret_cast<long>(residuals));
    //}
    int n = marginalization_info->n; // 上一次边缘化保留的残差块的local size的和,也就是残差维数
    int m = marginalization_info->m;
    Eigen::VectorXd dx(n);  // 用来存储残差
    for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)//遍历几种变量块
    {
        int size = marginalization_info->keep_block_size[i];
        int idx = marginalization_info->keep_block_idx[i] - m;
        // 当前参数块的值（）新的
        Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);//Eigen::Map : https://zhuanlan.zhihu.com/p/348622852
        // 当时参数块的值，//上一次边缘化时刻状态向量的值
        Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size);//参数表示是7个维度，残差只用6个
        if (size != 7)
            dx.segment(idx, size) = x - x0;// 不需要local param的直接做差
        else// 代表位姿的param
        {
            dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>(); // 位移直接做差
            dx.segment<3>(idx + 3) = 2.0 * Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();    // 旋转就是李代数做差
            if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).w() >= 0))   // 确保实部大于0
            {
                dx.segment<3>(idx + 3) = 2.0 * -Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            }
        }
    }

    //残差的更新，边缘化后的先验误差  ep= e0 + J*dx
    // 个人理解：根据FEJ算法．雅克比保持不变，但是残差随着优化会变化，因此下面不更新雅克比　只更新残差
    //不更新雅克比的原因： https://blog.csdn.net/hltt3838/article/details/109577950  http://events.jianshu.io/p/b8b479c48d8f
    // 可以参考　https://blog.csdn.net/weixin_41394379/article/details/89975386
    Eigen::Map<Eigen::VectorXd>(residuals, n) = marginalization_info->linearized_residuals + marginalization_info->linearized_jacobians * dx;
    if (jacobians)
    {

        for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); i++)
        {
            if (jacobians[i])
            {
                int size = marginalization_info->keep_block_size[i], local_size = marginalization_info->localSize(size);
                int idx = marginalization_info->keep_block_idx[i] - m;
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], n, size);
                jacobian.setZero();
                jacobian.leftCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx, local_size);//雅克比由local size转为global size
            }
        }
    }
    return true;
}
