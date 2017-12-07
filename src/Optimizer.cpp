//
// Created by rain on 17-11-17.
//

#include "Optimizer.h"

namespace RAIN_VIO
{

Eigen::Vector2d Optimizer::ReprojectionError(const ceres::Problem& problem, ceres::ResidualBlockId id)
{
    auto cost = problem.GetCostFunctionForResidualBlock(id);

    std::vector<double*> parameterBlocks;
    problem.GetParameterBlocksForResidualBlock(id, &parameterBlocks);

    Eigen::Vector2d residual;
    cost->Evaluate(parameterBlocks.data(), residual.data(), nullptr);

    return residual;
}

std::vector<double> Optimizer::GetReprojectionErrorNorms(const ceres::Problem& problem)
{
    std::vector<double> result;
    std::vector<ceres::ResidualBlockId> ids;

    problem.GetResidualBlocks(&ids);

    for (auto& id : ids)
    {
        result.push_back(ReprojectionError(problem, id).norm());
    }

    return result;
}

void Optimizer::RemoveOutliers(ceres::Problem& problem, double threshold)
{
    std::vector<ceres::ResidualBlockId> ids;
    problem.GetResidualBlocks(&ids);

    for (auto & id: ids)
    {
        if (ReprojectionError(problem, id).norm() > threshold)
        {
            problem.RemoveResidualBlock(id);
        }
    }
}

void Optimizer::PoseOptimization(int IdxWin, Frame *pFrame, Map *pMap)
{
    ceres::Problem problem;
    ceres::LocalParameterization* localParameterization = new ceres::QuaternionParameterization();

    double Rdcw[4];
    double tdcw[4];
    double Point3dw[3];

//    Rdcw = &Eigen::Quaterniond(pFrame->GetRotation()).x();

    Rdcw[3] = Eigen::Quaterniond(pFrame->GetRotation()).w();
    Rdcw[0] = Eigen::Quaterniond(pFrame->GetRotation()).x();
    Rdcw[1] = Eigen::Quaterniond(pFrame->GetRotation()).y();
    Rdcw[2] = Eigen::Quaterniond(pFrame->GetRotation()).z();

    tdcw[0] = pFrame->GetTranslation()[0];
    tdcw[1] = pFrame->GetTranslation()[1];
    tdcw[2] = pFrame->GetTranslation()[2];

    problem.AddParameterBlock(Rdcw, 4, localParameterization);
    problem.AddParameterBlock(tdcw, 3);

    for (auto &MapPoint : pMap->mlMapPoints)
    {
        if (!(MapPoint.mdEstimatedDepth > 0))
            continue;

        if ((MapPoint.mnUsedNum >= 2 && MapPoint.mnStartFrame < gWindowSize-1 && MapPoint.mnStartFrame < IdxWin && MapPoint.EndFrame() > IdxWin))
        {
            FeaturePerFrame featurePerFrame = MapPoint.mvFeaturePerFrame.at((size_t)(IdxWin-MapPoint.mnStartFrame));

            ceres::CostFunction* costFunction = ReprojectionError2::Create(featurePerFrame.Point[0], featurePerFrame.Point[1]);

            Point3dw[0] = MapPoint.mPoint3d[0];
            Point3dw[1] = MapPoint.mPoint3d[1];
            Point3dw[2] = MapPoint.mPoint3d[2];

            problem.AddResidualBlock(costFunction, nullptr, Rdcw, tdcw, Point3dw);
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_solver_time_in_seconds = 0.3;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    LOG(INFO) << "the " << IdxWin  << " frame: " <<  endl;
    LOG(INFO) << summary.BriefReport()
              << " residuals number: " << summary.num_residuals << endl
              << " Initial RMSE: " << sqrt(summary.initial_cost / summary.num_residuals) << endl
              << " Final RMSE: " << sqrt(summary.final_cost / summary.num_residuals) << endl
              << " Time (s): " << summary.total_time_in_seconds << endl;
}

void Optimizer::ComputeReprojectionCost(int IdxWin, Frame *pFrame, Map *pMap)
{
    double cost=0;
    vector<Eigen::Vector2d> vresiduals;

    vresiduals.resize(pFrame->mvFraPointsPts.size());

    double Point3dw[3];
    double Point3dw0[3];
    double Point3dw1[3];

    size_t i=0;

    for (auto &MapPoint : pMap->mlMapPoints)
    {
        if (!(MapPoint.mdEstimatedDepth > 0))
            continue;

        if ((MapPoint.mnUsedNum >= 2 && MapPoint.mnStartFrame < gWindowSize-1 && MapPoint.mnStartFrame < IdxWin && MapPoint.EndFrame() > IdxWin))
        {
            FeaturePerFrame featurePerFrame = MapPoint.mvFeaturePerFrame.at((size_t)(IdxWin-MapPoint.mnStartFrame));

            // 2d feature point

            Point3dw[0] = MapPoint.mPoint3d[0];
            Point3dw[1] = MapPoint.mPoint3d[1];
            Point3dw[2] = MapPoint.mPoint3d[2];

            ceres::QuaternionRotatePoint(&Eigen::Quaterniond(pFrame->GetRotation()).x(), Point3dw, Point3dw1);

            Point3dw1[0] += pFrame->GetTranslation()[0];
            Point3dw1[1] += pFrame->GetTranslation()[1];
            Point3dw1[2] += pFrame->GetTranslation()[2];

            double xp = Point3dw1[0]/Point3dw1[2];
            double yp = Point3dw1[1]/Point3dw1[2];

            vresiduals[i][0] = xp - featurePerFrame.Point[0];
            vresiduals[i][1] = yp - featurePerFrame.Point[1];

            i++;
        }
    }

    for (auto residul:vresiduals)
    {
        cost += (residul[0]*residul[0] + residul[1]*residul[1]);
    }

    cost *= 0.5;

     cout << "cost: " << cost << endl;
}

} // namespace RAIN_VIO
