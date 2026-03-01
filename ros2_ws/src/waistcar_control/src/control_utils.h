#pragma once
#include <Eigen/Dense>
// #include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include <complex>
#include <iostream>

// Expand polynomial coefficients from a set of roots.
// roots: polynomial roots (complex).
// Returns: real coefficient vector of length n+1 for s^n + a_{n-1}s^{n-1} + ... + a_0.
inline Eigen::VectorXd polyFromRoots(const std::vector<std::complex<double>>& roots)
{
    int n = static_cast<int>(roots.size());
    std::vector<std::complex<double>> coeffs = {1.0}; // Start with the constant polynomial 1.

    // Multiply by (s - r_i) for each root.
    for (const auto& r : roots)
    {
        std::vector<std::complex<double>> new_coeffs(coeffs.size() + 1, 0.0);
        for (size_t i = 0; i < coeffs.size(); ++i)
        {
            new_coeffs[i]     -= r * coeffs[i];  // term from (-r) * s^i
            new_coeffs[i + 1] += coeffs[i];      // shift to s^{i+1}
        }
        coeffs = std::move(new_coeffs);
    }

    // Convert complex coefficients to real values (assumes imaginary parts are near zero).
    Eigen::VectorXd real_coeffs(n + 1);
    for (int i = 0; i <= n; ++i)
    {
        real_coeffs(i) = coeffs[i].real();
    }

    return real_coeffs;
}

// Compute controllability matrix C = [B, AB, A^2B, ..., A^{N-1}B].
template<int N>
Eigen::Matrix<double, N, N> computeControllabilityMatrix(const Eigen::Matrix<double, N, N>& A,
                                                         const Eigen::Matrix<double, N, 1>& B)
{
    Eigen::Matrix<double, N, N> ctrb;
    Eigen::Matrix<double, N, 1> AiB = B;
    for (int i = 0; i < N; ++i)
    {
        ctrb.col(i) = AiB;
        AiB = A * AiB;
    }
    return ctrb;
}

// Compute state-feedback gain K using Ackermann's formula.
template<int N>
Eigen::Matrix<double, 1, N> computeKWithAckermann(const Eigen::Matrix<double, N, N>& A,
                                                  const Eigen::Matrix<double, N, 1>& B,
                                                  const std::vector<std::complex<double>>& desired_poles)
{
    Eigen::VectorXd coeffs = polyFromRoots(desired_poles); // polynomial coefficients (length N+1)

    Eigen::Matrix<double, N, N> ctrb = computeControllabilityMatrix<N>(A, B);
    if (ctrb.determinant() == 0)
    {
        std::cerr << "Error: System is not controllable!" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Build phi(A) = A^n + a_{n-1}A^{n-1} + ... + a_0 I.
    Eigen::Matrix<double, N, N> phiA = Eigen::Matrix<double, N, N>::Zero();
    Eigen::Matrix<double, N, N> An = Eigen::Matrix<double, N, N>::Identity();
    for (int i = 0; i <= N; ++i)
    {
        phiA += coeffs(i) * An;
        An = An * A;
    }

    // Compute [0 0 ... 1] * C^{-1} * phi(A).
    Eigen::Matrix<double, 1, N> selector = Eigen::Matrix<double, 1, N>::Zero();
    selector(0, N - 1) = 1.0;

    Eigen::Matrix<double, 1, N> K = selector * ctrb.inverse() * phiA;
    return K;
}

// Convenience wrapper for pole placement of a 4th-order SISO system.
inline Eigen::Matrix<double, 1, 4> place_poles(const Eigen::Matrix<double, 4, 4>& A,
                                               const Eigen::Matrix<double, 4, 1>& B,
                                               const std::vector<std::complex<double>>& desired_poles)
{
    return computeKWithAckermann<4>(A, B, desired_poles);
}

// // (Optional) 4th-order MIMO pole placement wrapper (requires Eigen pole placement support).
// inline Eigen::Matrix<double, 3, 4> place_poles_43(const Eigen::Matrix<double, 4, 4>& A,
//                                                const Eigen::Matrix<double, 4, 3>& B,
//                                                const std::vector<std::complex<double>>& desired_poles)
// {
//     auto result = Eigen::place_poles(A, B, desired_poles);
//     return result.getGain();  // returns K (M×N)
// }

// Convenience wrapper for pole placement of a 3rd-order SISO system.
inline Eigen::Matrix<double, 1, 3> place_poles(const Eigen::Matrix<double, 3, 3>& A,
                                               const Eigen::Matrix<double, 3, 1>& B,
                                               const std::vector<std::complex<double>>& desired_poles)
{
    return computeKWithAckermann<3>(A, B, desired_poles);
}
