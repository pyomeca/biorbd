#define BIORBD_API_EXPORTS
#include "Utils/Rotation.h"

#include "Utils/Error.h"
#include "Utils/Vector3d.h"
#include "Utils/String.h"
#include "Utils/Vector.h"
#include "RigidBody/NodeSegment.h"

using namespace BIORBD_NAMESPACE;

utils::Rotation::Rotation(
    const utils::Matrix3d& matrix) :
    utils::Matrix3d(matrix)
{
    checkUnitary();
}

#ifdef BIORBD_USE_CASADI_MATH

utils::Rotation::Rotation(
    const RBDLCasadiMath::MX_Xd_static<3, 3> &other) :
    utils::Matrix3d(other)
{

}

utils::Rotation::Rotation(
    const RBDLCasadiMath::MX_Xd_SubMatrix& other) :
    utils::Matrix3d(other)
{

}
#endif

utils::Rotation::Rotation(
    const utils::Scalar& v00, const utils::Scalar& v01, const utils::Scalar& v02,
    const utils::Scalar& v10, const utils::Scalar& v11, const utils::Scalar& v12,
    const utils::Scalar& v20, const utils::Scalar& v21, const utils::Scalar& v22) :
    utils::Matrix3d (v00, v01, v02, v10, v11, v12, v20, v21, v22)
{

}

utils::Rotation::Rotation(
    const utils::Vector& rotation,
    const utils::String& rotationSequence) :
    utils::Matrix3d(fromEulerAngles(rotation, rotationSequence))
{

}

utils::Rotation::Rotation(
    const RigidBodyDynamics::Math::SpatialTransform &st) :
    utils::Matrix3d(st.E)
{

}

utils::Vector3d utils::Rotation::axe(unsigned int idx) const
{
    utils::Error::check(idx<=2, "Axis must be between 0 and 2 included");
    return this->block(0,idx, 3, 1);
}

utils::Rotation utils::Rotation::fromSpatialTransform(
    const RigidBodyDynamics::Math::SpatialTransform& st)
{
    return st.E;
}

utils::Rotation utils::Rotation::fromEulerAngles(
    const utils::Vector &rot,
    const utils::String& seq)
{
    // Check for size consistency
    utils::Error::check(
        seq.length() == static_cast<unsigned int>(rot.rows()),
        "Rotation and sequence of rotation must be the same length");

    utils::Rotation out;
    out.setIdentity();
    // Set the actual rotation matrix to this
    utils::Matrix3d tp;
    for (unsigned int i=0; i<seq.length(); ++i) {
        utils::Scalar cosVi(std::cos(rot[i]));
        utils::Scalar sinVi(std::sin(rot[i]));
        if (seq.tolower()[i] == 'x')
            tp = utils::Matrix3d(1,     0,      0,
                                 0, cosVi, -sinVi,
                                 0, sinVi,  cosVi);

        else if (seq.tolower()[i] == 'y')
            tp = utils::Matrix3d(cosVi, 0, sinVi,
                                 0, 1,     0,
                                 -sinVi, 0, cosVi);

        else if (seq.tolower()[i] == 'z')
            tp = utils::Matrix3d(cosVi, -sinVi,  0,
                                 sinVi,  cosVi,  0,
                                 0,      0,  1);
        else {
            utils::Error::raise("Rotation sequence not recognized");
        }

        out.block(0,0,3,3) = out.block(0,0,3,3) * tp;
    }
    return out;
}

utils::Rotation utils::Rotation::fromMarkersNonNormalized(
    const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment> &axis1markers,
    const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment> &axis2markers,
    const std::pair<utils::String, utils::String>& axesNames,
    const utils::String &axisToRecalculate)
{
    if (!axesNames.first.compare("") || !axesNames.second.compare("")) {
        utils::Error::raise("axesNames must be defined with a pair of \"x\", \"y\" or \"z\"");
    }

    // Figure out where to put the axes
    std::vector<unsigned int> map(3);
    std::vector<unsigned int> toMultiply(2);
    if (!axesNames.first.tolower().compare("x")) {
        map[0] = 0;
        if (!axesNames.second.tolower().compare("y")) {
            map[1] = 1;
            map[2] = 2;
            toMultiply[0] = 0;
            toMultiply[1] = 1;
        } else {
            map[1] = 2;
            map[2] = 1;
            toMultiply[0] = 2;
            toMultiply[1] = 0;
        }
    } else if (!axesNames.first.tolower().compare("y")) {
        map[0] = 1;
        if (!axesNames.second.tolower().compare("x")) {
            map[1] = 0;
            map[2] = 2;
            toMultiply[0] = 0;
            toMultiply[1] = 1;
        } else {
            map[1] = 2;
            map[2] = 0;
            toMultiply[0] = 1;
            toMultiply[1] = 2;
        }
    } else if (!axesNames.first.tolower().compare("z")) {
        map[0] = 2;
        if (!axesNames.second.tolower().compare("x")) {
            map[1] = 0;
            map[2] = 1;
            toMultiply[0] = 2;
            toMultiply[1] = 0;
        } else {
            map[1] = 1;
            map[2] = 0;
            toMultiply[0] = 1;
            toMultiply[1] = 2;
        }
    }

    // Get the system of axis XYZ
    std::vector<utils::Vector3d> axes(3);
    axes[map[0]] = axis1markers.second - axis1markers.first;
    axes[map[1]] = axis2markers.second - axis2markers.first;
    axes[map[2]] = axes[toMultiply[0]].cross(axes[toMultiply[1]]);

    // Recalculate one axis
    if (!axisToRecalculate.tolower().compare("x")) {
        axes[0] = axes[1].cross(axes[2]);
    } else if (!axisToRecalculate.tolower().compare("y")) {
        axes[1] = axes[2].cross(axes[0]);
    } else if (!axisToRecalculate.tolower().compare("z")) {
        axes[2] = axes[0].cross(axes[1]);
    }

    // Organize them in a non-normalized matrix
    utils::Rotation r_out;
    for (unsigned int i=0; i<3; ++i) {
        r_out.block(0, i, 3, 1) = axes[i];
    }
    return r_out;
}

utils::Rotation utils::Rotation::fromMarkers(
    const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment> &axis1markers,
    const std::pair<rigidbody::NodeSegment, rigidbody::NodeSegment> &axis2markers,
    const std::pair<utils::String, utils::String>& axesNames,
    const utils::String &axisToRecalculate)
{
    utils::Rotation r_out(
        fromMarkersNonNormalized(axis1markers, axis2markers, axesNames,
                                 axisToRecalculate));

    // Organize them in a normalized matrix
    for (unsigned int i=0; i<3; ++i) {
        // Normalize axes
        r_out.block<3, 1>(0, i).normalize();
    }

    return r_out;
}

utils::Vector utils::Rotation::toEulerAngles(
    const utils::Rotation &r,
    const utils::String &seq)
{
    utils::Vector v;
    if (!seq.compare("zyzz")) {
        v = utils::Vector(3);
    } else {
        v = utils::Vector(static_cast<unsigned int>(seq.length()));
    }

    if (!seq.compare("x")) {
        v[0] = std::asin(r(2, 1));           // x
    } else if (!seq.compare("y")) {
        v[0] = std::asin(r(0, 2));           // y
    } else if (!seq.compare("z")) {
        v[0] = std::asin(r(1, 0));           // z
    } else if (!seq.compare("xy")) {
        v[0] = std::asin(r(2,1));            // x
        v[1] = std::asin(r(0,2));            // y
    } else if (!seq.compare("xz")) {
        v[0] = -std::asin(r(1,2));           // x
        v[1] = -std::asin(r(0,1));           // z
    } else if (!seq.compare("yx")) {
        v[0] = -std::asin(r(2,0));           // y
        v[1] = -std::asin(r(1,2));           // x
    } else if (!seq.compare("yz")) {
        v[0] = std::asin(r(0,2));            // y
        v[1] = std::asin(r(1,0));            // z
    } else if (!seq.compare("zx")) {
        v[0] = std::asin(r(1,0));            // z
        v[1] = std::asin(r(2,1));            // x
    } else if (!seq.compare("zy")) {
        v[0] = -std::asin(r(0,1));           // z
        v[1] = -std::asin(r(2,0));           // y
    } else if (!seq.compare("xyz")) {
        v[0] = std::atan2(-r(1,2), r(2,2));  // x
        v[1] = std::asin(r(0,2));            // y
        v[2] = std::atan2(-r(0,1), r(0,0));  // z
    } else if (!seq.compare("xzy")) {
        v[0] = std::atan2(r(2,1), r(1,1));   // x
        v[1] = std::asin(-r(0,1));           // z
        v[2] = std::atan2(r(0,2), r(0,0));   // y
    } else if (!seq.compare("yxz")) {
        v[0] = std::atan2(r(0,2), r(2,2));   // y
        v[1] = std::asin(-r(1,2));           // x
        v[2] = std::atan2(r(1,0), r(1,1));   // z
    } else if (!seq.compare("yzx")) {
        v[0] = std::atan2(-r(2,0), r(0,0));  // y
        v[1] = std::asin(r(1,0));            // z
        v[2] = std::atan2(-r(1,2), r(1,1));  // x
    } else if (!seq.compare("zxy")) {
        v[0] = std::atan2(-r(0,1), r(1,1));  // z
        v[1] = std::asin(r(2,1));            // x
        v[2] = std::atan2(-r(2,0), r(2,2));  // y
    } else if (!seq.compare("zyx")) {
        v[0] = std::atan2(r(1,0), r(0,0));   // z
        v[1] = std::asin(-r(2,0));           // y
        v[2] = std::atan2(r(2,1), r(2,2));   // x
    } else if (!seq.compare("zxz")) {
        v[0] = std::atan2(r(0,2), -r(1,2));  // z
        v[1] = std::acos(r(2,2));            // x
        v[2] = std::atan2(r(2,0), r(2,1));   // z
    } else if (!seq.compare("zyz")) {
        v[0] = std::atan2(r(1,2), r(0,2));   // z
        v[1] = std::acos(r(2,2));            // y
        v[2] = std::atan2(r(2,1), -r(2,0));  // z
    } else if (!seq.compare("zyzz")) {
        v[0] = std::atan2(r(1,2), r(0,2));   // z
        v[1] = std::acos(r(2,2));            // y
        v[2] = std::atan2(r(2,1), -r(2,0)) + v[0];   // z+z
    } else {
        utils::Error::raise("Angle sequence is not recognized");
    }

    return v;
}

#ifndef BIORBD_USE_CASADI_MATH
utils::Rotation utils::Rotation::mean(
    const std::vector<utils::Rotation> & mToMean)
{
    utils::Matrix3d m_tp;
    m_tp.setZero();

    // Initial guess being the actual arithmetic mean
    for (unsigned int i = 0; i<mToMean.size(); ++i) {
        m_tp += mToMean[i];
    }
    m_tp = m_tp/mToMean.size();

    // SVD decomposition
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        m_tp, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Normalize the matrix
    utils::Rotation m_out(svd.matrixU() * svd.matrixV().transpose());

    return m_out;
}
#endif

void utils::Rotation::checkUnitary()
{
#ifndef BIORBD_USE_CASADI_MATH
#ifndef SKIP_ASSERT
    double sqrtNorm = static_cast<double>(this->squaredNorm());
    bool checkUnit(fabs(sqrtNorm - 3.) < 1e-4);
    if (!checkUnit)
    {

        // Code addapted from https://github.com/brainexcerpts/3x3_polar_decomposition/blob/master/src/polar_decomposition_3x3.inl

        // Placer inf_norm and one norm in a function
        // créer fonction polar_decomposition et déplacer dans Matrix3d

        utils::Scalar tolerance = 1e-6;
        utils::Matrix3d M = this->block(0, 0, 3, 3);
        utils::Matrix3d R;
        utils::Matrix3d S;
        utils::Matrix3d Mk;
        utils::Matrix3d Ek;
        utils::Scalar det, E_one_norm;
        bool use_svd = false;

        
        // Mk = mat^T
        Mk = M.transpose();
        
        //M one-norm
        utils::Scalar M_one_norm = 0.0f;
        for (int i = 0; i < 3; i++)
        {
            utils::Scalar col_abs_sum = fabs(M(i, 0)) + fabs(M(i, 1)) + fabs(M(i, 2));
            if (col_abs_sum > M_one_norm)
                M_one_norm = col_abs_sum;
        }

        //M infinity-norm
        utils::Scalar M_inf_norm = 0.0;
        for (int i = 0; i < 3; i++)
        {
            utils::Scalar row_sum = fabs(M(i, 0)) + fabs(M(i, 1)) + fabs(M(i, 2));
            if (row_sum > M_inf_norm)
                M_inf_norm = row_sum;
        }

        do
        {
            utils::Matrix3d M_adj_Tk;

            /*
            // row 2 x row 3
            cross(&(Mk[3]), &(Mk[6]), &M_adj_Tk(0, 0)));
            // row 3 x row 1
            cross_product(&(Mk[6]), &(Mk[0]), &(M_adj_Tk[3]));
            // row 1 x row 2
            cross_product(&(Mk[0]), &(Mk[3]), &(M_adj_Tk[6]));
            */

            det = Mk(0, 0) * M_adj_Tk(0, 0) + Mk(0, 1) * M_adj_Tk(0, 1) + Mk(0, 2) * M_adj_Tk(0, 2);

            if( det <= 1e-6 )
            {
                use_svd = true;
                std::cout << "Oups, SVD serait utile!\n";
                break;
            }

            if( det == 0.0 )
            {
                printf("Warning (polarDecomposition) : zero determinant encountered.\n");
                break;
            }

            //mat_adj_T one-norm
            utils::Scalar MadjT_one_norm = 0.0f;
            for (int i = 0; i < 3; i++)
            {
                utils::Scalar col_abs_sum = fabs(M_adj_Tk(i, 0)) + fabs(M_adj_Tk(i, 1)) + fabs(M_adj_Tk(i, 2));
                if (col_abs_sum > MadjT_one_norm)
                    MadjT_one_norm = col_abs_sum;
            }

            //mat infinity-norm
            utils::Scalar MadjT_inf_norm = 0.0;
            for (int i = 0; i < 3; i++)
            {
                utils::Scalar row_sum = fabs(M_adj_Tk(i, 0)) + fabs(M_adj_Tk(i, 1)) + fabs(M_adj_Tk(i, 2));
                if (row_sum > M_inf_norm)
                    M_inf_norm = row_sum;
            }

            
            utils::Scalar gamma = sqrtf(sqrtf((MadjT_one_norm * MadjT_inf_norm) / (M_one_norm * M_inf_norm * det * det)));
            utils::Scalar g1 = gamma * 0.5f;
            utils::Scalar g2 = 0.5f / (gamma * det);

            for(int i = 0; i < 3; i++)
            {
                for(int j = 0; i < 3; j++)
                    {
                        Ek(i, j) = Mk(i, j);
                        Mk(i, j) = g1 * Mk(i, j) + g2 * M_adj_Tk(i, j);
                        Ek(i, j) -= Mk(i, j);
                    }
            }

            //Ek one-norm
            utils::Scalar E_one_norm = 0.0f;
            for (int i = 0; i < 3; i++)
            {
                utils::Scalar col_abs_sum = fabs(Ek(i, 0)) + fabs(Ek(i, 1)) + fabs(Ek(i, 2));
                if (col_abs_sum > E_one_norm)
                    E_one_norm = col_abs_sum;
            }

            //M one-norm
            utils::Scalar M_one_norm = 0.0f;
            for (int i = 0; i < 3; i++)
            {
                utils::Scalar col_abs_sum = fabs(M(i, 0)) + fabs(M(i, 1)) + fabs(M(i, 2));
                if (col_abs_sum > M_one_norm)
                    M_one_norm = col_abs_sum;
            }

            //M infinity-norm
            utils::Scalar M_inf_norm = 0.0;
            for (int i = 0; i < 3; i++)
            {
                utils::Scalar row_sum = fabs(M(i, 0)) + fabs(M(i, 1)) + fabs(M(i, 2));
                if (row_sum > M_inf_norm)
                    M_inf_norm = row_sum;
            }

        } while ( E_one_norm > M_one_norm * tolerance );

        
        if(use_svd) // && force_rotation
        {
            // use the SVD algorithm to compute R
            utils::Matrix3d Mm = M;
            utils::Matrix3d Um, Vm;
            utils::Vector3d lambda;
            utils::Vector3d lambda_inverse;
            int modified_SVD = 1;

            // form Um^T Um and do eigendecomposition
            utils::Matrix3d normalEq = Mm.transpose() * Mm;
            utils::Vector3d eigen_vals;
            utils::Matrix3d eigen_vecs;
            utils::Matrix3d V;
            utils::Vector3d d;

            int n = 3;

            utils::Vector3d e;
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    V(i, j) = normalEq(i, j);
                }
            }


            // Symmetric Householder reduction to tridiagonal form. (tred2)
            for (int j = 0; j < n; j++) {
                d(j) = V(n-1, j);
            }

            // Householder reduction to tridiagonal form.

            for (int i = n-1; i > 0; i--) {

                // Scale to avoid under/overflow.

                utils::Scalar scale = 0.0;
                utils::Scalar h = 0.0;
                for (int k = 0; k < i; k++) {
                    scale = scale + fabs(d(k));
                }
                if (scale == 0.0) {
                    e(i) = d(i-1);
                    for (int j = 0; j < i; j++) {
                        d(j) = V(i-1, j);
                        V(i, j) = 0.0;
                        V(j, i) = 0.0;
                    }
                } else {

                    // Generate Householder vector.
                    for (int k = 0; k < i; k++) {
                        d(k) /= scale;
                        h += d(k) * d(k);
                    }
                    utils::Scalar f = d[i-1];
                    utils::Scalar g = sqrtf(h);
                    if (f > 0) {
                        g = -g;
                    }
                    e(i) = scale * g;
                    h = h - f * g;
                    d(i-1) = f - g;
                    for (int j = 0; j < i; j++) {
                        e(j) = 0.0;
                    }

                    // Apply similarity transformation to remaining columns.
                    for (int j = 0; j < i; j++) {
                        f = d(j);
                        V(j, i) = f;
                        g = e(j) + V(j, j) * f;
                        for (int k = j+1; k <= i-1; k++) {
                            g += V(k, j) * d(k);
                            e(k) += V(k, j) * f;
                        }
                        e(j) = g;
                    }
                    f = 0.0;
                    for (int j = 0; j < i; j++) {
                        e(j) /= h;
                        f += e(j) * d(j);
                    }
                    utils::Scalar hh = f / (h + h);
                    for (int j = 0; j < i; j++) {
                        e[j] -= hh * d(j);
                    }
                    for (int j = 0; j < i; j++) {
                        f = d(j);
                        g = e(j);
                        for (int k = j; k <= i-1; k++) {
                            V(k, j) -= (f * e(k) + g * d(k));
                        }
                        d(j) = V(i-1, j);
                        V(i, j) = 0.0;
                    }
                }
                d(i) = h;
            }

            // Accumulate transformations
            for (int i = 0; i < n-1; i++) {
                V(n-1, i) = V(i, i);
                V(i, i) = 1.0;
                utils::Scalar h = d(i+1);
                if (h != 0.0) {
                    for (int k = 0; k <= i; k++) {
                        d(k) = V(k, i+1) / h;
                    }
                    for (int j = 0; j <= i; j++) {
                        utils::Scalar g = 0.0;
                        for (int k = 0; k <= i; k++) {
                            g += V(k, i+1) * V(k, j);
                        }
                        for (int k = 0; k <= i; k++) {
                            V(k, j) -= g * d(k);
                        }
                    }
                }
                for (int k = 0; k <= i; k++) {
                    V(k, i+1) = 0.0;
                }
            }
            for (int j = 0; j < n; j++) {
                d(j) = V(-1, j);
                V(n-1, j) = 0.0;
            }
            V(n-1, n-1) = 1.0;
            e(0) = 0.0;


            // Symmetric tridiagonal QL algorithm. (td12)
            for (int i = 1; i < n; i++) {
                e(i-1) = e(i);
            }
            e(n-1) = 0.0;

            utils::Scalar f = 0.0f;
            utils::Scalar tst1 = 0.0f;
            utils::Scalar eps = 1e-16;
            for (int l = 0; l < n; l++) {

                // Find small subdiagonal element
                tst1 = fmax(tst1, fabs(d(l)) + fabs(e(l)));
                int m = l;
                while (m < n) {
                    if (fabs(e(m)) <= eps*tst1) {
                        break;
                    }
                    m++;
                }

                // If m == l, d[l] is an eigenvalue,
                // otherwise, iterate.
                if (m > l) {
                    int iter = 0;
                    do {
                        iter = iter + 1;  // (Could check iteration count here.)

                        // Compute implicit shift
                        utils::Scalar g = d(l);
                        utils::Scalar p = (d(l+1) - g) / (2.0f * e(l));

                        utils::Scalar r = sqrtf(p*p+1.0f*1.0f);
                        if (p < 0) {
                            r = -r;
                        }
                        d(l) = e(l) / (p + r);
                        d(l+1) = e(l) * (p + r);
                        utils::Scalar dl1 = d(l+1);
                        utils::Scalar h = g - d(l);
                        for (int i = l+2; i < n; i++) {
                            d(i) -= h;
                        }
                        f = f + h;

                        // Implicit QL transformation.
                        p = d(m);
                        utils::Scalar c = 1.0;
                        utils::Scalar c2 = c;
                        utils::Scalar c3 = c;
                        utils::Scalar el1 = e(l+1);
                        utils::Scalar s = 0.0;
                        utils::Scalar s2 = 0.0;
                        for (int i = m-1; i >= l; i--) {
                            c3 = c2;
                            c2 = c;
                            s2 = s;
                            g = c * e(i);
                            h = c * p;
                            r = sqrtf(p*p+e(i)*e(i));
                            e(i+1) = s * r;
                            s = e(i) / r;
                            c = p / r;
                            p = c * d(i) - s * g;
                            d(i+1) = h + s * (c * g + s * d(i));

                            // Accumulate transformation.
                            for (int k = 0; k < n; k++) {
                                h = V(k, i+1);
                                V(k, i+1) = s * V(k, i) + c * h;
                                V(k, i) = c * V(k, i) - s * h;
                            }
                        }
                        p = -s * s2 * c3 * el1 * e(l) / dl1;
                        e(l) = s * p;
                        d(l) = c * p;

                        // Check for convergence.

                    } while (fabs(e(l)) > eps*tst1);
                }
                d(l) = d(l) + f;
                e(l) = 0.0;
            }

            // Sort eigenvalues and corresponding vectors.
            for (int i = 0; i < n-1; i++) {
                int k = i;
                utils::Scalar p = d(i);
                for (int j = i+1; j < n; j++) {
                    if (d(j) < p) {
                        k = j;
                        p = d(j);
                    }
                }
                if (k != i) {
                    d(k) = d(i);
                    d(i) = p;
                    for (int j = 0; j < n; j++) {
                        p = V(j, i);
                        V(j, i) = V(j, k);
                        V(j, k) = p;
                    }
                }
            }

            eigen_vals(0) = d(2);
            eigen_vals(1) = d(1);
            eigen_vals(2) = d(0);

            eigen_vecs(0, 0) = V(0, 2); 
            eigen_vecs(0, 1) = V(1, 2); 
            eigen_vecs(0, 2) = V(2, 2);
            eigen_vecs(1, 0) = V(0, 1); 
            eigen_vecs(1, 1) = V(1, 1); 
            eigen_vecs(1, 2) = V(2, 1);
            eigen_vecs(2, 0) = V(0, 0); 
            eigen_vecs(2, 1) = V(1, 0); 
            eigen_vecs(2, 2) = V(2, 0);


            Vm = eigen_vecs.transpose();

            // Handle situation:
            // det(Vm) == -1
            //    - multiply the first column of V by -1
            if (Vm.determinant() < 0.0)
            {
                // convert V into a rotation (multiply column 1 by -1)
                Vm(0,0) *= -1.0;
                Vm(1,0) *= -1.0;
                Vm(2,0) *= -1.0;
            }

            lambda(0) = (eigen_vals(0) > 0.0f) ? sqrtf(eigen_vals(0)) : 0.0f;
            lambda(1) = (eigen_vals(1) > 0.0f) ? sqrtf(eigen_vals(1)) : 0.0f;
            lambda(2) = (eigen_vals(2) > 0.0f) ? sqrtf(eigen_vals(2)) : 0.0f;

            // compute inverse of singular values
            // also check if singular values are close to zero
            lambda_inverse(0) = (lambda(0) > tolerance) ? (1.0f / lambda(0)) : 0.0f;
            lambda_inverse(1) = (lambda(1) > tolerance) ? (1.0f / lambda(1)) : 0.0f;
            lambda_inverse(2) = (lambda(2) > tolerance) ? (1.0f / lambda(2)) : 0.0f;

            // compute U using the formula:
            // U = F * V * diag(SigmaInverse)
            Um = Mm * Vm;

            utils::Matrix3d result;

            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    result(i, j) = Um(i, j);
                }
            }

            result(0,0) *= lambda_inverse(0);
            result(1,0) *= lambda_inverse(0);
            result(2,0) *= lambda_inverse(0);

            result(0,1) *= lambda_inverse(1);
            result(1,1) *= lambda_inverse(1);
            result(2,1) *= lambda_inverse(1);

            result(0,2) *= lambda_inverse(2);
            result(1,2) *= lambda_inverse(2);
            result(2,2) *= lambda_inverse(2);

            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    Um(i, j) = result(i, j);
                }
            }

            // In theory, U is now orthonormal, U^T U = U U^T = I ..
            // it may be a rotation or a reflection, depending on F.
            // But in practice, if singular values are small or zero,
            // it may not be orthonormal, so we need to fix it.
            // Handle situation:
            // 2. An entry of Sigma is near zero
            // ---------------------------------------------------------

            if ((lambda(0) < tolerance) && (lambda(1) < tolerance) && (lambda(2) < tolerance))
            {
                // extreme case, all singular values are small,
                // material has collapsed almost to a point
                // see [Irving 04], p. 4
                Um(0, 0) = 1.0;
                Um(0, 1) = 0.0;
                Um(0, 2) = 0.0;
                Um(1, 0) = 0.0;
                Um(1, 1) = 1.0;
                Um(1, 2) = 0.0;
                Um(2, 0) = 0.0;
                Um(2, 1) = 0.0;
                Um(2, 2) = 1.0;

            }
            else
            {
                // handle the case where two singular values are small,
                // but the third one is not handle it by computing
                // two (arbitrary) vectors orthogonal to the eigenvector
                // for the large singular value
                int done = 0;
                for(int dim = 0; dim < 3; dim++)
                {
                    int dim_a = dim;
                    int dim_b = (dim + 1) % 3;
                    int dim_c = (dim + 2) % 3;
                    if ((lambda(dim_b) < tolerance) && (lambda(dim_c) < tolerance))
                    {
                        // only the column dimA can be trusted,
                        // columns dimB and dimC correspond to tiny singular values
                        utils::Vector3d vec1(Um(0,dim_a), Um(1,dim_a), Um(2,dim_a)); // column dimA
                        utils::Vector3d vec2;

                        // find smallest abs component of v
                        int smallest_idx = 0;
                        for(int dim = 1; dim < 3; dim++)
                            if (fabs(vec1(dim)) < fabs(vec1(smallest_idx)))
                                smallest_idx = dim;

                        utils::Vector3d axis;
                        axis(smallest_idx) = 1.0;

                        // this cross-product will be non-zero (as long as v is not zero)
                        vec2 = (vec1.cross(axis)).normalized();

                        utils::Vector3d vec3 = (vec1.cross(vec2)).normalized();
                        Um(0, dim_b) = vec2(0);
                        Um(1, dim_b) = vec2(1);
                        Um(2, dim_b) = vec2(2);
                        Um(0, dim_c) = vec3(0);
                        Um(1, dim_c) = vec3(1);
                        Um(2, dim_c) = vec3(2);
                        if (Um.determinant() < 0.0)
                        {
                            Um(0, dim_b) *= -1.0;
                            Um(1, dim_b) *= -1.0;
                            Um(2, dim_b) *= -1.0;
                        }
                        done = 1;
                        break; // out of for
                    }
                }

                // handle the case where one singular value is small,
                // but the other two are not
                // handle it by computing the cross product of the two eigenvectors
                // for the two large singular values
                if(!done)
                {
                    for(int dim = 0; dim < 3; dim++)
                    {
                        int dim_a = dim;
                        int dim_b = (dim + 1) % 3;
                        int dim_c = (dim + 2) % 3;

                        if(lambda(dim_a) < tolerance)
                        {
                            // columns dimB and dimC are both good,
                            // but column dimA corresponds to a tiny singular value

                            utils::Vector3d vec1(Um(0,dim_b), Um(1,dim_b), Um(2,dim_b)); // column dimB
                            utils::Vector3d vec2(Um(0,dim_c), Um(1,dim_c), Um(2,dim_c)); // column dimC
                            utils::Vector3d vec3 = (vec1.cross(vec2)).normalized();
                            Um(0, dim_a) = vec3(0);
                            Um(1, dim_a) = vec3(1);
                            Um(2, dim_a) = vec3(2);
                            if(Um.determinant() < 0.0)
                            {
                                Um(0, dim_a) *= -1.0;
                                Um(1, dim_a) *= -1.0;
                                Um(2, dim_a) *= -1.0;
                            }
                            done = 1;
                            break; // out of for
                        }
                    }
                }

                if ((!done) && (modified_SVD == 1))
                {
                    // Handle situation:
                    // negative determinant (Tet is inverted in solid mechanics)
                    //    - check if det(U) == -1
                    //    - If yes, then negate the minimal element of Sigma
                    //      and the corresponding column of U

                    utils::Scalar det_U = Um.determinant();
                    if (det_U < 0.0)
                    {
                        // negative determinant
                        // find the smallest singular value (they are all non-negative)
                        int smallest_singular_value_idx = 0;
                        for(int dim=1; dim<3; dim++)
                            if (lambda(dim) < lambda(smallest_singular_value_idx))
                                smallest_singular_value_idx = dim;

                        // negate the smallest singular value
                        lambda(smallest_singular_value_idx) *= -1.0;
                        Um(0, smallest_singular_value_idx) *= -1.0;
                        Um(1, smallest_singular_value_idx) *= -1.0;
                        Um(2, smallest_singular_value_idx) *= -1.0;
                    }
                }
            }
            R = Um * Vm.transpose();
        }
        else
        {
            // R = Mk^T
            R = Mk.transpose();
        }


        std::cout << "On s'est rendu, youpi!\n";

        // utils::Matrix3d mat;

        utils::Error::check(false,
                        utils::String("The Rotation matrix square norm is equal to ")
                        + sqrtNorm + ", but should be equal to 3. Consider replacing the RT matrix by this closest orthonormal matrix \n[" 
                        + R(0, 0) + ", " + R(0, 1) + ", " + R(0, 2) + "\n" 
                        + R(1, 0) + ", " + R(1, 1) + ", " + R(1, 2) + "\n" 
                        + R(2, 0) + ", " + R(2, 1) + ", " + R(2, 2) + "]\n" 
                        + "Alternatively, you can compile with SKIP_ASSERT set to ON to turn off this error message");

    }

#endif
#endif
}

std::ostream &operator<<(std::ostream &os, const utils::Rotation &a)
{
    os << a.block(0,0,3,3);
    return os;
}

#ifdef BIORBD_USE_CASADI_MATH
void utils::Rotation::operator=(
    const RBDLCasadiMath::MX_Xd_static<3, 3> &other)
{
    this->Matrix3d::operator=(other);
}

void utils::Rotation::operator=(
    const RBDLCasadiMath::MX_Xd_SubMatrix& other)
{
    this->Matrix3d::operator=(other);
}
#endif
