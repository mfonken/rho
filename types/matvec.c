//
//  matvec.c
//  combine
//
//  Created by Matthew Fonken on 9/3/19.
//  Copyright © 2019 Matthew Fonken. All rights reserved.
//

#include "matvec.h"

float SafeExp(floating_t x)
{
    if (x < -UNCALCULABILTY_BOUND_FOR_SAFE_EXPONENT)
        return 0.0f;
    else if (x > UNCALCULABILTY_BOUND_FOR_SAFE_EXPONENT)
        return SAFE_EXPONENT_MAX_VALUE;
    else
        return exp(x);
}

/* Vec2 Functions */
void Vec2SubVec2(vec2_t * A, vec2_t * B, vec2_t * C)
{
    C->a = A->a - B->a;
    C->b = A->b - B->b;
}

void Vec2AddVec2(vec2_t * A, vec2_t * B, vec2_t * C)
{
    C->a = A->a + B->a;
    C->b = A->b + B->b;
}

void ScalarMulVec2(floating_t A, vec2_t * B, vec2_t * C)
{
    C->a = A * B->a;
    C->b = A * B->b;
}

floating_t Vec2DotVec2( vec2_t * A, vec2_t * B )
{
    return A->a * B->a + A->b * B->b;
}

void Vec2AAT( vec2_t * A, mat2x2 * m )
{
    floating_t ab = A->a * A->b;
    m->a = A->a * A->a;
    m->b = ab;
    m->c = ab;
    m->d = A->b * A->b;
}

/* Mat2x2 Functions */
void Mat2x2AddMat2x2(mat2x2 * A, mat2x2 * B, mat2x2 * C)
{
    C->a = A->a + B->a;
    C->b = A->b + B->b;
    C->c = A->c + B->c;
    C->d = A->d + B->d;
}

void Mat2x2SubMat2x2(mat2x2 * A, mat2x2 * B, mat2x2 * C)
{
    C->a = A->a - B->a;
    C->b = A->b - B->b;
    C->c = A->c - B->c;
    C->d = A->d - B->d;
}

void ScalarMulMat2x2( floating_t A, mat2x2 * B, mat2x2 * C )
{
    C->a = A * B->a;
    C->b = A * B->b;
    C->c = A * B->c;
    C->d = A * B->d;
}

void Mat2x2DotVec2(mat2x2 * mat, vec2_t * vec, vec2_t * res)
{
    res->a = mat->a * vec->a + mat->b * vec->b;
    res->b = mat->c * vec->a + mat->d * vec->b;
}

void MulTwoMat2x2s( mat2x2 * A, mat2x2 * B, mat2x2 * C )
{
    MatVec.Mat2x2.DotVec2( A, (vec2_t *)B, (vec2_t *)C );
    MatVec.Mat2x2.DotVec2( A, (vec2_t *)&(((floating_t*)B)[2]), (vec2_t *)&(((floating_t*)C)[2]) );
    SWAP( C->b, C->c );
}

floating_t Mat2x2Determinant( mat2x2 * mat )
{
    return ( ( mat->a * mat->d ) - ( mat->c * mat->b ) );
}

void Mat2x2Inverse( mat2x2 * mat, mat2x2 * res )
{
    floating_t det = MatVec.Mat2x2.Determinant( mat );
    if( det )
    {
        res->d = mat->a / det;
        res->c = -mat->b / det;
        res->b = -mat->c / det;
        res->a = mat->d / det;
    }
    else
        memset( res, ZDIV_LNUM, sizeof(mat2x2) );
}

void Mat2x2LLT( mat2x2 * mat, mat2x2 * llt )
{
    mat2x2 L = { 0., 0., 0., 0. };
    L.a = sqrt( mat->a );
    L.b = mat->b;
    L.c = ZDIV( mat->c, L.a );
    L.d = sqrt( mat->d - L.c * L.c );
    if( L.d == 0) L.d = 1;
    *llt = L;
}

/* Gaussian1D Functions */
#ifdef INCLUDE_FULL_GAUSSIAN_1D
void MulGaussian1d( gaussian1d_t * a, gaussian1d_t * b, gaussian1d_t * c )
{
    floating_t cov_y_a_var = a->std_dev * a->std_dev, cov_y_b_var = b->std_dev * b->std_dev;
    floating_t k = cov_y_a_var / ( cov_y_a_var + cov_y_b_var), mean_diff = b->mean - a->mean;
    c->mean = a->mean + k * mean_diff;
    c->std_dev = a->std_dev * ( 1 - k );
}

void DivGaussian1d( gaussian1d_t * a, gaussian1d_t * b, gaussian1d_t * c )
{
    floating_t cov_y_a_var = a->std_dev * a->std_dev, cov_y_b_var = b->std_dev * b->std_dev;
    floating_t k = cov_y_a_var / ( cov_y_a_var - cov_y_b_var), mean_diff = a->mean - b->mean;
    c->mean = -a->mean + k * mean_diff;
    c->std_dev = a->std_dev * ( 1 + k );
}
void CopyGaussian1d( gaussian1d_t * a, gaussian1d_t * b )
{
    b->mean = a->mean;
    b->std_dev = a->std_dev;
}
#endif
floating_t ProbabilityFromGaussian1d( gaussian1d_t * a, floating_t v )
{
    floating_t two_std_dev = 2. * a->std_dev,
    diff = ( v - a->mean ),
    e = SafeExp( - ( diff * diff ) / two_std_dev ),
    f = sqrt( M_PI * two_std_dev );
    return ZDIV( e, f );
}

/* Gaussian2D Functions */
void MultiplyGaussian2d( gaussian2d_t * a, gaussian2d_t * b, gaussian2d_t * c )
{
//    /* See https://math.stackexchange.com/questions/157172/product-of-two-multivariate-gaussians-distributions */
//    mat2x2 a_computation, b_computation, c_computation;
//
//    MatVec.Mat2x2.Inverse( &a->covariance, &a_computation );
//    MatVec.Mat2x2.Inverse( &b->covariance, &b_computation );
//    MatVec.Mat2x2.Add( &a_computation, &b_computation, &c_computation );
//    MatVec.Mat2x2.Inverse( &c_computation, &c->covariance );
//
//    vec2_t a_computation_vec, b_computation_vec, c_computation_vec;
//    MatVec.Mat2x2.DotVec2( &a_computation, &a->mean, &a_computation_vec );
//    MatVec.Mat2x2.DotVec2( &b_computation, &b->mean, &b_computation_vec );
//    MatVec.Vec2.Add( &a_computation_vec, &b_computation_vec, &c_computation_vec );
//    MatVec.Mat2x2.DotVec2( &c->covariance, &c_computation_vec, &c->mean );
    
//    vec2_t a_bc = { a->covariance.b, a->covariance.c };
//    vec2_t b_bc = { b->covariance.b, b->covariance.c };
//    /// Temporary check for reducing error compounding
//    a->covariance.b = 0; a->covariance.c = 0;
//    b->covariance.b = 0; b->covariance.c = 0;
    
    /* https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/ - eq's 14 & 15 */
    mat2x2 covariance_sum, inverse_covariance_sum, K, a_covariance_K;
    MatVec.Mat2x2.Add( &a->covariance, &b->covariance, &covariance_sum );
    MatVec.Mat2x2.Inverse( &covariance_sum, &inverse_covariance_sum);
    MatVec.Mat2x2.Multiply( &a->covariance, &inverse_covariance_sum, &K );
    MatVec.Mat2x2.Multiply( &a->covariance, &K, &a_covariance_K );
    
    vec2_t mean_diff, K_mean_diff;
    MatVec.Vec2.Subtract( &b->mean, &a->mean, &mean_diff );
    MatVec.Mat2x2.DotVec2( &K, &mean_diff, &K_mean_diff );
    MatVec.Vec2.Add( &a->mean, &K_mean_diff, &c->mean );
    
    MatVec.Mat2x2.Subtract( &a->covariance, &a_covariance_K, &c->covariance );
    
//    a->covariance.b = a_bc.a; a->covariance.c = a_bc.b;
//    b->covariance.b = b_bc.a; b->covariance.c = b_bc.b;
    
    c->combinations = a->combinations + b->combinations + 1;
}
floating_t ProbabilityFromGaussian2d( gaussian2d_t * a, vec2_t * v )
{
    floating_t two_pi = 2. * M_PI, mahalanobis_sq, e, f,
    cov_det = MatVec.Mat2x2.Determinant( &a->covariance );
    if(fabs(cov_det) < SMALL_VALUE_ERROR_OFFSET) return 0;
    vec2_t mean_diff;
    mat2x2 cov_inv;
    
    MatVec.Vec2.Subtract( v, &a->mean, &mean_diff );
    MatVec.Mat2x2.Inverse( &a->covariance, &cov_inv );
    mahalanobis_sq = MatVec.Gaussian2D.Covariance.MahalanobisSq( &cov_inv, &mean_diff );
    e = SafeExp( -0.5 * mahalanobis_sq );
    
    f = sqrt( two_pi * cov_det );
    return ( e / f );
}

void UpdateGaussianWithWeightGaussian2d( vec2_t * A, vec2_t * B, gaussian2d_t * gaussian, floating_t weight )
{
    floating_t delta_mean_a_a = A->a * B->a,
    delta_mean_a_b = A->a * B->b,
    delta_mean_b_b = A->b * B->b;
    if( delta_mean_a_a == 0. ) delta_mean_a_a += SMALL_VALUE_ERROR_OFFSET;
    if( delta_mean_a_b == 0. ) delta_mean_a_b += SMALL_VALUE_ERROR_OFFSET;
    if( delta_mean_b_b == 0. ) delta_mean_b_b += SMALL_VALUE_ERROR_OFFSET;
    mat2x2 covariance_delta_factor, unweighted_covariance_factor =
    { delta_mean_a_a, delta_mean_a_b, delta_mean_a_b, delta_mean_b_b };
    
    //printf("m_covf: <%.2f %.2f> [%.2f %.2f | %.2f %.2f]\n", A->a, A->b, unweighted_covariance_factor.a, unweighted_covariance_factor.b, unweighted_covariance_factor.c, unweighted_covariance_factor.d);
    
    MatVec.Mat2x2.Subtract( &unweighted_covariance_factor, &gaussian->covariance, &covariance_delta_factor );
    MatVec.Mat2x2.ScalarMultiply( weight, &covariance_delta_factor, &covariance_delta_factor );
    MatVec.Mat2x2.Add( &gaussian->covariance, &covariance_delta_factor, &gaussian->covariance );
    
    MatVec.Gaussian2D.Covariance.Limit( &gaussian->covariance );
}

vec2_t WeightedMeanUpdateGaussian2d( vec2_t * new_val, gaussian2d_t * gaussian, floating_t weight )
{
    vec2_t delta_mean, weighted_mean;
    MatVec.Vec2.Subtract( new_val, &gaussian->mean, &delta_mean );
    MatVec.Vec2.ScalarMultiply( weight, &delta_mean, &weighted_mean );
    MatVec.Vec2.Add( &weighted_mean, &gaussian->mean, &gaussian->mean );
    return delta_mean;
}

floating_t MahalanobisDistanceSquaredGaussian2d( mat2x2 * inv_covariance, vec2_t * delta )
{
    vec2_t inv_covariance_delta;
    MatVec.Mat2x2.DotVec2(inv_covariance, delta, &inv_covariance_delta);
    floating_t mahalanobis_distance_squared;
    mahalanobis_distance_squared = MatVec.Vec2.Dot(delta, &inv_covariance_delta );
    return mahalanobis_distance_squared;
}

bool LimitCovarianceGaussian2d( mat2x2 * covariance )
{
    bool limited = false;
    floating_t max = covariance->a, check;
    for(uint8_t i = 1; i < 4; i++ )
    {
        check = ((floating_t *)covariance)[i];
        if( check > max )
            max = check;
    }
    if( max > MAX_COVARIANCE )
    {
        floating_t correction_factor = MAX_COVARIANCE / max;
        for(uint8_t i = 0; i < 4; i++ )
            ((floating_t *)covariance)[i] *= correction_factor;
        limited = true;
    }
    return limited;
}

floating_t CovarianceAngleGaussian2d( mat2x2 * covariance )
{
    floating_t a = covariance->a, b = covariance->b, d = covariance->d,
    a_minus_d = a - d,
    radius = sqrt( a_minus_d * a_minus_d + 4. * b * b ),
    lambda = a_minus_d + radius;
    return atan2( -2 * b, a - lambda ) * 180 / M_PI;
}
