#ifndef DM_INTRINSICS_H
#define DM_INTRINSICS_H

DM_INLINE
dm_mm_int dm_mm_cast_float_to_int(dm_mm_float mm)
{
#ifdef DM_SIMD_256
    return _mm256_castps_si256(mm);
#else
    return _mm_castps_si128(mm);
#endif
}

DM_INLINE
dm_mm_float dm_mm_cast_int_to_float(dm_mm_int mm)
{
#ifdef DM_SIMD_256
    return _mm256_castsi256_ps(mm);
#else
    return _mm_castsi128_ps(mm);
#endif
}

/*
float
*/
DM_INLINE
dm_mm_float dm_mm_load_ps(float* d)
{
#ifdef DM_SIMD_256
#ifdef DM_PLATFORM_LINUX
    return _mm256_loadu_ps(d);
#else
    return _mm256_load_ps(d);
#endif
    
#else
#ifdef DM_PLATFORM_LINUX
    return _mm_loadu_ps(d);
#else
    return _mm_load_ps(d);
#endif
#endif
}

DM_INLINE
dm_mm_float dm_mm_set1_ps(float d)
{
#ifdef DM_SIMD_256
    return _mm256_set1_ps(d);
#else
    return _mm_set1_ps(d);
#endif
}

DM_INLINE
void dm_mm_store_ps(float* d, dm_mm_float mm)
{
#ifdef DM_SIMD_256
    
#ifdef DM_PLATFORM_LINUX
    _mm256_storeu_ps(d, mm);
#else
    _mm256_store_ps(d, mm);
#endif
    
#else
#ifdef DM_PLATFORM_LINUX
    _mm_storeu_ps(d, mm);
#else
    _mm_store_ps(d, mm);
#endif
    
#endif // DM_SIMD_256
}

DM_INLINE
dm_mm_float dm_mm_add_ps(dm_mm_float left, dm_mm_float right)
{
#ifdef DM_SIMD_256
    return _mm256_add_ps(left, right);
#else
    return _mm_add_ps(left, right);
#endif
}

DM_INLINE
dm_mm_float dm_mm_sub_ps(dm_mm_float left, dm_mm_float right)
{
#ifdef DM_SIMD_256
    return _mm256_sub_ps(left, right);
#else
    return _mm_sub_ps(left, right);
#endif
}

DM_INLINE
dm_mm_float dm_mm_mul_ps(dm_mm_float left, dm_mm_float right)
{
#ifdef DM_SIMD_256
    return _mm256_mul_ps(left, right);
#else
    return _mm_mul_ps(left, right);
#endif
}

DM_INLINE
dm_mm_float dm_mm_div_ps(dm_mm_float left, dm_mm_float right)
{
#ifdef DM_SIMD_256
    return _mm256_div_ps(left, right);
#else
    return _mm_div_ps(left, right);
#endif
}

DM_INLINE
dm_mm_float dm_mm_sqrt_ps(dm_mm_float mm)
{
#ifdef DM_SIMD_256
    return _mm256_sqrt_ps(mm);
#else
    return _mm_sqrt_ps(mm);
#endif
}

DM_INLINE
dm_mm_float dm_mm_hadd_ps(dm_mm_float left, dm_mm_float right)
{
#ifdef DM_SIMD_256
    return _mm256_hadd_ps(left, right);
#else
    return _mm_hadd_ps(left, right);
#endif
}

DM_INLINE
dm_mm_float dm_mm_fmadd_ps(dm_mm_float a, dm_mm_float b, dm_mm_float c)
{
#ifdef DM_SIMD_256
    return _mm256_fmadd_ps(a, b, c);
#else
    return _mm_fmadd_ps(a, b, c);
#endif
}

DM_INLINE
dm_mm_float dm_mm_max_ps(dm_mm_float a, dm_mm_float b)
{
#ifdef DM_SIMD_256
    return _mm256_max_ps(a, b);
#else
    return _mm_max_ps(a, b)
#endif
}

DM_INLINE
dm_mm_float dm_mm_min_ps(dm_mm_float a, dm_mm_float b)
{
#ifdef DM_SIMD_256
    return _mm256_min_ps(a, b);
#else
    return _mm_min_ps(a, b);
#endif
}

DM_INLINE
float dm_mm_extract_float(dm_mm_float mm)
{
#ifdef DM_SIMD_256
    return _mm256_cvtss_f32(mm);
#else
    return _mm_cvtss_f32(mm);
#endif
}

// https://stackoverflow.com/questions/13219146/how-to-sum-m256-horizontally
DM_INLINE
float dm_mm_sum_elements(dm_mm_float mm)
{
#ifdef DM_SIMD_256
    // hiQuad = ( x7, x6, x5, x4 )
    const __m128 hiQuad = _mm256_extractf128_ps(mm, 1);
    // loQuad = ( x3, x2, x1, x0 )
    const __m128 loQuad = _mm256_castps256_ps128(mm);
    // sumQuad = ( x3 + x7, x2 + x6, x1 + x5, x0 + x4 )
    const __m128 sumQuad = _mm_add_ps(loQuad, hiQuad);
    // loDual = ( -, -, x1 + x5, x0 + x4 )
    const __m128 loDual = sumQuad;
    // hiDual = ( -, -, x3 + x7, x2 + x6 )
    const __m128 hiDual = _mm_movehl_ps(sumQuad, sumQuad);
    // sumDual = ( -, -, x1 + x3 + x5 + x7, x0 + x2 + x4 + x6 )
    const __m128 sumDual = _mm_add_ps(loDual, hiDual);
    // lo = ( -, -, -, x0 + x2 + x4 + x6 )
    const __m128 lo = sumDual;
    // hi = ( -, -, -, x1 + x3 + x5 + x7 )
    const __m128 hi = _mm_shuffle_ps(sumDual, sumDual, 0x1);
    // sum = ( -, -, -, x0 + x1 + x2 + x3 + x4 + x5 + x6 + x7 )
    const __m128 sum = _mm_add_ss(lo, hi);
    return _mm_cvtss_f32(sum);
#endif
}

/*
 int
*/
DM_INLINE
dm_mm_int dm_mm_load_i(int* d)
{
#ifdef DM_SIMD_256
    return _mm256_load_si256((dm_mm_int*)d);
#else
    return _mm_load_si128((dm_mm_int*)d);
#endif
}

DM_INLINE
dm_mm_int dm_mm_set1_i(int d)
{
#ifdef DM_SIMD_256
    return _mm256_set1_epi32(d);
#else
    return _mm_set1_epi32(d);
#endif
}

DM_INLINE
void dm_mm_store_i(int* i, dm_mm_int mm)
{
#ifdef DM_SIMD_256
    _mm256_store_si256((dm_mm_int*)i, mm);
#else
    _mm_store_si128((dm_mm_int*)i, mm);
#endif
}

DM_INLINE
dm_mm_int dm_mm_add_i(dm_mm_int left, dm_mm_int right)
{
#ifdef DM_SIMD_256
    return _mm256_add_epi32(left, right);
#else
    return _mm_add_epi32(left, right);
#endif
}

DM_INLINE
dm_mm_int dm_mm_sub_i(dm_mm_int left, dm_mm_int right)
{
#ifdef DM_SIMD_256
    return _mm256_sub_epi32(left, right);
#else
    return _mm_sub_epi32(left, right);
#endif
}

DM_INLINE
dm_mm_int dm_mm_mul_i(dm_mm_int left, dm_mm_int right)
{
#ifdef DM_SIMD_256
    return _mm256_mul_epi32(left, right);
#else
    return _mm_mul_epi32(left, right);
#endif
}

DM_INLINE
dm_mm_int dm_mm_hadd_i(dm_mm_int left, dm_mm_int right)
{
#ifdef DM_SIMD_256
    return _mm256_hadd_epi32(left, right);
#else
    return _mm_hadd_epi32(left, right);
#endif
}

DM_INLINE
dm_mm_int dm_mm_shiftl_1(dm_mm_int mm)
{
#ifdef DM_SIMD_256
    return _mm256_slli_si256(mm, sizeof(int));
#else
    return _mm_bslli_si128(mm, sizeof(int));
#endif
}

DM_INLINE
dm_mm_int dm_mm_shiftr_1(dm_mm_int mm)
{
#ifdef DM_SIMD_256
    return _mm256_bsrli_epi128(mm, sizeof(int));
#else
    return _mm_bsrli_si128(mm, sizeof(int));
#endif
}

#endif //DM_INTRINSICS_H
