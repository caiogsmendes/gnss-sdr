// ####################################################################################################
// ######################   NCO - Numerically Controlled Oscillator
// ##################################
// ####################################################################################################
//  Implementação em OpenCL v1.2
//  Author: Caio Mendes, Eng.; contato: caio.mendes@horuseye.com.br

// Uma forma que eu encontrei de usar complexos dentro do Kernel
// (Ref.: Embree,Paul. C algorithms for real-time DSP p.85)

// NCO (Ref.: Kaplan(2017))

// #include <math.h>
typedef struct
{
    float real[1024];
    float imag[1024];
} COMPLEX_VEC;

typedef struct
{
    float real;
    float imag;
} COMPLEX;

typedef struct
{
    int N;                   // Length of holding register
    int M;                   // Frequency Selection digital input value
    float HoldingReg[1024];  // Holding Register
    int freqRES;             // Frequency Resolution
    float step;
    int Count_length;  // Count Length
    float outputFreq;  // Output Frequency of NCO
} NCO_PARAM;

double realmod(double x, double y)
{
    if (y == 0)
        {
            return x;
        }
    double result = fmod(x, y);
    return ((result >= 0 && y > 0) || (x <= 0 && y < 0)) ? result : (result + y);
}

void complex_add(float *sig_in_A_real, float *sig_in_A_imag,
    float *sig_in_B_real, float *sig_in_B_imag,
    float *sig_out_real, float *sig_out_imag)
{
    *sig_out_real = *sig_in_A_real + *sig_in_B_real;
    *sig_out_imag = *sig_in_A_imag + *sig_in_B_imag;
}

void complex_sub(float *sig_in_A_real, float *sig_in_A_imag,
    float *sig_in_B_real, float *sig_in_B_imag,
    float *sig_out_real, float *sig_out_imag)
{
    *sig_out_real = *sig_in_A_real - *sig_in_B_real;
    *sig_out_imag = *sig_in_A_imag - *sig_in_B_imag;
}

void complex_mult(float *sig_in_A_real, float *sig_in_A_imag,
    float *sig_in_B_real, float *sig_in_B_imag,
    float *sig_out_real, float *sig_out_imag)
{
    *sig_out_real =
        (*sig_in_A_real) * (*sig_in_B_real) - (*sig_in_B_imag) * (*sig_in_A_imag);
    *sig_out_imag =
        (*sig_in_A_real) * (*sig_in_B_imag) + (*sig_in_A_imag) * (*sig_in_B_real);
}

void complex_div(float *sig_in_A_real, float *sig_in_A_imag,
    float *sig_in_B_real, float *sig_in_B_imag,
    float *sig_out_real, float *sig_out_imag)
{
    float square_sig_in_B_real = (*sig_in_B_real) * (*sig_in_B_real);
    float square_sig_in_B_imag = (*sig_in_B_imag) * (*sig_in_B_imag);

    *sig_out_real = ((*sig_in_A_real) * (*sig_in_B_real) +
                        ((*sig_in_A_imag) * (*sig_in_B_imag))) /
                    (square_sig_in_B_real + square_sig_in_B_imag);
    *sig_out_imag = (((*sig_in_A_imag) * (*sig_in_B_real) -
                        (*sig_in_A_real) * (*sig_in_B_imag))) /
                    (square_sig_in_B_real + square_sig_in_B_imag);
}

__kernel void multicorr(
    __global float *corr_out_real,
    __global float *corr_out_imag,
    __global float *sig_in_real,
    __global float *sig_in_imag,
    float phase_step_rad,
    float phase_step_rad_inc,
    float phase_offset_real,
    float phase_offset_imag,
    // __global float *local_code_resampled_real,  // Isso deveria ser um vetor
    // __global float *local_code_resampled_imag,
    __global float *local_code_resampled_early,
    __global float *local_code_resampled_prompt,
    __global float *local_code_resampled_late,
    int n_correlators,
    int n_sample)
{
    // float16 seno[get_global_size(0)];
    // float16 cosseno[get_global_size(0)];
    // int i = get_global_id(0);
    // float pi = 3.14;
    // seno[i] = sincos(i/pi,&cosseno[0]);
      int i = get_global_id(0);
      int j = get_group_id(0);
      int k = get_local_id(0);
    COMPLEX tmp32_1;
    COMPLEX phase_doppler_rate;
    COMPLEX phase_doppler;
    COMPLEX phase_inc;
    COMPLEX phase_inc_rate;
    phase_inc.real = 1;
    phase_inc.imag = -phase_step_rad;
    phase_inc_rate.real = 1;
    phase_inc_rate.imag = -phase_step_rad_inc;
    int n_vec;
    unsigned int n;
    float arga;
    phase_doppler.real = phase_offset_real;
    phase_doppler.imag = phase_offset_imag;

    float16 result[1024];

    // evaluates and return the phase of the complex number
    arga = atan(phase_step_rad_inc / 1.0);  // fazer ajustes no intervalo da função

    for (n = 0; n < n_sample; n++)
        {
            // tmp32_1 = *in_common++ * (*phase);
            complex_mult(sig_in_real++, sig_in_imag++, &phase_doppler.real, &phase_doppler.imag, &tmp32_1.real, &tmp32_1.imag);
            // phase_doppler *= phase_inc;
            complex_mult(&phase_doppler.real, &phase_doppler.imag, &phase_inc.real, &phase_inc.imag, &phase_doppler.real, &phase_doppler.imag);
            // const float theta = (float)(n * n) * arga;
            const float theta = (float)(n * n) * arga;  // Eq.(1.92)
            // phase_doppler_rate = lv_cmake(cosf(theta), sinf(theta));
            phase_doppler_rate.real = cos(theta); phase_doppler_rate.imag = sin(theta);
            //(*phase) = phase_doppler * phase_doppler_rate;
            complex_mult(&phase_doppler.real, &phase_doppler.imag, &phase_doppler_rate.real, &phase_doppler_rate.imag, &phase_offset_real, &phase_offset_imag);

            // For Structure for reduction
            // for (n_vec = 0; n_vec < n_correlators; n_vec++)
                // {
                    corr_out_real[n] = tmp32_1.real * local_code_resampled_early[n] + tmp32_1.real*local_code_resampled_prompt[n]+tmp32_1.real*local_code_resampled_late[n];
                    corr_out_imag[n] = tmp32_1.imag * local_code_resampled_early[n] + tmp32_1.imag*local_code_resampled_prompt[n]+tmp32_1.imag*local_code_resampled_late[n];
                // }

            // __kernel void reduction_complete(__global float4 * data,
            //     __local float4 * partial_sums, __global float *sum)
            // {
            //     int lid = get_local_id(0);
            //     int group_size = get_local_size(0);
            //     partial_sums[lid] = data[get_local_id(0)];
            //     barrier(CLK_LOCAL_MEM_FENCE);
            //     for (int i = group_size / 2; i > 0; i >>= 1)
            //         {
            //             if (lid < i)
            //                 {
            //                     partial_sums[lid] += partial_sums[lid + i];
            //                 }
            //             barrier(CLK_LOCAL_MEM_FENCE);
            //         }
            //     if (lid == 0)
            //         {
            //             *sum = partial_sums[0].s0 + partial_sums[0].s1 +
            //                    partial_sums[0].s2 + partial_sums[0].s3;
            //         }
            // }
        }

}