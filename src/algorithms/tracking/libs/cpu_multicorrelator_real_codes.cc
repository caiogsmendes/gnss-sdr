/*!
 * \file cpu_multicorrelator_real_codes.cc
 * \brief Highly optimized CPU vector multiTAP correlator class with real-valued local codes
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          <li> Cillian O'Driscoll, 2017. cillian.odriscoll(at)gmail.com
 *          </ul>
 *
 * Class that implements a highly optimized vector multiTAP correlator class for CPUs
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "cpu_multicorrelator_real_codes.h"
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <CL/cl.h>
#include <chrono>
#include <cmath>
#include <complex.h>
#include <display.h>
#include <fstream>
#include <iostream>
#include <tgmath.h>
#include <vector>
// #include <../gperftools/profiler.h>


// Caio
#define LENGTH_OCL 1024
#define __CL_ENABLE_EXCEPTIONS
#ifndef DEVICE
#define DEVICE CL_DEVICE_TYPE_GPU
#endif

#include "err_code.h"
#include "cl.hpp"
#include "util.hpp"  // utility library


using namespace std::chrono;

// Caio End

Cpu_Multicorrelator_Real_Codes::~Cpu_Multicorrelator_Real_Codes()
{
    if (d_local_codes_resampled != nullptr)
        {
            Cpu_Multicorrelator_Real_Codes::free();
        }
}


bool Cpu_Multicorrelator_Real_Codes::init(
    int max_signal_length_samples,
    int n_correlators)
{
    // ALLOCATE MEMORY FOR INTERNAL vectors
    size_t size = max_signal_length_samples * sizeof(float);

    d_local_codes_resampled = static_cast<float**>(volk_gnsssdr_malloc(n_correlators * sizeof(float*), volk_gnsssdr_get_alignment()));
    for (int n = 0; n < n_correlators; n++)
        {
            d_local_codes_resampled[n] = static_cast<float*>(volk_gnsssdr_malloc(size, volk_gnsssdr_get_alignment()));
        }
    d_n_correlators = n_correlators;
    return true;
}


bool Cpu_Multicorrelator_Real_Codes::set_local_code_and_taps(
    int code_length_chips,
    const float* local_code_in,
    float* shifts_chips)
{
    d_local_code_in = local_code_in;
    d_shifts_chips = shifts_chips;
    d_code_length_chips = code_length_chips;

    return true;
}


bool Cpu_Multicorrelator_Real_Codes::set_input_output_vectors(std::complex<float>* corr_out, const std::complex<float>* sig_in)
{
    // Save CPU pointers
    d_sig_in = sig_in;
    d_corr_out = corr_out;
    return true;
}


void Cpu_Multicorrelator_Real_Codes::update_local_code(int correlator_length_samples, float rem_code_phase_chips, float code_phase_step_chips, float code_phase_rate_step_chips)
{
    if (d_use_high_dynamics_resampler)
        {
            volk_gnsssdr_32f_xn_high_dynamics_resampler_32f_xn(d_local_codes_resampled,
                d_local_code_in,
                rem_code_phase_chips,
                code_phase_step_chips,
                code_phase_rate_step_chips,
                d_shifts_chips,
                d_code_length_chips,
                d_n_correlators,
                correlator_length_samples);
        }
    else
        {
            volk_gnsssdr_32f_xn_resampler_32f_xn(d_local_codes_resampled,
                d_local_code_in,
                rem_code_phase_chips,
                code_phase_step_chips,
                d_shifts_chips,
                d_code_length_chips,
                d_n_correlators,
                correlator_length_samples);
        }
}


bool Cpu_Multicorrelator_Real_Codes::Carrier_wipeoff_multicorrelator_resampler(
    float rem_carrier_phase_in_rad,
    float phase_step_rad,
    float phase_rate_step_rad,
    float rem_code_phase_chips,
    float code_phase_step_chips,
    float code_phase_rate_step_chips,
    int signal_length_samples)
{
    // ### Caio's Mod  ####
    // std::fstream filename;
    // filename.open("d_local_codes_resampled.txt",std::ios::out|std::ios::app);
    // filename.open("timing_multicorrlator.txt",std::ios::out|std::ios::app);
    // filename.open("correlator.txt",std::ios::out|std::ios::app);
    // auto start = high_resolution_clock::now();
    // ProfilerStart("Output_Carrier_Wipeoff_Multicorrelator.prof");
    // ###################

    update_local_code(signal_length_samples, rem_code_phase_chips, code_phase_step_chips, code_phase_rate_step_chips);

    // Regenerate phase at each call in order to avoid numerical issues
    lv_32fc_t phase_offset_as_complex[1];
    phase_offset_as_complex[0] = lv_cmake(std::cos(rem_carrier_phase_in_rad), -std::sin(rem_carrier_phase_in_rad));
    // int counter = 0;
    // float d_corr_out_real[signal_length_samples];
    // float d_corr_out_imag[signal_length_samples];
    // int n = 0;
    // while (n < signal_length_samples)
    //     {
    //         d_corr_out_real[n] = d_sig_in->real();
    //         d_corr_out_imag[n] = d_sig_in->imag();
    //         d_sig_in++;
    //         n++;
    //     }

    // print_codes(const_cast<const float**>(d_local_codes_resampled),signal_length_samples);
    // while (counter < signal_length_samples)
    //     {
    //         *d_sin_in++;
    //         float d_sig_in_real = d_corr_out->real();
    //         float d_sig_in_imag = d_corr_out->imag();
    //         filename << d_sig_in_real << " " << d_sig_in_imag << "\n";
    //     }

    // float d_local_codes_resampled = const_cast<float**>(d_local_codes_resampled->real());
    // float d_local_codes_resampled = const_cast<float**>(d_local_codes_resampled->imag());
    // float d_local_codes_resampled_real[signal_length_samples][d_n_correlators] = {const_cast<const float**>(d_local_codes_resampled)};
    // float d_local_codes_resampled_imag[signal_length_samples][d_n_correlators];
    // float phase_offset_real = std::cos(rem_carrier_phase_in_rad);
    // float phase_offset_imag = -std::sin(rem_carrier_phase_in_rad);

    // filename<<d_local_codes_resampled_real[signal_length_samples][0]<<" "<<d_local_codes_resampled_imag[signal_length_samples][0]<<"\n";

    // std::cout<<TEXT_CYAN<<"Carrier_wipeoff\n";
    // std::cout<<TEXT_RESET<<"\n";
    // std::cout<<const_cast<const lv_32fc_t**>(d_local_codes_resampled)<<"\n";

    // std::cout << "Number of Samples Processed: " << signal_length_samples << "\n";
    // std::cout << contador << "\n";
    // contador++;
    // std::fstream sampler;
    // sampler.open("Sampler_comparador_GPU_SIMD.txt",std::ios::out|std::ios::app);
    
    
    // GPU_multicorrelator(
    //     d_corr_out,
    //     d_sig_in,
    //     phase_step_rad,
    //     phase_rate_step_rad,
    //     rem_carrier_phase_in_rad,
    //     const_cast<const float**>(d_local_codes_resampled),
    //     d_n_correlators,
    //     signal_length_samples);

    if (d_use_high_dynamics_resampler)
        {
            // Caio's Mod
            // Equation from the Book
            // yp[k] = SUM(r[n+kN_t]*c[n-tau]*N_scode*exp(-jtheta[n;k]))
            /*  Output: d_corr_out
                Input: d_sig_in
                    phase_offset (Complex)
                    exp(phase_step_rad)
                    local_codes_resampled
                    d_n_correlators
                    signal_length_samples
            */


            // Esse kernel é chamado para cada canal ou dentro das samples para todos os canais
            volk_gnsssdr_32fc_32f_high_dynamic_rotator_dot_prod_32fc_xn(
                d_corr_out,
                d_sig_in,
                std::exp(lv_32fc_t(0.0, -phase_step_rad)),
                std::exp(lv_32fc_t(0.0, -phase_rate_step_rad)),
                phase_offset_as_complex,
                const_cast<const float**>(d_local_codes_resampled),
                d_n_correlators,
                signal_length_samples);
            // signal_length_samples);
            // filename<<d_local_codes_resampled<<" "<<const_cast<const float**>(d_local_codes_resampled)<<"\n";
            // Caio
            // Substituir o high_dynamics_rotator_dot_prod
            // pelo multicorrelator em OpenCL
            // GPU_multicorrelator(&d_corr_out_real, &d_corr_out_imag,
            // &d_sig_in_real, &d_sig_in_imag,
            // phase_step_rad,
            // phase_rate_step_rad,
            // phase_offset_real,
            // phase_offset_imag,
            // &d_local_codes_resampled_real[0], &d_local_codes_resampled_imag[0],
            // d_n_correlators,
            // signal_length_samples);
        }
    else
        {
            volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn(d_corr_out, d_sig_in, std::exp(lv_32fc_t(0.0, -phase_step_rad)), phase_offset_as_complex, const_cast<const float**>(d_local_codes_resampled), d_n_correlators, signal_length_samples);  // signal_length_samples);
        }
    //  auto stop = high_resolution_clock::now();
    //  auto duration = duration_cast<nanoseconds>(stop - start);
    //  filename<<"Carrier Wipeoff_multicorrelator: "<<duration.count()<<"ns | "<<" rem_carrier_phase_in_rad:  "<<rem_carrier_phase_in_rad<<"\n";
    // // ProfilerStop();
    // filename.close();
    // sampler<<d_corr_out<<"\n";
    // sampler.close();
    return true;
}


bool Cpu_Multicorrelator_Real_Codes::Carrier_wipeoff_multicorrelator_resampler(
    float rem_carrier_phase_in_rad,
    float phase_step_rad,
    float rem_code_phase_chips,
    float code_phase_step_chips,
    float code_phase_rate_step_chips,
    int signal_length_samples)
{
    update_local_code(signal_length_samples, rem_code_phase_chips, code_phase_step_chips, code_phase_rate_step_chips);
    // Regenerate phase at each call in order to avoid numerical issues
    lv_32fc_t phase_offset_as_complex[1];
    phase_offset_as_complex[0] = lv_cmake(std::cos(rem_carrier_phase_in_rad), -std::sin(rem_carrier_phase_in_rad));
    // call VOLK_GNSSSDR kernel
    volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn(d_corr_out, d_sig_in, std::exp(lv_32fc_t(0.0, -phase_step_rad)), phase_offset_as_complex, const_cast<const float**>(d_local_codes_resampled), d_n_correlators, signal_length_samples);
    return true;
}


bool Cpu_Multicorrelator_Real_Codes::free()
{
    // Free memory
    if (d_local_codes_resampled != nullptr)
        {
            for (int n = 0; n < d_n_correlators; n++)
                {
                    volk_gnsssdr_free(d_local_codes_resampled[n]);
                }
            volk_gnsssdr_free(d_local_codes_resampled);
            d_local_codes_resampled = nullptr;
        }
    return true;
}


void Cpu_Multicorrelator_Real_Codes::set_high_dynamics_resampler(
    bool use_high_dynamics_resampler)
{
    d_use_high_dynamics_resampler = use_high_dynamics_resampler;
}

void Cpu_Multicorrelator_Real_Codes::GPU_multicorrelator(
    std::complex<float>* h_corr_out,
    const std::complex<float>* d_sig_in,
    float phase_step_rad,
    float phase_step_rad_inc,
    float rem_carrier_phase_in_rad,  // No lugar de float* phase_offset,
    const float** d_local_codes_resampled,
    int n_correlators,
    int signal_length_samples)
{
    int first = 0;
    int last = signal_length_samples;
    std::vector<float> h_local_codes_resampled_early(last);
    std::vector<float> h_local_codes_resampled_prompt(last);
    std::vector<float> h_local_codes_resampled_late(last);
    // std::fstream sampling;
    // sampling.open("sampling_GPU.txt",std::ios::out|std::ios::app);

    // Preparo de dados de entrada
    float h_sig_in_real[last];
    float h_sig_in_imag[last];
    // for (int j = 0; j < 3; j++)
        // {
            for (int i = first; i < last; i++)
                {
                    h_sig_in_real[i] = d_sig_in->real();
                    h_sig_in_imag[i] = d_sig_in->imag();
                    d_sig_in++;
                    h_local_codes_resampled_early[i] = d_local_codes_resampled[0][i];
                    h_local_codes_resampled_prompt[i] = d_local_codes_resampled[1][i];
                    h_local_codes_resampled_late[i] = d_local_codes_resampled[2][i];
                    // sampling<<d_local_codes_resampled[0][i]<<" "<<d_local_codes_resampled[1][i]<<" "<<d_local_codes_resampled[2][i]<<"\n";
                }
        // }
    // sampling.close();

    // ############## Passar um program de Exemplo pra testar a GPU da Colibri ################
    //  Pegar plataforma genérica
    std::vector<cl::Platform> all_platforms;
    cl::Platform::get(&all_platforms);
    cl::Platform default_platform = all_platforms[0];
    std::vector<cl::Device> all_devices;
    default_platform.getDevices(CL_DEVICE_TYPE_GPU, &all_devices);
    cl::Device default_device = all_devices[0];

    // build program
    // Kernel com Multicorrelator (Sequencial)
    cl::Context context({default_device});
    cl::Program::Sources sources;
    std::string kernel_code =
        "void complex_mult(float *sig_in_A_real, float *sig_in_A_imag, float *sig_in_B_real, float *sig_in_B_imag, float *sig_out_real, float *sig_out_imag){*sig_out_real =(*sig_in_A_real) * (*sig_in_B_real) - (*sig_in_B_imag) * (*sig_in_A_imag);*sig_out_imag = (*sig_in_A_real) * (*sig_in_B_imag) + (*sig_in_A_imag) * (*sig_in_B_real);}"
        "__kernel void multicorr("
        "__global float *corr_out_real,"
        "__global float *corr_out_imag,"
        "__global float *sig_in_real,"
        "__global float *sig_in_imag,"
        "float phase_step_rad,"
        "float phase_step_rad_inc,"
        "float phase_offset_real,"
        "float phase_offset_imag,"
        "__global float *local_code_resampled_early,"
        "__global float *local_code_resampled_prompt,"
        "__global float *local_code_resampled_late,"
        "int n_correlators,"
        "int n_sample){"
        "int i = get_global_id(0);"
        "float tmp32_1_real;"
        "float tmp32_1_imag;"
        "float phase_doppler_rate_real = 0;"
        "float phase_doppler_rate_imag = 0;"
        "float phase_doppler_real = 0;"
        "float phase_doppler_imag = 0;"
        "float phase_inc_real = 1;"
        "float phase_inc_imag = phase_step_rad;"
        "float phase_inc_rate_real = 1;"
        "float phase_inc_rate_imag = phase_step_rad_inc;"
        "int n_vec;"
        "unsigned int n;"
        "phase_doppler_real = phase_offset_real;"
        "phase_doppler_imag = phase_offset_imag;"
        "float arga = atan(phase_step_rad_inc / 1.0);"
        "for (n = 1; n < n_sample; n++)"
        "{"
        "complex_mult(sig_in_real++, sig_in_imag++, &phase_doppler_real, &phase_doppler_imag, &tmp32_1_real, &tmp32_1_imag);"
        "complex_mult(&phase_doppler_real, &phase_doppler_imag, &phase_inc_real, &phase_inc_imag, &phase_doppler_real, &phase_doppler_imag);"
        "const float theta = (float)(n * n) * arga;"
        "phase_doppler_rate_real = cos(theta); phase_doppler_rate_imag = sin(theta);"
        "complex_mult(&phase_doppler_real, &phase_doppler_imag, &phase_doppler_rate_real, &phase_doppler_rate_imag, &phase_offset_real, &phase_offset_imag);"
        "corr_out_real[n] = corr_out_real[n-1]+tmp32_1_real*local_code_resampled_early[n]+tmp32_1_real*local_code_resampled_prompt[n]+tmp32_1_real*local_code_resampled_late[n];"
        "corr_out_imag[n] = corr_out_imag[n-1]+tmp32_1_imag*local_code_resampled_early[n]+tmp32_1_imag*local_code_resampled_prompt[n]+tmp32_1_imag*local_code_resampled_late[n];"
        "}"
        "}";
    sources.push_back({kernel_code.c_str(), kernel_code.length()});
    cl::Program program(context, sources);
    try
        {
            program.build({default_device});
        }
    catch (cl::Error err)
        {
            std::cout << "Error building: " << program.getBuildInfo<CL_PROGRAM_BUILD_LOG>({default_device}) << "\n";
            exit(1);
        }

    // Buffers e Copy memory
    // Buffers e Const de Entrada
    cl::Buffer d_sig_in_real(context, CL_MEM_READ_WRITE, sizeof(float) * signal_length_samples);
    cl::Buffer d_sig_in_imag(context, CL_MEM_READ_WRITE, sizeof(float) * signal_length_samples);
    // cl::Buffer d_local_codes_resampled = cl::Buffer(context, h_local_codes_resampled.begin(), h_local_codes_resampled.end(), true);
    cl::Buffer d_local_codes_resampled_early = cl::Buffer(context, h_local_codes_resampled_early.begin(), h_local_codes_resampled_early.end(), true);
    cl::Buffer d_local_codes_resampled_prompt = cl::Buffer(context, h_local_codes_resampled_prompt.begin(), h_local_codes_resampled_prompt.end(), true);
    cl::Buffer d_local_codes_resampled_late = cl::Buffer(context, h_local_codes_resampled_late.begin(), h_local_codes_resampled_late.end(), true);

    // Buffers de Saída
    cl::Buffer d_corr_out_real(context, CL_MEM_READ_WRITE, sizeof(float) * signal_length_samples);
    cl::Buffer d_corr_out_imag(context, CL_MEM_READ_WRITE, sizeof(float) * signal_length_samples);

    cl::CommandQueue queue(context, default_device);
    queue.enqueueWriteBuffer(d_sig_in_real, CL_TRUE, 0, sizeof(float) * signal_length_samples, h_sig_in_real);
    queue.enqueueWriteBuffer(d_sig_in_imag, CL_TRUE, 0, sizeof(float) * signal_length_samples, h_sig_in_imag);

    // Execute Kernel e Retrieve
    cl::Kernel kernel(program, "multicorr");
    kernel.setArg(0, d_corr_out_real);
    kernel.setArg(1, d_corr_out_imag);
    kernel.setArg(2, d_sig_in_real);
    kernel.setArg(3, d_sig_in_imag);
    kernel.setArg(4, phase_step_rad);
    kernel.setArg(5, phase_step_rad_inc);
    kernel.setArg(6, std::cos(rem_carrier_phase_in_rad));
    kernel.setArg(7, -std::sin(rem_carrier_phase_in_rad));
    kernel.setArg(8, d_local_codes_resampled_early);
    kernel.setArg(9, d_local_codes_resampled_prompt);
    kernel.setArg(10, d_local_codes_resampled_late);
    kernel.setArg(11, n_correlators);
    kernel.setArg(12, signal_length_samples);

    queue.enqueueNDRangeKernel(kernel, cl::NullRange, cl::NDRange(signal_length_samples), cl::NullRange);
    float h_corr_out_real[signal_length_samples];
    float h_corr_out_imag[signal_length_samples];
    queue.enqueueReadBuffer(d_corr_out_real, CL_TRUE, 0, sizeof(float) * signal_length_samples, h_corr_out_real);
    queue.enqueueReadBuffer(d_corr_out_imag, CL_TRUE, 0, sizeof(float) * signal_length_samples, h_corr_out_imag);
    queue.finish();

    // std::cout << "result: \n";
    for (int i = first; i < last; i++)
        {
            *h_corr_out = lv_cmake(h_corr_out_real[i], h_corr_out_imag[i]);
            h_corr_out++;
        }
    // std::cout << "\n";
}
// ########################################################################################
// }