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

#include "HEtechSerial.h"

// Caio
#define LENGTH_OCL 1024
#define __CL_ENABLE_EXCEPTIONS
#ifndef DEVICE
#define DEVICE CL_DEVICE_TYPE_GPU
#endif

// #include "err_code.h"
// #include "cl.hpp"
// #include "util.hpp"  // utility library


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

    // // std::cout<<TEXT_CYAN<<"Carrier_wipeoff\n";
    // // std::cout<<TEXT_RESET<<"\n";
    // // std::cout<<const_cast<const lv_32fc_t**>(d_local_codes_resampled)<<"\n";

    
    

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

            // GPU_track_record(d_corr_out,
            //     d_sig_in,
            //     phase_step_rad,
            //     phase_rate_step_rad,
            //     rem_carrier_phase_in_rad,
            //     const_cast<const float**>(d_local_codes_resampled),
            //     d_n_correlators,
            //     signal_length_samples);
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
        }
    else
        {
            // GPU_track_record(d_corr_out,
            //     d_sig_in,
            //     phase_step_rad,
            //     phase_rate_step_rad,
            //     rem_carrier_phase_in_rad,
            //     const_cast<const float**>(d_local_codes_resampled),
            //     d_n_correlators,
            //     signal_length_samples);
            volk_gnsssdr_32fc_32f_rotator_dot_prod_32fc_xn(d_corr_out, d_sig_in, std::exp(lv_32fc_t(0.0, -phase_step_rad)), phase_offset_as_complex, const_cast<const float**>(d_local_codes_resampled), d_n_correlators, signal_length_samples);  // signal_length_samples);
        }

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




