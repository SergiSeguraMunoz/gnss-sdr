/*!
 * \file cpu_multicorrelator.cc
 * \brief High optimized CPU vector multiTAP correlator class
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
 *
 * Class that implements a high optimized vector multiTAP correlator class for CPUs
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "cpu_multicorrelator_reflectometry.h"
#include <cmath>
#include <iostream>
#include <volk_gnsssdr/volk_gnsssdr.h>


cpu_multicorrelator_reflectometry::cpu_multicorrelator_reflectometry()
{
    d_sig_in = nullptr;
    d_sig_in_reflectometry_LHCP = nullptr;
    d_sig_in_reflectometry_RHCP = nullptr;
    d_local_code_in = nullptr;
    d_local_code_in_delayed = nullptr;
    d_shifts_chips = nullptr;
    d_shifts_chips_reflecto = nullptr;
    d_corr_out = nullptr;
    d_corr_out_reflectometry_LHCP = nullptr;
    d_corr_out_reflectometry_noDelay = nullptr;
    d_corr_out_reflectometry_RHCP = nullptr;
    d_local_codes_resampled = nullptr;
    d_local_codes_resampled_reflectometry_LHCP = nullptr;
    d_local_codes_resampled_reflectometry_noDelay = nullptr;
    d_local_codes_resampled_reflectometry_RHCP = nullptr;
    d_code_length_chips = 0;
    d_n_correlators = 0;
}


cpu_multicorrelator_reflectometry::~cpu_multicorrelator_reflectometry()
{
    if(d_local_codes_resampled != nullptr)
    {
        cpu_multicorrelator_reflectometry::free();
    }
    delete[] d_shifts_chips_reflecto;
}


bool cpu_multicorrelator_reflectometry::init(
        int max_signal_length_samples,
        int n_correlators)
{
    // ALLOCATE MEMORY FOR INTERNAL vectors
    size_t size = max_signal_length_samples * sizeof(std::complex<float>);
    d_local_codes_resampled = static_cast<std::complex<float>**>(volk_gnsssdr_malloc(n_correlators * sizeof(std::complex<float>*), volk_gnsssdr_get_alignment()));
    for (int n = 0; n < n_correlators; n++)
    {
        d_local_codes_resampled[n] = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(size, volk_gnsssdr_get_alignment()));
    }
    d_n_correlators = n_correlators;
    return true;
}


bool cpu_multicorrelator_reflectometry::init_reflectometry(
        int max_signal_length_samples,
        int n_correlators,
        int n_voie,
        bool CalibrationOutputEnabled)
{
    d_CalibrationOutputEnabled = CalibrationOutputEnabled;
    size_t size = max_signal_length_samples * sizeof(std::complex<float>);
    d_local_codes_resampled_reflectometry_LHCP = static_cast<std::complex<float>**>(volk_gnsssdr_malloc(n_correlators * sizeof(std::complex<float>*), volk_gnsssdr_get_alignment()));
    d_RHCP_enabled = false;

    for (int n = 0; n < n_correlators; n++)
    {
        d_local_codes_resampled_reflectometry_LHCP[n] = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(size, volk_gnsssdr_get_alignment()));
    }
    if(d_CalibrationOutputEnabled){
        d_local_codes_resampled_reflectometry_noDelay = static_cast<std::complex<float>**>(volk_gnsssdr_malloc(1 * sizeof(std::complex<float>*), volk_gnsssdr_get_alignment()));
        d_local_codes_resampled_reflectometry_noDelay[0] = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(size, volk_gnsssdr_get_alignment()));
    }

    if(n_voie==3){
        d_local_codes_resampled_reflectometry_RHCP = static_cast<std::complex<float>**>(volk_gnsssdr_malloc(n_correlators * sizeof(std::complex<float>*), volk_gnsssdr_get_alignment()));
        d_RHCP_enabled = true;
        for (int n = 0; n < n_correlators; n++)
        {
            d_local_codes_resampled_reflectometry_RHCP[n] = static_cast<std::complex<float>*>(volk_gnsssdr_malloc(size, volk_gnsssdr_get_alignment()));
        }
    }

    d_n_correlators_reflectometry = n_correlators;
    return true;
}

bool cpu_multicorrelator_reflectometry::set_local_code_and_taps(
        int code_length_chips,
        const std::complex<float>* local_code_in,
        float* shifts_chips, float* shifts_chips_reflectometry)
{
    d_local_code_in = local_code_in;
    d_shifts_chips = shifts_chips;
    d_code_length_chips = code_length_chips;

    //############ Modification Algorithme de reflectometrie ############
    //Délai sur la voie réfléchie
    d_shifts_chips_reflecto = new float[d_n_correlators_reflectometry];
    for(int i=0; i<d_n_correlators_reflectometry; i++){

        d_shifts_chips_reflecto[i]=(-(d_n_correlators_reflectometry-1)/2)*shifts_chips_reflectometry[0]+i*shifts_chips_reflectometry[0];
    }
    return true;
}


bool cpu_multicorrelator_reflectometry::set_input_output_vectors(std::complex<float>* corr_out, const std::complex<float>* sig_in)
{
    // Save CPU pointers
    d_sig_in = sig_in;
    d_corr_out = corr_out;
    return true;
}

bool cpu_multicorrelator_reflectometry::set_input_output_vectors_LHCP_reflectometry(std::complex<float>* corr_out, const std::complex<float>* sig_in)
{
    // Save CPU pointers
    d_sig_in_reflectometry_LHCP = sig_in;
    d_corr_out_reflectometry_LHCP = corr_out;
    return true;
}

bool cpu_multicorrelator_reflectometry::set_output_noDelay_reflectometry(std::complex<float>* corr_out_noDelay){
    d_corr_out_reflectometry_noDelay = corr_out_noDelay;
    return true;
}

bool cpu_multicorrelator_reflectometry::set_delay_reflecto(float delay){
    d_delay_reflectometry = delay;
    for(int i=0; i<d_n_correlators_reflectometry; i++){
        d_shifts_chips_reflecto[i]-=d_delay_reflectometry;
    }
    return true;
}

bool cpu_multicorrelator_reflectometry::set_input_output_vectors_RHCP_reflectometry(std::complex<float>* corr_out, const std::complex<float>* sig_in)
{
    // Save CPU pointers
    d_sig_in_reflectometry_RHCP = sig_in;
    d_corr_out_reflectometry_RHCP = corr_out;
    return true;
}

void cpu_multicorrelator_reflectometry::update_local_code(int correlator_length_samples, float rem_code_phase_chips, float code_phase_step_chips)
{
    volk_gnsssdr_32fc_xn_resampler_32fc_xn(d_local_codes_resampled,
                                           d_local_code_in,
                                           rem_code_phase_chips,
                                           code_phase_step_chips,
                                           d_shifts_chips,
                                           d_code_length_chips,
                                           d_n_correlators,
                                           correlator_length_samples);

    //############ Modification Algorithme de reflectometrie ############
    volk_gnsssdr_32fc_xn_resampler_32fc_xn(d_local_codes_resampled_reflectometry_LHCP,
                                           d_local_code_in,
                                           rem_code_phase_chips,
                                           code_phase_step_chips,
                                           d_shifts_chips_reflecto,
                                           d_code_length_chips,
                                           d_n_correlators_reflectometry,
                                           correlator_length_samples);

    float d_shifts_chips_noDelay = 0;

    if(d_CalibrationOutputEnabled){
        volk_gnsssdr_32fc_xn_resampler_32fc_xn(d_local_codes_resampled_reflectometry_noDelay,
                                               d_local_code_in,
                                               rem_code_phase_chips,
                                               code_phase_step_chips,
                                               &d_shifts_chips_noDelay,
                                               d_code_length_chips,
                                               1,
                                               correlator_length_samples);
    }

    if(d_RHCP_enabled==true){
        volk_gnsssdr_32fc_xn_resampler_32fc_xn(d_local_codes_resampled_reflectometry_RHCP,
                                               d_local_code_in,
                                               rem_code_phase_chips,
                                               code_phase_step_chips,
                                               d_shifts_chips_reflecto,
                                               d_code_length_chips,
                                               d_n_correlators_reflectometry,
                                               correlator_length_samples);
    }
    //################# Fin de la modification #################
}


bool cpu_multicorrelator_reflectometry::Carrier_wipeoff_multicorrelator_resampler(
        float rem_carrier_phase_in_rad,
        float phase_step_rad,
        float rem_code_phase_chips,
        float code_phase_step_chips,
        int signal_length_samples)
{
    update_local_code(signal_length_samples, rem_code_phase_chips, code_phase_step_chips);
    // Regenerate phase at each call in order to avoid numerical issues
    lv_32fc_t phase_offset_as_complex[1];
    phase_offset_as_complex[0] = lv_cmake(std::cos(rem_carrier_phase_in_rad), -std::sin(rem_carrier_phase_in_rad));
    // call VOLK_GNSSSDR kernel
    lv_32fc_t phase_offset_copy[1];
    lv_32fc_t phase_offset_copy_copy[1];
    lv_32fc_t phase_offset_copy_copy_copy[1];
    phase_offset_copy[0] = phase_offset_as_complex[0];
    phase_offset_copy_copy[0] = phase_offset_as_complex[0];
    phase_offset_copy_copy_copy[0] = phase_offset_as_complex[0];

    volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn(d_corr_out, d_sig_in, std::exp(lv_32fc_t(0, - phase_step_rad)), phase_offset_as_complex, (const lv_32fc_t**)d_local_codes_resampled, d_n_correlators, signal_length_samples);
    volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn(d_corr_out_reflectometry_LHCP, d_sig_in_reflectometry_LHCP, std::exp(lv_32fc_t(0, - phase_step_rad)), phase_offset_copy, (const lv_32fc_t**)d_local_codes_resampled_reflectometry_LHCP, d_n_correlators_reflectometry, signal_length_samples);
    if(d_CalibrationOutputEnabled){
        volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn(d_corr_out_reflectometry_noDelay, d_sig_in_reflectometry_LHCP, std::exp(lv_32fc_t(0, - phase_step_rad)), phase_offset_copy_copy, (const lv_32fc_t**)d_local_codes_resampled_reflectometry_noDelay, 1, signal_length_samples);
    }
    if(d_RHCP_enabled){
        volk_gnsssdr_32fc_x2_rotator_dot_prod_32fc_xn(d_corr_out_reflectometry_RHCP, d_sig_in_reflectometry_RHCP, std::exp(lv_32fc_t(0, - phase_step_rad)), phase_offset_copy_copy_copy, (const lv_32fc_t**)d_local_codes_resampled_reflectometry_RHCP, d_n_correlators_reflectometry, signal_length_samples);
    }
    return true;
}


bool cpu_multicorrelator_reflectometry::free()
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
    // Free memory
    if (d_local_codes_resampled_reflectometry_LHCP != nullptr)
    {
        for (int n = 0; n < d_n_correlators_reflectometry; n++)
        {
            volk_gnsssdr_free(d_local_codes_resampled_reflectometry_LHCP[n]);
        }
        volk_gnsssdr_free(d_local_codes_resampled_reflectometry_LHCP);
        d_local_codes_resampled_reflectometry_LHCP = nullptr;
    }
    // Free memory
    if (d_local_codes_resampled_reflectometry_noDelay != nullptr)
    {
        volk_gnsssdr_free(d_local_codes_resampled_reflectometry_noDelay[0]);
        volk_gnsssdr_free(d_local_codes_resampled_reflectometry_noDelay);
        d_local_codes_resampled_reflectometry_noDelay = nullptr;
    }
    // Free memory
    if (d_local_codes_resampled_reflectometry_RHCP != nullptr)
    {
        for (int n = 0; n < d_n_correlators_reflectometry; n++)
        {
            volk_gnsssdr_free(d_local_codes_resampled_reflectometry_RHCP[n]);
        }
        volk_gnsssdr_free(d_local_codes_resampled_reflectometry_RHCP);
        d_local_codes_resampled_reflectometry_RHCP = nullptr;
    }
    return true;
}

