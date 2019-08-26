/*!
 * \file cpu_multicorrelator_reflectometry.h
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

#ifndef GNSS_SDR_CPU_MULTICORRELATOR_REFLECTOMETRY_H_
#define GNSS_SDR_CPU_MULTICORRELATOR_REFLECTOMETRY_H_


#include <complex>

/*!
 * \brief Class that implements carrier wipe-off and correlators.
 */
class cpu_multicorrelator_reflectometry
{
public:
    cpu_multicorrelator_reflectometry();
    ~cpu_multicorrelator_reflectometry();
    bool init(int max_signal_length_samples, int n_correlators);
    bool init_reflectometry(int max_signal_length_samples, int n_correlators, int n_voie, bool CalibrationOutputEnabled = false);
    bool set_local_code_and_taps(int code_length_chips, const std::complex<float>* local_code_in, float* shifts_chips, float* shifts_chips_reflectometry); //Was float *shifts_chips_reflectometry
    bool set_delay_reflecto(float delay);
    bool set_input_output_vectors(std::complex<float>* corr_out, const std::complex<float>* sig_in);
    bool set_input_output_vectors_LHCP_reflectometry(std::complex<float>* corr_out, const std::complex<float>* sig_in);
    bool set_input_output_vectors_RHCP_reflectometry(std::complex<float>* corr_out, const std::complex<float>* sig_in);
    bool set_output_noDelay_reflectometry(std::complex<float>* corr_out_noDelay);
    void update_local_code(int correlator_length_samples, float rem_code_phase_chips, float code_phase_step_chips);
    bool Carrier_wipeoff_multicorrelator_resampler(float rem_carrier_phase_in_rad, float phase_step_rad, float rem_code_phase_chips, float code_phase_step_chips, int signal_length_samples);
    bool free();

private:
    // Allocate the device input vectors
    const std::complex<float> *d_sig_in;
    const std::complex<float> *d_sig_in_reflectometry_LHCP;
    const std::complex<float> *d_sig_in_reflectometry_RHCP;
    std::complex<float> **d_local_codes_resampled;
    std::complex<float> **d_local_codes_resampled_reflectometry_LHCP;
    std::complex<float> **d_local_codes_resampled_reflectometry_noDelay;
    std::complex<float> **d_local_codes_resampled_reflectometry_RHCP;
    const std::complex<float> *d_local_code_in;
    const std::complex<float> *d_local_code_in_delayed;
    std::complex<float> *d_corr_out;
    std::complex<float> *d_corr_out_reflectometry_LHCP;
    std::complex<float> *d_corr_out_reflectometry_noDelay;
    std::complex<float> *d_corr_out_reflectometry_RHCP;
    bool d_CalibrationOutputEnabled;
    float *d_shifts_chips;
    float *d_shifts_chips_reflecto;
    float d_delay_reflectometry;
    int d_code_length_chips;
    int d_n_correlators;
    int d_n_correlators_reflectometry;
    int d_RHCP_enabled;
};


#endif /* CPU_MULTICORRELATOR_REFLECTOMETRY_H_ */
