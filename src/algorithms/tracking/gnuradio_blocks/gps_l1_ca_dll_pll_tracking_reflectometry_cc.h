/*!
 * \file gps_l1_ca_dll_pll_tracking_reflectometry_cc.h
 * \brief Interface of a code DLL + carrier PLL tracking block
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency Approach,
 * Birkhauser, 2007
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

#ifndef GNSS_SDR_GPS_L1_CA_DLL_PLL_TRACKING_REFLECTOMETRY_CC_H
#define GNSS_SDR_GPS_L1_CA_DLL_PLL_TRACKING_REFLECTOMETRY_CC_H

#include <fstream>
#include <map>
#include <string>
#include <gnuradio/block.h>
#include "gnss_synchro.h"
#include "tracking_2nd_DLL_filter.h"
#include "tracking_2nd_PLL_filter.h"
#include "cpu_multicorrelator_reflectometry.h"

class Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc;

using gps_l1_ca_dll_pll_tracking_reflectometry_cc_sptr = boost::shared_ptr<Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc>;

gps_l1_ca_dll_pll_tracking_reflectometry_cc_sptr gps_l1_ca_dll_pll_make_tracking_reflectometry_cc(
                   long if_freq,
                   long fs_in,
                   unsigned int vector_length,
                   bool dump,
                   std::string dump_filename,
                   float pll_bw_hz,
                   float dll_bw_hz,
                   float early_late_space_chips,
				   float early_late_space_chips_reflecto,
                   int nbCorr, std::string elev_filename, int recept_height, int in_streams, bool calibrationOutputEnabled,
                    std::string fileInPath, bool enableOutputFilter);

/*!
 * \brief This class implements a DLL + PLL tracking loop block
 */
class Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc: public gr::block
{
public:
    ~Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc();

    void set_channel(uint32_t channel);
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);
    void start_tracking();
    void stop_tracking();

    int general_work(int noutput_items, gr_vector_int& ninput_items,
        gr_vector_const_void_star& input_items, gr_vector_void_star& output_items);

    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

private:
    friend gps_l1_ca_dll_pll_tracking_reflectometry_cc_sptr
    gps_l1_ca_dll_pll_make_tracking_reflectometry_cc(
            long if_freq,
            long fs_in,
            unsigned int vector_length,
            bool dump,
            std::string dump_filename,
            float pll_bw_hz,
            float dll_bw_hz,
            float early_late_space_chips,
			float early_late_space_chips_reflecto,
            int nbCorr, std::string elev_filename, int recept_height, int in_streams, bool calibrationOutputEnabled, std::string FileInPath, bool enabledOutputFilter);

    Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc(long if_freq,
            long fs_in, unsigned
            int vector_length,
            bool dump,
            std::string dump_filename,
            float pll_bw_hz,
            float dll_bw_hz,
            float early_late_space_chips,
			float early_late_space_chips_reflecto,
            int nbCorr, std::string elev_filename, int recept_height, int in_streams, bool calibrationOutputEnabled, std::string fileInPath, const std::vector<int> output_size, bool enabledOutputFilter);

    // tracking configuration vars
    unsigned int d_vector_length;
    bool d_dump;

    Gnss_Synchro* d_acquisition_gnss_synchro;
    unsigned int d_channel;

    long d_if_freq;
    long d_fs_in;

    double d_early_late_spc_chips;
    //////////////////////////////////////////
    double d_early_late_spc_chips_reflecto;
    ////////////////////////////////////////////

    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    double d_rem_code_phase_chips;
    double d_rem_carr_phase_rad;

    // PLL and DLL filter library
    Tracking_2nd_DLL_filter d_code_loop_filter;
    Tracking_2nd_PLL_filter d_carrier_loop_filter;

    // acquisition
    double d_acq_code_phase_samples;
    double d_acq_carrier_doppler_hz;
    // correlator
    int d_n_correlator_taps;
    gr_complex* d_ca_code;
    float* d_local_code_shift_chips;
    ///////////////////////////////////////////////
    float* d_local_code_shift_chips_reflecto;
    ///////////////////////////////////////////////
    gr_complex* d_correlator_outs;
    cpu_multicorrelator_reflectometry multicorrelator_cpu;


    // tracking vars
    double d_code_freq_chips;
    double d_code_phase_step_chips;
    double d_carrier_doppler_hz;
    double d_carrier_phase_step_rad;
    double d_acc_carrier_phase_rad;
    double d_code_phase_samples;
    double d_acc_code_phase_secs;

    //PRN period in samples
    int d_current_prn_length_samples;

    //processing samples counters
    unsigned long int d_sample_counter;
    unsigned long int d_acq_sample_stamp;

    // CN0 estimation and lock detector
    int d_cn0_estimation_counter;
    gr_complex* d_Prompt_buffer;
    double d_carrier_lock_test;
    double d_CN0_SNV_dB_Hz;
    double d_carrier_lock_threshold;
    int d_carrier_lock_fail_counter;

    // control vars
    bool d_enable_tracking;
    bool d_pull_in;

    //############ Modification Algorithme de reflectometrie ############
    // file dump
    std::string d_dump_filename_LHCP_way;
    std::string d_dump_filename_RHCP_way;
    std::string d_dump_filename_LHCP_wayLong;
    std::string d_dump_filename_LHCP_noDelay;
    std::string d_dump_filename_RHCP_wayLong;
    std::ofstream d_dump_file_LHCP_way;
    std::ofstream d_dump_file_LHCP_noDelay;
    std::ofstream d_dump_file_RHCP_way;

    // param
    int nbCorr_;
    float d_delay_reflectometry;
    void delay_calculation_reflectometry();
    std::string d_elev_filename;
    unsigned int in_streams_;
    double d_recept_height;
    std::ifstream e_file;
    int sample_counter_reflecto;
    bool d_CalibrationOutputEnabled;
    gr_complex* d_code_delayed;
    gr_complex* d_out_corr_LHCP;
    gr_complex* d_out_corr_noDelay;
    gr_complex* d_out_corr_RHCP;
    std::string fileInPath_;
    std::vector<float> reflected_corr_outputs_LHCP_way;
    std::vector<float> reflected_corr_outputs_noDelay;
    std::vector<float> reflected_corr_outputs_RHCP_way;
    bool enabledOutputFilter_;
    //################# Fin de la modification #################

    std::map<std::string, std::string> systemName;
    std::string sys;
};

#endif //GNSS_SDR_GPS_L1_CA_DLL_PLL_TRACKING_REFLECTOMETRY_CC_H
