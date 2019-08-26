/*!
 * \file gps_l1_ca_dll_pll_tracking_reflectometry_cc.cc
 * \brief Implementation of a code DLL + carrier PLL tracking block
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Javier Arribas, 2011. jarribas(at)cttc.es
 *
 * Code DLL + carrier PLL according to the algorithms described in:
 * [1] K.Borre, D.M.Akos, N.Bertelsen, P.Rinder, and S.H.Jensen,
 * A Software-Defined GPS and Galileo Receiver. A Single-Frequency
 * Approach, Birkhauser, 2007
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

#include "gps_l1_ca_dll_pll_tracking_reflectometry_cc.h"
#include <cmath>
#include <iostream>
#include <memory>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <gnuradio/io_signature.h>
#include <glog/logging.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include "gps_sdr_signal_processing.h"
#include "tracking_discriminators.h"
#include "lock_detectors.h"
#include "GPS_L1_CA.h"
//#include "control_message_factory.h"


/*!
 * \todo Include in definition header file
 */
#define CN0_ESTIMATION_SAMPLES 20
#define MINIMUM_VALID_CN0 25
#define MAXIMUM_LOCK_FAIL_COUNTER 50
#define CARRIER_LOCK_THRESHOLD 0.85


using google::LogMessage;

gps_l1_ca_dll_pll_tracking_reflectometry_cc_sptr
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
        int nbCorr,
        std::string elev_filename,              // to be changed : elevations computed by GNSS-SDR
        int recept_height,                      // to be changed : receiver height computed by GNSS-SDR
        int in_streams,
        bool calibrationOutputEnabled, std::string fileInPath, bool enabledOutputFilter)
{
    std::vector<int> output_sizes;
    output_sizes.push_back(sizeof(Gnss_Synchro));
    for(int i=0; i<2*(nbCorr+3); i++){
        output_sizes.push_back(sizeof(float));
    }
    return gps_l1_ca_dll_pll_tracking_reflectometry_cc_sptr(new Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc(if_freq,
                                                                                                            fs_in,
																											vector_length,
																											dump,
																											dump_filename,
																											pll_bw_hz,
																											dll_bw_hz,
																											early_late_space_chips,
																											early_late_space_chips_reflecto,
																											nbCorr,
																											elev_filename,
																											recept_height,
																											in_streams,
																											calibrationOutputEnabled,
																											fileInPath, output_sizes,
																											enabledOutputFilter));
}



void Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc::forecast (int noutput_items,
                                                            gr_vector_int &ninput_items_required)
{
    if (noutput_items != 0)
    {
        ninput_items_required[0] = static_cast<int>(d_vector_length) * 2; //set the required available samples in each call
        //############ Modification Algorithme de reflectometrie ############
        ninput_items_required[in_streams_-1] = static_cast<int>(d_vector_length) * 2; //set the required available samples in each call
        ninput_items_required[in_streams_] = static_cast<int>(d_vector_length) * 2; //set the required available samples in each call
        //################# Fin de la modification #################
    }
}



Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc::Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc(long if_freq,
                                                                                         long fs_in,
                                                                                         unsigned int vector_length,
                                                                                         bool dump,
                                                                                         std::string dump_filename,
                                                                                         float pll_bw_hz,
                                                                                         float dll_bw_hz,
                                                                                         float early_late_space_chips,float early_late_space_chips_reflecto,
																						 int nbCorr, std::string elev_filename, int recept_height, int in_streams, bool calibrationOutputEnabled, std::string fileInPath, std::vector<int> output_size, bool enabledOutputFilter) :
    gr::block("Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc", gr::io_signature::make(in_streams, in_streams, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    std::cout << enabledOutputFilter*2*(nbCorr+3)+1 << std::endl;
    // Telemetry bit synchronization message port input
    this->message_port_register_out(pmt::mp("events"));
    this->message_port_register_in(pmt::mp("telemetry_to_trk"));

    // initialize internal vars
    enabledOutputFilter_ = enabledOutputFilter;
    in_streams_ = in_streams;
    d_dump = dump;
    d_if_freq = if_freq;
    d_fs_in = fs_in;
    d_vector_length = vector_length;

    d_dump_filename_LHCP_way = dump_filename;
    d_dump_filename_RHCP_way = dump_filename;
    nbCorr_ = nbCorr;
    fileInPath_ = fileInPath;
    d_elev_filename = elev_filename;
    d_recept_height = recept_height;
    d_current_prn_length_samples = static_cast<int>(d_vector_length);

    // Initialize tracking  ==========================================
    d_code_loop_filter.set_DLL_BW(dll_bw_hz);
    d_carrier_loop_filter.set_PLL_BW(pll_bw_hz);

    //--- DLL variables --------------------------------------------------------
    d_early_late_spc_chips = early_late_space_chips; // Define early-late offset (in chips)

    // Initialization of local code replica
    // Get space for a vector with the C/A code replica sampled 1x/chip
    d_ca_code = static_cast<gr_complex*>(volk_gnsssdr_malloc(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS) * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
      // correlator outputs (scalar)
    d_n_correlator_taps = 3; // Early, Prompt, and Late
    d_correlator_outs = static_cast<gr_complex*>(volk_gnsssdr_malloc(d_n_correlator_taps*sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    for (int n = 0; n <d_n_correlator_taps; n++)
    {
        d_correlator_outs[n] = gr_complex(0,0);
    }

    d_local_code_shift_chips = static_cast<float*>(volk_gnsssdr_malloc(d_n_correlator_taps*sizeof(float), volk_gnsssdr_get_alignment()));
    // Set TAPs delay values [chips]
    d_local_code_shift_chips[0] = - d_early_late_spc_chips;
    d_local_code_shift_chips[1] = 0.0;
    d_local_code_shift_chips[2] = d_early_late_spc_chips;

    multicorrelator_cpu.init(2 * d_current_prn_length_samples, d_n_correlator_taps);

    //############ Modification Algorithme de reflectometrie ############
    d_CalibrationOutputEnabled = calibrationOutputEnabled;
    sample_counter_reflecto = 0;

    multicorrelator_cpu.init_reflectometry(2 * d_current_prn_length_samples, nbCorr_, in_streams_, d_CalibrationOutputEnabled);

    d_code_delayed = static_cast<gr_complex*>(volk_gnsssdr_malloc(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS) * sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_out_corr_LHCP = static_cast<gr_complex*>(volk_gnsssdr_malloc(nbCorr_*sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_out_corr_noDelay = static_cast<gr_complex*>(volk_gnsssdr_malloc(sizeof(gr_complex), volk_gnsssdr_get_alignment()));
    d_out_corr_RHCP = static_cast<gr_complex*>(volk_gnsssdr_malloc(nbCorr_*sizeof(gr_complex), volk_gnsssdr_get_alignment()));
   for (int n = 0; n < nbCorr_; n++){
        d_out_corr_LHCP[n] = gr_complex(0,0);
        d_out_corr_RHCP[n] = gr_complex(0,0);
    }
    d_out_corr_noDelay[0] = gr_complex(0,0);


    //################# Fin de la modification #################

    //--- Perform initializations ------------------------------
    // define initial code frequency basis of NCO
    d_code_freq_chips = GPS_L1_CA_CODE_RATE_HZ;
    // define residual code phase (in chips)
    d_rem_code_phase_samples = 0.0;
    // define residual carrier phase
    d_rem_carr_phase_rad = 0.0;

    // sample synchronization
    d_sample_counter = 0;
    //d_sample_counter_seconds = 0;
    d_acq_sample_stamp = 0;

    d_enable_tracking = false;
    d_pull_in = false;

    // CN0 estimation and lock detector buffers
    d_cn0_estimation_counter = 0;
    d_Prompt_buffer = new gr_complex[CN0_ESTIMATION_SAMPLES];
    d_carrier_lock_test = 1;
    d_CN0_SNV_dB_Hz = 0;
    d_carrier_lock_fail_counter = 0;
    d_carrier_lock_threshold = CARRIER_LOCK_THRESHOLD;

    systemName["G"] = std::string("GPS");
    systemName["S"] = std::string("SBAS");

    d_acquisition_gnss_synchro = 0;
    d_channel = 0;
    d_acq_code_phase_samples = 0.0;
    d_acq_carrier_doppler_hz = 0.0;
    d_carrier_doppler_hz = 0.0;
    d_acc_carrier_phase_rad = 0.0;
    d_code_phase_samples = 0.0;
    d_acc_code_phase_secs = 0.0;
    d_rem_code_phase_chips = 0.0;
    d_code_phase_step_chips = 0.0;
    d_carrier_phase_step_rad = 0.0;

    set_relative_rate(1.0 / static_cast<double>(d_vector_length));
    //////////////////////////////////////////////////Modification Ray//////////////////////////////////////////////////////
    		d_early_late_spc_chips_reflecto = early_late_space_chips_reflecto;
        	d_local_code_shift_chips_reflecto = static_cast<float*>(volk_gnsssdr_malloc(d_n_correlator_taps*sizeof(float), volk_gnsssdr_get_alignment()));
                       	   d_local_code_shift_chips_reflecto[0] = - d_early_late_spc_chips_reflecto;
                           d_local_code_shift_chips_reflecto[1] = 0.0;
                           d_local_code_shift_chips_reflecto[2] = d_early_late_spc_chips_reflecto;

                           // for(int i=0; i<nbCorr_; i++){

               //   d_local_code_shift_chips_reflecto[i] = (-(nbCorr_-1)/2)*d_early_late_spc_chips_reflecto + i*d_early_late_spc_chips_reflecto;

                 //   }
                   //if (i<(nbCorr_-1)/2){
                       //d_local_code_shift_chips_reflecto[i] = d_early_late_spc_chips_reflecto; //((-(nbCorr_-1)/2)+i)*-d_early_late_spc_chips_reflecto;
                		// if (i=((nbCorr_-1)/2)){
                		//	 d_local_code_shift_chips_reflecto[i] = 0.0;
                         //}
                		 //if (i>(nbCorr_-1)/2){
                		//	 d_local_code_shift_chips_reflecto[i] = d_early_late_spc_chips_reflecto; //(i-((nbCorr_-1)/2))*-d_early_late_spc_chips_reflecto;
                		// }
                       //}

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}


void Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc::start_tracking()
{
    /*
     *  correct the code phase according to the delay between acq and trk
     */
    std::cout << "Start tracking\n";
    d_acq_code_phase_samples = d_acquisition_gnss_synchro->Acq_delay_samples;
    d_acq_carrier_doppler_hz = d_acquisition_gnss_synchro->Acq_doppler_hz;
    d_acq_sample_stamp = d_acquisition_gnss_synchro->Acq_samplestamp_samples;

    long int acq_trk_diff_samples;
    double acq_trk_diff_seconds;
    acq_trk_diff_samples = static_cast<long int>(d_sample_counter) - static_cast<long int>(d_acq_sample_stamp);//-d_vector_length;
    DLOG(INFO) << "Number of samples between Acquisition and Tracking =" << acq_trk_diff_samples;
    acq_trk_diff_seconds = static_cast<float>(acq_trk_diff_samples) / static_cast<float>(d_fs_in);
    //doppler effect
    // Fd=(C/(C+Vr))*F
    double radial_velocity = (GPS_L1_FREQ_HZ + d_acq_carrier_doppler_hz) / GPS_L1_FREQ_HZ;
    // new chip and prn sequence periods based on acq Doppler
    double T_chip_mod_seconds;
    double T_prn_mod_seconds;
    double T_prn_mod_samples;
    d_code_freq_chips = radial_velocity * GPS_L1_CA_CODE_RATE_HZ;
    d_code_phase_step_chips = static_cast<double>(d_code_freq_chips) / static_cast<double>(d_fs_in);
    T_chip_mod_seconds = 1/d_code_freq_chips;
    T_prn_mod_seconds = T_chip_mod_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
    T_prn_mod_samples = T_prn_mod_seconds * static_cast<double>(d_fs_in);

    d_current_prn_length_samples = round(T_prn_mod_samples);

    double T_prn_true_seconds = GPS_L1_CA_CODE_LENGTH_CHIPS / GPS_L1_CA_CODE_RATE_HZ;
    double T_prn_true_samples = T_prn_true_seconds * static_cast<double>(d_fs_in);
    double T_prn_diff_seconds = T_prn_true_seconds - T_prn_mod_seconds;
    double N_prn_diff = acq_trk_diff_seconds / T_prn_true_seconds;
    double corrected_acq_phase_samples, delay_correction_samples;
    corrected_acq_phase_samples = fmod((d_acq_code_phase_samples + T_prn_diff_seconds * N_prn_diff * static_cast<double>(d_fs_in)), T_prn_true_samples);
    if (corrected_acq_phase_samples < 0)
    {
        corrected_acq_phase_samples = T_prn_mod_samples + corrected_acq_phase_samples;
    }
    delay_correction_samples = d_acq_code_phase_samples - corrected_acq_phase_samples;

    d_acq_code_phase_samples = corrected_acq_phase_samples;

    d_carrier_doppler_hz = d_acq_carrier_doppler_hz;
    d_carrier_phase_step_rad = GPS_TWO_PI * d_carrier_doppler_hz / static_cast<double>(d_fs_in);

    // DLL/PLL filter initialization
    d_carrier_loop_filter.initialize(); // initialize the carrier filter
    d_code_loop_filter.initialize();    // initialize the code filter

    // generate local reference ALWAYS starting at chip 1 (1 sample per chip)
    gps_l1_ca_code_gen_complex(d_ca_code, d_acquisition_gnss_synchro->PRN, 0);

    //############ Modification for reflectometry ############
    multicorrelator_cpu.set_local_code_and_taps(static_cast<int>(GPS_L1_CA_CODE_LENGTH_CHIPS), d_ca_code,d_local_code_shift_chips, d_local_code_shift_chips_reflecto); //d_local_code_shift_chips_reflecto
    for (int n = 0; n <d_n_correlator_taps; n++)
    {
       d_correlator_outs[n] = gr_complex(0,0);
    }
    for (int n = 0; n < nbCorr_; n++){
        d_out_corr_LHCP[n] = gr_complex(0,0);
        d_out_corr_RHCP[n] = gr_complex(0,0);
    }
    //################# End of reflectometry modification #################

    d_carrier_lock_fail_counter = 0;
    d_rem_code_phase_samples = 0;
    d_rem_carr_phase_rad = 0.0;
    d_rem_code_phase_chips = 0.0;
    d_acc_carrier_phase_rad = 0.0;
    d_acc_code_phase_secs = 0.0;

    d_code_phase_samples = d_acq_code_phase_samples;

    std::string sys_ = &d_acquisition_gnss_synchro->System;
    sys = sys_.substr(0,1);

    // DEBUG OUTPUT-
    std::cout << "Tracking start on channel " << d_channel << " for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " whith Doppler="<<d_acq_carrier_doppler_hz<<" Hz"<< std::endl;
    LOG(INFO) << "Starting tracking of satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " on  channel " << d_channel;

    // enable tracking
    d_pull_in = true;
    d_enable_tracking = true;

    LOG(INFO) << "PULL-IN Doppler [Hz]=" << d_carrier_doppler_hz
              << " Code Phase correction [samples]=" << delay_correction_samples
              << " PULL-IN Code Phase [samples]=" << d_acq_code_phase_samples;

    //############ Modification for reflectometry ############
    if(d_dump == true){
        if(d_dump_file_LHCP_way.is_open()){
            d_dump_file_LHCP_way.close();
        }
        try{
            boost::filesystem::path p(fileInPath_);
            d_dump_filename_LHCP_wayLong = d_dump_filename_LHCP_way+std::string("Raw_Corr_D3R_R1L_1C_G")+boost::lexical_cast<std::string>(d_acquisition_gnss_synchro->PRN)+std::string("_")+p.filename().c_str();
            d_dump_file_LHCP_way.open(d_dump_filename_LHCP_wayLong.c_str(), std::ios::out | std::ios::trunc);
            LOG(INFO) << "Tracking dump enabled on channel" << d_channel << " dumpfile:" <<  d_dump_filename_LHCP_wayLong.c_str() << "\n";
        }
        catch(std::ifstream::failure e){
            LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file:" << e.what() << "\n";
        }
        if(d_dump_file_LHCP_noDelay.is_open()){
            d_dump_file_LHCP_noDelay.close();
        }
// [LL] Il semble y avoir un pb dans ce qui suit : même nom d_dump_filename_LHCP_wayLong utilisé
        if(d_CalibrationOutputEnabled == true){
            try{
                boost::filesystem::path p(fileInPath_);
                d_dump_filename_LHCP_wayLong = d_dump_filename_LHCP_way+std::string("Raw_Corr_D3R_R1L_1C_G")+boost::lexical_cast<std::string>(d_acquisition_gnss_synchro->PRN)+std::string("_")+p.filename().c_str();
                d_dump_file_LHCP_noDelay.open(d_dump_filename_LHCP_wayLong.c_str(), std::ios::out | std::ios::trunc);
                LOG(INFO) << "Tracking dump no delay enabled on channel" << d_channel << " dumpfile:" <<  d_dump_filename_LHCP_wayLong.c_str() << "\n";
                std::cout << "Tracking dump no delay enabled on channel" << d_channel << " dumpfile:" <<  d_dump_filename_LHCP_wayLong.c_str() << "\n";
            }
            catch(std::ifstream::failure e){
                LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump no delay file:" << e.what() << "\n";
            }
        }
// Case of dual RHCP+LHCP reflected antenna
        if(in_streams_ == 3){
            if(d_dump_file_RHCP_way.is_open()){
                d_dump_file_RHCP_way.close();
            }
 /*          try{
                d_dump_filename_RHCP_wayLong = d_dump_filename_RHCP_way;
                d_dump_filename_RHCP_wayLong.append(boost::lexical_cast<std::string>(nbCorr_));
                d_dump_filename_RHCP_wayLong.append("_corr_tracking_RHCP_PRN_");
                d_dump_filename_RHCP_wayLong.append(boost::lexical_cast<std::string>(d_acquisition_gnss_synchro->PRN));
                d_dump_filename_RHCP_wayLong.append(".txt");
                d_dump_file_RHCP_way.open(d_dump_filename_RHCP_wayLong.c_str(), std::ios::out | std::ios::trunc);
                LOG(INFO) << "Tracking dump RHCP enabled on channel" << d_channel << " dumpfile:" <<  d_dump_filename_RHCP_wayLong.c_str() << "\n";
            }
            catch(std::ifstream::failure e){
                LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump RHCP file:" << e.what() << "\n";
            }
        }
   }*/

    try{
              boost::filesystem::path p(fileInPath_);
              d_dump_filename_RHCP_wayLong = d_dump_filename_RHCP_way+std::string("Raw_Corr_D3R_R1R_1C_G")+boost::lexical_cast<std::string>(d_acquisition_gnss_synchro->PRN)+std::string("_")+p.filename().c_str();
              d_dump_file_RHCP_way.open(d_dump_filename_RHCP_wayLong.c_str(), std::ios::out | std::ios::trunc);
              LOG(INFO) << "Tracking dump RHCP enabled on channel" << d_channel << " dumpfile:" <<  d_dump_filename_RHCP_wayLong.c_str() << "\n";
          }
          catch(std::ifstream::failure e){
              LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump RHCP file:" << e.what() << "\n";
          }
        }
    }

// XXX Use of a file containing sat. elevations : to be changed in a function that computes elevations
    std::string e_filename = d_elev_filename;
    if(e_file.is_open() == false){
        e_file.close();
    }
    try{
        e_filename.append(boost::lexical_cast<std::string>(d_acquisition_gnss_synchro->PRN)),
                e_filename.append(".txt");
        e_file.open(e_filename.c_str(), std::ifstream::in | std::ifstream::binary);
    }
    catch(std::ifstream::failure e){
        LOG(WARNING) << "Error opening file containing elevations:" << e.what() << "\n";
    }
    delay_calculation_reflectometry();
    multicorrelator_cpu.set_delay_reflecto(d_delay_reflectometry);
    //################# End of reflectometry modification #################
}

void Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc::stop_tracking()
{
}


Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc::~Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc()
{
    //d_dump_file.close();
    //############ Modification Algorithme de reflectometrie ############
    d_dump_file_LHCP_way.close();
    d_dump_file_RHCP_way.close();
    e_file.close();
    //################# Fin de la modification #################
    volk_gnsssdr_free(d_local_code_shift_chips);
    volk_gnsssdr_free(d_local_code_shift_chips_reflecto);
    volk_gnsssdr_free(d_correlator_outs);
    volk_gnsssdr_free(d_ca_code);
    volk_gnsssdr_free(d_out_corr_LHCP);
    volk_gnsssdr_free(d_out_corr_RHCP);
    volk_gnsssdr_free(d_out_corr_noDelay);
    volk_gnsssdr_free(d_code_delayed);
    delete[] d_Prompt_buffer;
    multicorrelator_cpu.free();
}

// [LL] fonction suivante à revoir : calcul de l'élévation et de la hauteur du Rx
void Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc::delay_calculation_reflectometry(){
    std::string line;
    e_file.clear();
    e_file.seekg(0, std::ios::beg);
    std::getline(e_file, line);
    std::stringstream iss(line);
    double elevation;
    iss >> elevation;
    d_delay_reflectometry = 2*d_recept_height*sin(elevation*(M_PI/180));
    d_delay_reflectometry = d_delay_reflectometry*(GPS_L1_CA_CODE_RATE_HZ/GPS_C_M_S);
    LOG(INFO) << "Delay reflectometry for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " (elev=" << elevation << "°):" << d_delay_reflectometry << "\n";
    std::cout << "Delay reflectometry for satellite " << Gnss_Satellite(systemName[sys], d_acquisition_gnss_synchro->PRN) << " (elev=" << elevation << "°):" << d_delay_reflectometry << "\n";
}

int Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc::general_work (int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
                                                               gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{
    // process vars
    double carr_error_hz = 0.0;
    double carr_error_filt_hz = 0.0;
    double code_error_chips = 0.0;
    double code_error_filt_chips = 0.0;

    // Block input data and block output stream pointers
    const gr_complex* in = (gr_complex*) input_items[0]; //PRN start block alignment
    //############ Modification Algorithme de reflectometrie ############
    const gr_complex* inLHCP;
    const gr_complex* inRHCP;

    inLHCP = (gr_complex*)input_items[ninput_items.size()-1];
    if(in_streams_==3){
        inLHCP = (gr_complex*)input_items[ninput_items.size()-2];
        inRHCP = (gr_complex*)input_items[ninput_items.size()-1];
    }
    //################# Fin de la modification #################
    Gnss_Synchro **out = (Gnss_Synchro **) &output_items[0];

    // GNSS_SYNCHRO OBJECT to interchange data between tracking->telemetry_decoder
    Gnss_Synchro current_synchro_data = Gnss_Synchro();

    if (d_enable_tracking == true)
    {
        // Fill the acquisition data
        current_synchro_data = *d_acquisition_gnss_synchro;
        // Receiver signal alignment
        if (d_pull_in == true)
        {
            int samples_offset;
            double acq_trk_shif_correction_samples;
            int acq_to_trk_delay_samples;
            acq_to_trk_delay_samples = d_sample_counter - d_acq_sample_stamp;
            acq_trk_shif_correction_samples = d_current_prn_length_samples - fmod(static_cast<float>(acq_to_trk_delay_samples), static_cast<float>(d_current_prn_length_samples));
            samples_offset = round(d_acq_code_phase_samples + acq_trk_shif_correction_samples);
            current_synchro_data.Tracking_sample_counter = d_sample_counter + static_cast<uint64_t>(samples_offset);
            d_sample_counter = d_sample_counter + samples_offset; //count for the processed samples
            sample_counter_reflecto = sample_counter_reflecto+1;
            d_pull_in = false;
            *out[0] = current_synchro_data;
            consume_each(samples_offset); //shift input to perform alignment with local replica
            return 1;
        }

        // ################# CARRIER WIPEOFF AND CORRELATORS ##############################
        // perform carrier wipe-off and compute Early, Prompt and Late correlation
        multicorrelator_cpu.set_input_output_vectors(d_correlator_outs, in);
        //############ Modification Algorithme de reflectometrie ############
        multicorrelator_cpu.set_input_output_vectors_LHCP_reflectometry(d_out_corr_LHCP, inLHCP);
        if(d_CalibrationOutputEnabled){
            multicorrelator_cpu.set_output_noDelay_reflectometry(d_out_corr_noDelay);
        }
        if(in_streams_== 3){
            multicorrelator_cpu.set_input_output_vectors_RHCP_reflectometry(d_out_corr_RHCP, inRHCP);
        }
        //################# Fin de la modification #################
        multicorrelator_cpu.Carrier_wipeoff_multicorrelator_resampler(d_rem_carr_phase_rad,
                                                                      d_carrier_phase_step_rad,
                                                                      d_rem_code_phase_chips,
                                                                      d_code_phase_step_chips,
                                                                      d_current_prn_length_samples);

        // ################## PLL ##########################################################
        // PLL discriminator
        // Update PLL discriminator [rads/Ti -> Secs/Ti]
        carr_error_hz = pll_cloop_two_quadrant_atan(d_correlator_outs[1]) / GPS_TWO_PI; //prompt output
        // Carrier discriminator filter
        carr_error_filt_hz = d_carrier_loop_filter.get_carrier_nco(carr_error_hz);
        // New carrier Doppler frequency estimation
        d_carrier_doppler_hz = d_acq_carrier_doppler_hz + carr_error_filt_hz;

        // New code Doppler frequency estimation
        d_code_freq_chips = GPS_L1_CA_CODE_RATE_HZ + ((d_carrier_doppler_hz * GPS_L1_CA_CODE_RATE_HZ) / GPS_L1_FREQ_HZ);
        //carrier phase accumulator for (K) doppler estimation
        d_acc_carrier_phase_rad -= GPS_TWO_PI * d_carrier_doppler_hz * GPS_L1_CA_CODE_PERIOD;
        //remanent carrier phase to prevent overflow in the code NCO
        d_rem_carr_phase_rad = d_rem_carr_phase_rad + GPS_TWO_PI * ( d_if_freq + d_carrier_doppler_hz ) * GPS_L1_CA_CODE_PERIOD;
        d_rem_carr_phase_rad = fmod(d_rem_carr_phase_rad, GPS_TWO_PI);

        // ################## DLL ##########################################################
        // DLL discriminator
        code_error_chips = dll_nc_e_minus_l_normalized(d_correlator_outs[0], d_correlator_outs[2]); //[chips/Ti] //early and late
        // Code discriminator filter
        code_error_filt_chips = d_code_loop_filter.get_code_nco(code_error_chips); //[chips/second]
        //Code phase accumulator
        double code_error_filt_secs;
        code_error_filt_secs = (GPS_L1_CA_CODE_PERIOD * code_error_filt_chips) / GPS_L1_CA_CODE_RATE_HZ; //[seconds]
        d_acc_code_phase_secs = d_acc_code_phase_secs + code_error_filt_secs;

        // ################## CARRIER AND CODE NCO BUFFER ALIGNEMENT #######################
        // keep alignment parameters for the next input buffer
        double T_chip_seconds;
        double T_prn_seconds;
        double T_prn_samples;
        double K_blk_samples;
        // Compute the next buffer length based in the new period of the PRN sequence and the code phase error estimation
        T_chip_seconds = 1 / static_cast<double>(d_code_freq_chips);
        T_prn_seconds = T_chip_seconds * GPS_L1_CA_CODE_LENGTH_CHIPS;
        T_prn_samples = T_prn_seconds * static_cast<double>(d_fs_in);
        K_blk_samples = T_prn_samples + d_rem_code_phase_samples + code_error_filt_secs * static_cast<double>(d_fs_in);
        d_current_prn_length_samples = round(K_blk_samples); //round to a discrete samples

        //################### PLL COMMANDS #################################################
        //carrier phase step (NCO phase increment per sample) [rads/sample]
        d_carrier_phase_step_rad = GPS_TWO_PI * d_carrier_doppler_hz / static_cast<double>(d_fs_in);

        //################### DLL COMMANDS #################################################
        //code phase step (Code resampler phase increment per sample) [chips/sample]
        d_code_phase_step_chips = d_code_freq_chips / static_cast<double>(d_fs_in);
        //remnant code phase [chips]
        d_rem_code_phase_chips = d_rem_code_phase_samples * (d_code_freq_chips / static_cast<double>(d_fs_in));

        // ####### CN0 ESTIMATION AND LOCK DETECTORS ######
        if (d_cn0_estimation_counter < CN0_ESTIMATION_SAMPLES)
        {
            // fill buffer with prompt correlator output values
            d_Prompt_buffer[d_cn0_estimation_counter] = d_correlator_outs[1]; //prompt
            d_cn0_estimation_counter++;
        }
        else
        {
            d_cn0_estimation_counter = 0;
            // Code lock indicator
            d_CN0_SNV_dB_Hz = cn0_svn_estimator(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES, GPS_L1_CA_CODE_PERIOD);
            // Carrier lock indicator
            d_carrier_lock_test = carrier_lock_detector(d_Prompt_buffer, CN0_ESTIMATION_SAMPLES);
            // Loss of lock detection
            if (d_carrier_lock_test < d_carrier_lock_threshold or d_CN0_SNV_dB_Hz < MINIMUM_VALID_CN0)
            {
                d_carrier_lock_fail_counter++;
            }
            else
            {
                if (d_carrier_lock_fail_counter > 0) d_carrier_lock_fail_counter--;
            }
            if (d_carrier_lock_fail_counter > MAXIMUM_LOCK_FAIL_COUNTER)
            {
                std::cout << "Loss of lock in channel " << d_channel << "!" << std::endl;
                LOG(INFO) << "Loss of lock in channel " << d_channel << "!";
                this->message_port_pub(pmt::mp("events"), pmt::from_long(3));//3 -> loss of lock
                d_carrier_lock_fail_counter = 0;
                d_enable_tracking = false; // TODO: check if disabling tracking is consistent with the channel state machine
                // [LL] :ca pourrait aussi expliquer le bug observé
                // [LL] Voir si ajouter un d_dump_file_LHCP_way.close() corrige le bug idendifié.
            }
        }
        // ########### Output the tracking data to navigation and PVT ##########
        current_synchro_data.Prompt_I = static_cast<double>((d_correlator_outs[1]).real());
        current_synchro_data.Prompt_Q = static_cast<double>((d_correlator_outs[1]).imag());

        // Tracking_timestamp_secs is aligned with the CURRENT PRN start sample (Hybridization OK!, but some glitches??)
        current_synchro_data.Tracking_sample_counter = d_sample_counter + static_cast<uint64_t>(d_current_prn_length_samples);
        //compute remnant code phase samples AFTER the Tracking timestamp
        d_rem_code_phase_samples = K_blk_samples - d_current_prn_length_samples; //rounding error < 1 sample

        //current_synchro_data.Tracking_timestamp_secs = ((double)d_sample_counter)/static_cast<double>(d_fs_in);
        // This tracking block aligns the Tracking_timestamp_secs with the start sample of the PRN, thus, Code_phase_secs=0
        current_synchro_data.Code_phase_samples = 0; //o.o rem?
        current_synchro_data.Carrier_phase_rads = d_acc_carrier_phase_rad;
        current_synchro_data.Carrier_Doppler_hz = d_carrier_doppler_hz;
        current_synchro_data.CN0_dB_hz = d_CN0_SNV_dB_Hz;
        current_synchro_data.Flag_valid_symbol_output = true;
        current_synchro_data.correlation_length_ms = 1;
    }
    else
    {
        for (int n = 0; n < d_n_correlator_taps; n++)
        {
            d_correlator_outs[n] = gr_complex(0,0);
        }
        for (int n = 0; n < nbCorr_; n++){
            d_out_corr_LHCP[n] = gr_complex(0,0);
            d_out_corr_RHCP[n] = gr_complex(0,0);
        }
        d_out_corr_noDelay[0] = gr_complex(0,0);
        current_synchro_data.Tracking_sample_counter = d_sample_counter + static_cast<uint64_t>(d_current_prn_length_samples);
        current_synchro_data.System = {'G'};
    }

    //assign the GNURadio block output data
    *out[0] = current_synchro_data;

   if(d_dump)
    {
        // MULTIPLEXED FILE RECORDING - Record results to file
		float prompt_I, Early_I, Late_I;
        float prompt_Q, Early_Q, Late_Q;
        //float tmp_E, tmp_P, tmp_L;
        //double tmp_double;
        prompt_I = d_correlator_outs[1].real();
        prompt_Q = d_correlator_outs[1].imag();
        Early_I = d_correlator_outs[0].real();
        Early_Q = d_correlator_outs[0].imag();
        Late_I = d_correlator_outs[2].real();
        Late_Q = d_correlator_outs[2].imag();
        /*tmp_E = std::abs<float>(d_correlator_outs[0]);
        tmp_P = std::abs<float>(d_correlator_outs[1]);
        tmp_L = std::abs<float>(d_correlator_outs[2]);*/

        // pseudo-range computation


        // acc_carrier phase expressed in cycles


        //############ Modification Algorithme de reflectometrie ############
        reflected_corr_outputs_LHCP_way.resize(nbCorr_*2);
        int indice_LHCP = 0;
        for(int i=0; i<nbCorr_; i++){
            reflected_corr_outputs_LHCP_way[indice_LHCP] = static_cast<float>((d_out_corr_LHCP[i]).real());
            reflected_corr_outputs_LHCP_way[indice_LHCP+1] = static_cast<float>((d_out_corr_LHCP[i]).imag());
            indice_LHCP = indice_LHCP+2;
        }
        if(d_CalibrationOutputEnabled){
            reflected_corr_outputs_noDelay.resize(2);
            reflected_corr_outputs_noDelay[0] = static_cast<float>((d_out_corr_noDelay[0]).real());
            reflected_corr_outputs_noDelay[1] = static_cast<float>((d_out_corr_noDelay[0]).imag());
        }
        if(in_streams_==3){
            reflected_corr_outputs_RHCP_way.resize(nbCorr_*2);
            int indice_RHCP = 0;
            for(int i=0; i<nbCorr_; i++){
                reflected_corr_outputs_RHCP_way[indice_RHCP] = static_cast<float>((d_out_corr_RHCP[i]).real());
                reflected_corr_outputs_RHCP_way[indice_RHCP+1] = static_cast<float>((d_out_corr_RHCP[i]).imag());
                indice_RHCP = indice_RHCP+2;
            }
        }

        try{
           d_dump_file_LHCP_way << sample_counter_reflecto << "\t" << current_synchro_data.Tracking_sample_counter << "\t" << Early_I << "\t" << Early_Q << "\t" << prompt_I << "\t" << prompt_Q << "\t" << Late_I << "\t" << Late_Q << "\t";
            //d_dump_file_LHCP_way << sample_counter_reflecto << "\t" << current_synchro_data.Tracking_timestamp_secs << "\t";
            for (unsigned int i=0; i<reflected_corr_outputs_LHCP_way.size(); i++){
                d_dump_file_LHCP_way << reflected_corr_outputs_LHCP_way[i] << "\t";
            }
            d_dump_file_LHCP_way << "\n";

            /*if(enabledOutputFilter_){
            for(int i=0;i<2*(nbCorr_); i++){
                     *((float*) output_items[1+i]) = reflected_corr_outputs_LHCP_way[i];
                            }
            }*/

            if(enabledOutputFilter_){
                *((float*) output_items[1]) = Early_I;
               *((float*) output_items[2]) = Early_Q;
               *((float*) output_items[3]) = prompt_I;
               *((float*) output_items[4]) = prompt_Q;
               *((float*) output_items[5]) = Late_I;
               *((float*) output_items[6]) = Late_Q;
                for(int i=0;i<2*(nbCorr_); i++){
                    *((float*) output_items[7+i]) = reflected_corr_outputs_LHCP_way[i];
                }
            }
        }
            catch(std::ifstream::failure e){
            LOG(WARNING) << "Exception writing trk dump LHCP file " << e.what();
        }
        if(d_CalibrationOutputEnabled){
            d_dump_file_LHCP_noDelay << sample_counter_reflecto << "\t" << current_synchro_data.Tracking_sample_counter << "\t" << reflected_corr_outputs_noDelay[0] << "\t" << reflected_corr_outputs_noDelay[1] << "\n";
        }
        if(in_streams_==3){
            try{
                d_dump_file_RHCP_way << Early_I << "\t" << Early_Q << "\t" << prompt_I << "\t" << prompt_Q << "\t" << Late_I << "\t" << Late_Q << "\t";
               // d_dump_file_RHCP_way;
                for (unsigned int i=0; i<reflected_corr_outputs_RHCP_way.size(); i++){
                    d_dump_file_RHCP_way << reflected_corr_outputs_RHCP_way[i] << "\t";
                }
                d_dump_file_RHCP_way << "\n";
            }
            catch(std::ifstream::failure e){
                LOG(WARNING) << "Exception writing trk dump RHCP file " << e.what();
            }
        }
    }
        //################# Fin de la modification #################

      /*  try
        {
            // EPR
            d_dump_file.write(reinterpret_cast<char*>(&tmp_E), sizeof(float));
            d_dump_file.write(reinterpret_cast<char*>(&tmp_P), sizeof(float));
            d_dump_file.write(reinterpret_cast<char*>(&tmp_L), sizeof(float));
            // PROMPT I and Q (to analyze navigation symbols)
            d_dump_file.write(reinterpret_cast<char*>(&prompt_I), sizeof(float));
            d_dump_file.write(reinterpret_cast<char*>(&prompt_Q), sizeof(float));
            // PRN start sample stamp
            //tmp_float=(float)d_sample_counter;
            d_dump_file.write(reinterpret_cast<char*>(&d_sample_counter), sizeof(unsigned long int));
            // accumulated carrier phase
            d_dump_file.write(reinterpret_cast<char*>(&d_acc_carrier_phase_rad), sizeof(double));

            // carrier and code frequency
            d_dump_file.write(reinterpret_cast<char*>(&d_carrier_doppler_hz), sizeof(double));
            //d_dump_file.write(reinterpret_cast<char*>(&d_code_freq_chips), sizeof(double));

            //PLL commands
            d_dump_file.write(reinterpret_cast<char*>(&carr_error_hz), sizeof(double));
            d_dump_file.write(reinterpret_cast<char*>(&d_carrier_doppler_hz), sizeof(double));

            //DLL commands
            d_dump_file.write(reinterpret_cast<char*>(&code_error_chips), sizeof(double));
            d_dump_file.write(reinterpret_cast<char*>(&code_error_filt_chips), sizeof(double));

            // CN0 and carrier lock test
            d_dump_file.write(reinterpret_cast<char*>(&d_CN0_SNV_dB_Hz), sizeof(double));
            d_dump_file.write(reinterpret_cast<char*>(&d_carrier_lock_test), sizeof(double));

            // AUX vars (for debug purposes)
            tmp_double = d_rem_code_phase_samples;
            d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
            tmp_double = static_cast<double>(d_sample_counter + d_current_prn_length_samples);
            d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
        }
        catch (const std::ifstream::failure &e)
        {
            LOG(WARNING) << "Exception writing trk dump file " << e.what();
        }*/



    consume_each(d_current_prn_length_samples); // this is necessary in gr::block derivates
    d_sample_counter += d_current_prn_length_samples; //count for the processed samples
    sample_counter_reflecto = sample_counter_reflecto+1;
    return 1; //output tracking result ALWAYS even in the case of d_enable_tracking==false
}



void Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc::set_channel(uint32_t channel)
{
    gr::thread::scoped_lock l(d_setlock);
    d_channel = channel;
    LOG(INFO) << "Tracking Channel set to " << d_channel;
    std::cout << "Tracking Channel set to " << d_channel << "(Debug : ouverture fichier log supprimée\n";
    //############ Modification Algorithme de reflectometrie ############
    /* SUPPRESSION DE LA SORTIE d_dump_file
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
    {
        if (d_dump_file.is_open() == false)
        {
            try
            {
                d_dump_filename.append(boost::lexical_cast<std::string>(d_channel));
                d_dump_filename.append(".dat");
                d_dump_file.exceptions (std::ifstream::failbit | std::ifstream::badbit);
                d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                LOG(INFO) << "Tracking dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
            }
            catch (const std::ifstream::failure &e)
            {
                LOG(WARNING) << "channel " << d_channel << " Exception opening trk dump file " << e.what();
            }
        }
    }
    */
    //################# Fin de la modification #################
}


void Gps_L1_Ca_Dll_Pll_Tracking_Reflectometry_cc::set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
{
    d_acquisition_gnss_synchro = p_gnss_synchro;
}
