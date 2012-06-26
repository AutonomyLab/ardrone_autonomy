#include <time.h>
#ifndef _WIN32
	#include <sys/time.h>
#else

 #include <sys/timeb.h>
 #include <Winsock2.h>  // for timeval structure

 int gettimeofday (struct timeval *tp, void *tz)
 {
	 struct _timeb timebuffer;
	 _ftime (&timebuffer);
	 tp->tv_sec = (long)timebuffer.time;
	 tp->tv_usec = (long)timebuffer.millitm * 1000;
	 return 0;
 }
#endif

#include <stdlib.h>

#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_print.h>

#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/Navdata/ardrone_navdata_file.h>
#include <ardrone_tool/UI/ardrone_input.h>

uint32_t num_picture_decoded = 0;

float32_t nd_iphone_gaz = 0.0;
float32_t nd_iphone_yaw = 0.0;
int32_t nd_iphone_flag = 0;
float32_t nd_iphone_phi = 0.0;
float32_t nd_iphone_theta = 0.0;
float32_t nd_iphone_magneto_psi_accuracy = 0.0;
float32_t nd_iphone_magneto_psi = 0.0;

// Public declaration of navdata_file allowing other handlers to write into
FILE* navdata_file = NULL;

// Private declaration of navdata_file
// Allow this handler to disable other handlers that write in navdata file
static FILE* navdata_file_private = NULL;
static ardrone_navdata_file_data *navdata_file_data = NULL;

static void ardrone_navdata_file_print_version( void )
{
  unsigned int i;
  fprintf(navdata_file,"VERSION 19c\n");  // TODO : CHANGE VERSION NUMBER EVERY TIME THE FILE STRUCTURE CHANGES
  fprintf(navdata_file,
  "Control_state [-]; ARDrone_state [-]; Time [s]; nd_seq_num [-]; \
    AccX_raw [LSB]; AccY_raw [LSB]; AccZ_raw [LSB]; \
    GyroX_raw [LSB]; GyroY_raw [LSB]; GyroZ_raw [LSB]; GyroZIMMU3000 [LSB]; GyroX_110_raw [LSB]; GyroY_110_raw [LSB];Battery_Voltage_raw [mV]; \
    Alim_3V3 [LSB]; vrefEpson [LSB]; vrefIDG [LSB]; flag_echo_ini [LSB]; us_debut_echo [LSB]; us_fin_echo [LSB]; us_association_echo; us_distance_echo [LSB];\
    us_courbe_temps [LSB]; us_courbe_valeur [LSB]; us_courbe_ref [LSB]; nombre_echo [LSB]; sum_echo[LSB]; alt_temp_raw [mm];\
    gradient [LSB];\
    UP [LSB]; UT [LSB]; Temperature_meas [0_1C]; Pression_meas [Pa];\
    Prediction_US [mm];\
    Somme_inno [mm];\
    Offset_Pressure [Pa];\
    Estimated_altitude [mm];\
    Estimated_velocity [m/s];\
    Estimated_Bias_PWM [m/s2];\
    Estimated_Biais_Pressure [Pa];\
    Estimated_cov_alt [m];\
    Estimated_offset_US [m];\
    Estimated_cov_PWM;\
    Bool_effet_sol [SU];\
    Bool_rejet_US [SU];\
    Estimated_cov_Vz [m/s];\
    mx [LSB]; my [LSB]; mz [LSB];\
    mx [mG]; my [mG]; mz [mG];\
    mx_rect [mG]; my_rect [mG]; mz_rect [mG];\
    offsetx [mG]; offsety [mG]; offsetz [mG];\
    heading_unwrapped [deg];	heading_gyro_unwrapped [deg]; heading_fusion_unwrapped [mdeg];\
    magneto_calibration_ok [bool];\
    magneto_state [SU];\
    wind_speed [m/s]; wind_angle [deg];\
    wind_estimator_state_x1 [SU]; wind_estimator_state_x2 [SU]; wind_estimator_state_x3 [SU]; wind_estimator_state_x4 [SU]; wind_estimator_state_x5 [SU]; wind_estimator_state_x6 [SU];\
    Phi_wind_compensation [rad]; Theta_wind_compensation [rad];\
    magneto_radius [mG];\
    magneto_error_mean [mG]; magneto_error_var [mG2];\
    magneto_debug1 [SU]; magneto_debug2 [SU]; magneto_debug3 [SU];\
    Accs_temperature [K]; Gyro_temperature [LSB]; AccX_phys_filt [mg]; AccY_phys_filt [mg]; AccZ_phys_filt [mg]; \
    GyroX_phys_filt [deg/s]; GyroY_phys_filt [deg/s]; GyroZ_phys_filt [deg/s]; GyroZ_immu3000 [deg/s];\
    GyroX_offset [deg/s]; GyroY_offset [deg/s]; GyroZ_offset [deg/s]; \
    Theta_acc [mdeg]; Phi_acc [mdeg];\
    Theta_ref_embedded [mdeg]; Phi_ref_embedded [mdeg]; Psi_ref_embedded [mdeg]; Theta_ref_int [mdeg]; Phi_ref_int [mdeg]; \
    Pitch_ref_embedded [mdeg]; Roll_ref_embedded [mdeg]; Yaw_ref_embedded [mdeg/s]; \
    Theta_trim_embedded [mdeg]; Phi_trim_embedded [mdeg]; Yaw_trim_embedded [mdeg/s]; \
    Pitch_rc_embedded [-]; Roll_rc_embedded [-]; Yaw_rc_embedded [-]; Gaz_rc_embedded [-]; Ag_rc_embedded [-]; User_Input [-]; \
    PWM1 [PWM]; PWM2 [PWM]; PWM3 [PWM]; PWM4 [PWM];\
    SAT_PWM1 [PWM]; SAT_PWM2 [PWM]; SAT_PWM3 [PWM]; SAT_PWM4 [PWM];\
    Gaz_feed_forward [PWM]; Gaz_altitude [PWM];\
    Altitude_integral [mm/s]; Vz_ref [mm/s];\
    Altitude_prop [PWM]; Altitude_der [PWM]; \
    u_pitch [PWM]; u_roll [PWM]; u_yaw [PWM]; yaw_u_I [PWM];\
    u_pitch_planif [PWM]; u_roll_planif [PWM]; u_yaw_planif [PWM]; u_gaz_planif [PWM];\
    Current_motor1 [mA]; Current_motor2 [mA]; Current_motor3 [mA]; Current_motor4 [mA];\
    Altitude_vision [mm]; Altitude_vz [mm/s]; Altitude_ref_embedded [mm]; Altitude_raw [mm]; \
    Observer AccZ [m/s2]; Observer altitude US [m]; Estimated Altitude[m]; Estimated Vz [m/s]; Estimated acc bias [m/s2];\
    Observer state [-]; vb 1; vb 2;	Observer flight state [-];\
    Vision_tx_raw [-]; Vision_ty_raw [-]; Vision_tz_raw [-];   Vision_State [-]; Vision_defined [-];\
    Vision_phi_trim[rad]; Vision_phi_ref_prop[rad]; Vision_theta_trim[rad]; Vision_theta_ref_prop[rad];\
    Vx_body [mm/s]; Vy_body [mm/s]; Vz_body [mm/s];\
    New_raw_picture [-]; T_capture; F_capture; P_capture; delta_Phi; delta_Theta; delta_Psi; Alt_capture [-]; time_capture [s];\
    Demo_vbat [-]; Demo_theta [mdeg]; Demo_phi [mdeg]; Demo_psi [mdeg];  Demo_altitude [mm];  Demo_vx [mm/s]; Demo_vy [mm/s]; Demo_vz [mm/s];Demo_num_frames [-];\
    Demo_detect_rot_m11 [-]; Demo_detect_rot_m12 [-]; Demo_detect_rot_m13 [-]; Demo_detect_rot_m21 [-]; Demo_detect_rot_m22 [-]; Demo_detect_rot_m23 [-];\
    Demo_detect_rot_m31 [-]; Demo_detect_rot_m32 [-]; Demo_detect_rot_m33 [-]; Demo_detect_trans_v1 [-]; Demo_detect_trans_v2 [-]; Demo_detect_trans_v3 [-]; \
	  Demo_detect_tag_index [-]; Demo_camera_type [-]; Demo_drone_camera_rot_m11 [-]; Demo_drone_camera_rot_m12 [-]; Demo_drone_camera_rot_m13 [-];\
    Demo_drone_camera_rot_m21 [-]; Demo_drone_camera_rot_m22 [-]; Demo_drone_camera_rot_m23 [-]; Demo_drone_camera_rot_m31 [-]; Demo_drone_camera_rot_m32 [-];\
    Demo_drone_camera_rot_m33 [-]; Demo_drone_camera_trans_x [-]; Demo_drone_camera_trans_y [-]; Demo_drone_camera_trans_z [-];\
	  nd_iphone_flag [-]; nd_iphone_phi [-]; nd_iphone_theta [-]; nd_iphone_gaz [-]; nd_iphone_yaw [-];\
	  quant [-]; encoded_frame_size [bytes]; encoded_frame_number [-]; atcmd_ref_seq [-]; atcmd_mean_ref_gap [ms]; atcmd_var_ref_gap [SU]; atcmd_ref_quality[-]; out_bitrate[-]; desired_bitrate[-]; tcp_queue_level[-]; fifo_queue_level[-];\
    hdvideo_frame_number [-];");

/*  for(i = 0 ; i < DEFAULT_NB_TRACKERS_WIDTH*DEFAULT_NB_TRACKERS_HEIGHT ; i++)
    fprintf(navdata_file, "Locked_%u; X_%u; Y_%u; ", i, i, i);
*/
  fprintf(navdata_file, "Nb_detected; ");

  for(i = 0 ; i < 4 ; i++)
    fprintf(navdata_file, "Type_%u; Xd_%u; Yd_%u; W_%u; H_%u; D_%u; O_%u; ", i, i, i, i, i, i, i);

  fprintf(navdata_file, "Perf_szo [ms]; Perf_corners [ms]; Perf_compute [ms]; Perf_tracking [ms]; Perf_trans [ms]; Perf_update [ms]; ");

	for(i=0; i<NAVDATA_MAX_CUSTOM_TIME_SAVE; i++)
		fprintf(navdata_file, "Perf_Custom_%u; ", i);

  // tags after "flag_new_picture" will be written on the IHM side
  fprintf(navdata_file, "Watchdog Control [-]; flag_new_picture [-]; Sample time [s]; ");

  fprintf(navdata_file, "Vx_Ref_[mm/s]; Vy_Ref_[mm/s]; Theta_modele [rad]; Phi_modele [rad]; k_v_x [-]; k_v_y [-]; k_mode [-];UI Time [-]; Theta UI [-]; Phi UI [-];Psi UI [-];Psi_accuracy UI [-]; UI_Seq;");

  #ifdef PC_USE_POLARIS
    fprintf( navdata_file,
    "POLARIS_X [mm]; POLARIS_Y [mm]; POLARIS_Z [mm]; \
  POLARIS_QX [deg]; POLARIS_QY [deg]; POLARIS_QZ [deg];\
  POLARIS_Q0 [deg]; Time s [s]; Time us [us]; ");
  #endif

  #ifdef USE_TABLE_PILOTAGE
      fprintf( navdata_file, " Table_Pilotage_position [mdeg]; Table_Pilotage_vitesse [deg/s]; ");
  #endif

	if((navdata_file_data != NULL) && (navdata_file_data->print_header != NULL))
		navdata_file_data->print_header(navdata_file);
}

struct tm *navdata_atm = NULL;

C_RESULT ardrone_navdata_file_init( void* data )
{
  char filename[1024];
  struct timeval tv;
  time_t temptime;

  navdata_file_data = (ardrone_navdata_file_data*)data;

  gettimeofday(&tv,NULL);
  temptime = (time_t)tv.tv_sec;
  navdata_atm = localtime(&temptime);
  strcpy(filename, root_dir);
  strcat(filename, "/");

  if((navdata_file_data != NULL) && (navdata_file_data->filename != NULL))
  {
	  strcpy(filename, navdata_file_data->filename);
  }
  else
  {
      sprintf(filename, "%s/mesures_%04d%02d%02d_%02d%02d%02d.txt",
        filename,
        navdata_atm->tm_year+1900, navdata_atm->tm_mon+1, navdata_atm->tm_mday,
        navdata_atm->tm_hour, navdata_atm->tm_min, navdata_atm->tm_sec);
  }

  // private for instance
  navdata_file_private = fopen(filename, "wb");

  return navdata_file_private != NULL ? C_OK : C_FAIL;
}

C_RESULT ardrone_navdata_file_process( const navdata_unpacked_t* const pnd )
{
	uint32_t i;
	char str[50];
	int32_t* locked_ptr;
	screen_point_t* point_ptr;
	struct timeval time;
	input_state_t *input_state = NULL;

	gettimeofday(&time,NULL);

	if( navdata_file_private == NULL )
		return C_FAIL;

	if( ardrone_get_mask_from_state(pnd->ardrone_state, ARDRONE_NAVDATA_BOOTSTRAP) )
		return C_OK;

	if( navdata_file == NULL )
	{
		navdata_file = navdata_file_private;

		if( ardrone_get_mask_from_state(pnd->ardrone_state, ARDRONE_NAVDATA_DEMO_MASK) )
		{
			printf("Receiving navdata demo\n");
		}
		else
		{
			printf("Receiving all navdata\n");
		}
		ardrone_navdata_file_print_version();
	}

	// Handle the case where user asked for a new navdata file
	if( navdata_file != navdata_file_private )
	{
		fclose(navdata_file);
		navdata_file = navdata_file_private;

		if( ardrone_get_mask_from_state(pnd->ardrone_state, ARDRONE_NAVDATA_DEMO_MASK) )
		{
			printf("Receiving navdata demo\n");
		}
		else
		{
			printf("Receiving all navdata\n");
		}
		ardrone_navdata_file_print_version();
	}

	vp_os_memset(&str[0], 0, sizeof(str));
	input_state = ardrone_tool_input_get_state();
	fprintf( navdata_file,"\n" );
	fprintf( navdata_file, "%u; %u", (unsigned int) pnd->navdata_demo.ctrl_state, (unsigned int) pnd->ardrone_state );

	sprintf( str, "%d.%06d", (int)((pnd->navdata_time.time & TSECMASK) >> TSECDEC), (int)(pnd->navdata_time.time & TUSECMASK) );
	fprintf( navdata_file, ";%s", str );
	fprintf( navdata_file, "; %u", (unsigned int) pnd->nd_seq);

	fprintf( navdata_file, "; %04u; %04u; %04u; %04d; %04d; %04d; %04d; %04u; %04u; %04u; %04u; %04u; %04u; %04u; %04u; %04u; %04u; %04u; %04u; %04u; %04u; %04u; %04u; %04d; %04d",
			(unsigned int) pnd->navdata_raw_measures.raw_accs[ACC_X],
			(unsigned int) pnd->navdata_raw_measures.raw_accs[ACC_Y],
			(unsigned int) pnd->navdata_raw_measures.raw_accs[ACC_Z],
			(int) pnd->navdata_raw_measures.raw_gyros[GYRO_X],
			(int) pnd->navdata_raw_measures.raw_gyros[GYRO_Y],
			(int) pnd->navdata_raw_measures.raw_gyros[GYRO_Z],
			(int) pnd->navdata_zimmu_3000.vzimmuLSB,
			(unsigned int) pnd->navdata_raw_measures.raw_gyros_110[0],
			(unsigned int) pnd->navdata_raw_measures.raw_gyros_110[1],
			(unsigned int) pnd->navdata_raw_measures.vbat_raw,
			(unsigned int) pnd->navdata_phys_measures.alim3V3,
			(unsigned int) pnd->navdata_phys_measures.vrefEpson,
			(unsigned int) pnd->navdata_phys_measures.vrefIDG,
			(unsigned int) pnd->navdata_raw_measures.flag_echo_ini,
			(unsigned int) pnd->navdata_raw_measures.us_debut_echo,
			(unsigned int) pnd->navdata_raw_measures.us_fin_echo,
			(unsigned int) pnd->navdata_raw_measures.us_association_echo,
			(unsigned int) pnd->navdata_raw_measures.us_distance_echo,
			(unsigned int) pnd->navdata_raw_measures.us_courbe_temps,
			(unsigned int) pnd->navdata_raw_measures.us_courbe_valeur,
			(unsigned int) pnd->navdata_raw_measures.us_courbe_ref,
			(unsigned int) pnd->navdata_raw_measures.nb_echo,
			(unsigned int) pnd->navdata_raw_measures.sum_echo,
			(int) pnd->navdata_raw_measures.alt_temp_raw,
      (int) pnd->navdata_raw_measures.gradient

	        );


	fprintf( navdata_file, "; %10u; %10u; %10u; %10u",
			(signed int) pnd->navdata_pressure_raw.up,
			(signed int) pnd->navdata_pressure_raw.ut,
			(signed int) pnd->navdata_pressure_raw.Temperature_meas,
			(signed int) pnd->navdata_pressure_raw.Pression_meas );


	fprintf( navdata_file, "; %6f; %6f; %6f",
			pnd->navdata_kalman_pressure.prediction_US,
			pnd->navdata_kalman_pressure.somme_inno,
			pnd->navdata_kalman_pressure.offset_pressure);

	fprintf( navdata_file, "; %6f; %6f; %6f; %6f; %6f; %6f; %6f; %6d; %6d; %6f",
			pnd->navdata_kalman_pressure.est_z,
			pnd->navdata_kalman_pressure.est_zdot,
			pnd->navdata_kalman_pressure.est_bias_PWM,
			pnd->navdata_kalman_pressure.est_biais_pression,
			pnd->navdata_kalman_pressure.cov_alt,
			pnd->navdata_kalman_pressure.offset_US,
			pnd->navdata_kalman_pressure.cov_PWM ,
			pnd->navdata_kalman_pressure.bool_effet_sol,
			pnd->navdata_kalman_pressure.flag_rejet_US,
			pnd->navdata_kalman_pressure.cov_vitesse);

	fprintf( navdata_file, "; %5d; %5d; %5d; %6f; %6f; %6f; %6f; %6f; %6f",
			(signed int) pnd->navdata_magneto.mx,
			(signed int) pnd->navdata_magneto.my,
			(signed int) pnd->navdata_magneto.mz,
			pnd->navdata_magneto.magneto_raw.x,
			pnd->navdata_magneto.magneto_raw.y,
			pnd->navdata_magneto.magneto_raw.z,
			pnd->navdata_magneto.magneto_rectified.x,
			pnd->navdata_magneto.magneto_rectified.y,
			pnd->navdata_magneto.magneto_rectified.z);

	fprintf( navdata_file, "; %6f; %6f; %6f; %6f; %6f; %6f",
			pnd->navdata_magneto.magneto_offset.x,
			pnd->navdata_magneto.magneto_offset.y,
			pnd->navdata_magneto.magneto_offset.z,
			pnd->navdata_magneto.heading_unwrapped,
			pnd->navdata_magneto.heading_gyro_unwrapped,
			pnd->navdata_magneto.heading_fusion_unwrapped);

	fprintf( navdata_file, "; %d; %d; %6f; %6f; %6f; %6f; %6f; %6f; %6f; %6f; %6f; %6f; %6f; %6f; %6f; %6f; %6f; %6f",
			pnd->navdata_magneto.magneto_calibration_ok,
			pnd->navdata_magneto.magneto_state,
			pnd->navdata_wind_speed.wind_speed,
			pnd->navdata_wind_speed.wind_angle,
			pnd->navdata_wind_speed.state_x1,
			pnd->navdata_wind_speed.state_x2,
			pnd->navdata_wind_speed.state_x3,
			pnd->navdata_wind_speed.state_x4,
			pnd->navdata_wind_speed.state_x5,
			pnd->navdata_wind_speed.state_x6,
			pnd->navdata_wind_speed.wind_compensation_phi,
			pnd->navdata_wind_speed.wind_compensation_theta,
			pnd->navdata_magneto.magneto_radius,
			pnd->navdata_magneto.error_mean,
			pnd->navdata_magneto.error_var,
			pnd->navdata_wind_speed.magneto_debug1,
			pnd->navdata_wind_speed.magneto_debug2,
			pnd->navdata_wind_speed.magneto_debug3);

	fprintf( navdata_file, "; %f; %04u; % 5f; % 5f; % 5f; % 6f; % 6f; % 6f; %6f",
			pnd->navdata_phys_measures.accs_temp,
			(unsigned int)pnd->navdata_phys_measures.gyro_temp,
			pnd->navdata_phys_measures.phys_accs[ACC_X],
			pnd->navdata_phys_measures.phys_accs[ACC_Y],
			pnd->navdata_phys_measures.phys_accs[ACC_Z],
			pnd->navdata_phys_measures.phys_gyros[GYRO_X],
			pnd->navdata_phys_measures.phys_gyros[GYRO_Y],
			pnd->navdata_phys_measures.phys_gyros[GYRO_Z],
			pnd->navdata_zimmu_3000.vzfind                );

	fprintf( navdata_file, "; % f; % f; % f",
			pnd->navdata_gyros_offsets.offset_g[GYRO_X],
			pnd->navdata_gyros_offsets.offset_g[GYRO_Y],
			pnd->navdata_gyros_offsets.offset_g[GYRO_Z] );

	fprintf( navdata_file, "; % f; % f",
			pnd->navdata_euler_angles.theta_a,
			pnd->navdata_euler_angles.phi_a);

	fprintf( navdata_file, ";  %04d; %04d; %04d; %04d; %04d; %06d; %06d; %06d",
			(int) pnd->navdata_references.ref_theta,
			(int) pnd->navdata_references.ref_phi,
			(int) pnd->navdata_references.ref_psi,
			(int) pnd->navdata_references.ref_theta_I,
			(int) pnd->navdata_references.ref_phi_I,
			(int) pnd->navdata_references.ref_pitch,
			(int) pnd->navdata_references.ref_roll,
			(int) pnd->navdata_references.ref_yaw );

	fprintf( navdata_file, "; % 8.6f; % 8.6f; % 8.6f",
			pnd->navdata_trims.euler_angles_trim_theta,
			pnd->navdata_trims.euler_angles_trim_phi,
			pnd->navdata_trims.angular_rates_trim_r );

	fprintf( navdata_file, "; %04d; %04d; %04d; %04d; %04d; %04u",
			(int) pnd->navdata_rc_references.rc_ref_pitch,
			(int) pnd->navdata_rc_references.rc_ref_roll,
			(int) pnd->navdata_rc_references.rc_ref_yaw,
			(int) pnd->navdata_rc_references.rc_ref_gaz,
			(int) pnd->navdata_rc_references.rc_ref_ag,
			(unsigned int) input_state->user_input );

	fprintf( navdata_file, "; %03u; %03u; %03u; %03u; %03u; %03u; %03u; %03u",
			(unsigned int) pnd->navdata_pwm.motor1,
			(unsigned int) pnd->navdata_pwm.motor2,
			(unsigned int) pnd->navdata_pwm.motor3,
			(unsigned int) pnd->navdata_pwm.motor4,
			(unsigned int) pnd->navdata_pwm.sat_motor1,
			(unsigned int) pnd->navdata_pwm.sat_motor2,
			(unsigned int) pnd->navdata_pwm.sat_motor3,
			(unsigned int) pnd->navdata_pwm.sat_motor4);

	fprintf( navdata_file, "; %6f; %6f;%6f; %6f;%6f; %6f",
			pnd->navdata_pwm.gaz_feed_forward,
			pnd->navdata_pwm.gaz_altitude,
			pnd->navdata_pwm.altitude_integral,
			pnd->navdata_pwm.vz_ref,
			pnd->navdata_pwm.altitude_prop,
			pnd->navdata_pwm.altitude_der);

	fprintf( navdata_file, "; %03d; %03d; %03d; %f",
			(int) pnd->navdata_pwm.u_pitch,
			(int) pnd->navdata_pwm.u_roll,
			(int) pnd->navdata_pwm.u_yaw,
			pnd->navdata_pwm.yaw_u_I);

	fprintf( navdata_file, "; %03d; %03d; %03d; %f",
			(int) pnd->navdata_pwm.u_pitch_planif,
			(int) pnd->navdata_pwm.u_roll_planif,
			(int) pnd->navdata_pwm.u_yaw_planif,
			pnd->navdata_pwm.u_gaz_planif);

	fprintf(navdata_file, "; %04d; %04d; %04d; %04d",
			(int) pnd->navdata_pwm.current_motor1,
			(int) pnd->navdata_pwm.current_motor2,
			(int) pnd->navdata_pwm.current_motor3,
			(int) pnd->navdata_pwm.current_motor4 );

	fprintf( navdata_file, "; %04d; %f; %04d;%04u",
			(int) pnd->navdata_altitude.altitude_vision,
			pnd->navdata_altitude.altitude_vz,
			(int) pnd->navdata_altitude.altitude_ref,
			(unsigned int) pnd->navdata_altitude.altitude_raw );

	fprintf( navdata_file, "; %f; %f; %f; %f; %f; %04u; %f; %f; %04u",
			pnd->navdata_altitude.obs_accZ,
			pnd->navdata_altitude.obs_alt,
			pnd->navdata_altitude.obs_x.v[0],
			pnd->navdata_altitude.obs_x.v[1],
			pnd->navdata_altitude.obs_x.v[2],
			pnd->navdata_altitude.obs_state,
			pnd->navdata_altitude.est_vb.v[0],
			pnd->navdata_altitude.est_vb.v[1],
			pnd->navdata_altitude.est_state );

	vp_os_memset(&str[0], 0, sizeof(str));
	sprintf( str, "%d.%06d", (int)((pnd->navdata_vision.time_capture & TSECMASK) >> TSECDEC), (int)(pnd->navdata_vision.time_capture & TUSECMASK) );

	fprintf( navdata_file, "; % 8.6f; % 8.6f; % 8.6f; %u; %u; % f;% f;% f;% f; % f; % f; % f; %u; % f; % f; % f; % f; % f; % f; % d; %s",
			pnd->navdata_vision_raw.vision_tx_raw,
			pnd->navdata_vision_raw.vision_ty_raw,
			pnd->navdata_vision_raw.vision_tz_raw,
			(unsigned int) pnd->navdata_vision.vision_state,
			(unsigned int) pnd->vision_defined,
			pnd->navdata_vision.vision_phi_trim,
			pnd->navdata_vision.vision_phi_ref_prop,
			pnd->navdata_vision.vision_theta_trim,
			pnd->navdata_vision.vision_theta_ref_prop,
			pnd->navdata_vision.body_v.x,
			pnd->navdata_vision.body_v.y,
			pnd->navdata_vision.body_v.z,
			(unsigned int) pnd->navdata_vision.new_raw_picture,
			pnd->navdata_vision.theta_capture,
			pnd->navdata_vision.phi_capture,
			pnd->navdata_vision.psi_capture,
			pnd->navdata_vision.delta_phi,
			pnd->navdata_vision.delta_theta,
			pnd->navdata_vision.delta_psi,
			(int)pnd->navdata_vision.altitude_capture,
			str );

	fprintf( navdata_file, "; %04u",
			(unsigned int) pnd->navdata_demo.vbat_flying_percentage );

	fprintf( navdata_file, "; % f; % f; % f",
			pnd->navdata_demo.theta,
			pnd->navdata_demo.phi,
			pnd->navdata_demo.psi );

	fprintf( navdata_file, "; %04d",
			(int) pnd->navdata_demo.altitude );

	fprintf( navdata_file, "; %f; %f; %f ",
			pnd->navdata_demo.vx,
			pnd->navdata_demo.vy,
			pnd->navdata_demo.vz );

	fprintf( navdata_file, "; %04u", (unsigned int) pnd->navdata_demo.num_frames );

	fprintf( navdata_file, "; %f; %f; %f; %f; %f; %f; %f; %f; %f", pnd->navdata_demo.detection_camera_rot.m11,
			pnd->navdata_demo.detection_camera_rot.m12,
			pnd->navdata_demo.detection_camera_rot.m13,
			pnd->navdata_demo.detection_camera_rot.m21,
			pnd->navdata_demo.detection_camera_rot.m22,
			pnd->navdata_demo.detection_camera_rot.m23,
			pnd->navdata_demo.detection_camera_rot.m31,
			pnd->navdata_demo.detection_camera_rot.m32,
			pnd->navdata_demo.detection_camera_rot.m33);

	fprintf( navdata_file, "; %f; %f; %f", pnd->navdata_demo.detection_camera_trans.x,
			pnd->navdata_demo.detection_camera_trans.y,
			pnd->navdata_demo.detection_camera_trans.z);

	fprintf( navdata_file, "; %04u; %04u",
			(unsigned int) pnd->navdata_demo.detection_tag_index,
			(unsigned int) pnd->navdata_demo.detection_camera_type);

	fprintf( navdata_file, "; %f; %f; %f; %f; %f; %f; %f; %f; %f", pnd->navdata_demo.drone_camera_rot.m11,
			pnd->navdata_demo.drone_camera_rot.m12,
			pnd->navdata_demo.drone_camera_rot.m13,
			pnd->navdata_demo.drone_camera_rot.m21,
			pnd->navdata_demo.drone_camera_rot.m22,
			pnd->navdata_demo.drone_camera_rot.m23,
			pnd->navdata_demo.drone_camera_rot.m31,
			pnd->navdata_demo.drone_camera_rot.m32,
			pnd->navdata_demo.drone_camera_rot.m33);

	fprintf( navdata_file, "; %f; %f; %f", pnd->navdata_demo.drone_camera_trans.x,
			pnd->navdata_demo.drone_camera_trans.y,
			pnd->navdata_demo.drone_camera_trans.z);

	fprintf( navdata_file, "; %d; %f; %f; %f; %f",
			(int)nd_iphone_flag,
			nd_iphone_phi,
			nd_iphone_theta,
			nd_iphone_gaz,
			nd_iphone_yaw);

	/* Store information regarding the live video stream and the associated rate control */
	fprintf( navdata_file, "; %d; %d; %d; %d; %d; %f; %d; %d; %d; %d; %d",
			pnd->navdata_video_stream.quant,
			pnd->navdata_video_stream.frame_size,
			pnd->navdata_video_stream.frame_number,
			pnd->navdata_video_stream.atcmd_ref_seq,
			pnd->navdata_video_stream.atcmd_mean_ref_gap,
			pnd->navdata_video_stream.atcmd_var_ref_gap,
			pnd->navdata_video_stream.atcmd_ref_quality,
			pnd->navdata_video_stream.out_bitrate,
			pnd->navdata_video_stream.desired_bitrate,
			pnd->navdata_video_stream.tcp_queue_level,
			pnd->navdata_video_stream.fifo_queue_level
	);

	/* Store information regarding the HD storage stream */
	fprintf( navdata_file, "; %d",
			pnd->navdata_hdvideo_stream.frame_number
			);



	locked_ptr  = (int32_t*) &pnd->navdata_trackers_send.locked[0];
	point_ptr   = (screen_point_t*) &pnd->navdata_trackers_send.point[0];

/*
	for(i = 0; i < DEFAULT_NB_TRACKERS_WIDTH*DEFAULT_NB_TRACKERS_HEIGHT; i++)
	{
		fprintf( navdata_file, "; %d; %u; %u",
				(int) *locked_ptr++,
				(unsigned int) point_ptr->x,
				(unsigned int) point_ptr->y );
		point_ptr++;
	}
*/
	fprintf( navdata_file, "; %u", (unsigned int) pnd->navdata_vision_detect.nb_detected );
	for(i = 0 ; i < 4 ; i++)
	{
		fprintf( navdata_file, "; %u; %u; %u; %u; %u; %u; %f",
				(unsigned int) pnd->navdata_vision_detect.type[i],
				(unsigned int) pnd->navdata_vision_detect.xc[i],
				(unsigned int) pnd->navdata_vision_detect.yc[i],
				(unsigned int) pnd->navdata_vision_detect.width[i],
				(unsigned int) pnd->navdata_vision_detect.height[i],
				(unsigned int) pnd->navdata_vision_detect.dist[i],
				pnd->navdata_vision_detect.orientation_angle[i]);
	}

	fprintf( navdata_file, "; %f; %f; %f; %f; %f; %f",
			pnd->navdata_vision_perf.time_szo,
			pnd->navdata_vision_perf.time_corners,
			pnd->navdata_vision_perf.time_compute,
			pnd->navdata_vision_perf.time_tracking,
			pnd->navdata_vision_perf.time_trans,
			pnd->navdata_vision_perf.time_update );

	for(i = 0 ; i < NAVDATA_MAX_CUSTOM_TIME_SAVE ; i++)
	{
		fprintf( navdata_file, "; %f", pnd->navdata_vision_perf.time_custom[i]);
	}

	fprintf( navdata_file, "; %d", (int) pnd->navdata_watchdog.watchdog );

	fprintf( navdata_file, "; %u", (unsigned int) num_picture_decoded );

	sprintf( str, "%d.%06d", (int)time.tv_sec, (int)time.tv_usec);
	fprintf( navdata_file, "; %s", str );

    fprintf( navdata_file, ";%f;%f;%f;%f;%f;%f;%d;%f;%f;%f;%f;%f;%d",
      (float) pnd->navdata_references.vx_ref,
      (float) pnd->navdata_references.vy_ref,
      (float) pnd->navdata_references.theta_mod,
      (float) pnd->navdata_references.phi_mod,
      (float) pnd->navdata_references.k_v_x,
      (float) pnd->navdata_references.k_v_y,
      (int) pnd->navdata_references.k_mode,
      (float) pnd->navdata_references.ui_time,
      (float) pnd->navdata_references.ui_theta,
      (float) pnd->navdata_references.ui_phi,
      (float) pnd->navdata_references.ui_psi,
      (float) pnd->navdata_references.ui_psi_accuracy,
      (int) pnd->navdata_references.ui_seq);

    if((navdata_file_data != NULL) && (navdata_file_data->print != NULL))
  	  navdata_file_data->print(navdata_file);

	return C_OK;
}

C_RESULT ardrone_navdata_file_release( void )
{
  if( navdata_file != NULL )
  {
    navdata_file = NULL;

    fprintf(navdata_file_private,"\n");

    fclose( navdata_file_private );

    navdata_file_private = NULL;

    navdata_file_data = NULL;
  }

  return C_OK;
}
