#ifndef NAVDATA_OPTION_DEMO
	#define NAVDATA_OPTION_DEMO(x,y,z)
#endif
#ifndef NAVDATA_OPTION
	#define NAVDATA_OPTION(x,y,z)
#endif
#ifndef NAVDATA_OPTION_CKS
	#define NAVDATA_OPTION_CKS(x,y,z)
#endif


NAVDATA_OPTION_DEMO ( navdata_demo_t , navdata_demo , NAVDATA_DEMO_TAG)

NAVDATA_OPTION( navdata_time_t,           navdata_time            , NAVDATA_TIME_TAG             )
NAVDATA_OPTION( navdata_raw_measures_t,   navdata_raw_measures    , NAVDATA_RAW_MEASURES_TAG     )
NAVDATA_OPTION( navdata_phys_measures_t,  navdata_phys_measures   , NAVDATA_PHYS_MEASURES_TAG    )
NAVDATA_OPTION( navdata_gyros_offsets_t,  navdata_gyros_offsets   , NAVDATA_GYROS_OFFSETS_TAG    )
NAVDATA_OPTION( navdata_euler_angles_t,   navdata_euler_angles    , NAVDATA_EULER_ANGLES_TAG     )
NAVDATA_OPTION( navdata_references_t,     navdata_references      , NAVDATA_REFERENCES_TAG       )
NAVDATA_OPTION( navdata_trims_t,          navdata_trims           , NAVDATA_TRIMS_TAG            )
NAVDATA_OPTION( navdata_rc_references_t,  navdata_rc_references   , NAVDATA_RC_REFERENCES_TAG    )
NAVDATA_OPTION( navdata_pwm_t,            navdata_pwm             , NAVDATA_PWM_TAG              )
NAVDATA_OPTION( navdata_altitude_t,       navdata_altitude        , NAVDATA_ALTITUDE_TAG         )
NAVDATA_OPTION( navdata_vision_raw_t,     navdata_vision_raw      , NAVDATA_VISION_RAW_TAG       )
NAVDATA_OPTION( navdata_vision_of_t,      navdata_vision_of       , NAVDATA_VISION_OF_TAG        )
NAVDATA_OPTION( navdata_vision_t,         navdata_vision          , NAVDATA_VISION_TAG           )
NAVDATA_OPTION( navdata_vision_perf_t ,   navdata_vision_perf     , NAVDATA_VISION_PERF_TAG      )
NAVDATA_OPTION( navdata_trackers_send_t,  navdata_trackers_send   , NAVDATA_TRACKERS_SEND_TAG    )
NAVDATA_OPTION( navdata_vision_detect_t,  navdata_vision_detect   , NAVDATA_VISION_DETECT_TAG    )
NAVDATA_OPTION( navdata_watchdog_t  ,     navdata_watchdog        , NAVDATA_WATCHDOG_TAG         )
NAVDATA_OPTION( navdata_adc_data_frame_t, navdata_adc_data_frame  , NAVDATA_ADC_DATA_FRAME_TAG   )
NAVDATA_OPTION( navdata_video_stream_t,   navdata_video_stream    , NAVDATA_VIDEO_STREAM_TAG     )
NAVDATA_OPTION( navdata_games_t,          navdata_games           , NAVDATA_GAMES_TAG            )


NAVDATA_OPTION_CKS( navdata_cks_t,        navdata_cks             , NAVDATA_CKS_TAG    )

#undef NAVDATA_OPTION_DEMO
#undef NAVDATA_OPTION
#undef NAVDATA_OPTION_CKS
