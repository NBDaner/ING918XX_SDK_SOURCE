#ifndef _SIG_UUID_H
#define _SIG_UUID_H

#define COMPANY_ID_INGCHIPS        0x06AC
#define VENDOR_COMPANY_ID          COMPANY_ID_INGCHIPS

// Assigned UUIDs by Bluetooth SIG

// Services
#define SIG_UUID_SERVICE_INDOOR_POSITIONING                0x1821
#define SIG_UUID_SERVICE_HTTP_PROXY                        0x1823
#define SIG_UUID_SERVICE_LOCATION_AND_NAVIGATION           0x1819
#define SIG_UUID_SERVICE_CYCLING_POWER                     0x1818
#define SIG_UUID_SERVICE_LINK_LOSS                         0x1803
#define SIG_UUID_SERVICE_IMMEDIATE_ALERT                   0x1802
#define SIG_UUID_SERVICE_GENERIC_ATTRIBUTE                 0x1801
#define SIG_UUID_SERVICE_GENERIC_ACCESS                    0x1800
#define SIG_UUID_SERVICE_ALERT_NOTIFICATION                0x1811
#define SIG_UUID_SERVICE_BLOOD_PRESSURE                    0x1810
#define SIG_UUID_SERVICE_CURRENT_TIME                      0x1805
#define SIG_UUID_SERVICE_HUMAN_INTERFACE_DEVICE            0x1812
#define SIG_UUID_SERVICE_AUTOMATION_IO                     0x1815
#define SIG_UUID_SERVICE_RUNNING_SPEED_AND_CADENCE         0x1814
#define SIG_UUID_SERVICE_CYCLING_SPEED_AND_CADENCE         0x1816
#define SIG_UUID_SERVICE_TX_POWER                          0x1804
#define SIG_UUID_SERVICE_RECONNECTION_CONFIGURATION        0x1829
#define SIG_UUID_SERVICE_MESH_PROXY                        0x1828
#define SIG_UUID_SERVICE_MESH_PROVISIONING                 0x1827
#define SIG_UUID_SERVICE_FITNESS_MACHINE                   0x1826
#define SIG_UUID_SERVICE_OBJECT_TRANSFER                   0x1825
#define SIG_UUID_SERVICE_TRANSPORT_DISCOVERY               0x1824
#define SIG_UUID_SERVICE_INSULIN_DELIVERY                  0x183a
#define SIG_UUID_SERVICE_SCAN_PARAMETERS                   0x1813
#define SIG_UUID_SERVICE_REFERENCE_TIME_UPDATE             0x1806
#define SIG_UUID_SERVICE_NEXT_DST_CHANGE                   0x1807
#define SIG_UUID_SERVICE_GLUCOSE                           0x1808
#define SIG_UUID_SERVICE_HEALTH_THERMOMETER                0x1809
#define SIG_UUID_SERVICE_DEVICE_INFORMATION                0x180a
#define SIG_UUID_SERVICE_PULSE_OXIMETER                    0x1822
#define SIG_UUID_SERVICE_HEART_RATE                        0x180d
#define SIG_UUID_SERVICE_PHONE_ALERT_STATUS                0x180e
#define SIG_UUID_SERVICE_BATTERY_SERVICE                   0x180f
#define SIG_UUID_SERVICE_BOND_MANAGEMENT                   0x181e
#define SIG_UUID_SERVICE_CONTINUOUS_GLUCOSE_MONITORING     0x181f
#define SIG_UUID_SERVICE_USER_DATA                         0x181c
#define SIG_UUID_SERVICE_WEIGHT_SCALE                      0x181d
#define SIG_UUID_SERVICE_ENVIRONMENTAL_SENSING             0x181a
#define SIG_UUID_SERVICE_BODY_COMPOSITION                  0x181b
#define SIG_UUID_SERVICE_INTERNET_PROTOCOL_SUPPORT         0x1820
        
// Characteristics
#define SIG_UUID_CHARACT_ALERT_CATEGORY_ID                 0x2a43
#define SIG_UUID_CHARACT_AGGREGATE                         0x2a5a
#define SIG_UUID_CHARACT_FIRST_NAME                        0x2a8a
#define SIG_UUID_CHARACT_EXACT_TIME_256                    0x2a0c
#define SIG_UUID_CHARACT_TREADMILL_DATA                    0x2acd
#define SIG_UUID_CHARACT_SUPPORTED_HEART_RATE_RANGE        0x2ad7
#define SIG_UUID_CHARACT_DATE_OF_BIRTH                     0x2a85
#define SIG_UUID_CHARACT_TIME_WITH_DST                     0x2a11
#define SIG_UUID_CHARACT_GAP_DEVICE_NAME                   0x2a00
#define SIG_UUID_CHARACT_TIME_SOURCE                       0x2a13
#define SIG_UUID_CHARACT_SCAN_INTERVAL_WINDOW              0x2a4f
#define SIG_UUID_CHARACT_TWO_ZONE_HEART_RATE_LIMIT         0x2a95
#define SIG_UUID_CHARACT_GAP_PERIPHERAL_PRIVACY_FLAG       0x2a02
#define SIG_UUID_CHARACT_MANUFACTURER_NAME_STRING          0x2a29
#define SIG_UUID_CHARACT_CGM_STATUS                        0x2aa9
#define SIG_UUID_CHARACT_BOOT_KEYBOARD_OUTPUT_REPORT       0x2a32
#define SIG_UUID_CHARACT_CYCLING_POWER_FEATURE             0x2a65
#define SIG_UUID_CHARACT_PLX_SPOT_CHECK_MEASUREMENT        0x2a5e
#define SIG_UUID_CHARACT_POLLEN_CONCENTRATION              0x2a75
#define SIG_UUID_CHARACT_HTTP_STATUS_CODE                  0x2ab8
#define SIG_UUID_CHARACT_GAP_PERIPHERAL_PREFERRED_CONNECTION_PARAMETERS 0x2a04
#define SIG_UUID_CHARACT_BODY_SENSOR_LOCATION              0x2a38
#define SIG_UUID_CHARACT_TIME_UPDATE_STATE                 0x2a17
#define SIG_UUID_CHARACT_ALERT_LEVEL                       0x2a06
#define SIG_UUID_CHARACT_FITNESS_MACHINE_FEATURE           0x2acc
#define SIG_UUID_CHARACT_LN_FEATURE                        0x2a6a
#define SIG_UUID_CHARACT_TIME_ZONE                         0x2a0e
#define SIG_UUID_CHARACT_HIP_CIRCUMFERENCE                 0x2a8f
#define SIG_UUID_CHARACT_BAROMETRIC_PRESSURE_TREND         0x2aa3
#define SIG_UUID_CHARACT_FLOOR_NUMBER                      0x2ab2
#define SIG_UUID_CHARACT_MESH_PROVISIONING_DATA_IN         0x2adb
#define SIG_UUID_CHARACT_BATTERY_LEVEL                     0x2a19
#define SIG_UUID_CHARACT_IDD_COMMAND_DATA                  0x2b26
#define SIG_UUID_CHARACT_HID_CONTROL_POINT                 0x2a4c
#define SIG_UUID_CHARACT_UNREAD_ALERT_STATUS               0x2a45
#define SIG_UUID_CHARACT_PNP_ID                            0x2a50
#define SIG_UUID_CHARACT_IDD_RECORD_ACCESS_CONTROL_POINT   0x2b27
#define SIG_UUID_CHARACT_PLX_CONTINUOUS_MEASUREMENT        0x2a5f
#define SIG_UUID_CHARACT_ANAEROBIC_THRESHOLD               0x2a83
#define SIG_UUID_CHARACT_CSC_MEASUREMENT                   0x2a5b
#define SIG_UUID_CHARACT_BOOT_MOUSE_INPUT_REPORT           0x2a33
#define SIG_UUID_CHARACT_LOCAL_TIME_INFORMATION            0x2a0f
#define SIG_UUID_CHARACT_PRESSURE                          0x2a6d
#define SIG_UUID_CHARACT_FIVE_ZONE_HEART_RATE_LIMITS       0x2a8b
#define SIG_UUID_CHARACT_ANAEROBIC_HEART_RATE_UPPER_LIMIT  0x2a82
#define SIG_UUID_CHARACT_TEMPERATURE                       0x2a6e
#define SIG_UUID_CHARACT_SERIAL_NUMBER_STRING              0x2a25
#define SIG_UUID_CHARACT_CURRENT_TIME                      0x2a2b
#define SIG_UUID_CHARACT_OBJECT_ACTION_CONTROL_POINT       0x2ac5
#define SIG_UUID_CHARACT_BLOOD_PRESSURE_FEATURE            0x2a49
#define SIG_UUID_CHARACT_ANALOG                            0x2a58
#define SIG_UUID_CHARACT_UV_INDEX                          0x2a76
#define SIG_UUID_CHARACT_WEIGHT_SCALE_FEATURE              0x2a9e
#define SIG_UUID_CHARACT_HTTPS_SECURITY                    0x2abb
#define SIG_UUID_CHARACT_TEMPERATURE_MEASUREMENT           0x2a1c
#define SIG_UUID_CHARACT_DESCRIPTOR_VALUE_CHANGED          0x2a7d
#define SIG_UUID_CHARACT_IDD_STATUS                        0x2b21
#define SIG_UUID_CHARACT_GAP_APPEARANCE                    0x2a01
#define SIG_UUID_CHARACT_IDD_HISTORY_DATA                  0x2b28
#define SIG_UUID_CHARACT_AEROBIC_THRESHOLD                 0x2a7f
#define SIG_UUID_CHARACT_CYCLING_POWER_VECTOR              0x2a64
#define SIG_UUID_CHARACT_LOCATION_NAME                     0x2ab5
#define SIG_UUID_CHARACT_ALERT_STATUS                      0x2a3f
#define SIG_UUID_CHARACT_URI                               0x2ab6
#define SIG_UUID_CHARACT_CROSS_TRAINER_DATA                0x2ace
#define SIG_UUID_CHARACT_LATITUDE                          0x2aae
#define SIG_UUID_CHARACT_HTTP_HEADERS                      0x2ab7
#define SIG_UUID_CHARACT_RESTING_HEART_RATE                0x2a92
#define SIG_UUID_CHARACT_TX_POWER_LEVEL                    0x2a07
#define SIG_UUID_CHARACT_OBJECT_LIST_FILTER                0x2ac7
#define SIG_UUID_CHARACT_RECONNECTION_CONFIGURATION_CONTROL_POINT 0x2b1f
#define SIG_UUID_CHARACT_CGM_SESSION_RUN_TIME              0x2aab
#define SIG_UUID_CHARACT_APPARENT_WIND_SPEED               0x2a72
#define SIG_UUID_CHARACT_BOND_MANAGEMENT_CONTROL_POINT     0x2aa4
#define SIG_UUID_CHARACT_HEART_RATE_MAX                    0x2a8d
#define SIG_UUID_CHARACT_REPORT_MAP                        0x2a4b
#define SIG_UUID_CHARACT_TRUE_WIND_DIRECTION               0x2a71
#define SIG_UUID_CHARACT_AGE                               0x2a80
#define SIG_UUID_CHARACT_MAGNETIC_DECLINATION              0x2a2c
#define SIG_UUID_CHARACT_MEASUREMENT_INTERVAL              0x2a21
#define SIG_UUID_CHARACT_HEAT_INDEX                        0x2a7a
#define SIG_UUID_CHARACT_SUPPORTED_INCLINATION_RANGE       0x2ad5
#define SIG_UUID_CHARACT_GUST_FACTOR                       0x2a74
#define SIG_UUID_CHARACT_INDOOR_BIKE_DATA                  0x2ad2
#define SIG_UUID_CHARACT_TIME_ACCURACY                     0x2a12
#define SIG_UUID_CHARACT_ANAEROBIC_HEART_RATE_LOWER_LIMIT  0x2a81
#define SIG_UUID_CHARACT_DST_OFFSET                        0x2a0d
#define SIG_UUID_CHARACT_MAXIMUM_RECOMMENDED_HEART_RATE    0x2a91
#define SIG_UUID_CHARACT_OBJECT_SIZE                       0x2ac0
#define SIG_UUID_CHARACT_CGM_FEATURE                       0x2aa8
#define SIG_UUID_CHARACT_HEART_RATE_MEASUREMENT            0x2a37
#define SIG_UUID_CHARACT_SUPPORTED_NEW_ALERT_CATEGORY      0x2a47
#define SIG_UUID_CHARACT_DATE_OF_THRESHOLD_ASSESSMENT      0x2a86
#define SIG_UUID_CHARACT_THREE_ZONE_HEART_RATE_LIMITS      0x2a94
#define SIG_UUID_CHARACT_FITNESS_MACHINE_STATUS            0x2ada
#define SIG_UUID_CHARACT_OBJECT_LIST_CONTROL_POINT         0x2ac6
#define SIG_UUID_CHARACT_CYCLING_POWER_CONTROL_POINT       0x2a66
#define SIG_UUID_CHARACT_BOOT_KEYBOARD_INPUT_REPORT        0x2a22
#define SIG_UUID_CHARACT_MESH_PROVISIONING_DATA_OUT        0x2adc
#define SIG_UUID_CHARACT_ELEVATION                         0x2a6c
#define SIG_UUID_CHARACT_IDD_FEATURES                      0x2b23
#define SIG_UUID_CHARACT_INTERMEDIATE_TEMPERATURE          0x2a1e
#define SIG_UUID_CHARACT_ALERT_NOTIFICATION_CONTROL_POINT  0x2a44
#define SIG_UUID_CHARACT_SC_CONTROL_POINT                  0x2a55
#define SIG_UUID_CHARACT_RINGER_SETTING                    0x2a41
#define SIG_UUID_CHARACT_MAGNETIC_FLUX_DENSITY_3D          0x2aa1
#define SIG_UUID_CHARACT_FIRMWARE_REVISION_STRING          0x2a26
#define SIG_UUID_CHARACT_HARDWARE_REVISION_STRING          0x2a27
#define SIG_UUID_CHARACT_HTTP_CONTROL_POINT                0x2aba
#define SIG_UUID_CHARACT_BODY_COMPOSITION_MEASUREMENT      0x2a9c
#define SIG_UUID_CHARACT_GLUCOSE_FEATURE                   0x2a51
#define SIG_UUID_CHARACT_PLX_FEATURES                      0x2a60
#define SIG_UUID_CHARACT_IDD_STATUS_READER_CONTROL_POINT   0x2b24
#define SIG_UUID_CHARACT_OBJECT_TYPE                       0x2abf
#define SIG_UUID_CHARACT_STEP_CLIMBER_DATA                 0x2acf
#define SIG_UUID_CHARACT_TDS_CONTROL_POINT                 0x2abc
#define SIG_UUID_CHARACT_LANGUAGE                          0x2aa2
#define SIG_UUID_CHARACT_LN_CONTROL_POINT                  0x2a6b
#define SIG_UUID_CHARACT_GATT_SERVICE_CHANGED              0x2a05
#define SIG_UUID_CHARACT_OBJECT_PROPERTIES                 0x2ac4
#define SIG_UUID_CHARACT_HTTP_ENTITY_BODY                  0x2ab9
#define SIG_UUID_CHARACT_OBJECT_LAST_MODIFIED              0x2ac2
#define SIG_UUID_CHARACT_AEROBIC_HEART_RATE_LOWER_LIMIT    0x2a7e
#define SIG_UUID_CHARACT_DATE_TIME                         0x2a08
#define SIG_UUID_CHARACT_USER_INDEX                        0x2a9a
#define SIG_UUID_CHARACT_OTS_FEATURE                       0x2abd
#define SIG_UUID_CHARACT_GENDER                            0x2a8c
#define SIG_UUID_CHARACT_FITNESS_MACHINE_CONTROL_POINT     0x2ad9
#define SIG_UUID_CHARACT_INTERMEDIATE_CUFF_PRESSURE        0x2a36
#define SIG_UUID_CHARACT_REPORT                            0x2a4d
#define SIG_UUID_CHARACT_GLUCOSE_MEASUREMENT               0x2a18
#define SIG_UUID_CHARACT_FAT_BURN_HEART_RATE_UPPER_LIMIT   0x2a89
#define SIG_UUID_CHARACT_DAY_DATE_TIME                     0x2a0a
#define SIG_UUID_CHARACT_TEMPERATURE_TYPE                  0x2a1d
#define SIG_UUID_CHARACT_DATABASE_CHANGE_INCREMENT         0x2a99
#define SIG_UUID_CHARACT_VO2_MAX                           0x2a96
#define SIG_UUID_CHARACT_OBJECT_ID                         0x2ac3
#define SIG_UUID_CHARACT_APPARENT_WIND_DIRECTION           0x2a73
#define SIG_UUID_CHARACT_ALERT_CATEGORY_ID_BIT_MASK        0x2a42
#define SIG_UUID_CHARACT_BLOOD_PRESSURE_MEASUREMENT        0x2a35
#define SIG_UUID_CHARACT_PROTOCOL_MODE                     0x2a4e
#define SIG_UUID_CHARACT_CGM_SPECIFIC_OPS_CONTROL_POINT    0x2aac
#define SIG_UUID_CHARACT_BOND_MANAGEMENT_FEATURE           0x2aa5
#define SIG_UUID_CHARACT_MESH_PROXY_DATA_OUT               0x2ade
#define SIG_UUID_CHARACT_SOFTWARE_REVISION_STRING          0x2a28
#define SIG_UUID_CHARACT_GLUCOSE_MEASUREMENT_CONTEXT       0x2a34
#define SIG_UUID_CHARACT_LONGITUDE                         0x2aaf
#define SIG_UUID_CHARACT_DIGITAL                           0x2a56
#define SIG_UUID_CHARACT_LOCAL_NORTH_COORDINATE            0x2ab0
#define SIG_UUID_CHARACT_CSC_FEATURE                       0x2a5c
#define SIG_UUID_CHARACT_WEIGHT                            0x2a98
#define SIG_UUID_CHARACT_IDD_COMMAND_CONTROL_POINT         0x2b25
#define SIG_UUID_CHARACT_SYSTEM_ID                         0x2a23
#define SIG_UUID_CHARACT_TRAINING_STATUS                   0x2ad3
#define SIG_UUID_CHARACT_TRUE_WIND_SPEED                   0x2a70
#define SIG_UUID_CHARACT_NEW_ALERT                         0x2a46
#define SIG_UUID_CHARACT_DEW_POINT                         0x2a7b
#define SIG_UUID_CHARACT_GAP_RECONNECTION_ADDRESS          0x2a03
#define SIG_UUID_CHARACT_RSC_FEATURE                       0x2a54
#define SIG_UUID_CHARACT_LAST_NAME                         0x2a90
#define SIG_UUID_CHARACT_WEIGHT_MEASUREMENT                0x2a9d
#define SIG_UUID_CHARACT_IDD_STATUS_CHANGED                0x2b20
#define SIG_UUID_CHARACT_SUPPORTED_UNREAD_ALERT_CATEGORY   0x2a48
#define SIG_UUID_CHARACT_REFERENCE_TIME_INFORMATION        0x2a14
#define SIG_UUID_CHARACT_CGM_SESSION_START_TIME            0x2aaa
#define SIG_UUID_CHARACT_POSITION_QUALITY                  0x2a69
#define SIG_UUID_CHARACT_EMAIL_ADDRESS                     0x2a87
#define SIG_UUID_CHARACT_DAY_OF_WEEK                       0x2a09
#define SIG_UUID_CHARACT_SCAN_REFRESH                      0x2a31
#define SIG_UUID_CHARACT_RC_SETTINGS                       0x2b1e
#define SIG_UUID_CHARACT_MESH_PROXY_DATA_IN                0x2add
#define SIG_UUID_CHARACT_ALTITUDE                          0x2ab3
#define SIG_UUID_CHARACT_OBJECT_NAME                       0x2abe
#define SIG_UUID_CHARACT_OBJECT_FIRST_CREATED              0x2ac1
#define SIG_UUID_CHARACT_SUPPORTED_POWER_RANGE             0x2ad8
#define SIG_UUID_CHARACT_SUPPORTED_RESISTANCE_LEVEL_RANGE  0x2ad6
#define SIG_UUID_CHARACT_STAIR_CLIMBER_DATA                0x2ad0
#define SIG_UUID_CHARACT_RINGER_CONTROL_POINT              0x2a40
#define SIG_UUID_CHARACT_RAINFALL                          0x2a78
#define SIG_UUID_CHARACT_HID_INFORMATION                   0x2a4a
#define SIG_UUID_CHARACT_RC_FEATURE                        0x2b1d
#define SIG_UUID_CHARACT_USER_CONTROL_POINT                0x2a9f
#define SIG_UUID_CHARACT_AEROBIC_HEART_RATE_UPPER_LIMIT    0x2a84
#define SIG_UUID_CHARACT_LOCATION_AND_SPEED                0x2a67
#define SIG_UUID_CHARACT_OBJECT_CHANGED                    0x2ac8
#define SIG_UUID_CHARACT_IDD_ANNUNCIATION_STATUS           0x2b22
#define SIG_UUID_CHARACT_TIME_UPDATE_CONTROL_POINT         0x2a16
#define SIG_UUID_CHARACT_MAGNETIC_FLUX_DENSITY_2D          0x2aa0
#define SIG_UUID_CHARACT_RECORD_ACCESS_CONTROL_POINT       0x2a52
#define SIG_UUID_CHARACT_CGM_MEASUREMENT                   0x2aa7
#define SIG_UUID_CHARACT_INDOOR_POSITIONING_CONFIGURATION  0x2aad
#define SIG_UUID_CHARACT_IRRADIANCE                        0x2a77
#define SIG_UUID_CHARACT_NAVIGATION                        0x2a68
#define SIG_UUID_CHARACT_CYCLING_POWER_MEASUREMENT         0x2a63
#define SIG_UUID_CHARACT_SUPPORTED_SPEED_RANGE             0x2ad4
#define SIG_UUID_CHARACT_RSC_MEASUREMENT                   0x2a53
#define SIG_UUID_CHARACT_SENSOR_LOCATION                   0x2a5d
#define SIG_UUID_CHARACT_BODY_COMPOSITION_FEATURE          0x2a9b
#define SIG_UUID_CHARACT_IEEE_11073_20601_REGULATORY_CERTIFICATION_DATA_LIST 0x2a2a
#define SIG_UUID_CHARACT_WIND_CHILL                        0x2a79
#define SIG_UUID_CHARACT_HEART_RATE_CONTROL_POINT          0x2a39
#define SIG_UUID_CHARACT_ROWER_DATA                        0x2ad1
#define SIG_UUID_CHARACT_HEIGHT                            0x2a8e
#define SIG_UUID_CHARACT_WAIST_CIRCUMFERENCE               0x2a97
#define SIG_UUID_CHARACT_FAT_BURN_HEART_RATE_LOWER_LIMIT   0x2a88
#define SIG_UUID_CHARACT_UNCERTAINTY                       0x2ab4
#define SIG_UUID_CHARACT_HUMIDITY                          0x2a6f
#define SIG_UUID_CHARACT_MODEL_NUMBER_STRING               0x2a24
#define SIG_UUID_CHARACT_SPORT_TYPE_FOR_AEROBIC_AND_ANAEROBIC_THRESHOLDS 0x2a93
#define SIG_UUID_CHARACT_LOCAL_EAST_COORDINATE             0x2ab1
        
// Descriptors
#define SIG_UUID_DESCRIP_ES_MEASUREMENT                    0x290c
#define SIG_UUID_DESCRIP_GATT_CLIENT_CHARACTERISTIC_CONFIGURATION 0x2902
#define SIG_UUID_DESCRIP_ES_TRIGGER_SETTING                0x290d
#define SIG_UUID_DESCRIP_GATT_CHARACTERISTIC_AGGREGATE_FORMAT 0x2905
#define SIG_UUID_DESCRIP_TIME_TRIGGER_SETTING              0x290e
#define SIG_UUID_DESCRIP_GATT_SERVER_CHARACTERISTIC_CONFIGURATION 0x2903
#define SIG_UUID_DESCRIP_GATT_CHARACTERISTIC_PRESENTATION_FORMAT 0x2904
#define SIG_UUID_DESCRIP_GATT_CHARACTERISTIC_USER_DESCRIPTION 0x2901
#define SIG_UUID_DESCRIP_GATT_CHARACTERISTIC_EXTENDED_PROPERTIES 0x2900
#define SIG_UUID_DESCRIP_EXTERNAL_REPORT_REFERENCE         0x2907
#define SIG_UUID_DESCRIP_VALID_RANGE                       0x2906
#define SIG_UUID_DESCRIP_NUMBER_OF_DIGITALS                0x2909
#define SIG_UUID_DESCRIP_REPORT_REFERENCE                  0x2908
#define SIG_UUID_DESCRIP_ES_CONFIGURATION                  0x290b
#define SIG_UUID_DESCRIP_VALUE_TRIGGER_SETTING             0x290a


#endif
