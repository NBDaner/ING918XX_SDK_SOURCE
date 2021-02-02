att_dispatch_client_can_send_now = 0x00005ae5;
att_dispatch_client_request_can_send_now_event = 0x00005aeb;
att_dispatch_register_client = 0x00005af1;
att_dispatch_register_server = 0x00005b05;
att_dispatch_server_can_send_now = 0x00005b19;
att_dispatch_server_request_can_send_now_event = 0x00005b1f;
att_emit_general_event = 0x00005bd1;
att_server_can_send_packet_now = 0x000062f1;
att_server_get_mtu = 0x000062f5;
att_server_indicate = 0x00006369;
att_server_init = 0x000063ed;
att_server_notify = 0x00006429;
att_server_register_packet_handler = 0x00006511;
att_server_request_can_send_now_event = 0x0000651d;
att_set_db = 0x00006539;
att_set_read_callback = 0x0000654d;
att_set_write_callback = 0x00006559;
bd_addr_cmp = 0x0000669d;
bd_addr_copy = 0x000066a3;
bd_addr_to_str = 0x000066ad;
big_endian_read_16 = 0x000066e5;
big_endian_read_32 = 0x000066ed;
big_endian_store_16 = 0x00006701;
big_endian_store_32 = 0x0000670d;
btstack_memory_pool_create = 0x0000696b;
btstack_memory_pool_free = 0x00006995;
btstack_memory_pool_get = 0x000069f5;
btstack_push_user_msg = 0x00006a11;
char_for_nibble = 0x00006c89;
eTaskConfirmSleepModeStatus = 0x00006f19;
gap_add_dev_to_periodic_list = 0x00007605;
gap_add_whitelist = 0x0000761d;
gap_aes_encrypt = 0x00007631;
gap_clear_white_lists = 0x00007659;
gap_clr_adv_set = 0x00007669;
gap_clr_periodic_adv_list = 0x00007679;
gap_create_connection_cancel = 0x000076a5;
gap_default_periodic_adv_sync_transfer_param = 0x000076b5;
gap_disconnect = 0x000076cd;
gap_disconnect_all = 0x000076f9;
gap_ext_create_connection = 0x000077a1;
gap_get_connection_parameter_range = 0x00007865;
gap_le_read_channel_map = 0x0000789d;
gap_periodic_adv_create_sync = 0x00007911;
gap_periodic_adv_create_sync_cancel = 0x00007935;
gap_periodic_adv_set_info_transfer = 0x00007945;
gap_periodic_adv_sync_transfer = 0x0000795d;
gap_periodic_adv_sync_transfer_param = 0x00007975;
gap_periodic_adv_term_sync = 0x00007991;
gap_read_antenna_info = 0x00007a25;
gap_read_periodic_adv_list_size = 0x00007a35;
gap_read_phy = 0x00007a45;
gap_read_remote_info = 0x00007a59;
gap_read_remote_used_features = 0x00007a6d;
gap_read_rssi = 0x00007a81;
gap_remove_whitelist = 0x00007a95;
gap_rmv_adv_set = 0x00007b21;
gap_rmv_dev_from_periodic_list = 0x00007b35;
gap_set_adv_set_random_addr = 0x00007b75;
gap_set_connection_cte_request_enable = 0x00007bc1;
gap_set_connection_cte_response_enable = 0x00007bdd;
gap_set_connection_cte_rx_param = 0x00007bf1;
gap_set_connection_cte_tx_param = 0x00007c4d;
gap_set_connection_parameter_range = 0x00007ca1;
gap_set_connectionless_cte_tx_enable = 0x00007cb9;
gap_set_connectionless_cte_tx_param = 0x00007ccd;
gap_set_connectionless_iq_sampling_enable = 0x00007d2d;
gap_set_def_phy = 0x00007d91;
gap_set_ext_adv_data = 0x00007da9;
gap_set_ext_adv_enable = 0x00007dc1;
gap_set_ext_adv_para = 0x00007e3d;
gap_set_ext_scan_enable = 0x00007f1d;
gap_set_ext_scan_para = 0x00007f35;
gap_set_ext_scan_response_data = 0x00007fdd;
gap_set_host_channel_classification = 0x00007ff5;
gap_set_periodic_adv_data = 0x00008009;
gap_set_periodic_adv_enable = 0x0000807d;
gap_set_periodic_adv_para = 0x00008091;
gap_set_periodic_adv_rx_enable = 0x000080a9;
gap_set_phy = 0x000080bd;
gap_set_random_device_address = 0x000080d9;
gap_start_ccm = 0x0000813d;
gap_update_connection_parameters = 0x00008171;
gatt_client_cancel_write = 0x00008689;
gatt_client_discover_characteristic_descriptors = 0x000086af;
gatt_client_discover_characteristics_for_handle_range_by_uuid128 = 0x000086ef;
gatt_client_discover_characteristics_for_handle_range_by_uuid16 = 0x0000873f;
gatt_client_discover_characteristics_for_service = 0x0000878f;
gatt_client_discover_primary_services = 0x000087c5;
gatt_client_discover_primary_services_by_uuid128 = 0x000087f7;
gatt_client_discover_primary_services_by_uuid16 = 0x0000883b;
gatt_client_execute_write = 0x00008877;
gatt_client_find_included_services_for_service = 0x0000889d;
gatt_client_get_mtu = 0x000088cb;
gatt_client_is_ready = 0x00008975;
gatt_client_listen_for_characteristic_value_updates = 0x0000898b;
gatt_client_prepare_write = 0x000089af;
gatt_client_pts_suppress_mtu_exchange = 0x000089ed;
gatt_client_read_characteristic_descriptor_using_descriptor_handle = 0x000089f9;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle = 0x00008a23;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x00008a29;
gatt_client_read_long_value_of_characteristic_using_value_handle = 0x00008a57;
gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset = 0x00008a5d;
gatt_client_read_multiple_characteristic_values = 0x00008a8b;
gatt_client_read_value_of_characteristic_using_value_handle = 0x00008abb;
gatt_client_read_value_of_characteristics_by_uuid128 = 0x00008ae9;
gatt_client_read_value_of_characteristics_by_uuid16 = 0x00008b35;
gatt_client_register_handler = 0x00008b81;
gatt_client_reliable_write_long_value_of_characteristic = 0x00008b8d;
gatt_client_signed_write_without_response = 0x00008fbd;
gatt_client_write_characteristic_descriptor_using_descriptor_handle = 0x00009081;
gatt_client_write_client_characteristic_configuration = 0x000090bb;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle = 0x0000910d;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x0000911d;
gatt_client_write_long_value_of_characteristic = 0x00009159;
gatt_client_write_long_value_of_characteristic_with_offset = 0x00009169;
gatt_client_write_value_of_characteristic = 0x000091a5;
gatt_client_write_value_of_characteristic_without_response = 0x000091db;
hci_add_event_handler = 0x0000a6a9;
hci_power_control = 0x0000ae55;
hci_register_acl_packet_handler = 0x0000b009;
kv_commit = 0x0000b691;
kv_get = 0x0000b6e9;
kv_init = 0x0000b701;
kv_put = 0x0000b769;
kv_remove = 0x0000b7e1;
kv_remove_all = 0x0000b81d;
kv_value_modified = 0x0000b861;
kv_visit = 0x0000b865;
l2cap_can_send_fixed_channel_packet_now = 0x0000b925;
l2cap_can_send_packet_now = 0x0000b929;
l2cap_create_channel = 0x0000bae1;
l2cap_disconnect = 0x0000bc19;
l2cap_get_remote_mtu_for_local_cid = 0x0000bebd;
l2cap_init = 0x0000c33d;
l2cap_le_send_flow_control_credit = 0x0000c385;
l2cap_max_le_mtu = 0x0000c5e9;
l2cap_max_mtu = 0x0000c5ed;
l2cap_next_local_cid = 0x0000c5f1;
l2cap_next_sig_id = 0x0000c601;
l2cap_register_fixed_channel = 0x0000c699;
l2cap_register_packet_handler = 0x0000c6b5;
l2cap_register_service = 0x0000c6c1;
l2cap_request_can_send_fix_channel_now_event = 0x0000c7a9;
l2cap_request_can_send_now_event = 0x0000c7cd;
l2cap_request_connection_parameter_update = 0x0000c7e7;
l2cap_require_security_level_2_for_outgoing_sdp = 0x0000c819;
l2cap_send = 0x0000cbdd;
l2cap_send_connectionless = 0x0000cc55;
l2cap_send_connectionless3 = 0x0000cce5;
l2cap_send_echo_request = 0x0000cd7d;
l2cap_send_signaling_le = 0x0000cde1;
l2cap_unregister_service = 0x0000ce39;
ll_free = 0x0000d007;
ll_hint_on_ce_len = 0x0000d011;
ll_malloc = 0x0000d04d;
ll_query_timing_info = 0x0000d185;
ll_set_adv_coded_scheme = 0x0000d331;
ll_set_conn_coded_scheme = 0x0000d361;
ll_set_conn_latency = 0x0000d38d;
ll_set_conn_tx_power = 0x0000d3bd;
ll_set_def_antenna = 0x0000d405;
ll_set_initiating_coded_scheme = 0x0000d421;
nibble_for_char = 0x0001de25;
platform_32k_rc_auto_tune = 0x0001dedd;
platform_32k_rc_tune = 0x0001df59;
platform_calibrate_32k = 0x0001df6d;
platform_config = 0x0001df71;
platform_get_heap_status = 0x0001e021;
platform_get_us_time = 0x0001e039;
platform_get_version = 0x0001e03d;
platform_hrng = 0x0001e045;
platform_install_isr_stack = 0x0001e04d;
platform_patch_rf_init_data = 0x0001e059;
platform_printf = 0x0001e065;
platform_raise_assertion = 0x0001e079;
platform_rand = 0x0001e08d;
platform_read_info = 0x0001e091;
platform_read_persistent_reg = 0x0001e0ad;
platform_reset = 0x0001e0bd;
platform_set_evt_callback = 0x0001e0f1;
platform_set_irq_callback = 0x0001e105;
platform_set_rf_clk_source = 0x0001e13d;
platform_set_rf_init_data = 0x0001e149;
platform_set_rf_power_mapping = 0x0001e155;
platform_shutdown = 0x0001e161;
platform_switch_app = 0x0001e165;
platform_trace_raw = 0x0001e191;
platform_write_persistent_reg = 0x0001e1a5;
printf_hexdump = 0x0001e1b5;
pvPortMalloc = 0x0001ece9;
pvTaskIncrementMutexHeldCount = 0x0001edd1;
pvTimerGetTimerID = 0x0001ede9;
pxPortInitialiseStack = 0x0001ee15;
reverse_128 = 0x0001eff5;
reverse_24 = 0x0001effb;
reverse_48 = 0x0001f001;
reverse_56 = 0x0001f007;
reverse_64 = 0x0001f00d;
reverse_bd_addr = 0x0001f013;
reverse_bytes = 0x0001f019;
sm_add_event_handler = 0x0001f2b9;
sm_address_resolution_lookup = 0x0001f3e5;
sm_authenticated = 0x0001f4bf;
sm_authorization_decline = 0x0001f4cd;
sm_authorization_grant = 0x0001f4ed;
sm_authorization_state = 0x0001f50d;
sm_bonding_decline = 0x0001f529;
sm_config = 0x0001f901;
sm_encryption_key_size = 0x0001fa0d;
sm_just_works_confirm = 0x000201dd;
sm_le_device_key = 0x0002042d;
sm_passkey_input = 0x000204c5;
sm_private_random_address_generation_get = 0x00020851;
sm_private_random_address_generation_get_mode = 0x00020859;
sm_private_random_address_generation_set_mode = 0x00020865;
sm_private_random_address_generation_set_update_period = 0x0002088d;
sm_register_oob_data_callback = 0x000208c5;
sm_request_pairing = 0x000208d1;
sm_send_security_request = 0x0002130b;
sm_set_accepted_stk_generation_methods = 0x00021331;
sm_set_authentication_requirements = 0x0002133d;
sm_set_encryption_key_size_range = 0x00021349;
sscanf_bd_addr = 0x00021645;
sysSetPublicDeviceAddr = 0x00021961;
uuid128_to_str = 0x00022095;
uuid_add_bluetooth_prefix = 0x000220ed;
uuid_has_bluetooth_prefix = 0x0002210d;
uxQueueMessagesWaiting = 0x00022151;
uxQueueMessagesWaitingFromISR = 0x00022179;
uxQueueSpacesAvailable = 0x00022195;
uxTaskGetStackHighWaterMark = 0x000221c1;
vPortEnterCritical = 0x00022279;
vPortExitCritical = 0x000222b9;
vPortFree = 0x000222e5;
vPortSuppressTicksAndSleep = 0x00022379;
vPortValidateInterruptPriority = 0x00022451;
vQueueDelete = 0x000224a5;
vQueueWaitForMessageRestricted = 0x000224d1;
vTaskDelay = 0x00022519;
vTaskInternalSetTimeOutState = 0x00022565;
vTaskMissedYield = 0x00022575;
vTaskPlaceOnEventList = 0x00022581;
vTaskPlaceOnEventListRestricted = 0x000225b9;
vTaskPriorityDisinheritAfterTimeout = 0x000225f9;
vTaskStartScheduler = 0x000226a5;
vTaskStepTick = 0x00022735;
vTaskSuspendAll = 0x00022765;
vTaskSwitchContext = 0x00022775;
xPortStartScheduler = 0x0002281d;
xQueueAddToSet = 0x000228d9;
xQueueCreateCountingSemaphore = 0x000228fd;
xQueueCreateCountingSemaphoreStatic = 0x00022939;
xQueueCreateMutex = 0x0002297d;
xQueueCreateMutexStatic = 0x00022993;
xQueueCreateSet = 0x000229ad;
xQueueGenericCreate = 0x000229b5;
xQueueGenericCreateStatic = 0x00022a01;
xQueueGenericReset = 0x00022a69;
xQueueGenericSend = 0x00022af5;
xQueueGenericSendFromISR = 0x00022c61;
xQueueGiveFromISR = 0x00022d21;
xQueueGiveMutexRecursive = 0x00022dc5;
xQueueIsQueueEmptyFromISR = 0x00022e05;
xQueueIsQueueFullFromISR = 0x00022e29;
xQueuePeek = 0x00022e51;
xQueuePeekFromISR = 0x00022f79;
xQueueReceive = 0x00022fe5;
xQueueReceiveFromISR = 0x00023111;
xQueueRemoveFromSet = 0x000231a5;
xQueueSelectFromSet = 0x000231c7;
xQueueSelectFromSetFromISR = 0x000231d9;
xQueueSemaphoreTake = 0x000231ed;
xQueueTakeMutexRecursive = 0x00023359;
xTaskCheckForTimeOut = 0x0002339d;
xTaskCreate = 0x0002340d;
xTaskCreateStatic = 0x00023469;
xTaskGetCurrentTaskHandle = 0x000234d9;
xTaskGetSchedulerState = 0x000234e5;
xTaskGetTickCount = 0x00023501;
xTaskGetTickCountFromISR = 0x0002350d;
xTaskIncrementTick = 0x0002351d;
xTaskPriorityDisinherit = 0x000235e9;
xTaskPriorityInherit = 0x0002367d;
xTaskRemoveFromEventList = 0x00023711;
xTaskResumeAll = 0x00023791;
xTimerCreate = 0x00023859;
xTimerCreateStatic = 0x0002388d;
xTimerCreateTimerTask = 0x000238c5;
xTimerGenericCommand = 0x00023931;
xTimerGetExpiryTime = 0x000239a1;
