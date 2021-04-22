att_dispatch_client_can_send_now = 0x00005b1d;
att_dispatch_client_request_can_send_now_event = 0x00005b23;
att_dispatch_register_client = 0x00005b29;
att_dispatch_register_server = 0x00005b3d;
att_dispatch_server_can_send_now = 0x00005b51;
att_dispatch_server_request_can_send_now_event = 0x00005b57;
att_emit_general_event = 0x00005c09;
att_server_can_send_packet_now = 0x00006329;
att_server_get_mtu = 0x0000632d;
att_server_indicate = 0x000063a1;
att_server_init = 0x00006425;
att_server_notify = 0x00006461;
att_server_register_packet_handler = 0x00006549;
att_server_request_can_send_now_event = 0x00006555;
att_set_db = 0x00006571;
att_set_read_callback = 0x00006585;
att_set_write_callback = 0x00006591;
bd_addr_cmp = 0x000066d5;
bd_addr_copy = 0x000066db;
bd_addr_to_str = 0x000066e5;
big_endian_read_16 = 0x0000671d;
big_endian_read_32 = 0x00006725;
big_endian_store_16 = 0x00006739;
big_endian_store_32 = 0x00006745;
btstack_memory_pool_create = 0x000069af;
btstack_memory_pool_free = 0x000069d9;
btstack_memory_pool_get = 0x00006a39;
btstack_push_user_msg = 0x00006a55;
char_for_nibble = 0x00006d1d;
eTaskConfirmSleepModeStatus = 0x00006fc9;
gap_add_dev_to_periodic_list = 0x000076e9;
gap_add_whitelist = 0x00007701;
gap_aes_encrypt = 0x00007715;
gap_clear_white_lists = 0x0000773d;
gap_clr_adv_set = 0x0000774d;
gap_clr_periodic_adv_list = 0x0000775d;
gap_create_connection_cancel = 0x00007789;
gap_default_periodic_adv_sync_transfer_param = 0x00007799;
gap_disconnect = 0x000077b1;
gap_disconnect_all = 0x000077dd;
gap_ext_create_connection = 0x00007885;
gap_get_connection_parameter_range = 0x00007949;
gap_le_read_channel_map = 0x00007985;
gap_periodic_adv_create_sync = 0x000079f9;
gap_periodic_adv_create_sync_cancel = 0x00007a1d;
gap_periodic_adv_set_info_transfer = 0x00007a2d;
gap_periodic_adv_sync_transfer = 0x00007a45;
gap_periodic_adv_sync_transfer_param = 0x00007a5d;
gap_periodic_adv_term_sync = 0x00007a79;
gap_read_antenna_info = 0x00007b0d;
gap_read_periodic_adv_list_size = 0x00007b1d;
gap_read_phy = 0x00007b2d;
gap_read_remote_info = 0x00007b41;
gap_read_remote_used_features = 0x00007b55;
gap_read_rssi = 0x00007b69;
gap_remove_whitelist = 0x00007b7d;
gap_rmv_adv_set = 0x00007c09;
gap_rmv_dev_from_periodic_list = 0x00007c1d;
gap_rx_test_v2 = 0x00007c35;
gap_rx_test_v3 = 0x00007c4d;
gap_set_adv_set_random_addr = 0x00007c9d;
gap_set_connection_cte_request_enable = 0x00007ce9;
gap_set_connection_cte_response_enable = 0x00007d05;
gap_set_connection_cte_rx_param = 0x00007d19;
gap_set_connection_cte_tx_param = 0x00007d75;
gap_set_connection_parameter_range = 0x00007dc9;
gap_set_connectionless_cte_tx_enable = 0x00007de1;
gap_set_connectionless_cte_tx_param = 0x00007df5;
gap_set_connectionless_iq_sampling_enable = 0x00007e55;
gap_set_def_phy = 0x00007eb9;
gap_set_ext_adv_data = 0x00007ed1;
gap_set_ext_adv_enable = 0x00007ee9;
gap_set_ext_adv_para = 0x00007f65;
gap_set_ext_scan_enable = 0x00008045;
gap_set_ext_scan_para = 0x0000805d;
gap_set_ext_scan_response_data = 0x00008105;
gap_set_host_channel_classification = 0x0000811d;
gap_set_periodic_adv_data = 0x00008131;
gap_set_periodic_adv_enable = 0x000081a5;
gap_set_periodic_adv_para = 0x000081b9;
gap_set_periodic_adv_rx_enable = 0x000081d1;
gap_set_phy = 0x000081e5;
gap_set_random_device_address = 0x00008201;
gap_start_ccm = 0x00008265;
gap_test_end = 0x00008299;
gap_tx_test_v2 = 0x000082a9;
gap_tx_test_v3 = 0x000082c1;
gap_update_connection_parameters = 0x000082e9;
gap_vendor_tx_continuous_wave = 0x0000830d;
gatt_client_cancel_write = 0x0000881d;
gatt_client_discover_characteristic_descriptors = 0x00008843;
gatt_client_discover_characteristics_for_handle_range_by_uuid128 = 0x00008883;
gatt_client_discover_characteristics_for_handle_range_by_uuid16 = 0x000088d3;
gatt_client_discover_characteristics_for_service = 0x00008923;
gatt_client_discover_primary_services = 0x00008959;
gatt_client_discover_primary_services_by_uuid128 = 0x0000898b;
gatt_client_discover_primary_services_by_uuid16 = 0x000089cf;
gatt_client_execute_write = 0x00008a0b;
gatt_client_find_included_services_for_service = 0x00008a31;
gatt_client_get_mtu = 0x00008a5f;
gatt_client_is_ready = 0x00008b09;
gatt_client_listen_for_characteristic_value_updates = 0x00008b1f;
gatt_client_prepare_write = 0x00008b43;
gatt_client_pts_suppress_mtu_exchange = 0x00008b81;
gatt_client_read_characteristic_descriptor_using_descriptor_handle = 0x00008b8d;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle = 0x00008bb7;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x00008bbd;
gatt_client_read_long_value_of_characteristic_using_value_handle = 0x00008beb;
gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset = 0x00008bf1;
gatt_client_read_multiple_characteristic_values = 0x00008c1f;
gatt_client_read_value_of_characteristic_using_value_handle = 0x00008c4f;
gatt_client_read_value_of_characteristics_by_uuid128 = 0x00008c7d;
gatt_client_read_value_of_characteristics_by_uuid16 = 0x00008cc9;
gatt_client_register_handler = 0x00008d15;
gatt_client_reliable_write_long_value_of_characteristic = 0x00008d21;
gatt_client_signed_write_without_response = 0x00009151;
gatt_client_write_characteristic_descriptor_using_descriptor_handle = 0x00009215;
gatt_client_write_client_characteristic_configuration = 0x0000924f;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle = 0x000092a1;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x000092b1;
gatt_client_write_long_value_of_characteristic = 0x000092ed;
gatt_client_write_long_value_of_characteristic_with_offset = 0x000092fd;
gatt_client_write_value_of_characteristic = 0x00009339;
gatt_client_write_value_of_characteristic_without_response = 0x0000936f;
hci_add_event_handler = 0x0000a83d;
hci_power_control = 0x0000afd5;
hci_register_acl_packet_handler = 0x0000b189;
kv_commit = 0x0000b811;
kv_get = 0x0000b869;
kv_init = 0x0000b881;
kv_put = 0x0000b8e9;
kv_remove = 0x0000b961;
kv_remove_all = 0x0000b99d;
kv_value_modified = 0x0000b9e1;
kv_visit = 0x0000b9e5;
l2cap_can_send_fixed_channel_packet_now = 0x0000baa5;
l2cap_can_send_packet_now = 0x0000baa9;
l2cap_create_channel = 0x0000bc61;
l2cap_disconnect = 0x0000bd99;
l2cap_get_remote_mtu_for_local_cid = 0x0000c00d;
l2cap_init = 0x0000c48d;
l2cap_le_send_flow_control_credit = 0x0000c4d5;
l2cap_max_le_mtu = 0x0000c759;
l2cap_max_mtu = 0x0000c75d;
l2cap_next_local_cid = 0x0000c761;
l2cap_next_sig_id = 0x0000c771;
l2cap_register_fixed_channel = 0x0000c809;
l2cap_register_packet_handler = 0x0000c825;
l2cap_register_service = 0x0000c831;
l2cap_request_can_send_fix_channel_now_event = 0x0000c919;
l2cap_request_can_send_now_event = 0x0000c93d;
l2cap_request_connection_parameter_update = 0x0000c957;
l2cap_require_security_level_2_for_outgoing_sdp = 0x0000c989;
l2cap_send = 0x0000cd1d;
l2cap_send_connectionless = 0x0000cd95;
l2cap_send_connectionless3 = 0x0000ce25;
l2cap_send_echo_request = 0x0000cebd;
l2cap_send_signaling_le = 0x0000cf21;
l2cap_unregister_service = 0x0000cf79;
ll_ackable_packet_alloc = 0x0000d147;
ll_ackable_packet_get_status = 0x0000d22f;
ll_ackable_packet_run = 0x0000d2a1;
ll_ackable_packet_set_tx_data = 0x0000d349;
ll_free = 0x0000d363;
ll_hint_on_ce_len = 0x0000d36d;
ll_malloc = 0x0000d3ad;
ll_query_timing_info = 0x0000d4e5;
ll_raw_packet_alloc = 0x0000d531;
ll_raw_packet_free = 0x0000d5e5;
ll_raw_packet_get_iq_samples = 0x0000d60f;
ll_raw_packet_get_rx_data = 0x0000d6a9;
ll_raw_packet_recv = 0x0000d741;
ll_raw_packet_send = 0x0000d7d5;
ll_raw_packet_set_param = 0x0000d845;
ll_raw_packet_set_rx_cte = 0x0000d8ab;
ll_raw_packet_set_tx_cte = 0x0000d941;
ll_raw_packet_set_tx_data = 0x0000d97f;
ll_scan_set_fixed_channel = 0x0000da41;
ll_set_adv_coded_scheme = 0x0000db55;
ll_set_conn_coded_scheme = 0x0000db85;
ll_set_conn_latency = 0x0000dbb1;
ll_set_conn_tx_power = 0x0000dbe1;
ll_set_def_antenna = 0x0000dc29;
ll_set_initiating_coded_scheme = 0x0000dc45;
ll_set_max_conn_number = 0x0000dc51;
nibble_for_char = 0x0001e7d5;
platform_32k_rc_auto_tune = 0x0001e875;
platform_32k_rc_tune = 0x0001e8f1;
platform_calibrate_32k = 0x0001e905;
platform_config = 0x0001e909;
platform_get_heap_status = 0x0001e9b9;
platform_get_us_time = 0x0001e9d1;
platform_get_version = 0x0001e9d5;
platform_hrng = 0x0001e9dd;
platform_install_isr_stack = 0x0001e9e5;
platform_patch_rf_init_data = 0x0001e9f1;
platform_printf = 0x0001e9fd;
platform_raise_assertion = 0x0001ea11;
platform_rand = 0x0001ea25;
platform_read_info = 0x0001ea29;
platform_read_persistent_reg = 0x0001ea45;
platform_reset = 0x0001ea55;
platform_set_evt_callback = 0x0001ea89;
platform_set_irq_callback = 0x0001ea9d;
platform_set_rf_clk_source = 0x0001ead5;
platform_set_rf_init_data = 0x0001eae1;
platform_set_rf_power_mapping = 0x0001eaed;
platform_shutdown = 0x0001eaf9;
platform_switch_app = 0x0001eafd;
platform_trace_raw = 0x0001eb29;
platform_write_persistent_reg = 0x0001eb3d;
printf_hexdump = 0x0001eb4d;
pvPortMalloc = 0x0001f60d;
pvTaskIncrementMutexHeldCount = 0x0001f6f5;
pvTimerGetTimerID = 0x0001f70d;
pxPortInitialiseStack = 0x0001f739;
reverse_128 = 0x0001f919;
reverse_24 = 0x0001f91f;
reverse_48 = 0x0001f925;
reverse_56 = 0x0001f92b;
reverse_64 = 0x0001f931;
reverse_bd_addr = 0x0001f937;
reverse_bytes = 0x0001f93d;
sm_add_event_handler = 0x0001fbfd;
sm_address_resolution_lookup = 0x0001fd29;
sm_authenticated = 0x0001fe03;
sm_authorization_decline = 0x0001fe11;
sm_authorization_grant = 0x0001fe31;
sm_authorization_state = 0x0001fe51;
sm_bonding_decline = 0x0001fe6d;
sm_config = 0x00020245;
sm_encryption_key_size = 0x00020351;
sm_just_works_confirm = 0x00020b0d;
sm_le_device_key = 0x00020d5d;
sm_passkey_input = 0x00020df5;
sm_private_random_address_generation_get = 0x00021181;
sm_private_random_address_generation_get_mode = 0x00021189;
sm_private_random_address_generation_set_mode = 0x00021195;
sm_private_random_address_generation_set_update_period = 0x000211bd;
sm_register_oob_data_callback = 0x000211f5;
sm_request_pairing = 0x00021201;
sm_send_security_request = 0x00021c3b;
sm_set_accepted_stk_generation_methods = 0x00021c61;
sm_set_authentication_requirements = 0x00021c6d;
sm_set_encryption_key_size_range = 0x00021c79;
sscanf_bd_addr = 0x00021f75;
sysSetPublicDeviceAddr = 0x000222d5;
uuid128_to_str = 0x00022a09;
uuid_add_bluetooth_prefix = 0x00022a61;
uuid_has_bluetooth_prefix = 0x00022a81;
uxQueueMessagesWaiting = 0x00022ac5;
uxQueueMessagesWaitingFromISR = 0x00022aed;
uxQueueSpacesAvailable = 0x00022b09;
uxTaskGetStackHighWaterMark = 0x00022b35;
vPortEnterCritical = 0x00022bed;
vPortExitCritical = 0x00022c2d;
vPortFree = 0x00022c59;
vPortSuppressTicksAndSleep = 0x00022ced;
vPortValidateInterruptPriority = 0x00022dc5;
vQueueDelete = 0x00022e19;
vQueueWaitForMessageRestricted = 0x00022e45;
vTaskDelay = 0x00022e8d;
vTaskInternalSetTimeOutState = 0x00022ed9;
vTaskMissedYield = 0x00022ee9;
vTaskPlaceOnEventList = 0x00022ef5;
vTaskPlaceOnEventListRestricted = 0x00022f2d;
vTaskPriorityDisinheritAfterTimeout = 0x00022f6d;
vTaskStartScheduler = 0x00023019;
vTaskStepTick = 0x000230a9;
vTaskSuspendAll = 0x000230d9;
vTaskSwitchContext = 0x000230e9;
xPortStartScheduler = 0x00023191;
xQueueAddToSet = 0x0002324d;
xQueueCreateCountingSemaphore = 0x00023271;
xQueueCreateCountingSemaphoreStatic = 0x000232ad;
xQueueCreateMutex = 0x000232f1;
xQueueCreateMutexStatic = 0x00023307;
xQueueCreateSet = 0x00023321;
xQueueGenericCreate = 0x00023329;
xQueueGenericCreateStatic = 0x00023375;
xQueueGenericReset = 0x000233dd;
xQueueGenericSend = 0x00023469;
xQueueGenericSendFromISR = 0x000235d5;
xQueueGiveFromISR = 0x00023695;
xQueueGiveMutexRecursive = 0x00023739;
xQueueIsQueueEmptyFromISR = 0x00023779;
xQueueIsQueueFullFromISR = 0x0002379d;
xQueuePeek = 0x000237c5;
xQueuePeekFromISR = 0x000238ed;
xQueueReceive = 0x00023959;
xQueueReceiveFromISR = 0x00023a85;
xQueueRemoveFromSet = 0x00023b19;
xQueueSelectFromSet = 0x00023b3b;
xQueueSelectFromSetFromISR = 0x00023b4d;
xQueueSemaphoreTake = 0x00023b61;
xQueueTakeMutexRecursive = 0x00023ccd;
xTaskCheckForTimeOut = 0x00023d11;
xTaskCreate = 0x00023d81;
xTaskCreateStatic = 0x00023ddd;
xTaskGetCurrentTaskHandle = 0x00023e4d;
xTaskGetSchedulerState = 0x00023e59;
xTaskGetTickCount = 0x00023e75;
xTaskGetTickCountFromISR = 0x00023e81;
xTaskIncrementTick = 0x00023e91;
xTaskPriorityDisinherit = 0x00023f5d;
xTaskPriorityInherit = 0x00023ff1;
xTaskRemoveFromEventList = 0x00024085;
xTaskResumeAll = 0x00024105;
xTimerCreate = 0x000241cd;
xTimerCreateStatic = 0x00024201;
xTimerCreateTimerTask = 0x00024239;
xTimerGenericCommand = 0x000242a5;
xTimerGetExpiryTime = 0x00024315;
