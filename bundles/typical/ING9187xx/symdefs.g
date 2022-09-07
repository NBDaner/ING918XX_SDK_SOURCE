att_dispatch_client_can_send_now = 0x000059e1;
att_dispatch_client_request_can_send_now_event = 0x000059e7;
att_dispatch_register_client = 0x000059ed;
att_dispatch_register_server = 0x00005a01;
att_dispatch_server_can_send_now = 0x00005a15;
att_dispatch_server_request_can_send_now_event = 0x00005a1b;
att_emit_general_event = 0x00005acd;
att_server_can_send_packet_now = 0x000061e1;
att_server_deferred_read_response = 0x000061e5;
att_server_get_mtu = 0x000061fd;
att_server_indicate = 0x00006275;
att_server_init = 0x000062f9;
att_server_notify = 0x00006335;
att_server_register_packet_handler = 0x0000644d;
att_server_request_can_send_now_event = 0x00006459;
att_set_db = 0x00006475;
att_set_read_callback = 0x00006489;
att_set_write_callback = 0x00006495;
bd_addr_cmp = 0x00006605;
bd_addr_copy = 0x0000660b;
bd_addr_to_str = 0x00006615;
big_endian_read_16 = 0x0000664d;
big_endian_read_32 = 0x00006655;
big_endian_store_16 = 0x00006669;
big_endian_store_32 = 0x00006675;
btstack_config = 0x000067c9;
btstack_memory_pool_create = 0x00006907;
btstack_memory_pool_free = 0x00006931;
btstack_memory_pool_get = 0x00006991;
btstack_push_user_msg = 0x000069f9;
char_for_nibble = 0x00006cc1;
eTaskConfirmSleepModeStatus = 0x00006f69;
gap_add_dev_to_periodic_list = 0x00007589;
gap_add_whitelist = 0x000075a1;
gap_aes_encrypt = 0x000075b5;
gap_clear_white_lists = 0x000075f9;
gap_clr_adv_set = 0x00007609;
gap_clr_periodic_adv_list = 0x00007619;
gap_create_connection_cancel = 0x00007629;
gap_disconnect = 0x00007639;
gap_disconnect_all = 0x00007665;
gap_ext_create_connection = 0x000076a5;
gap_get_connection_parameter_range = 0x00007795;
gap_le_read_channel_map = 0x000077d1;
gap_periodic_adv_create_sync = 0x00007845;
gap_periodic_adv_create_sync_cancel = 0x00007869;
gap_periodic_adv_term_sync = 0x00007879;
gap_read_periodic_adv_list_size = 0x0000790d;
gap_read_phy = 0x0000791d;
gap_read_remote_info = 0x00007931;
gap_read_remote_used_features = 0x00007945;
gap_read_rssi = 0x00007959;
gap_remove_whitelist = 0x0000796d;
gap_rmv_adv_set = 0x000079f1;
gap_rmv_dev_from_periodic_list = 0x00007a05;
gap_rx_test_v2 = 0x00007a1d;
gap_set_adv_set_random_addr = 0x00007a5d;
gap_set_connection_parameter_range = 0x00007aa9;
gap_set_data_length = 0x00007ac1;
gap_set_def_phy = 0x00007add;
gap_set_ext_adv_data = 0x00007af5;
gap_set_ext_adv_enable = 0x00007b0d;
gap_set_ext_adv_para = 0x00007b89;
gap_set_ext_scan_enable = 0x00007c69;
gap_set_ext_scan_para = 0x00007c81;
gap_set_ext_scan_response_data = 0x00007d29;
gap_set_host_channel_classification = 0x00007d41;
gap_set_periodic_adv_data = 0x00007d55;
gap_set_periodic_adv_enable = 0x00007dc9;
gap_set_periodic_adv_para = 0x00007ddd;
gap_set_phy = 0x00007df5;
gap_set_random_device_address = 0x00007e11;
gap_start_ccm = 0x00007e41;
gap_test_end = 0x00007e75;
gap_tx_test_v2 = 0x00007e85;
gap_tx_test_v4 = 0x00007e9d;
gap_update_connection_parameters = 0x00007ec5;
gap_vendor_tx_continuous_wave = 0x00007ee9;
gatt_client_cancel_write = 0x00008411;
gatt_client_discover_characteristic_descriptors = 0x00008437;
gatt_client_discover_characteristics_for_handle_range_by_uuid128 = 0x00008477;
gatt_client_discover_characteristics_for_handle_range_by_uuid16 = 0x000084c7;
gatt_client_discover_characteristics_for_service = 0x00008517;
gatt_client_discover_primary_services = 0x0000854d;
gatt_client_discover_primary_services_by_uuid128 = 0x0000857f;
gatt_client_discover_primary_services_by_uuid16 = 0x000085c3;
gatt_client_execute_write = 0x000085ff;
gatt_client_find_included_services_for_service = 0x00008625;
gatt_client_get_mtu = 0x00008653;
gatt_client_is_ready = 0x000086f5;
gatt_client_listen_for_characteristic_value_updates = 0x0000870b;
gatt_client_prepare_write = 0x0000872d;
gatt_client_read_characteristic_descriptor_using_descriptor_handle = 0x00008769;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle = 0x00008793;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x00008799;
gatt_client_read_long_value_of_characteristic_using_value_handle = 0x000087c7;
gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset = 0x000087cd;
gatt_client_read_multiple_characteristic_values = 0x000087fb;
gatt_client_read_value_of_characteristic_using_value_handle = 0x0000882b;
gatt_client_read_value_of_characteristics_by_uuid128 = 0x00008859;
gatt_client_read_value_of_characteristics_by_uuid16 = 0x000088a5;
gatt_client_register_handler = 0x000088f1;
gatt_client_reliable_write_long_value_of_characteristic = 0x000088fd;
gatt_client_signed_write_without_response = 0x00008d2d;
gatt_client_write_characteristic_descriptor_using_descriptor_handle = 0x00008df1;
gatt_client_write_client_characteristic_configuration = 0x00008e2b;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle = 0x00008e7d;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x00008e8d;
gatt_client_write_long_value_of_characteristic = 0x00008ec9;
gatt_client_write_long_value_of_characteristic_with_offset = 0x00008ed9;
gatt_client_write_value_of_characteristic = 0x00008f15;
gatt_client_write_value_of_characteristic_without_response = 0x00008f4b;
hci_add_event_handler = 0x0000a471;
hci_power_control = 0x0000ac65;
hci_register_acl_packet_handler = 0x0000ae19;
kv_commit = 0x0000b38d;
kv_get = 0x0000b3e5;
kv_init = 0x0000b3fd;
kv_put = 0x0000b465;
kv_remove = 0x0000b4dd;
kv_remove_all = 0x0000b519;
kv_value_modified = 0x0000b55d;
kv_visit = 0x0000b561;
l2cap_can_send_fixed_channel_packet_now = 0x0000b621;
l2cap_can_send_packet_now = 0x0000b625;
l2cap_create_channel = 0x0000b7dd;
l2cap_disconnect = 0x0000b915;
l2cap_get_remote_mtu_for_local_cid = 0x0000bb3d;
l2cap_init = 0x0000bf35;
l2cap_le_send_flow_control_credit = 0x0000bf75;
l2cap_max_le_mtu = 0x0000c231;
l2cap_max_mtu = 0x0000c235;
l2cap_register_fixed_channel = 0x0000c341;
l2cap_register_packet_handler = 0x0000c35d;
l2cap_register_service = 0x0000c369;
l2cap_request_can_send_fix_channel_now_event = 0x0000c44d;
l2cap_request_can_send_now_event = 0x0000c471;
l2cap_request_connection_parameter_update = 0x0000c48b;
l2cap_send = 0x0000c825;
l2cap_send_connectionless = 0x0000c89d;
l2cap_send_connectionless3 = 0x0000c92d;
l2cap_send_echo_request = 0x0000c9c5;
l2cap_unregister_service = 0x0000ca81;
le_device_db_add = 0x0000cad9;
le_device_db_find = 0x0000cbad;
le_device_db_from_key = 0x0000cbd9;
le_device_db_iter_cur = 0x0000cbe1;
le_device_db_iter_cur_key = 0x0000cbe5;
le_device_db_iter_init = 0x0000cbe9;
le_device_db_iter_next = 0x0000cbf1;
le_device_db_remove_key = 0x0000cc17;
ll_free = 0x0000cc43;
ll_hint_on_ce_len = 0x0000cc4d;
ll_legacy_adv_set_interval = 0x0000cc85;
ll_malloc = 0x0000cc95;
ll_query_timing_info = 0x0000cdcd;
ll_scan_set_fixed_channel = 0x0000ce71;
ll_set_adv_access_address = 0x0000cf85;
ll_set_adv_coded_scheme = 0x0000cf91;
ll_set_conn_coded_scheme = 0x0000cfc1;
ll_set_conn_latency = 0x0000cfed;
ll_set_conn_tx_power = 0x0000d01d;
ll_set_def_antenna = 0x0000d065;
ll_set_initiating_coded_scheme = 0x0000d081;
ll_set_max_conn_number = 0x0000d08d;
nibble_for_char = 0x0001ceb9;
platform_32k_rc_auto_tune = 0x0001cf55;
platform_32k_rc_tune = 0x0001cfd1;
platform_calibrate_32k = 0x0001cfe5;
platform_config = 0x0001cfe9;
platform_get_heap_status = 0x0001d0bd;
platform_get_task_handle = 0x0001d0d5;
platform_get_us_time = 0x0001d0f5;
platform_get_version = 0x0001d0f9;
platform_hrng = 0x0001d101;
platform_install_isr_stack = 0x0001d109;
platform_patch_rf_init_data = 0x0001d115;
platform_printf = 0x0001d121;
platform_raise_assertion = 0x0001d135;
platform_rand = 0x0001d149;
platform_read_info = 0x0001d14d;
platform_read_persistent_reg = 0x0001d169;
platform_reset = 0x0001d179;
platform_set_evt_callback = 0x0001d1ad;
platform_set_irq_callback = 0x0001d1c1;
platform_set_rf_clk_source = 0x0001d1f9;
platform_set_rf_init_data = 0x0001d205;
platform_set_rf_power_mapping = 0x0001d211;
platform_set_timer = 0x0001d21d;
platform_shutdown = 0x0001d221;
platform_switch_app = 0x0001d225;
platform_trace_raw = 0x0001d251;
platform_write_persistent_reg = 0x0001d269;
printf_hexdump = 0x0001d279;
pvPortMalloc = 0x0001dd81;
pvTaskIncrementMutexHeldCount = 0x0001de69;
pvTimerGetTimerID = 0x0001de81;
pxPortInitialiseStack = 0x0001dead;
reverse_128 = 0x0001e055;
reverse_24 = 0x0001e05b;
reverse_48 = 0x0001e061;
reverse_56 = 0x0001e067;
reverse_64 = 0x0001e06d;
reverse_bd_addr = 0x0001e073;
reverse_bytes = 0x0001e079;
sm_add_event_handler = 0x0001e1e5;
sm_address_resolution_lookup = 0x0001e33d;
sm_authenticated = 0x0001e695;
sm_authorization_decline = 0x0001e6a3;
sm_authorization_grant = 0x0001e6c3;
sm_authorization_state = 0x0001e6e3;
sm_bonding_decline = 0x0001e6fd;
sm_config = 0x0001eb1d;
sm_config_conn = 0x0001eb35;
sm_encryption_key_size = 0x0001eceb;
sm_just_works_confirm = 0x0001f225;
sm_le_device_key = 0x0001f561;
sm_passkey_input = 0x0001f5f7;
sm_private_random_address_generation_get = 0x0001f9a5;
sm_private_random_address_generation_get_mode = 0x0001f9ad;
sm_private_random_address_generation_set_mode = 0x0001f9b9;
sm_private_random_address_generation_set_update_period = 0x0001f9e1;
sm_register_oob_data_callback = 0x0001fb1d;
sm_request_pairing = 0x0001fb29;
sm_send_security_request = 0x00020547;
sm_set_accepted_stk_generation_methods = 0x0002056d;
sm_set_authentication_requirements = 0x00020579;
sm_set_encryption_key_size_range = 0x00020585;
sscanf_bd_addr = 0x000208e1;
sysSetPublicDeviceAddr = 0x00020c49;
uuid128_to_str = 0x0002122d;
uuid_add_bluetooth_prefix = 0x00021285;
uuid_has_bluetooth_prefix = 0x000212a5;
uxListRemove = 0x000212c1;
uxQueueMessagesWaiting = 0x000212e9;
uxQueueMessagesWaitingFromISR = 0x00021311;
uxQueueSpacesAvailable = 0x0002132d;
uxTaskGetStackHighWaterMark = 0x00021359;
uxTaskPriorityGet = 0x00021379;
uxTaskPriorityGetFromISR = 0x00021395;
vListInitialise = 0x00021447;
vListInitialiseItem = 0x0002145d;
vListInsert = 0x00021463;
vListInsertEnd = 0x00021493;
vPortEndScheduler = 0x000214ad;
vPortEnterCritical = 0x000214d5;
vPortExitCritical = 0x00021519;
vPortFree = 0x00021549;
vPortSuppressTicksAndSleep = 0x000215dd;
vPortValidateInterruptPriority = 0x000216e5;
vQueueDelete = 0x0002173d;
vQueueWaitForMessageRestricted = 0x00021769;
vTaskDelay = 0x000217b1;
vTaskInternalSetTimeOutState = 0x000217fd;
vTaskMissedYield = 0x0002180d;
vTaskPlaceOnEventList = 0x00021819;
vTaskPlaceOnEventListRestricted = 0x00021851;
vTaskPriorityDisinheritAfterTimeout = 0x00021891;
vTaskPrioritySet = 0x0002193d;
vTaskResume = 0x00021a05;
vTaskStartScheduler = 0x00021a89;
vTaskStepTick = 0x00021b19;
vTaskSuspend = 0x00021b49;
vTaskSuspendAll = 0x00021c05;
vTaskSwitchContext = 0x00021c15;
xPortStartScheduler = 0x00021cbd;
xQueueAddToSet = 0x00021d7d;
xQueueCreateCountingSemaphore = 0x00021da1;
xQueueCreateCountingSemaphoreStatic = 0x00021ddd;
xQueueCreateMutex = 0x00021e21;
xQueueCreateMutexStatic = 0x00021e37;
xQueueCreateSet = 0x00021e51;
xQueueGenericCreate = 0x00021e59;
xQueueGenericCreateStatic = 0x00021ea5;
xQueueGenericReset = 0x00021f0d;
xQueueGenericSend = 0x00021f99;
xQueueGenericSendFromISR = 0x00022105;
xQueueGiveFromISR = 0x000221c5;
xQueueGiveMutexRecursive = 0x00022269;
xQueueIsQueueEmptyFromISR = 0x000222a9;
xQueueIsQueueFullFromISR = 0x000222cd;
xQueuePeek = 0x000222f5;
xQueuePeekFromISR = 0x0002241d;
xQueueReceive = 0x00022489;
xQueueReceiveFromISR = 0x000225b5;
xQueueRemoveFromSet = 0x00022649;
xQueueSelectFromSet = 0x0002266b;
xQueueSelectFromSetFromISR = 0x0002267d;
xQueueSemaphoreTake = 0x00022691;
xQueueTakeMutexRecursive = 0x000227fd;
xTaskCheckForTimeOut = 0x00022841;
xTaskCreate = 0x000228b1;
xTaskCreateStatic = 0x0002290d;
xTaskGetCurrentTaskHandle = 0x0002297d;
xTaskGetSchedulerState = 0x00022989;
xTaskGetTickCount = 0x000229a5;
xTaskGetTickCountFromISR = 0x000229b1;
xTaskIncrementTick = 0x000229c1;
xTaskPriorityDisinherit = 0x00022a8d;
xTaskPriorityInherit = 0x00022b21;
xTaskRemoveFromEventList = 0x00022bb5;
xTaskResumeAll = 0x00022c35;
xTaskResumeFromISR = 0x00022cfd;
xTimerCreate = 0x00022d89;
xTimerCreateStatic = 0x00022dbd;
xTimerCreateTimerTask = 0x00022df5;
xTimerGenericCommand = 0x00022e61;
xTimerGetExpiryTime = 0x00022ed1;
xTimerGetTimerDaemonTaskHandle = 0x00022ef1;
