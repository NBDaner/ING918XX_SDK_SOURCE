att_dispatch_client_can_send_now = 0x00005a39;
att_dispatch_client_request_can_send_now_event = 0x00005a3f;
att_dispatch_register_client = 0x00005a45;
att_dispatch_register_server = 0x00005a59;
att_dispatch_server_can_send_now = 0x00005a6d;
att_dispatch_server_request_can_send_now_event = 0x00005a73;
att_emit_general_event = 0x00005b25;
att_server_can_send_packet_now = 0x00006239;
att_server_deferred_read_response = 0x0000623d;
att_server_get_mtu = 0x00006255;
att_server_indicate = 0x000062cd;
att_server_init = 0x00006351;
att_server_notify = 0x0000638d;
att_server_register_packet_handler = 0x000064a5;
att_server_request_can_send_now_event = 0x000064b1;
att_set_db = 0x000064cd;
att_set_read_callback = 0x000064e1;
att_set_write_callback = 0x000064ed;
bd_addr_cmp = 0x0000665d;
bd_addr_copy = 0x00006663;
bd_addr_to_str = 0x0000666d;
big_endian_read_16 = 0x000066a5;
big_endian_read_32 = 0x000066ad;
big_endian_store_16 = 0x000066c1;
big_endian_store_32 = 0x000066cd;
btstack_config = 0x00006805;
btstack_memory_pool_create = 0x00006953;
btstack_memory_pool_free = 0x0000697d;
btstack_memory_pool_get = 0x000069dd;
btstack_push_user_msg = 0x00006a25;
char_for_nibble = 0x00006cb5;
gap_add_dev_to_periodic_list = 0x000075c1;
gap_add_whitelist = 0x000075d9;
gap_aes_encrypt = 0x000075ed;
gap_clear_white_lists = 0x00007631;
gap_clr_adv_set = 0x00007641;
gap_clr_periodic_adv_list = 0x00007651;
gap_create_connection_cancel = 0x00007661;
gap_default_periodic_adv_sync_transfer_param = 0x00007671;
gap_disconnect = 0x00007689;
gap_disconnect_all = 0x000076b5;
gap_ext_create_connection = 0x000076f5;
gap_get_connection_parameter_range = 0x000077e5;
gap_le_read_channel_map = 0x00007821;
gap_periodic_adv_create_sync = 0x00007895;
gap_periodic_adv_create_sync_cancel = 0x000078b9;
gap_periodic_adv_set_info_transfer = 0x000078c9;
gap_periodic_adv_sync_transfer = 0x000078e1;
gap_periodic_adv_sync_transfer_param = 0x000078f9;
gap_periodic_adv_term_sync = 0x00007915;
gap_read_antenna_info = 0x000079a9;
gap_read_periodic_adv_list_size = 0x000079b9;
gap_read_phy = 0x000079c9;
gap_read_remote_info = 0x000079dd;
gap_read_remote_used_features = 0x000079f1;
gap_read_rssi = 0x00007a05;
gap_remove_whitelist = 0x00007a19;
gap_rmv_adv_set = 0x00007a9d;
gap_rmv_dev_from_periodic_list = 0x00007ab1;
gap_rx_test_v2 = 0x00007ac9;
gap_rx_test_v3 = 0x00007ae1;
gap_set_adv_set_random_addr = 0x00007b31;
gap_set_connection_cte_request_enable = 0x00007b7d;
gap_set_connection_cte_response_enable = 0x00007b99;
gap_set_connection_cte_rx_param = 0x00007bad;
gap_set_connection_cte_tx_param = 0x00007c09;
gap_set_connection_parameter_range = 0x00007c5d;
gap_set_connectionless_cte_tx_enable = 0x00007c75;
gap_set_connectionless_cte_tx_param = 0x00007c89;
gap_set_connectionless_iq_sampling_enable = 0x00007ce9;
gap_set_data_length = 0x00007d4d;
gap_set_def_phy = 0x00007d69;
gap_set_ext_adv_data = 0x00007d81;
gap_set_ext_adv_enable = 0x00007d99;
gap_set_ext_adv_para = 0x00007e15;
gap_set_ext_scan_enable = 0x00007ef5;
gap_set_ext_scan_para = 0x00007f0d;
gap_set_ext_scan_response_data = 0x00007fb5;
gap_set_host_channel_classification = 0x00007fcd;
gap_set_periodic_adv_data = 0x00007fe1;
gap_set_periodic_adv_enable = 0x00008055;
gap_set_periodic_adv_para = 0x00008069;
gap_set_periodic_adv_rx_enable = 0x00008081;
gap_set_phy = 0x00008095;
gap_set_random_device_address = 0x000080b1;
gap_start_ccm = 0x000080e1;
gap_test_end = 0x00008115;
gap_tx_test_v2 = 0x00008125;
gap_tx_test_v4 = 0x0000813d;
gap_update_connection_parameters = 0x00008165;
gap_vendor_tx_continuous_wave = 0x00008189;
gatt_client_cancel_write = 0x000086b1;
gatt_client_discover_characteristic_descriptors = 0x000086d7;
gatt_client_discover_characteristics_for_handle_range_by_uuid128 = 0x00008717;
gatt_client_discover_characteristics_for_handle_range_by_uuid16 = 0x00008767;
gatt_client_discover_characteristics_for_service = 0x000087b7;
gatt_client_discover_primary_services = 0x000087ed;
gatt_client_discover_primary_services_by_uuid128 = 0x0000881f;
gatt_client_discover_primary_services_by_uuid16 = 0x00008863;
gatt_client_execute_write = 0x0000889f;
gatt_client_find_included_services_for_service = 0x000088c5;
gatt_client_get_mtu = 0x000088f3;
gatt_client_is_ready = 0x00008995;
gatt_client_listen_for_characteristic_value_updates = 0x000089ab;
gatt_client_prepare_write = 0x000089cd;
gatt_client_read_characteristic_descriptor_using_descriptor_handle = 0x00008a09;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle = 0x00008a33;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x00008a39;
gatt_client_read_long_value_of_characteristic_using_value_handle = 0x00008a67;
gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset = 0x00008a6d;
gatt_client_read_multiple_characteristic_values = 0x00008a9b;
gatt_client_read_value_of_characteristic_using_value_handle = 0x00008acb;
gatt_client_read_value_of_characteristics_by_uuid128 = 0x00008af9;
gatt_client_read_value_of_characteristics_by_uuid16 = 0x00008b45;
gatt_client_register_handler = 0x00008b91;
gatt_client_reliable_write_long_value_of_characteristic = 0x00008b9d;
gatt_client_signed_write_without_response = 0x00008fcd;
gatt_client_write_characteristic_descriptor_using_descriptor_handle = 0x00009091;
gatt_client_write_client_characteristic_configuration = 0x000090cb;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle = 0x0000911d;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x0000912d;
gatt_client_write_long_value_of_characteristic = 0x00009169;
gatt_client_write_long_value_of_characteristic_with_offset = 0x00009179;
gatt_client_write_value_of_characteristic = 0x000091b5;
gatt_client_write_value_of_characteristic_without_response = 0x000091eb;
hci_add_event_handler = 0x0000a72d;
hci_power_control = 0x0000af25;
hci_register_acl_packet_handler = 0x0000b0d9;
kv_commit = 0x0000b671;
kv_get = 0x0000b6c9;
kv_init = 0x0000b6e1;
kv_put = 0x0000b749;
kv_remove = 0x0000b7c1;
kv_remove_all = 0x0000b7fd;
kv_value_modified = 0x0000b841;
kv_visit = 0x0000b845;
l2cap_can_send_fixed_channel_packet_now = 0x0000b905;
l2cap_can_send_packet_now = 0x0000b909;
l2cap_create_channel = 0x0000bac1;
l2cap_disconnect = 0x0000bbf9;
l2cap_get_remote_mtu_for_local_cid = 0x0000be21;
l2cap_init = 0x0000c219;
l2cap_le_send_flow_control_credit = 0x0000c259;
l2cap_max_le_mtu = 0x0000c515;
l2cap_max_mtu = 0x0000c519;
l2cap_register_fixed_channel = 0x0000c625;
l2cap_register_packet_handler = 0x0000c641;
l2cap_register_service = 0x0000c64d;
l2cap_request_can_send_fix_channel_now_event = 0x0000c731;
l2cap_request_can_send_now_event = 0x0000c755;
l2cap_request_connection_parameter_update = 0x0000c76f;
l2cap_send = 0x0000cb09;
l2cap_send_connectionless = 0x0000cb81;
l2cap_send_connectionless3 = 0x0000cc11;
l2cap_send_echo_request = 0x0000cca9;
l2cap_unregister_service = 0x0000cd65;
le_device_db_add = 0x0000cdbd;
le_device_db_find = 0x0000ce91;
le_device_db_from_key = 0x0000cebd;
le_device_db_iter_cur = 0x0000cec5;
le_device_db_iter_cur_key = 0x0000cec9;
le_device_db_iter_init = 0x0000cecd;
le_device_db_iter_next = 0x0000ced5;
le_device_db_remove_key = 0x0000cefb;
ll_ackable_packet_alloc = 0x0000cf27;
ll_ackable_packet_get_status = 0x0000d02f;
ll_ackable_packet_run = 0x0000d0a1;
ll_ackable_packet_set_tx_data = 0x0000d149;
ll_attach_cte_to_adv_set = 0x0000d165;
ll_free = 0x0000d2fd;
ll_hint_on_ce_len = 0x0000d309;
ll_legacy_adv_set_interval = 0x0000d341;
ll_malloc = 0x0000d351;
ll_query_timing_info = 0x0000d489;
ll_raw_packet_alloc = 0x0000d4d5;
ll_raw_packet_free = 0x0000d5a9;
ll_raw_packet_get_bare_rx_data = 0x0000d5d3;
ll_raw_packet_get_iq_samples = 0x0000d699;
ll_raw_packet_get_rx_data = 0x0000d733;
ll_raw_packet_recv = 0x0000d7e9;
ll_raw_packet_send = 0x0000d8a5;
ll_raw_packet_set_bare_data = 0x0000d98d;
ll_raw_packet_set_bare_mode = 0x0000d9cb;
ll_raw_packet_set_fake_cte_info = 0x0000dad1;
ll_raw_packet_set_param = 0x0000daf3;
ll_raw_packet_set_rx_cte = 0x0000db51;
ll_raw_packet_set_tx_cte = 0x0000dbe7;
ll_raw_packet_set_tx_data = 0x0000dc25;
ll_scan_set_fixed_channel = 0x0000dce1;
ll_scanner_enable_iq_sampling = 0x0000dced;
ll_set_adv_access_address = 0x0000de9d;
ll_set_adv_coded_scheme = 0x0000dea9;
ll_set_conn_coded_scheme = 0x0000ded9;
ll_set_conn_interval_unit = 0x0000df05;
ll_set_conn_latency = 0x0000df11;
ll_set_conn_tx_power = 0x0000df41;
ll_set_def_antenna = 0x0000df89;
ll_set_initiating_coded_scheme = 0x0000dfa5;
ll_set_max_conn_number = 0x0000dfb1;
nibble_for_char = 0x0001eefd;
platform_32k_rc_auto_tune = 0x0001ef99;
platform_32k_rc_tune = 0x0001f015;
platform_calibrate_32k = 0x0001f029;
platform_config = 0x0001f02d;
platform_controller_run = 0x0001f0fd;
platform_get_task_handle = 0x0001f135;
platform_get_us_time = 0x0001f14d;
platform_get_version = 0x0001f151;
platform_hrng = 0x0001f159;
platform_init_controller = 0x0001f161;
platform_os_idle_resumed_hook = 0x0001f17d;
platform_patch_rf_init_data = 0x0001f181;
platform_post_sleep_processing = 0x0001f18d;
platform_pre_sleep_processing = 0x0001f193;
platform_pre_suppress_ticks_and_sleep_processing = 0x0001f199;
platform_printf = 0x0001f19d;
platform_raise_assertion = 0x0001f1b1;
platform_rand = 0x0001f1c5;
platform_read_info = 0x0001f1c9;
platform_read_persistent_reg = 0x0001f1e5;
platform_reset = 0x0001f1f5;
platform_set_evt_callback = 0x0001f219;
platform_set_irq_callback = 0x0001f22d;
platform_set_rf_clk_source = 0x0001f265;
platform_set_rf_init_data = 0x0001f271;
platform_set_rf_power_mapping = 0x0001f27d;
platform_set_timer = 0x0001f289;
platform_shutdown = 0x0001f28d;
platform_switch_app = 0x0001f291;
platform_trace_raw = 0x0001f2bd;
platform_write_persistent_reg = 0x0001f2d5;
printf_hexdump = 0x0001f2e5;
reverse_128 = 0x0001f659;
reverse_24 = 0x0001f65f;
reverse_48 = 0x0001f665;
reverse_56 = 0x0001f66b;
reverse_64 = 0x0001f671;
reverse_bd_addr = 0x0001f677;
reverse_bytes = 0x0001f67d;
sm_add_event_handler = 0x0001f95d;
sm_address_resolution_lookup = 0x0001fab5;
sm_authenticated = 0x0001fe0d;
sm_authorization_decline = 0x0001fe1b;
sm_authorization_grant = 0x0001fe3b;
sm_authorization_state = 0x0001fe5b;
sm_bonding_decline = 0x0001fe75;
sm_config = 0x00020295;
sm_config_conn = 0x000202ad;
sm_encryption_key_size = 0x00020463;
sm_just_works_confirm = 0x0002099d;
sm_le_device_key = 0x00020cd9;
sm_passkey_input = 0x00020d6f;
sm_private_random_address_generation_get = 0x0002111d;
sm_private_random_address_generation_get_mode = 0x00021125;
sm_private_random_address_generation_set_mode = 0x00021131;
sm_private_random_address_generation_set_update_period = 0x00021159;
sm_register_oob_data_callback = 0x00021295;
sm_request_pairing = 0x000212a1;
sm_send_security_request = 0x00021cbf;
sm_set_accepted_stk_generation_methods = 0x00021ce5;
sm_set_authentication_requirements = 0x00021cf1;
sm_set_encryption_key_size_range = 0x00021cfd;
sscanf_bd_addr = 0x000220cd;
sysSetPublicDeviceAddr = 0x00022435;
uuid128_to_str = 0x00022bb1;
uuid_add_bluetooth_prefix = 0x00022c09;
uuid_has_bluetooth_prefix = 0x00022c29;
