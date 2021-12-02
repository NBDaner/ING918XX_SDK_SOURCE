--define_symbol att_dispatch_client_can_send_now=0x000059c9
--define_symbol att_dispatch_client_request_can_send_now_event=0x000059cf
--define_symbol att_dispatch_register_client=0x000059d5
--define_symbol att_dispatch_register_server=0x000059e9
--define_symbol att_dispatch_server_can_send_now=0x000059fd
--define_symbol att_dispatch_server_request_can_send_now_event=0x00005a03
--define_symbol att_emit_general_event=0x00005ab5
--define_symbol att_server_can_send_packet_now=0x000061c9
--define_symbol att_server_deferred_read_response=0x000061cd
--define_symbol att_server_get_mtu=0x000061e5
--define_symbol att_server_indicate=0x0000625d
--define_symbol att_server_init=0x000062e1
--define_symbol att_server_notify=0x0000631d
--define_symbol att_server_register_packet_handler=0x00006435
--define_symbol att_server_request_can_send_now_event=0x00006441
--define_symbol att_set_db=0x0000645d
--define_symbol att_set_read_callback=0x00006471
--define_symbol att_set_write_callback=0x0000647d
--define_symbol bd_addr_cmp=0x000065ed
--define_symbol bd_addr_copy=0x000065f3
--define_symbol bd_addr_to_str=0x000065fd
--define_symbol big_endian_read_16=0x00006635
--define_symbol big_endian_read_32=0x0000663d
--define_symbol big_endian_store_16=0x00006651
--define_symbol big_endian_store_32=0x0000665d
--define_symbol btstack_config=0x000067b1
--define_symbol btstack_memory_pool_create=0x000068ef
--define_symbol btstack_memory_pool_free=0x00006919
--define_symbol btstack_memory_pool_get=0x00006979
--define_symbol btstack_push_user_msg=0x00006995
--define_symbol char_for_nibble=0x00006c5d
--define_symbol eTaskConfirmSleepModeStatus=0x00006f05
--define_symbol gap_add_dev_to_periodic_list=0x00007605
--define_symbol gap_add_whitelist=0x0000761d
--define_symbol gap_aes_encrypt=0x00007631
--define_symbol gap_clear_white_lists=0x00007675
--define_symbol gap_clr_adv_set=0x00007685
--define_symbol gap_clr_periodic_adv_list=0x00007695
--define_symbol gap_create_connection_cancel=0x000076c1
--define_symbol gap_disconnect=0x000076d1
--define_symbol gap_disconnect_all=0x000076fd
--define_symbol gap_ext_create_connection=0x000077a5
--define_symbol gap_get_connection_parameter_range=0x00007895
--define_symbol gap_le_read_channel_map=0x000078cd
--define_symbol gap_periodic_adv_create_sync=0x00007941
--define_symbol gap_periodic_adv_create_sync_cancel=0x00007965
--define_symbol gap_periodic_adv_term_sync=0x00007975
--define_symbol gap_read_periodic_adv_list_size=0x00007a09
--define_symbol gap_read_phy=0x00007a19
--define_symbol gap_read_remote_info=0x00007a2d
--define_symbol gap_read_remote_used_features=0x00007a41
--define_symbol gap_read_rssi=0x00007a55
--define_symbol gap_remove_whitelist=0x00007a69
--define_symbol gap_rmv_adv_set=0x00007af5
--define_symbol gap_rmv_dev_from_periodic_list=0x00007b09
--define_symbol gap_rx_test_v2=0x00007b21
--define_symbol gap_set_adv_set_random_addr=0x00007b61
--define_symbol gap_set_connection_parameter_range=0x00007bb1
--define_symbol gap_set_data_length=0x00007bcd
--define_symbol gap_set_def_phy=0x00007be9
--define_symbol gap_set_ext_adv_data=0x00007c01
--define_symbol gap_set_ext_adv_enable=0x00007c19
--define_symbol gap_set_ext_adv_para=0x00007c95
--define_symbol gap_set_ext_scan_enable=0x00007d75
--define_symbol gap_set_ext_scan_para=0x00007d8d
--define_symbol gap_set_ext_scan_response_data=0x00007e35
--define_symbol gap_set_host_channel_classification=0x00007e4d
--define_symbol gap_set_periodic_adv_data=0x00007e61
--define_symbol gap_set_periodic_adv_enable=0x00007ed5
--define_symbol gap_set_periodic_adv_para=0x00007ee9
--define_symbol gap_set_phy=0x00007f01
--define_symbol gap_set_random_device_address=0x00007f1d
--define_symbol gap_start_ccm=0x00007f81
--define_symbol gap_test_end=0x00007fb5
--define_symbol gap_tx_test_v2=0x00007fc5
--define_symbol gap_update_connection_parameters=0x00007fdd
--define_symbol gap_vendor_tx_continuous_wave=0x00008001
--define_symbol gatt_client_cancel_write=0x00008529
--define_symbol gatt_client_discover_characteristic_descriptors=0x0000854f
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x0000858f
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x000085df
--define_symbol gatt_client_discover_characteristics_for_service=0x0000862f
--define_symbol gatt_client_discover_primary_services=0x00008665
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x00008697
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x000086db
--define_symbol gatt_client_execute_write=0x00008717
--define_symbol gatt_client_find_included_services_for_service=0x0000873d
--define_symbol gatt_client_get_mtu=0x0000876b
--define_symbol gatt_client_is_ready=0x0000880d
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x00008823
--define_symbol gatt_client_prepare_write=0x00008845
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x00008881
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x000088ab
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x000088b1
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x000088df
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x000088e5
--define_symbol gatt_client_read_multiple_characteristic_values=0x00008913
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x00008943
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x00008971
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x000089bd
--define_symbol gatt_client_register_handler=0x00008a09
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x00008a15
--define_symbol gatt_client_signed_write_without_response=0x00008e45
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x00008f09
--define_symbol gatt_client_write_client_characteristic_configuration=0x00008f43
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x00008f95
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008fa5
--define_symbol gatt_client_write_long_value_of_characteristic=0x00008fe1
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x00008ff1
--define_symbol gatt_client_write_value_of_characteristic=0x0000902d
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x00009063
--define_symbol hci_add_event_handler=0x0000a549
--define_symbol hci_power_control=0x0000acf5
--define_symbol hci_register_acl_packet_handler=0x0000aea9
--define_symbol kv_commit=0x0000b519
--define_symbol kv_get=0x0000b571
--define_symbol kv_init=0x0000b589
--define_symbol kv_put=0x0000b5f1
--define_symbol kv_remove=0x0000b669
--define_symbol kv_remove_all=0x0000b6a5
--define_symbol kv_value_modified=0x0000b6e9
--define_symbol kv_visit=0x0000b6ed
--define_symbol l2cap_can_send_fixed_channel_packet_now=0x0000b7ad
--define_symbol l2cap_can_send_packet_now=0x0000b7b1
--define_symbol l2cap_create_channel=0x0000b969
--define_symbol l2cap_disconnect=0x0000baa1
--define_symbol l2cap_get_remote_mtu_for_local_cid=0x0000bd15
--define_symbol l2cap_init=0x0000c195
--define_symbol l2cap_le_send_flow_control_credit=0x0000c1dd
--define_symbol l2cap_max_le_mtu=0x0000c461
--define_symbol l2cap_max_mtu=0x0000c465
--define_symbol l2cap_next_local_cid=0x0000c469
--define_symbol l2cap_next_sig_id=0x0000c479
--define_symbol l2cap_register_fixed_channel=0x0000c511
--define_symbol l2cap_register_packet_handler=0x0000c52d
--define_symbol l2cap_register_service=0x0000c539
--define_symbol l2cap_request_can_send_fix_channel_now_event=0x0000c621
--define_symbol l2cap_request_can_send_now_event=0x0000c645
--define_symbol l2cap_request_connection_parameter_update=0x0000c65f
--define_symbol l2cap_require_security_level_2_for_outgoing_sdp=0x0000c691
--define_symbol l2cap_send=0x0000ca25
--define_symbol l2cap_send_connectionless=0x0000ca9d
--define_symbol l2cap_send_connectionless3=0x0000cb2d
--define_symbol l2cap_send_echo_request=0x0000cbc5
--define_symbol l2cap_send_signaling_le=0x0000cc29
--define_symbol l2cap_unregister_service=0x0000cc81
--define_symbol le_device_db_add=0x0000cce5
--define_symbol le_device_db_find=0x0000cdb9
--define_symbol le_device_db_from_key=0x0000cde5
--define_symbol le_device_db_iter_cur=0x0000cded
--define_symbol le_device_db_iter_cur_key=0x0000cdf1
--define_symbol le_device_db_iter_init=0x0000cdf5
--define_symbol le_device_db_iter_next=0x0000cdfd
--define_symbol le_device_db_remove_key=0x0000ce23
--define_symbol ll_free=0x0000ce4f
--define_symbol ll_hint_on_ce_len=0x0000ce59
--define_symbol ll_legacy_adv_set_interval=0x0000ce99
--define_symbol ll_malloc=0x0000cea9
--define_symbol ll_query_timing_info=0x0000cfe1
--define_symbol ll_scan_set_fixed_channel=0x0000d085
--define_symbol ll_set_adv_access_address=0x0000d199
--define_symbol ll_set_adv_coded_scheme=0x0000d1a5
--define_symbol ll_set_conn_coded_scheme=0x0000d1d5
--define_symbol ll_set_conn_latency=0x0000d201
--define_symbol ll_set_conn_tx_power=0x0000d231
--define_symbol ll_set_def_antenna=0x0000d279
--define_symbol ll_set_initiating_coded_scheme=0x0000d295
--define_symbol ll_set_max_conn_number=0x0000d2a1
--define_symbol nibble_for_char=0x0001d049
--define_symbol platform_32k_rc_auto_tune=0x0001d0e5
--define_symbol platform_32k_rc_tune=0x0001d161
--define_symbol platform_calibrate_32k=0x0001d175
--define_symbol platform_config=0x0001d179
--define_symbol platform_get_heap_status=0x0001d239
--define_symbol platform_get_us_time=0x0001d251
--define_symbol platform_get_version=0x0001d255
--define_symbol platform_hrng=0x0001d25d
--define_symbol platform_install_isr_stack=0x0001d265
--define_symbol platform_patch_rf_init_data=0x0001d271
--define_symbol platform_printf=0x0001d27d
--define_symbol platform_raise_assertion=0x0001d291
--define_symbol platform_rand=0x0001d2a5
--define_symbol platform_read_info=0x0001d2a9
--define_symbol platform_read_persistent_reg=0x0001d2c5
--define_symbol platform_reset=0x0001d2d5
--define_symbol platform_set_evt_callback=0x0001d309
--define_symbol platform_set_irq_callback=0x0001d31d
--define_symbol platform_set_rf_clk_source=0x0001d355
--define_symbol platform_set_rf_init_data=0x0001d361
--define_symbol platform_set_rf_power_mapping=0x0001d36d
--define_symbol platform_shutdown=0x0001d379
--define_symbol platform_switch_app=0x0001d37d
--define_symbol platform_trace_raw=0x0001d3a9
--define_symbol platform_write_persistent_reg=0x0001d3c1
--define_symbol printf_hexdump=0x0001d3d1
--define_symbol pvPortMalloc=0x0001dedd
--define_symbol pvTaskIncrementMutexHeldCount=0x0001dfc5
--define_symbol pvTimerGetTimerID=0x0001dfdd
--define_symbol pxPortInitialiseStack=0x0001e009
--define_symbol reverse_128=0x0001e1b1
--define_symbol reverse_24=0x0001e1b7
--define_symbol reverse_48=0x0001e1bd
--define_symbol reverse_56=0x0001e1c3
--define_symbol reverse_64=0x0001e1c9
--define_symbol reverse_bd_addr=0x0001e1cf
--define_symbol reverse_bytes=0x0001e1d5
--define_symbol sm_add_event_handler=0x0001e341
--define_symbol sm_address_resolution_lookup=0x0001e499
--define_symbol sm_authenticated=0x0001e7e5
--define_symbol sm_authorization_decline=0x0001e7f3
--define_symbol sm_authorization_grant=0x0001e813
--define_symbol sm_authorization_state=0x0001e833
--define_symbol sm_bonding_decline=0x0001e84d
--define_symbol sm_config=0x0001ec6d
--define_symbol sm_config_conn=0x0001ec85
--define_symbol sm_encryption_key_size=0x0001ee0b
--define_symbol sm_just_works_confirm=0x0001f339
--define_symbol sm_le_device_key=0x0001f675
--define_symbol sm_passkey_input=0x0001f70b
--define_symbol sm_private_random_address_generation_get=0x0001fab1
--define_symbol sm_private_random_address_generation_get_mode=0x0001fab9
--define_symbol sm_private_random_address_generation_set_mode=0x0001fac5
--define_symbol sm_private_random_address_generation_set_update_period=0x0001faed
--define_symbol sm_register_oob_data_callback=0x0001fc29
--define_symbol sm_request_pairing=0x0001fc35
--define_symbol sm_send_security_request=0x0002062f
--define_symbol sm_set_accepted_stk_generation_methods=0x00020655
--define_symbol sm_set_authentication_requirements=0x00020661
--define_symbol sm_set_encryption_key_size_range=0x0002066d
--define_symbol sscanf_bd_addr=0x000209c9
--define_symbol sysSetPublicDeviceAddr=0x00020d31
--define_symbol uuid128_to_str=0x00021325
--define_symbol uuid_add_bluetooth_prefix=0x0002137d
--define_symbol uuid_has_bluetooth_prefix=0x0002139d
--define_symbol uxListRemove=0x000213b9
--define_symbol uxQueueMessagesWaiting=0x000213e1
--define_symbol uxQueueMessagesWaitingFromISR=0x00021409
--define_symbol uxQueueSpacesAvailable=0x00021425
--define_symbol uxTaskGetStackHighWaterMark=0x00021451
--define_symbol vListInitialise=0x000214f7
--define_symbol vListInitialiseItem=0x0002150d
--define_symbol vListInsert=0x00021513
--define_symbol vListInsertEnd=0x00021543
--define_symbol vPortEnterCritical=0x0002155d
--define_symbol vPortExitCritical=0x000215a1
--define_symbol vPortFree=0x000215d1
--define_symbol vPortSuppressTicksAndSleep=0x00021665
--define_symbol vPortValidateInterruptPriority=0x00021785
--define_symbol vQueueDelete=0x000217dd
--define_symbol vQueueWaitForMessageRestricted=0x00021809
--define_symbol vTaskDelay=0x00021851
--define_symbol vTaskInternalSetTimeOutState=0x0002189d
--define_symbol vTaskMissedYield=0x000218ad
--define_symbol vTaskPlaceOnEventList=0x000218b9
--define_symbol vTaskPlaceOnEventListRestricted=0x000218f1
--define_symbol vTaskPriorityDisinheritAfterTimeout=0x00021931
--define_symbol vTaskResume=0x000219dd
--define_symbol vTaskStartScheduler=0x00021a61
--define_symbol vTaskStepTick=0x00021af1
--define_symbol vTaskSuspend=0x00021b21
--define_symbol vTaskSuspendAll=0x00021bdd
--define_symbol vTaskSwitchContext=0x00021bed
--define_symbol xPortStartScheduler=0x00021c95
--define_symbol xQueueAddToSet=0x00021d59
--define_symbol xQueueCreateCountingSemaphore=0x00021d7d
--define_symbol xQueueCreateCountingSemaphoreStatic=0x00021db9
--define_symbol xQueueCreateMutex=0x00021dfd
--define_symbol xQueueCreateMutexStatic=0x00021e13
--define_symbol xQueueCreateSet=0x00021e2d
--define_symbol xQueueGenericCreate=0x00021e35
--define_symbol xQueueGenericCreateStatic=0x00021e81
--define_symbol xQueueGenericReset=0x00021ee9
--define_symbol xQueueGenericSend=0x00021f75
--define_symbol xQueueGenericSendFromISR=0x000220e1
--define_symbol xQueueGiveFromISR=0x000221a1
--define_symbol xQueueGiveMutexRecursive=0x00022245
--define_symbol xQueueIsQueueEmptyFromISR=0x00022285
--define_symbol xQueueIsQueueFullFromISR=0x000222a9
--define_symbol xQueuePeek=0x000222d1
--define_symbol xQueuePeekFromISR=0x000223f9
--define_symbol xQueueReceive=0x00022465
--define_symbol xQueueReceiveFromISR=0x00022591
--define_symbol xQueueRemoveFromSet=0x00022625
--define_symbol xQueueSelectFromSet=0x00022647
--define_symbol xQueueSelectFromSetFromISR=0x00022659
--define_symbol xQueueSemaphoreTake=0x0002266d
--define_symbol xQueueTakeMutexRecursive=0x000227d9
--define_symbol xTaskCheckForTimeOut=0x0002281d
--define_symbol xTaskCreate=0x0002288d
--define_symbol xTaskCreateStatic=0x000228e9
--define_symbol xTaskGetCurrentTaskHandle=0x00022959
--define_symbol xTaskGetSchedulerState=0x00022965
--define_symbol xTaskGetTickCount=0x00022981
--define_symbol xTaskGetTickCountFromISR=0x0002298d
--define_symbol xTaskIncrementTick=0x0002299d
--define_symbol xTaskPriorityDisinherit=0x00022a69
--define_symbol xTaskPriorityInherit=0x00022afd
--define_symbol xTaskRemoveFromEventList=0x00022b91
--define_symbol xTaskResumeAll=0x00022c11
--define_symbol xTaskResumeFromISR=0x00022cd9
--define_symbol xTimerCreate=0x00022d65
--define_symbol xTimerCreateStatic=0x00022d99
--define_symbol xTimerCreateTimerTask=0x00022dd1
--define_symbol xTimerGenericCommand=0x00022e3d
--define_symbol xTimerGetExpiryTime=0x00022ead
