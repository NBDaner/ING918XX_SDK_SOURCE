--define_symbol att_dispatch_client_can_send_now=0x000059e1
--define_symbol att_dispatch_client_request_can_send_now_event=0x000059e7
--define_symbol att_dispatch_register_client=0x000059ed
--define_symbol att_dispatch_register_server=0x00005a01
--define_symbol att_dispatch_server_can_send_now=0x00005a15
--define_symbol att_dispatch_server_request_can_send_now_event=0x00005a1b
--define_symbol att_emit_general_event=0x00005acd
--define_symbol att_server_can_send_packet_now=0x000061e1
--define_symbol att_server_deferred_read_response=0x000061e5
--define_symbol att_server_get_mtu=0x000061fd
--define_symbol att_server_indicate=0x00006275
--define_symbol att_server_init=0x000062f9
--define_symbol att_server_notify=0x00006335
--define_symbol att_server_register_packet_handler=0x0000644d
--define_symbol att_server_request_can_send_now_event=0x00006459
--define_symbol att_set_db=0x00006475
--define_symbol att_set_read_callback=0x00006489
--define_symbol att_set_write_callback=0x00006495
--define_symbol bd_addr_cmp=0x00006605
--define_symbol bd_addr_copy=0x0000660b
--define_symbol bd_addr_to_str=0x00006615
--define_symbol big_endian_read_16=0x0000664d
--define_symbol big_endian_read_32=0x00006655
--define_symbol big_endian_store_16=0x00006669
--define_symbol big_endian_store_32=0x00006675
--define_symbol btstack_config=0x000067c9
--define_symbol btstack_memory_pool_create=0x00006907
--define_symbol btstack_memory_pool_free=0x00006931
--define_symbol btstack_memory_pool_get=0x00006991
--define_symbol btstack_push_user_msg=0x000069f9
--define_symbol char_for_nibble=0x00006cc1
--define_symbol eTaskConfirmSleepModeStatus=0x00006f69
--define_symbol gap_add_dev_to_periodic_list=0x00007589
--define_symbol gap_add_whitelist=0x000075a1
--define_symbol gap_aes_encrypt=0x000075b5
--define_symbol gap_clear_white_lists=0x000075f9
--define_symbol gap_clr_adv_set=0x00007609
--define_symbol gap_clr_periodic_adv_list=0x00007619
--define_symbol gap_create_connection_cancel=0x00007629
--define_symbol gap_disconnect=0x00007639
--define_symbol gap_disconnect_all=0x00007665
--define_symbol gap_ext_create_connection=0x000076a5
--define_symbol gap_get_connection_parameter_range=0x00007795
--define_symbol gap_le_read_channel_map=0x000077d1
--define_symbol gap_periodic_adv_create_sync=0x00007845
--define_symbol gap_periodic_adv_create_sync_cancel=0x00007869
--define_symbol gap_periodic_adv_term_sync=0x00007879
--define_symbol gap_read_periodic_adv_list_size=0x0000790d
--define_symbol gap_read_phy=0x0000791d
--define_symbol gap_read_remote_info=0x00007931
--define_symbol gap_read_remote_used_features=0x00007945
--define_symbol gap_read_rssi=0x00007959
--define_symbol gap_remove_whitelist=0x0000796d
--define_symbol gap_rmv_adv_set=0x000079f1
--define_symbol gap_rmv_dev_from_periodic_list=0x00007a05
--define_symbol gap_rx_test_v2=0x00007a1d
--define_symbol gap_set_adv_set_random_addr=0x00007a5d
--define_symbol gap_set_connection_parameter_range=0x00007aa9
--define_symbol gap_set_data_length=0x00007ac1
--define_symbol gap_set_def_phy=0x00007add
--define_symbol gap_set_ext_adv_data=0x00007af5
--define_symbol gap_set_ext_adv_enable=0x00007b0d
--define_symbol gap_set_ext_adv_para=0x00007b89
--define_symbol gap_set_ext_scan_enable=0x00007c69
--define_symbol gap_set_ext_scan_para=0x00007c81
--define_symbol gap_set_ext_scan_response_data=0x00007d29
--define_symbol gap_set_host_channel_classification=0x00007d41
--define_symbol gap_set_periodic_adv_data=0x00007d55
--define_symbol gap_set_periodic_adv_enable=0x00007dc9
--define_symbol gap_set_periodic_adv_para=0x00007ddd
--define_symbol gap_set_phy=0x00007df5
--define_symbol gap_set_random_device_address=0x00007e11
--define_symbol gap_start_ccm=0x00007e41
--define_symbol gap_test_end=0x00007e75
--define_symbol gap_tx_test_v2=0x00007e85
--define_symbol gap_tx_test_v4=0x00007e9d
--define_symbol gap_update_connection_parameters=0x00007ec5
--define_symbol gap_vendor_tx_continuous_wave=0x00007ee9
--define_symbol gatt_client_cancel_write=0x00008411
--define_symbol gatt_client_discover_characteristic_descriptors=0x00008437
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x00008477
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x000084c7
--define_symbol gatt_client_discover_characteristics_for_service=0x00008517
--define_symbol gatt_client_discover_primary_services=0x0000854d
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x0000857f
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x000085c3
--define_symbol gatt_client_execute_write=0x000085ff
--define_symbol gatt_client_find_included_services_for_service=0x00008625
--define_symbol gatt_client_get_mtu=0x00008653
--define_symbol gatt_client_is_ready=0x000086f5
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x0000870b
--define_symbol gatt_client_prepare_write=0x0000872d
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x00008769
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x00008793
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008799
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x000087c7
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x000087cd
--define_symbol gatt_client_read_multiple_characteristic_values=0x000087fb
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x0000882b
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x00008859
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x000088a5
--define_symbol gatt_client_register_handler=0x000088f1
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x000088fd
--define_symbol gatt_client_signed_write_without_response=0x00008d2d
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x00008df1
--define_symbol gatt_client_write_client_characteristic_configuration=0x00008e2b
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x00008e7d
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008e8d
--define_symbol gatt_client_write_long_value_of_characteristic=0x00008ec9
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x00008ed9
--define_symbol gatt_client_write_value_of_characteristic=0x00008f15
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x00008f4b
--define_symbol hci_add_event_handler=0x0000a471
--define_symbol hci_power_control=0x0000ac65
--define_symbol hci_register_acl_packet_handler=0x0000ae19
--define_symbol kv_commit=0x0000b38d
--define_symbol kv_get=0x0000b3e5
--define_symbol kv_init=0x0000b3fd
--define_symbol kv_put=0x0000b465
--define_symbol kv_remove=0x0000b4dd
--define_symbol kv_remove_all=0x0000b519
--define_symbol kv_value_modified=0x0000b55d
--define_symbol kv_visit=0x0000b561
--define_symbol l2cap_can_send_fixed_channel_packet_now=0x0000b621
--define_symbol l2cap_can_send_packet_now=0x0000b625
--define_symbol l2cap_create_channel=0x0000b7dd
--define_symbol l2cap_disconnect=0x0000b915
--define_symbol l2cap_get_remote_mtu_for_local_cid=0x0000bb3d
--define_symbol l2cap_init=0x0000bf35
--define_symbol l2cap_le_send_flow_control_credit=0x0000bf75
--define_symbol l2cap_max_le_mtu=0x0000c231
--define_symbol l2cap_max_mtu=0x0000c235
--define_symbol l2cap_register_fixed_channel=0x0000c341
--define_symbol l2cap_register_packet_handler=0x0000c35d
--define_symbol l2cap_register_service=0x0000c369
--define_symbol l2cap_request_can_send_fix_channel_now_event=0x0000c44d
--define_symbol l2cap_request_can_send_now_event=0x0000c471
--define_symbol l2cap_request_connection_parameter_update=0x0000c48b
--define_symbol l2cap_send=0x0000c825
--define_symbol l2cap_send_connectionless=0x0000c89d
--define_symbol l2cap_send_connectionless3=0x0000c92d
--define_symbol l2cap_send_echo_request=0x0000c9c5
--define_symbol l2cap_unregister_service=0x0000ca81
--define_symbol le_device_db_add=0x0000cad9
--define_symbol le_device_db_find=0x0000cbad
--define_symbol le_device_db_from_key=0x0000cbd9
--define_symbol le_device_db_iter_cur=0x0000cbe1
--define_symbol le_device_db_iter_cur_key=0x0000cbe5
--define_symbol le_device_db_iter_init=0x0000cbe9
--define_symbol le_device_db_iter_next=0x0000cbf1
--define_symbol le_device_db_remove_key=0x0000cc17
--define_symbol ll_free=0x0000cc43
--define_symbol ll_hint_on_ce_len=0x0000cc4d
--define_symbol ll_legacy_adv_set_interval=0x0000cc85
--define_symbol ll_malloc=0x0000cc95
--define_symbol ll_query_timing_info=0x0000cdcd
--define_symbol ll_scan_set_fixed_channel=0x0000ce71
--define_symbol ll_set_adv_access_address=0x0000cf85
--define_symbol ll_set_adv_coded_scheme=0x0000cf91
--define_symbol ll_set_conn_coded_scheme=0x0000cfc1
--define_symbol ll_set_conn_latency=0x0000cfed
--define_symbol ll_set_conn_tx_power=0x0000d01d
--define_symbol ll_set_def_antenna=0x0000d065
--define_symbol ll_set_initiating_coded_scheme=0x0000d081
--define_symbol ll_set_max_conn_number=0x0000d08d
--define_symbol nibble_for_char=0x0001cead
--define_symbol platform_32k_rc_auto_tune=0x0001cf49
--define_symbol platform_32k_rc_tune=0x0001cfc5
--define_symbol platform_calibrate_32k=0x0001cfd9
--define_symbol platform_config=0x0001cfdd
--define_symbol platform_get_heap_status=0x0001d0b1
--define_symbol platform_get_task_handle=0x0001d0c9
--define_symbol platform_get_us_time=0x0001d0e9
--define_symbol platform_get_version=0x0001d0ed
--define_symbol platform_hrng=0x0001d0f5
--define_symbol platform_install_isr_stack=0x0001d0fd
--define_symbol platform_patch_rf_init_data=0x0001d109
--define_symbol platform_printf=0x0001d115
--define_symbol platform_raise_assertion=0x0001d129
--define_symbol platform_rand=0x0001d13d
--define_symbol platform_read_info=0x0001d141
--define_symbol platform_read_persistent_reg=0x0001d15d
--define_symbol platform_reset=0x0001d16d
--define_symbol platform_set_evt_callback=0x0001d1a1
--define_symbol platform_set_irq_callback=0x0001d1b5
--define_symbol platform_set_rf_clk_source=0x0001d1ed
--define_symbol platform_set_rf_init_data=0x0001d1f9
--define_symbol platform_set_rf_power_mapping=0x0001d205
--define_symbol platform_set_timer=0x0001d211
--define_symbol platform_shutdown=0x0001d215
--define_symbol platform_switch_app=0x0001d219
--define_symbol platform_trace_raw=0x0001d245
--define_symbol platform_write_persistent_reg=0x0001d25d
--define_symbol printf_hexdump=0x0001d26d
--define_symbol pvPortMalloc=0x0001dd79
--define_symbol pvTaskIncrementMutexHeldCount=0x0001de61
--define_symbol pvTimerGetTimerID=0x0001de79
--define_symbol pxPortInitialiseStack=0x0001dea5
--define_symbol reverse_128=0x0001e04d
--define_symbol reverse_24=0x0001e053
--define_symbol reverse_48=0x0001e059
--define_symbol reverse_56=0x0001e05f
--define_symbol reverse_64=0x0001e065
--define_symbol reverse_bd_addr=0x0001e06b
--define_symbol reverse_bytes=0x0001e071
--define_symbol sm_add_event_handler=0x0001e1dd
--define_symbol sm_address_resolution_lookup=0x0001e335
--define_symbol sm_authenticated=0x0001e68d
--define_symbol sm_authorization_decline=0x0001e69b
--define_symbol sm_authorization_grant=0x0001e6bb
--define_symbol sm_authorization_state=0x0001e6db
--define_symbol sm_bonding_decline=0x0001e6f5
--define_symbol sm_config=0x0001eb15
--define_symbol sm_config_conn=0x0001eb2d
--define_symbol sm_encryption_key_size=0x0001ece3
--define_symbol sm_just_works_confirm=0x0001f21d
--define_symbol sm_le_device_key=0x0001f559
--define_symbol sm_passkey_input=0x0001f5ef
--define_symbol sm_private_random_address_generation_get=0x0001f99d
--define_symbol sm_private_random_address_generation_get_mode=0x0001f9a5
--define_symbol sm_private_random_address_generation_set_mode=0x0001f9b1
--define_symbol sm_private_random_address_generation_set_update_period=0x0001f9d9
--define_symbol sm_register_oob_data_callback=0x0001fb15
--define_symbol sm_request_pairing=0x0001fb21
--define_symbol sm_send_security_request=0x0002053f
--define_symbol sm_set_accepted_stk_generation_methods=0x00020565
--define_symbol sm_set_authentication_requirements=0x00020571
--define_symbol sm_set_encryption_key_size_range=0x0002057d
--define_symbol sscanf_bd_addr=0x000208d9
--define_symbol sysSetPublicDeviceAddr=0x00020c41
--define_symbol uuid128_to_str=0x00021225
--define_symbol uuid_add_bluetooth_prefix=0x0002127d
--define_symbol uuid_has_bluetooth_prefix=0x0002129d
--define_symbol uxListRemove=0x000212b9
--define_symbol uxQueueMessagesWaiting=0x000212e1
--define_symbol uxQueueMessagesWaitingFromISR=0x00021309
--define_symbol uxQueueSpacesAvailable=0x00021325
--define_symbol uxTaskGetStackHighWaterMark=0x00021351
--define_symbol uxTaskPriorityGet=0x00021371
--define_symbol uxTaskPriorityGetFromISR=0x0002138d
--define_symbol vListInitialise=0x0002143f
--define_symbol vListInitialiseItem=0x00021455
--define_symbol vListInsert=0x0002145b
--define_symbol vListInsertEnd=0x0002148b
--define_symbol vPortEndScheduler=0x000214a5
--define_symbol vPortEnterCritical=0x000214cd
--define_symbol vPortExitCritical=0x00021511
--define_symbol vPortFree=0x00021541
--define_symbol vPortSuppressTicksAndSleep=0x000215d5
--define_symbol vPortValidateInterruptPriority=0x000216dd
--define_symbol vQueueDelete=0x00021735
--define_symbol vQueueWaitForMessageRestricted=0x00021761
--define_symbol vTaskDelay=0x000217a9
--define_symbol vTaskInternalSetTimeOutState=0x000217f5
--define_symbol vTaskMissedYield=0x00021805
--define_symbol vTaskPlaceOnEventList=0x00021811
--define_symbol vTaskPlaceOnEventListRestricted=0x00021849
--define_symbol vTaskPriorityDisinheritAfterTimeout=0x00021889
--define_symbol vTaskPrioritySet=0x00021935
--define_symbol vTaskResume=0x000219fd
--define_symbol vTaskStartScheduler=0x00021a81
--define_symbol vTaskStepTick=0x00021b11
--define_symbol vTaskSuspend=0x00021b41
--define_symbol vTaskSuspendAll=0x00021bfd
--define_symbol vTaskSwitchContext=0x00021c0d
--define_symbol xPortStartScheduler=0x00021cb5
--define_symbol xQueueAddToSet=0x00021d75
--define_symbol xQueueCreateCountingSemaphore=0x00021d99
--define_symbol xQueueCreateCountingSemaphoreStatic=0x00021dd5
--define_symbol xQueueCreateMutex=0x00021e19
--define_symbol xQueueCreateMutexStatic=0x00021e2f
--define_symbol xQueueCreateSet=0x00021e49
--define_symbol xQueueGenericCreate=0x00021e51
--define_symbol xQueueGenericCreateStatic=0x00021e9d
--define_symbol xQueueGenericReset=0x00021f05
--define_symbol xQueueGenericSend=0x00021f91
--define_symbol xQueueGenericSendFromISR=0x000220fd
--define_symbol xQueueGiveFromISR=0x000221bd
--define_symbol xQueueGiveMutexRecursive=0x00022261
--define_symbol xQueueIsQueueEmptyFromISR=0x000222a1
--define_symbol xQueueIsQueueFullFromISR=0x000222c5
--define_symbol xQueuePeek=0x000222ed
--define_symbol xQueuePeekFromISR=0x00022415
--define_symbol xQueueReceive=0x00022481
--define_symbol xQueueReceiveFromISR=0x000225ad
--define_symbol xQueueRemoveFromSet=0x00022641
--define_symbol xQueueSelectFromSet=0x00022663
--define_symbol xQueueSelectFromSetFromISR=0x00022675
--define_symbol xQueueSemaphoreTake=0x00022689
--define_symbol xQueueTakeMutexRecursive=0x000227f5
--define_symbol xTaskCheckForTimeOut=0x00022839
--define_symbol xTaskCreate=0x000228a9
--define_symbol xTaskCreateStatic=0x00022905
--define_symbol xTaskGetCurrentTaskHandle=0x00022975
--define_symbol xTaskGetSchedulerState=0x00022981
--define_symbol xTaskGetTickCount=0x0002299d
--define_symbol xTaskGetTickCountFromISR=0x000229a9
--define_symbol xTaskIncrementTick=0x000229b9
--define_symbol xTaskPriorityDisinherit=0x00022a85
--define_symbol xTaskPriorityInherit=0x00022b19
--define_symbol xTaskRemoveFromEventList=0x00022bad
--define_symbol xTaskResumeAll=0x00022c2d
--define_symbol xTaskResumeFromISR=0x00022cf5
--define_symbol xTimerCreate=0x00022d81
--define_symbol xTimerCreateStatic=0x00022db5
--define_symbol xTimerCreateTimerTask=0x00022ded
--define_symbol xTimerGenericCommand=0x00022e59
--define_symbol xTimerGetExpiryTime=0x00022ec9
--define_symbol xTimerGetTimerDaemonTaskHandle=0x00022ee9
