--define_symbol att_dispatch_client_can_send_now=0x000059c1
--define_symbol att_dispatch_client_request_can_send_now_event=0x000059c7
--define_symbol att_dispatch_register_client=0x000059cd
--define_symbol att_dispatch_register_server=0x000059e1
--define_symbol att_dispatch_server_can_send_now=0x000059f5
--define_symbol att_dispatch_server_request_can_send_now_event=0x000059fb
--define_symbol att_emit_general_event=0x00005aad
--define_symbol att_server_can_send_packet_now=0x000061cd
--define_symbol att_server_get_mtu=0x000061d1
--define_symbol att_server_indicate=0x00006245
--define_symbol att_server_init=0x000062c9
--define_symbol att_server_notify=0x00006305
--define_symbol att_server_register_packet_handler=0x000063ed
--define_symbol att_server_request_can_send_now_event=0x000063f9
--define_symbol att_set_db=0x00006415
--define_symbol att_set_read_callback=0x00006429
--define_symbol att_set_write_callback=0x00006435
--define_symbol bd_addr_cmp=0x00006579
--define_symbol bd_addr_copy=0x0000657f
--define_symbol bd_addr_to_str=0x00006589
--define_symbol big_endian_read_16=0x000065c1
--define_symbol big_endian_read_32=0x000065c9
--define_symbol big_endian_store_16=0x000065dd
--define_symbol big_endian_store_32=0x000065e9
--define_symbol btstack_memory_pool_create=0x00006847
--define_symbol btstack_memory_pool_free=0x00006871
--define_symbol btstack_memory_pool_get=0x000068d1
--define_symbol btstack_push_user_msg=0x000068ed
--define_symbol char_for_nibble=0x00006b65
--define_symbol eTaskConfirmSleepModeStatus=0x00006df5
--define_symbol gap_add_dev_to_periodic_list=0x00007475
--define_symbol gap_add_whitelist=0x0000748d
--define_symbol gap_aes_encrypt=0x000074a1
--define_symbol gap_clear_white_lists=0x000074c9
--define_symbol gap_clr_adv_set=0x000074d9
--define_symbol gap_clr_periodic_adv_list=0x000074e9
--define_symbol gap_create_connection_cancel=0x00007515
--define_symbol gap_disconnect=0x00007525
--define_symbol gap_disconnect_all=0x00007551
--define_symbol gap_ext_create_connection=0x000075f9
--define_symbol gap_get_connection_parameter_range=0x000076bd
--define_symbol gap_le_read_channel_map=0x000076f5
--define_symbol gap_periodic_adv_create_sync=0x00007769
--define_symbol gap_periodic_adv_create_sync_cancel=0x0000778d
--define_symbol gap_periodic_adv_term_sync=0x0000779d
--define_symbol gap_read_periodic_adv_list_size=0x00007831
--define_symbol gap_read_phy=0x00007841
--define_symbol gap_read_remote_info=0x00007855
--define_symbol gap_read_remote_used_features=0x00007869
--define_symbol gap_read_rssi=0x0000787d
--define_symbol gap_remove_whitelist=0x00007891
--define_symbol gap_rmv_adv_set=0x0000791d
--define_symbol gap_rmv_dev_from_periodic_list=0x00007931
--define_symbol gap_set_adv_set_random_addr=0x00007971
--define_symbol gap_set_connection_parameter_range=0x000079c1
--define_symbol gap_set_def_phy=0x000079dd
--define_symbol gap_set_ext_adv_data=0x000079f5
--define_symbol gap_set_ext_adv_enable=0x00007a0d
--define_symbol gap_set_ext_adv_para=0x00007a89
--define_symbol gap_set_ext_scan_enable=0x00007b69
--define_symbol gap_set_ext_scan_para=0x00007b81
--define_symbol gap_set_ext_scan_response_data=0x00007c29
--define_symbol gap_set_host_channel_classification=0x00007c41
--define_symbol gap_set_periodic_adv_data=0x00007c55
--define_symbol gap_set_periodic_adv_enable=0x00007cc9
--define_symbol gap_set_periodic_adv_para=0x00007cdd
--define_symbol gap_set_phy=0x00007cf5
--define_symbol gap_set_random_device_address=0x00007d11
--define_symbol gap_start_ccm=0x00007d75
--define_symbol gap_update_connection_parameters=0x00007da9
--define_symbol gatt_client_cancel_write=0x000082c1
--define_symbol gatt_client_discover_characteristic_descriptors=0x000082e7
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x00008327
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x00008377
--define_symbol gatt_client_discover_characteristics_for_service=0x000083c7
--define_symbol gatt_client_discover_primary_services=0x000083fd
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x0000842f
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x00008473
--define_symbol gatt_client_execute_write=0x000084af
--define_symbol gatt_client_find_included_services_for_service=0x000084d5
--define_symbol gatt_client_get_mtu=0x00008503
--define_symbol gatt_client_is_ready=0x000085ad
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x000085c3
--define_symbol gatt_client_prepare_write=0x000085e7
--define_symbol gatt_client_pts_suppress_mtu_exchange=0x00008625
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x00008631
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x0000865b
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008661
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x0000868f
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x00008695
--define_symbol gatt_client_read_multiple_characteristic_values=0x000086c3
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x000086f3
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x00008721
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x0000876d
--define_symbol gatt_client_register_handler=0x000087b9
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x000087c5
--define_symbol gatt_client_signed_write_without_response=0x00008bf5
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x00008cb9
--define_symbol gatt_client_write_client_characteristic_configuration=0x00008cf3
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x00008d45
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008d55
--define_symbol gatt_client_write_long_value_of_characteristic=0x00008d91
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x00008da1
--define_symbol gatt_client_write_value_of_characteristic=0x00008ddd
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x00008e13
--define_symbol hci_add_event_handler=0x0000a2c5
--define_symbol hci_power_control=0x0000aa79
--define_symbol hci_register_acl_packet_handler=0x0000ac2d
--define_symbol kv_commit=0x0000b2b9
--define_symbol kv_get=0x0000b311
--define_symbol kv_init=0x0000b329
--define_symbol kv_put=0x0000b391
--define_symbol kv_remove=0x0000b409
--define_symbol kv_remove_all=0x0000b445
--define_symbol kv_value_modified=0x0000b489
--define_symbol kv_visit=0x0000b48d
--define_symbol l2cap_can_send_fixed_channel_packet_now=0x0000b54d
--define_symbol l2cap_can_send_packet_now=0x0000b551
--define_symbol l2cap_create_channel=0x0000b709
--define_symbol l2cap_disconnect=0x0000b841
--define_symbol l2cap_get_remote_mtu_for_local_cid=0x0000bae5
--define_symbol l2cap_init=0x0000bf65
--define_symbol l2cap_le_send_flow_control_credit=0x0000bfad
--define_symbol l2cap_max_le_mtu=0x0000c211
--define_symbol l2cap_max_mtu=0x0000c215
--define_symbol l2cap_next_local_cid=0x0000c219
--define_symbol l2cap_next_sig_id=0x0000c229
--define_symbol l2cap_register_fixed_channel=0x0000c2c1
--define_symbol l2cap_register_packet_handler=0x0000c2dd
--define_symbol l2cap_register_service=0x0000c2e9
--define_symbol l2cap_request_can_send_fix_channel_now_event=0x0000c3d1
--define_symbol l2cap_request_can_send_now_event=0x0000c3f5
--define_symbol l2cap_request_connection_parameter_update=0x0000c40f
--define_symbol l2cap_require_security_level_2_for_outgoing_sdp=0x0000c441
--define_symbol l2cap_send=0x0000c805
--define_symbol l2cap_send_connectionless=0x0000c87d
--define_symbol l2cap_send_connectionless3=0x0000c90d
--define_symbol l2cap_send_echo_request=0x0000c9a5
--define_symbol l2cap_send_signaling_le=0x0000ca09
--define_symbol l2cap_unregister_service=0x0000ca61
--define_symbol ll_free=0x0000cc2f
--define_symbol ll_hint_on_ce_len=0x0000cc39
--define_symbol ll_malloc=0x0000cc75
--define_symbol ll_query_timing_info=0x0000cdad
--define_symbol ll_set_adv_coded_scheme=0x0000cf59
--define_symbol ll_set_conn_coded_scheme=0x0000cf89
--define_symbol ll_set_conn_latency=0x0000cfb5
--define_symbol ll_set_conn_tx_power=0x0000cfe5
--define_symbol ll_set_def_antenna=0x0000d02d
--define_symbol ll_set_initiating_coded_scheme=0x0000d049
--define_symbol nibble_for_char=0x0001cbc1
--define_symbol platform_32k_rc_auto_tune=0x0001cc79
--define_symbol platform_32k_rc_tune=0x0001ccf5
--define_symbol platform_calibrate_32k=0x0001cd09
--define_symbol platform_config=0x0001cd0d
--define_symbol platform_get_heap_status=0x0001cdbd
--define_symbol platform_get_us_time=0x0001cdd5
--define_symbol platform_get_version=0x0001cdd9
--define_symbol platform_hrng=0x0001cde1
--define_symbol platform_install_isr_stack=0x0001cde9
--define_symbol platform_patch_rf_init_data=0x0001cdf5
--define_symbol platform_printf=0x0001ce01
--define_symbol platform_raise_assertion=0x0001ce15
--define_symbol platform_rand=0x0001ce29
--define_symbol platform_read_info=0x0001ce2d
--define_symbol platform_read_persistent_reg=0x0001ce49
--define_symbol platform_reset=0x0001ce59
--define_symbol platform_set_evt_callback=0x0001ce8d
--define_symbol platform_set_irq_callback=0x0001cea1
--define_symbol platform_set_rf_clk_source=0x0001ced9
--define_symbol platform_set_rf_init_data=0x0001cee5
--define_symbol platform_set_rf_power_mapping=0x0001cef1
--define_symbol platform_shutdown=0x0001cefd
--define_symbol platform_switch_app=0x0001cf01
--define_symbol platform_trace_raw=0x0001cf2d
--define_symbol platform_write_persistent_reg=0x0001cf41
--define_symbol printf_hexdump=0x0001cf51
--define_symbol pvPortMalloc=0x0001da85
--define_symbol pvTaskIncrementMutexHeldCount=0x0001db6d
--define_symbol pvTimerGetTimerID=0x0001db85
--define_symbol pxPortInitialiseStack=0x0001dbb1
--define_symbol reverse_128=0x0001dd59
--define_symbol reverse_24=0x0001dd5f
--define_symbol reverse_48=0x0001dd65
--define_symbol reverse_56=0x0001dd6b
--define_symbol reverse_64=0x0001dd71
--define_symbol reverse_bd_addr=0x0001dd77
--define_symbol reverse_bytes=0x0001dd7d
--define_symbol sm_add_event_handler=0x0001dec9
--define_symbol sm_address_resolution_lookup=0x0001dff5
--define_symbol sm_authenticated=0x0001e0cf
--define_symbol sm_authorization_decline=0x0001e0dd
--define_symbol sm_authorization_grant=0x0001e0fd
--define_symbol sm_authorization_state=0x0001e11d
--define_symbol sm_bonding_decline=0x0001e139
--define_symbol sm_config=0x0001e511
--define_symbol sm_encryption_key_size=0x0001e61d
--define_symbol sm_just_works_confirm=0x0001eded
--define_symbol sm_le_device_key=0x0001f03d
--define_symbol sm_passkey_input=0x0001f0d5
--define_symbol sm_private_random_address_generation_get=0x0001f461
--define_symbol sm_private_random_address_generation_get_mode=0x0001f469
--define_symbol sm_private_random_address_generation_set_mode=0x0001f475
--define_symbol sm_private_random_address_generation_set_update_period=0x0001f49d
--define_symbol sm_register_oob_data_callback=0x0001f4d5
--define_symbol sm_request_pairing=0x0001f4e1
--define_symbol sm_send_security_request=0x0001ff1b
--define_symbol sm_set_accepted_stk_generation_methods=0x0001ff41
--define_symbol sm_set_authentication_requirements=0x0001ff4d
--define_symbol sm_set_encryption_key_size_range=0x0001ff59
--define_symbol sscanf_bd_addr=0x00020255
--define_symbol sysSetPublicDeviceAddr=0x00020571
--define_symbol uuid128_to_str=0x00020b15
--define_symbol uuid_add_bluetooth_prefix=0x00020b6d
--define_symbol uuid_has_bluetooth_prefix=0x00020b8d
--define_symbol uxQueueMessagesWaiting=0x00020bd1
--define_symbol uxQueueMessagesWaitingFromISR=0x00020bf9
--define_symbol uxQueueSpacesAvailable=0x00020c15
--define_symbol uxTaskGetStackHighWaterMark=0x00020c41
--define_symbol vPortEnterCritical=0x00020cf9
--define_symbol vPortExitCritical=0x00020d39
--define_symbol vPortFree=0x00020d65
--define_symbol vPortSuppressTicksAndSleep=0x00020df9
--define_symbol vPortValidateInterruptPriority=0x00020ed1
--define_symbol vQueueDelete=0x00020f25
--define_symbol vQueueWaitForMessageRestricted=0x00020f51
--define_symbol vTaskDelay=0x00020f99
--define_symbol vTaskInternalSetTimeOutState=0x00020fe5
--define_symbol vTaskMissedYield=0x00020ff5
--define_symbol vTaskPlaceOnEventList=0x00021001
--define_symbol vTaskPlaceOnEventListRestricted=0x00021039
--define_symbol vTaskPriorityDisinheritAfterTimeout=0x00021079
--define_symbol vTaskStartScheduler=0x00021125
--define_symbol vTaskStepTick=0x000211b5
--define_symbol vTaskSuspendAll=0x000211e5
--define_symbol vTaskSwitchContext=0x000211f5
--define_symbol xPortStartScheduler=0x0002129d
--define_symbol xQueueAddToSet=0x00021359
--define_symbol xQueueCreateCountingSemaphore=0x0002137d
--define_symbol xQueueCreateCountingSemaphoreStatic=0x000213b9
--define_symbol xQueueCreateMutex=0x000213fd
--define_symbol xQueueCreateMutexStatic=0x00021413
--define_symbol xQueueCreateSet=0x0002142d
--define_symbol xQueueGenericCreate=0x00021435
--define_symbol xQueueGenericCreateStatic=0x00021481
--define_symbol xQueueGenericReset=0x000214e9
--define_symbol xQueueGenericSend=0x00021575
--define_symbol xQueueGenericSendFromISR=0x000216e1
--define_symbol xQueueGiveFromISR=0x000217a1
--define_symbol xQueueGiveMutexRecursive=0x00021845
--define_symbol xQueueIsQueueEmptyFromISR=0x00021885
--define_symbol xQueueIsQueueFullFromISR=0x000218a9
--define_symbol xQueuePeek=0x000218d1
--define_symbol xQueuePeekFromISR=0x000219f9
--define_symbol xQueueReceive=0x00021a65
--define_symbol xQueueReceiveFromISR=0x00021b91
--define_symbol xQueueRemoveFromSet=0x00021c25
--define_symbol xQueueSelectFromSet=0x00021c47
--define_symbol xQueueSelectFromSetFromISR=0x00021c59
--define_symbol xQueueSemaphoreTake=0x00021c6d
--define_symbol xQueueTakeMutexRecursive=0x00021dd9
--define_symbol xTaskCheckForTimeOut=0x00021e1d
--define_symbol xTaskCreate=0x00021e8d
--define_symbol xTaskCreateStatic=0x00021ee9
--define_symbol xTaskGetCurrentTaskHandle=0x00021f59
--define_symbol xTaskGetSchedulerState=0x00021f65
--define_symbol xTaskGetTickCount=0x00021f81
--define_symbol xTaskGetTickCountFromISR=0x00021f8d
--define_symbol xTaskIncrementTick=0x00021f9d
--define_symbol xTaskPriorityDisinherit=0x00022069
--define_symbol xTaskPriorityInherit=0x000220fd
--define_symbol xTaskRemoveFromEventList=0x00022191
--define_symbol xTaskResumeAll=0x00022211
--define_symbol xTimerCreate=0x000222d9
--define_symbol xTimerCreateStatic=0x0002230d
--define_symbol xTimerCreateTimerTask=0x00022345
--define_symbol xTimerGenericCommand=0x000223b1
--define_symbol xTimerGetExpiryTime=0x00022421
