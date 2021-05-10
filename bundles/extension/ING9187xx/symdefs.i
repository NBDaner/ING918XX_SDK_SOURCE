--define_symbol att_dispatch_client_can_send_now=0x000059d1
--define_symbol att_dispatch_client_request_can_send_now_event=0x000059d7
--define_symbol att_dispatch_register_client=0x000059dd
--define_symbol att_dispatch_register_server=0x000059f1
--define_symbol att_dispatch_server_can_send_now=0x00005a05
--define_symbol att_dispatch_server_request_can_send_now_event=0x00005a0b
--define_symbol att_emit_general_event=0x00005abd
--define_symbol att_server_can_send_packet_now=0x000061d1
--define_symbol att_server_get_mtu=0x000061d5
--define_symbol att_server_indicate=0x0000624d
--define_symbol att_server_init=0x000062d1
--define_symbol att_server_notify=0x0000630d
--define_symbol att_server_register_packet_handler=0x0000641d
--define_symbol att_server_request_can_send_now_event=0x00006429
--define_symbol att_set_db=0x00006445
--define_symbol att_set_read_callback=0x00006459
--define_symbol att_set_write_callback=0x00006465
--define_symbol bd_addr_cmp=0x000065d5
--define_symbol bd_addr_copy=0x000065db
--define_symbol bd_addr_to_str=0x000065e5
--define_symbol big_endian_read_16=0x0000661d
--define_symbol big_endian_read_32=0x00006625
--define_symbol big_endian_store_16=0x00006639
--define_symbol big_endian_store_32=0x00006645
--define_symbol btstack_config=0x00006781
--define_symbol btstack_memory_pool_create=0x000068bf
--define_symbol btstack_memory_pool_free=0x000068e9
--define_symbol btstack_memory_pool_get=0x00006949
--define_symbol btstack_push_user_msg=0x00006965
--define_symbol char_for_nibble=0x00006c2d
--define_symbol eTaskConfirmSleepModeStatus=0x00006ed9
--define_symbol gap_add_dev_to_periodic_list=0x00007589
--define_symbol gap_add_whitelist=0x000075a1
--define_symbol gap_aes_encrypt=0x000075b5
--define_symbol gap_clear_white_lists=0x000075dd
--define_symbol gap_clr_adv_set=0x000075ed
--define_symbol gap_clr_periodic_adv_list=0x000075fd
--define_symbol gap_create_connection_cancel=0x00007629
--define_symbol gap_disconnect=0x00007639
--define_symbol gap_disconnect_all=0x00007665
--define_symbol gap_ext_create_connection=0x0000770d
--define_symbol gap_get_connection_parameter_range=0x000077d1
--define_symbol gap_le_read_channel_map=0x0000780d
--define_symbol gap_periodic_adv_create_sync=0x00007881
--define_symbol gap_periodic_adv_create_sync_cancel=0x000078a5
--define_symbol gap_periodic_adv_term_sync=0x000078b5
--define_symbol gap_read_periodic_adv_list_size=0x00007949
--define_symbol gap_read_phy=0x00007959
--define_symbol gap_read_remote_info=0x0000796d
--define_symbol gap_read_remote_used_features=0x00007981
--define_symbol gap_read_rssi=0x00007995
--define_symbol gap_remove_whitelist=0x000079a9
--define_symbol gap_rmv_adv_set=0x00007a35
--define_symbol gap_rmv_dev_from_periodic_list=0x00007a49
--define_symbol gap_rx_test_v2=0x00007a61
--define_symbol gap_set_adv_set_random_addr=0x00007aa1
--define_symbol gap_set_connection_parameter_range=0x00007aed
--define_symbol gap_set_data_length=0x00007b05
--define_symbol gap_set_def_phy=0x00007b21
--define_symbol gap_set_ext_adv_data=0x00007b39
--define_symbol gap_set_ext_adv_enable=0x00007b51
--define_symbol gap_set_ext_adv_para=0x00007bcd
--define_symbol gap_set_ext_scan_enable=0x00007cad
--define_symbol gap_set_ext_scan_para=0x00007cc5
--define_symbol gap_set_ext_scan_response_data=0x00007d6d
--define_symbol gap_set_host_channel_classification=0x00007d85
--define_symbol gap_set_periodic_adv_data=0x00007d99
--define_symbol gap_set_periodic_adv_enable=0x00007e0d
--define_symbol gap_set_periodic_adv_para=0x00007e21
--define_symbol gap_set_phy=0x00007e39
--define_symbol gap_set_random_device_address=0x00007e55
--define_symbol gap_start_ccm=0x00007eb9
--define_symbol gap_test_end=0x00007eed
--define_symbol gap_tx_test_v2=0x00007efd
--define_symbol gap_update_connection_parameters=0x00007f15
--define_symbol gap_vendor_tx_continuous_wave=0x00007f39
--define_symbol gatt_client_cancel_write=0x00008461
--define_symbol gatt_client_discover_characteristic_descriptors=0x00008487
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x000084c7
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x00008517
--define_symbol gatt_client_discover_characteristics_for_service=0x00008567
--define_symbol gatt_client_discover_primary_services=0x0000859d
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x000085cf
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x00008613
--define_symbol gatt_client_execute_write=0x0000864f
--define_symbol gatt_client_find_included_services_for_service=0x00008675
--define_symbol gatt_client_get_mtu=0x000086a3
--define_symbol gatt_client_is_ready=0x00008745
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x0000875b
--define_symbol gatt_client_prepare_write=0x0000877d
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x000087b9
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x000087e3
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x000087e9
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x00008817
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x0000881d
--define_symbol gatt_client_read_multiple_characteristic_values=0x0000884b
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x0000887b
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x000088a9
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x000088f5
--define_symbol gatt_client_register_handler=0x00008941
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x0000894d
--define_symbol gatt_client_signed_write_without_response=0x00008d7d
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x00008e41
--define_symbol gatt_client_write_client_characteristic_configuration=0x00008e7b
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x00008ecd
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008edd
--define_symbol gatt_client_write_long_value_of_characteristic=0x00008f19
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x00008f29
--define_symbol gatt_client_write_value_of_characteristic=0x00008f65
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x00008f9b
--define_symbol hci_add_event_handler=0x0000a44d
--define_symbol hci_power_control=0x0000ac31
--define_symbol hci_register_acl_packet_handler=0x0000ade5
--define_symbol kv_commit=0x0000b3e9
--define_symbol kv_get=0x0000b441
--define_symbol kv_init=0x0000b459
--define_symbol kv_put=0x0000b4c1
--define_symbol kv_remove=0x0000b539
--define_symbol kv_remove_all=0x0000b575
--define_symbol kv_value_modified=0x0000b5b9
--define_symbol kv_visit=0x0000b5bd
--define_symbol l2cap_can_send_fixed_channel_packet_now=0x0000b67d
--define_symbol l2cap_can_send_packet_now=0x0000b681
--define_symbol l2cap_create_channel=0x0000b839
--define_symbol l2cap_disconnect=0x0000b971
--define_symbol l2cap_get_remote_mtu_for_local_cid=0x0000bbe5
--define_symbol l2cap_init=0x0000c065
--define_symbol l2cap_le_send_flow_control_credit=0x0000c0ad
--define_symbol l2cap_max_le_mtu=0x0000c331
--define_symbol l2cap_max_mtu=0x0000c335
--define_symbol l2cap_next_local_cid=0x0000c339
--define_symbol l2cap_next_sig_id=0x0000c349
--define_symbol l2cap_register_fixed_channel=0x0000c3e1
--define_symbol l2cap_register_packet_handler=0x0000c3fd
--define_symbol l2cap_register_service=0x0000c409
--define_symbol l2cap_request_can_send_fix_channel_now_event=0x0000c4f1
--define_symbol l2cap_request_can_send_now_event=0x0000c515
--define_symbol l2cap_request_connection_parameter_update=0x0000c52f
--define_symbol l2cap_require_security_level_2_for_outgoing_sdp=0x0000c561
--define_symbol l2cap_send=0x0000c8f5
--define_symbol l2cap_send_connectionless=0x0000c96d
--define_symbol l2cap_send_connectionless3=0x0000c9fd
--define_symbol l2cap_send_echo_request=0x0000ca95
--define_symbol l2cap_send_signaling_le=0x0000caf9
--define_symbol l2cap_unregister_service=0x0000cb51
--define_symbol ll_ackable_packet_alloc=0x0000cd1f
--define_symbol ll_ackable_packet_get_status=0x0000ce07
--define_symbol ll_ackable_packet_run=0x0000ce79
--define_symbol ll_ackable_packet_set_tx_data=0x0000cf21
--define_symbol ll_free=0x0000cf3b
--define_symbol ll_hint_on_ce_len=0x0000cf45
--define_symbol ll_legacy_adv_set_interval=0x0000cf85
--define_symbol ll_malloc=0x0000cf95
--define_symbol ll_query_timing_info=0x0000d0cd
--define_symbol ll_raw_packet_alloc=0x0000d119
--define_symbol ll_raw_packet_free=0x0000d1cd
--define_symbol ll_raw_packet_get_rx_data=0x0000d1f7
--define_symbol ll_raw_packet_recv=0x0000d28d
--define_symbol ll_raw_packet_send=0x0000d321
--define_symbol ll_raw_packet_set_param=0x0000d391
--define_symbol ll_raw_packet_set_tx_data=0x0000d3f7
--define_symbol ll_scan_set_fixed_channel=0x0000d4b9
--define_symbol ll_set_adv_coded_scheme=0x0000d5cd
--define_symbol ll_set_conn_coded_scheme=0x0000d5fd
--define_symbol ll_set_conn_latency=0x0000d629
--define_symbol ll_set_conn_tx_power=0x0000d659
--define_symbol ll_set_def_antenna=0x0000d6a1
--define_symbol ll_set_initiating_coded_scheme=0x0000d6bd
--define_symbol ll_set_max_conn_number=0x0000d6c9
--define_symbol nibble_for_char=0x0001d395
--define_symbol platform_32k_rc_auto_tune=0x0001d431
--define_symbol platform_32k_rc_tune=0x0001d4ad
--define_symbol platform_calibrate_32k=0x0001d4c1
--define_symbol platform_config=0x0001d4c5
--define_symbol platform_get_heap_status=0x0001d57d
--define_symbol platform_get_us_time=0x0001d595
--define_symbol platform_get_version=0x0001d599
--define_symbol platform_hrng=0x0001d5a1
--define_symbol platform_install_isr_stack=0x0001d5a9
--define_symbol platform_patch_rf_init_data=0x0001d5b5
--define_symbol platform_printf=0x0001d5c1
--define_symbol platform_raise_assertion=0x0001d5d5
--define_symbol platform_rand=0x0001d5e9
--define_symbol platform_read_info=0x0001d5ed
--define_symbol platform_read_persistent_reg=0x0001d609
--define_symbol platform_reset=0x0001d619
--define_symbol platform_set_evt_callback=0x0001d64d
--define_symbol platform_set_irq_callback=0x0001d661
--define_symbol platform_set_rf_clk_source=0x0001d699
--define_symbol platform_set_rf_init_data=0x0001d6a5
--define_symbol platform_set_rf_power_mapping=0x0001d6b1
--define_symbol platform_shutdown=0x0001d6bd
--define_symbol platform_switch_app=0x0001d6c1
--define_symbol platform_trace_raw=0x0001d6ed
--define_symbol platform_write_persistent_reg=0x0001d701
--define_symbol printf_hexdump=0x0001d711
--define_symbol pvPortMalloc=0x0001e1d9
--define_symbol pvTaskIncrementMutexHeldCount=0x0001e2c1
--define_symbol pvTimerGetTimerID=0x0001e2d9
--define_symbol pxPortInitialiseStack=0x0001e305
--define_symbol reverse_128=0x0001e4ad
--define_symbol reverse_24=0x0001e4b3
--define_symbol reverse_48=0x0001e4b9
--define_symbol reverse_56=0x0001e4bf
--define_symbol reverse_64=0x0001e4c5
--define_symbol reverse_bd_addr=0x0001e4cb
--define_symbol reverse_bytes=0x0001e4d1
--define_symbol sm_add_event_handler=0x0001e63d
--define_symbol sm_address_resolution_lookup=0x0001e769
--define_symbol sm_authenticated=0x0001e843
--define_symbol sm_authorization_decline=0x0001e851
--define_symbol sm_authorization_grant=0x0001e871
--define_symbol sm_authorization_state=0x0001e891
--define_symbol sm_bonding_decline=0x0001e8ad
--define_symbol sm_config=0x0001ec85
--define_symbol sm_encryption_key_size=0x0001ed91
--define_symbol sm_just_works_confirm=0x0001f549
--define_symbol sm_le_device_key=0x0001f7a1
--define_symbol sm_passkey_input=0x0001f839
--define_symbol sm_private_random_address_generation_get=0x0001fbc5
--define_symbol sm_private_random_address_generation_get_mode=0x0001fbcd
--define_symbol sm_private_random_address_generation_set_mode=0x0001fbd9
--define_symbol sm_private_random_address_generation_set_update_period=0x0001fc01
--define_symbol sm_register_oob_data_callback=0x0001fc39
--define_symbol sm_request_pairing=0x0001fc45
--define_symbol sm_send_security_request=0x00020695
--define_symbol sm_set_accepted_stk_generation_methods=0x000206bd
--define_symbol sm_set_authentication_requirements=0x000206c9
--define_symbol sm_set_encryption_key_size_range=0x000206d5
--define_symbol sscanf_bd_addr=0x000209c5
--define_symbol sysSetPublicDeviceAddr=0x00020d2d
--define_symbol uuid128_to_str=0x000212d1
--define_symbol uuid_add_bluetooth_prefix=0x00021329
--define_symbol uuid_has_bluetooth_prefix=0x00021349
--define_symbol uxQueueMessagesWaiting=0x0002138d
--define_symbol uxQueueMessagesWaitingFromISR=0x000213b5
--define_symbol uxQueueSpacesAvailable=0x000213d1
--define_symbol uxTaskGetStackHighWaterMark=0x000213fd
--define_symbol vPortEnterCritical=0x000214b5
--define_symbol vPortExitCritical=0x000214f5
--define_symbol vPortFree=0x00021521
--define_symbol vPortSuppressTicksAndSleep=0x000215b5
--define_symbol vPortValidateInterruptPriority=0x0002168d
--define_symbol vQueueDelete=0x000216e1
--define_symbol vQueueWaitForMessageRestricted=0x0002170d
--define_symbol vTaskDelay=0x00021755
--define_symbol vTaskInternalSetTimeOutState=0x000217a1
--define_symbol vTaskMissedYield=0x000217b1
--define_symbol vTaskPlaceOnEventList=0x000217bd
--define_symbol vTaskPlaceOnEventListRestricted=0x000217f5
--define_symbol vTaskPriorityDisinheritAfterTimeout=0x00021835
--define_symbol vTaskStartScheduler=0x000218e1
--define_symbol vTaskStepTick=0x00021971
--define_symbol vTaskSuspendAll=0x000219a1
--define_symbol vTaskSwitchContext=0x000219b1
--define_symbol xPortStartScheduler=0x00021a59
--define_symbol xQueueAddToSet=0x00021b15
--define_symbol xQueueCreateCountingSemaphore=0x00021b39
--define_symbol xQueueCreateCountingSemaphoreStatic=0x00021b75
--define_symbol xQueueCreateMutex=0x00021bb9
--define_symbol xQueueCreateMutexStatic=0x00021bcf
--define_symbol xQueueCreateSet=0x00021be9
--define_symbol xQueueGenericCreate=0x00021bf1
--define_symbol xQueueGenericCreateStatic=0x00021c3d
--define_symbol xQueueGenericReset=0x00021ca5
--define_symbol xQueueGenericSend=0x00021d31
--define_symbol xQueueGenericSendFromISR=0x00021e9d
--define_symbol xQueueGiveFromISR=0x00021f5d
--define_symbol xQueueGiveMutexRecursive=0x00022001
--define_symbol xQueueIsQueueEmptyFromISR=0x00022041
--define_symbol xQueueIsQueueFullFromISR=0x00022065
--define_symbol xQueuePeek=0x0002208d
--define_symbol xQueuePeekFromISR=0x000221b5
--define_symbol xQueueReceive=0x00022221
--define_symbol xQueueReceiveFromISR=0x0002234d
--define_symbol xQueueRemoveFromSet=0x000223e1
--define_symbol xQueueSelectFromSet=0x00022403
--define_symbol xQueueSelectFromSetFromISR=0x00022415
--define_symbol xQueueSemaphoreTake=0x00022429
--define_symbol xQueueTakeMutexRecursive=0x00022595
--define_symbol xTaskCheckForTimeOut=0x000225d9
--define_symbol xTaskCreate=0x00022649
--define_symbol xTaskCreateStatic=0x000226a5
--define_symbol xTaskGetCurrentTaskHandle=0x00022715
--define_symbol xTaskGetSchedulerState=0x00022721
--define_symbol xTaskGetTickCount=0x0002273d
--define_symbol xTaskGetTickCountFromISR=0x00022749
--define_symbol xTaskIncrementTick=0x00022759
--define_symbol xTaskPriorityDisinherit=0x00022825
--define_symbol xTaskPriorityInherit=0x000228b9
--define_symbol xTaskRemoveFromEventList=0x0002294d
--define_symbol xTaskResumeAll=0x000229cd
--define_symbol xTimerCreate=0x00022a95
--define_symbol xTimerCreateStatic=0x00022ac9
--define_symbol xTimerCreateTimerTask=0x00022b01
--define_symbol xTimerGenericCommand=0x00022b6d
--define_symbol xTimerGetExpiryTime=0x00022bdd
