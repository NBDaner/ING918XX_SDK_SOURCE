--define_symbol att_dispatch_client_can_send_now=0x00005aed
--define_symbol att_dispatch_client_request_can_send_now_event=0x00005af3
--define_symbol att_dispatch_register_client=0x00005af9
--define_symbol att_dispatch_register_server=0x00005b0d
--define_symbol att_dispatch_server_can_send_now=0x00005b21
--define_symbol att_dispatch_server_request_can_send_now_event=0x00005b27
--define_symbol att_emit_general_event=0x00005bd9
--define_symbol att_server_can_send_packet_now=0x000062ed
--define_symbol att_server_deferred_read_response=0x000062f1
--define_symbol att_server_get_mtu=0x00006309
--define_symbol att_server_indicate=0x00006381
--define_symbol att_server_init=0x00006405
--define_symbol att_server_notify=0x00006441
--define_symbol att_server_register_packet_handler=0x00006559
--define_symbol att_server_request_can_send_now_event=0x00006565
--define_symbol att_set_db=0x00006581
--define_symbol att_set_read_callback=0x00006595
--define_symbol att_set_write_callback=0x000065a1
--define_symbol bd_addr_cmp=0x00006711
--define_symbol bd_addr_copy=0x00006717
--define_symbol bd_addr_to_str=0x00006721
--define_symbol big_endian_read_16=0x00006759
--define_symbol big_endian_read_32=0x00006761
--define_symbol big_endian_store_16=0x00006775
--define_symbol big_endian_store_32=0x00006781
--define_symbol btstack_config=0x000068d5
--define_symbol btstack_memory_pool_create=0x00006a13
--define_symbol btstack_memory_pool_free=0x00006a3d
--define_symbol btstack_memory_pool_get=0x00006a9d
--define_symbol btstack_push_user_msg=0x00006ab9
--define_symbol char_for_nibble=0x00006d81
--define_symbol eTaskConfirmSleepModeStatus=0x00007029
--define_symbol gap_add_dev_to_periodic_list=0x00007799
--define_symbol gap_add_whitelist=0x000077b1
--define_symbol gap_aes_encrypt=0x000077c5
--define_symbol gap_clear_white_lists=0x00007809
--define_symbol gap_clr_adv_set=0x00007819
--define_symbol gap_clr_periodic_adv_list=0x00007829
--define_symbol gap_create_connection_cancel=0x00007855
--define_symbol gap_default_periodic_adv_sync_transfer_param=0x00007865
--define_symbol gap_disconnect=0x0000787d
--define_symbol gap_disconnect_all=0x000078a9
--define_symbol gap_ext_create_connection=0x00007951
--define_symbol gap_get_connection_parameter_range=0x00007a41
--define_symbol gap_le_read_channel_map=0x00007a79
--define_symbol gap_periodic_adv_create_sync=0x00007aed
--define_symbol gap_periodic_adv_create_sync_cancel=0x00007b11
--define_symbol gap_periodic_adv_set_info_transfer=0x00007b21
--define_symbol gap_periodic_adv_sync_transfer=0x00007b39
--define_symbol gap_periodic_adv_sync_transfer_param=0x00007b51
--define_symbol gap_periodic_adv_term_sync=0x00007b6d
--define_symbol gap_read_antenna_info=0x00007c01
--define_symbol gap_read_periodic_adv_list_size=0x00007c11
--define_symbol gap_read_phy=0x00007c21
--define_symbol gap_read_remote_info=0x00007c35
--define_symbol gap_read_remote_used_features=0x00007c49
--define_symbol gap_read_rssi=0x00007c5d
--define_symbol gap_remove_whitelist=0x00007c71
--define_symbol gap_rmv_adv_set=0x00007cfd
--define_symbol gap_rmv_dev_from_periodic_list=0x00007d11
--define_symbol gap_rx_test_v2=0x00007d29
--define_symbol gap_rx_test_v3=0x00007d41
--define_symbol gap_set_adv_set_random_addr=0x00007d91
--define_symbol gap_set_connection_cte_request_enable=0x00007de1
--define_symbol gap_set_connection_cte_response_enable=0x00007dfd
--define_symbol gap_set_connection_cte_rx_param=0x00007e11
--define_symbol gap_set_connection_cte_tx_param=0x00007e6d
--define_symbol gap_set_connection_parameter_range=0x00007ec1
--define_symbol gap_set_connectionless_cte_tx_enable=0x00007edd
--define_symbol gap_set_connectionless_cte_tx_param=0x00007ef1
--define_symbol gap_set_connectionless_iq_sampling_enable=0x00007f51
--define_symbol gap_set_data_length=0x00007fb5
--define_symbol gap_set_def_phy=0x00007fd1
--define_symbol gap_set_ext_adv_data=0x00007fe9
--define_symbol gap_set_ext_adv_enable=0x00008001
--define_symbol gap_set_ext_adv_para=0x0000807d
--define_symbol gap_set_ext_scan_enable=0x0000815d
--define_symbol gap_set_ext_scan_para=0x00008175
--define_symbol gap_set_ext_scan_response_data=0x0000821d
--define_symbol gap_set_host_channel_classification=0x00008235
--define_symbol gap_set_periodic_adv_data=0x00008249
--define_symbol gap_set_periodic_adv_enable=0x000082bd
--define_symbol gap_set_periodic_adv_para=0x000082d1
--define_symbol gap_set_periodic_adv_rx_enable=0x000082e9
--define_symbol gap_set_phy=0x000082fd
--define_symbol gap_set_random_device_address=0x00008319
--define_symbol gap_start_ccm=0x0000837d
--define_symbol gap_test_end=0x000083b1
--define_symbol gap_tx_test_v2=0x000083c1
--define_symbol gap_tx_test_v3=0x000083d9
--define_symbol gap_update_connection_parameters=0x00008401
--define_symbol gap_vendor_tx_continuous_wave=0x00008425
--define_symbol gatt_client_cancel_write=0x0000894d
--define_symbol gatt_client_discover_characteristic_descriptors=0x00008973
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid128=0x000089b3
--define_symbol gatt_client_discover_characteristics_for_handle_range_by_uuid16=0x00008a03
--define_symbol gatt_client_discover_characteristics_for_service=0x00008a53
--define_symbol gatt_client_discover_primary_services=0x00008a89
--define_symbol gatt_client_discover_primary_services_by_uuid128=0x00008abb
--define_symbol gatt_client_discover_primary_services_by_uuid16=0x00008aff
--define_symbol gatt_client_execute_write=0x00008b3b
--define_symbol gatt_client_find_included_services_for_service=0x00008b61
--define_symbol gatt_client_get_mtu=0x00008b8f
--define_symbol gatt_client_is_ready=0x00008c31
--define_symbol gatt_client_listen_for_characteristic_value_updates=0x00008c47
--define_symbol gatt_client_prepare_write=0x00008c69
--define_symbol gatt_client_read_characteristic_descriptor_using_descriptor_handle=0x00008ca5
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle=0x00008ccf
--define_symbol gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x00008cd5
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle=0x00008d03
--define_symbol gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset=0x00008d09
--define_symbol gatt_client_read_multiple_characteristic_values=0x00008d37
--define_symbol gatt_client_read_value_of_characteristic_using_value_handle=0x00008d67
--define_symbol gatt_client_read_value_of_characteristics_by_uuid128=0x00008d95
--define_symbol gatt_client_read_value_of_characteristics_by_uuid16=0x00008de1
--define_symbol gatt_client_register_handler=0x00008e2d
--define_symbol gatt_client_reliable_write_long_value_of_characteristic=0x00008e39
--define_symbol gatt_client_signed_write_without_response=0x00009269
--define_symbol gatt_client_write_characteristic_descriptor_using_descriptor_handle=0x0000932d
--define_symbol gatt_client_write_client_characteristic_configuration=0x00009367
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle=0x000093b9
--define_symbol gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset=0x000093c9
--define_symbol gatt_client_write_long_value_of_characteristic=0x00009405
--define_symbol gatt_client_write_long_value_of_characteristic_with_offset=0x00009415
--define_symbol gatt_client_write_value_of_characteristic=0x00009451
--define_symbol gatt_client_write_value_of_characteristic_without_response=0x00009487
--define_symbol hci_add_event_handler=0x0000a989
--define_symbol hci_power_control=0x0000b135
--define_symbol hci_register_acl_packet_handler=0x0000b2e9
--define_symbol kv_commit=0x0000b959
--define_symbol kv_get=0x0000b9b1
--define_symbol kv_init=0x0000b9c9
--define_symbol kv_put=0x0000ba31
--define_symbol kv_remove=0x0000baa9
--define_symbol kv_remove_all=0x0000bae5
--define_symbol kv_value_modified=0x0000bb29
--define_symbol kv_visit=0x0000bb2d
--define_symbol l2cap_can_send_fixed_channel_packet_now=0x0000bbed
--define_symbol l2cap_can_send_packet_now=0x0000bbf1
--define_symbol l2cap_create_channel=0x0000bda9
--define_symbol l2cap_disconnect=0x0000bee1
--define_symbol l2cap_get_remote_mtu_for_local_cid=0x0000c155
--define_symbol l2cap_init=0x0000c5d5
--define_symbol l2cap_le_send_flow_control_credit=0x0000c61d
--define_symbol l2cap_max_le_mtu=0x0000c8a1
--define_symbol l2cap_max_mtu=0x0000c8a5
--define_symbol l2cap_next_local_cid=0x0000c8a9
--define_symbol l2cap_next_sig_id=0x0000c8b9
--define_symbol l2cap_register_fixed_channel=0x0000c951
--define_symbol l2cap_register_packet_handler=0x0000c96d
--define_symbol l2cap_register_service=0x0000c979
--define_symbol l2cap_request_can_send_fix_channel_now_event=0x0000ca61
--define_symbol l2cap_request_can_send_now_event=0x0000ca85
--define_symbol l2cap_request_connection_parameter_update=0x0000ca9f
--define_symbol l2cap_require_security_level_2_for_outgoing_sdp=0x0000cad1
--define_symbol l2cap_send=0x0000ce65
--define_symbol l2cap_send_connectionless=0x0000cedd
--define_symbol l2cap_send_connectionless3=0x0000cf6d
--define_symbol l2cap_send_echo_request=0x0000d005
--define_symbol l2cap_send_signaling_le=0x0000d069
--define_symbol l2cap_unregister_service=0x0000d0c1
--define_symbol le_device_db_add=0x0000d125
--define_symbol le_device_db_find=0x0000d1f9
--define_symbol le_device_db_from_key=0x0000d225
--define_symbol le_device_db_iter_cur=0x0000d22d
--define_symbol le_device_db_iter_cur_key=0x0000d231
--define_symbol le_device_db_iter_init=0x0000d235
--define_symbol le_device_db_iter_next=0x0000d23d
--define_symbol le_device_db_remove_key=0x0000d263
--define_symbol ll_free=0x0000d28f
--define_symbol ll_hint_on_ce_len=0x0000d299
--define_symbol ll_legacy_adv_set_interval=0x0000d2d9
--define_symbol ll_malloc=0x0000d2e9
--define_symbol ll_query_timing_info=0x0000d421
--define_symbol ll_scan_set_fixed_channel=0x0000d4c5
--define_symbol ll_set_adv_access_address=0x0000d5d9
--define_symbol ll_set_adv_coded_scheme=0x0000d5e5
--define_symbol ll_set_conn_coded_scheme=0x0000d615
--define_symbol ll_set_conn_latency=0x0000d641
--define_symbol ll_set_conn_tx_power=0x0000d671
--define_symbol ll_set_def_antenna=0x0000d6b9
--define_symbol ll_set_initiating_coded_scheme=0x0000d6d5
--define_symbol ll_set_max_conn_number=0x0000d6e1
--define_symbol nibble_for_char=0x0001e351
--define_symbol platform_32k_rc_auto_tune=0x0001e3ed
--define_symbol platform_32k_rc_tune=0x0001e469
--define_symbol platform_calibrate_32k=0x0001e47d
--define_symbol platform_config=0x0001e481
--define_symbol platform_get_heap_status=0x0001e541
--define_symbol platform_get_us_time=0x0001e559
--define_symbol platform_get_version=0x0001e55d
--define_symbol platform_hrng=0x0001e565
--define_symbol platform_install_isr_stack=0x0001e56d
--define_symbol platform_patch_rf_init_data=0x0001e579
--define_symbol platform_printf=0x0001e585
--define_symbol platform_raise_assertion=0x0001e599
--define_symbol platform_rand=0x0001e5ad
--define_symbol platform_read_info=0x0001e5b1
--define_symbol platform_read_persistent_reg=0x0001e5cd
--define_symbol platform_reset=0x0001e5dd
--define_symbol platform_set_evt_callback=0x0001e611
--define_symbol platform_set_irq_callback=0x0001e625
--define_symbol platform_set_rf_clk_source=0x0001e65d
--define_symbol platform_set_rf_init_data=0x0001e669
--define_symbol platform_set_rf_power_mapping=0x0001e675
--define_symbol platform_shutdown=0x0001e681
--define_symbol platform_switch_app=0x0001e685
--define_symbol platform_trace_raw=0x0001e6b1
--define_symbol platform_write_persistent_reg=0x0001e6c9
--define_symbol printf_hexdump=0x0001e6d9
--define_symbol pvPortMalloc=0x0001f1e5
--define_symbol pvTaskIncrementMutexHeldCount=0x0001f2cd
--define_symbol pvTimerGetTimerID=0x0001f2e5
--define_symbol pxPortInitialiseStack=0x0001f311
--define_symbol reverse_128=0x0001f4f1
--define_symbol reverse_24=0x0001f4f7
--define_symbol reverse_48=0x0001f4fd
--define_symbol reverse_56=0x0001f503
--define_symbol reverse_64=0x0001f509
--define_symbol reverse_bd_addr=0x0001f50f
--define_symbol reverse_bytes=0x0001f515
--define_symbol sm_add_event_handler=0x0001f7d5
--define_symbol sm_address_resolution_lookup=0x0001f92d
--define_symbol sm_authenticated=0x0001fc79
--define_symbol sm_authorization_decline=0x0001fc87
--define_symbol sm_authorization_grant=0x0001fca7
--define_symbol sm_authorization_state=0x0001fcc7
--define_symbol sm_bonding_decline=0x0001fce1
--define_symbol sm_config=0x00020101
--define_symbol sm_config_conn=0x00020119
--define_symbol sm_encryption_key_size=0x0002029f
--define_symbol sm_just_works_confirm=0x000207cd
--define_symbol sm_le_device_key=0x00020b09
--define_symbol sm_passkey_input=0x00020b9f
--define_symbol sm_private_random_address_generation_get=0x00020f45
--define_symbol sm_private_random_address_generation_get_mode=0x00020f4d
--define_symbol sm_private_random_address_generation_set_mode=0x00020f59
--define_symbol sm_private_random_address_generation_set_update_period=0x00020f81
--define_symbol sm_register_oob_data_callback=0x000210bd
--define_symbol sm_request_pairing=0x000210c9
--define_symbol sm_send_security_request=0x00021ac3
--define_symbol sm_set_accepted_stk_generation_methods=0x00021ae9
--define_symbol sm_set_authentication_requirements=0x00021af5
--define_symbol sm_set_encryption_key_size_range=0x00021b01
--define_symbol sscanf_bd_addr=0x00021e5d
--define_symbol sysSetPublicDeviceAddr=0x000221c5
--define_symbol uuid128_to_str=0x00022949
--define_symbol uuid_add_bluetooth_prefix=0x000229a1
--define_symbol uuid_has_bluetooth_prefix=0x000229c1
--define_symbol uxListRemove=0x000229dd
--define_symbol uxQueueMessagesWaiting=0x00022a05
--define_symbol uxQueueMessagesWaitingFromISR=0x00022a2d
--define_symbol uxQueueSpacesAvailable=0x00022a49
--define_symbol uxTaskGetStackHighWaterMark=0x00022a75
--define_symbol vListInitialise=0x00022b1b
--define_symbol vListInitialiseItem=0x00022b31
--define_symbol vListInsert=0x00022b37
--define_symbol vListInsertEnd=0x00022b67
--define_symbol vPortEnterCritical=0x00022b81
--define_symbol vPortExitCritical=0x00022bc5
--define_symbol vPortFree=0x00022bf5
--define_symbol vPortSuppressTicksAndSleep=0x00022c89
--define_symbol vPortValidateInterruptPriority=0x00022da9
--define_symbol vQueueDelete=0x00022e01
--define_symbol vQueueWaitForMessageRestricted=0x00022e2d
--define_symbol vTaskDelay=0x00022e75
--define_symbol vTaskInternalSetTimeOutState=0x00022ec1
--define_symbol vTaskMissedYield=0x00022ed1
--define_symbol vTaskPlaceOnEventList=0x00022edd
--define_symbol vTaskPlaceOnEventListRestricted=0x00022f15
--define_symbol vTaskPriorityDisinheritAfterTimeout=0x00022f55
--define_symbol vTaskResume=0x00023001
--define_symbol vTaskStartScheduler=0x00023085
--define_symbol vTaskStepTick=0x00023115
--define_symbol vTaskSuspend=0x00023145
--define_symbol vTaskSuspendAll=0x00023201
--define_symbol vTaskSwitchContext=0x00023211
--define_symbol xPortStartScheduler=0x000232b9
--define_symbol xQueueAddToSet=0x0002337d
--define_symbol xQueueCreateCountingSemaphore=0x000233a1
--define_symbol xQueueCreateCountingSemaphoreStatic=0x000233dd
--define_symbol xQueueCreateMutex=0x00023421
--define_symbol xQueueCreateMutexStatic=0x00023437
--define_symbol xQueueCreateSet=0x00023451
--define_symbol xQueueGenericCreate=0x00023459
--define_symbol xQueueGenericCreateStatic=0x000234a5
--define_symbol xQueueGenericReset=0x0002350d
--define_symbol xQueueGenericSend=0x00023599
--define_symbol xQueueGenericSendFromISR=0x00023705
--define_symbol xQueueGiveFromISR=0x000237c5
--define_symbol xQueueGiveMutexRecursive=0x00023869
--define_symbol xQueueIsQueueEmptyFromISR=0x000238a9
--define_symbol xQueueIsQueueFullFromISR=0x000238cd
--define_symbol xQueuePeek=0x000238f5
--define_symbol xQueuePeekFromISR=0x00023a1d
--define_symbol xQueueReceive=0x00023a89
--define_symbol xQueueReceiveFromISR=0x00023bb5
--define_symbol xQueueRemoveFromSet=0x00023c49
--define_symbol xQueueSelectFromSet=0x00023c6b
--define_symbol xQueueSelectFromSetFromISR=0x00023c7d
--define_symbol xQueueSemaphoreTake=0x00023c91
--define_symbol xQueueTakeMutexRecursive=0x00023dfd
--define_symbol xTaskCheckForTimeOut=0x00023e41
--define_symbol xTaskCreate=0x00023eb1
--define_symbol xTaskCreateStatic=0x00023f0d
--define_symbol xTaskGetCurrentTaskHandle=0x00023f7d
--define_symbol xTaskGetSchedulerState=0x00023f89
--define_symbol xTaskGetTickCount=0x00023fa5
--define_symbol xTaskGetTickCountFromISR=0x00023fb1
--define_symbol xTaskIncrementTick=0x00023fc1
--define_symbol xTaskPriorityDisinherit=0x0002408d
--define_symbol xTaskPriorityInherit=0x00024121
--define_symbol xTaskRemoveFromEventList=0x000241b5
--define_symbol xTaskResumeAll=0x00024235
--define_symbol xTaskResumeFromISR=0x000242fd
--define_symbol xTimerCreate=0x00024389
--define_symbol xTimerCreateStatic=0x000243bd
--define_symbol xTimerCreateTimerTask=0x000243f5
--define_symbol xTimerGenericCommand=0x00024461
--define_symbol xTimerGetExpiryTime=0x000244d1
