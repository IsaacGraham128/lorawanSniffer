// Cleaning up the stinker_server for the sake of demo, don't want my ugly code

    // Original code with comments, will need to clean
    test_duration_secs = 20;
    for (int j = 0; j < 3; j++) {
        buffer_tx[2] = 12;
        send(new_socket, buffer_tx, sizeof(buffer_tx), 0);

        MSG_INFO("Loop %d\n", j);
        packets_per_minute = 8;
        wait_time_ms = (long)(ms_per_minute / packets_per_minute);
        
        while (wait_time_ms > packet_airtime_ms) {

            //sprintf(file_helper, "5min_ppm_%d_jamming_%d", packets_per_minute, j);
            // log_open(file_helper);
            MSG_INFO("PPM: %d. Delay %lums\n", packets_per_minute, wait_time_ms);
            //MSG_INFO("Jammer message delay is %lums\n", wait_time_ms);

            /* Re(set) FCnts */
            fcnt = 1; 
            fcnt_client = 1;
            
            for (int j = 27; j > 26; j--) {
                //MSG_INFO("(Jammer %d, Desired %d)\n", j, 12);
                pkt.rf_power = j;

                // Need to create a function here for the scaling jam
                // Have the function take in the desired ppm, then have it function like the other jammer
                jamming_scaling(&pkt, new_socket, 17, test_duration_secs, wait_time_ms, 1000, &fcnt, &fcnt_client);

                wait_ms(test_duration_secs * 1000);

                if (exit_sig || quit_sig) // Code to exit
                    interrupt_cleanup(new_socket, server_fd);
            }

            packets_per_minute *= scaler;                               // Packets per minute as per scaler
            wait_time_ms = (long)(ms_per_minute / packets_per_minute);  // Update the wait time

            // wait_ms(ms_per_minute);
        }
    }


    /* Send exit cmd to client sniffer */
    interrupt_cleanup(new_socket, server_fd);


    /* Case to testing forced collisions */
    /* Selectively interrupting transmitted packet sections */
   
   
    /* Decrease jammer radio power */
    // Set desired radio power to lowest and SF
    // desired_power = 12;
    // buffer_tx[2] = desired_power;
    // send(new_socket, buffer_tx, sizeof(buffer_tx), 0);
    // buffer_sf[2] = pkt.datarate;
    // send(socket, buffer_sf, sizeof(buffer_sf), 0);

    // for (i = 0; i < 3; i++) {
    //     /* Create log */
    //     sprintf(file_helper, "packet_interruption_section_%d", i);
    //     log_open(file_helper);

    //     /* Indicate what part of the packet we are interrupting */
    //     switch(i) {
    //         case 0 : MSG_INFO("Transmission will interrupt the Preamble and Sync Word.\n"); break;
    //         case 1 : MSG_INFO("Transmission will interrupt the PHDR and CRC.\n");           break;
    //         case 2 : MSG_INFO("Transmission will interrupt the FRMPayload.\n");             break;
    //         case 3 : MSG_INFO("Transmission will interrupt the final CRC.\n");              break;
    //         default: MSG_ERR("Bad frame section selected. Exiting function");
    //     }

    //     // Set the desired message power
    //     buffer_tx[2] = 12;
    //     send(new_socket, buffer_tx, sizeof(buffer_tx), 0);

    //     /* Lowering jammer transmission power */
    //     for (int j = 27; j > 11; j--) {
    //         MSG_INFO("(Jammer %d, Desired %d)\n", j-12, 12);
    //         pkt.rf_power = j;
    //         jamming_selective(&pkt, i, new_socket, 40);

    //         wait_ms(20000);

    //         if (exit_sig || quit_sig) // Code to exit
    //             interrupt_cleanup(new_socket, server_fd);
    //     }

    //     /* Increasing desired transmission power*/
    //     /* No need to lower pkt.rf_power, already at lowest from final test of for loop above */
    //     for (int j = 13; j < 20; j++) {
    //         MSG_INFO("(Jammer %d, Desired %d)\n", 0, j-12);
    //         buffer_tx[2] = j;
    //         send(new_socket, buffer_tx, sizeof(buffer_tx), 0);
    //         jamming_selective(&pkt, i, new_socket, 40);

    //         wait_ms(20000);

    //         if (exit_sig || quit_sig) // Code to exit
    //             interrupt_cleanup(new_socket, server_fd);
    //     }
    // }

    /* Old jammer code */
    // // USE BELOWWWWWWW!
    // for (int j = 27; j > 11; j--) {
    //     jammer_power = j;
    //     MSG_INFO("------- TX Power Difference is %ddB (Radio 0 (jammer): %ddB Radio 1: %ddB) -------\n",
    //     jammer_power - desired_power, jammer_power, desired_power);

    //     // Set jammer and desired powers here
    //     pkt.rf_power = jammer_power;

    //     for (i = 0; i < 4; i++) {
    //         jamming_selective(&pkt, i, new_socket);

    //         if (i != 3)
    //             wait_ms(10000); // Only do the 10 second wait if tests are still to be completed
    //     }

    //     MSG_INFO("------- TX Power Difference %ddB Interruption Test Complete -------\n", desired_power - jammer_power);
    //     if (j != 12) {
    //         MSG_INFO("Testing break (20 seconds).\n\n");
    //         while (wait_time_ms < 20000) {
    //             wait_ms(100);
    //             wait_time_ms += 100;

    //             if (exit_sig || quit_sig) {
    //                 MSG_ERR("Exit signal heard! Im gone...\n");
    //                 please_leave = true;
    //                 break;
    //             }
    //         }
    //         if (please_leave) {
    //             break;
    //         }
    //         wait_time_ms = 0; /* Reset for next sleep */
    //     }
    // }

    /* Case to test a set radio and a scaling one */
    

    
    /* Scaling test with new format */
    /* prep the desired */
    // HAS BEEN EDITED FOR DEMO SHOWCASE
    // airtime = (pkt.size + 8 + 4.25 + 8 + 2) * 8;
    // switch (pkt.datarate) {
    //     case DR_LORA_SF7:   airtime = airtime / BITRATE_DR5; break;
    //     case DR_LORA_SF8:   airtime = airtime / BITRATE_DR4; break;
    //     case DR_LORA_SF9:   airtime = airtime / BITRATE_DR3; break;
    //     case DR_LORA_SF10:  airtime = airtime / BITRATE_DR2; break;
    //     case DR_LORA_SF11:  airtime = airtime / BITRATE_DR1; break;
    //     case DR_LORA_SF12:  airtime = airtime / BITRATE_DR0; break;
    //     default:            MSG_ERR("Unknown spreading factor found");
    // }
    // packet_airtime_ms = (long int)(airtime * 1e3);
    // long potential_ppm, airtime_to_take_ms;
    // float toa_util = 0.015; // utilisation percentage in whole digits...
    // test_duration_secs = 60;
    
    // while (toa_util < 1.0) {
    //     airtime_to_take_ms = (long)((test_duration_secs * 1000.0) * toa_util);
    //     MSG_INFO("Airtime to take was %lu\n", airtime_to_take_ms);
    //     potential_ppm = airtime_to_take_ms / packet_airtime_ms;
    //     MSG_INFO("To acheieve %.3f airtime utilisation, we could use %luppm\n", toa_util, potential_ppm);
    //     toa_util *= 2;
    // }
    // interrupt_cleanup(new_socket, server_fd);

    

    // for (int j = 27; j > 11; j--) {
    //     sprintf(file_helper, "rf_power_check");
    //     log_open(file_helper);
    //     packets_per_minute = 4;
    //     jammer_power = j;
    //     desired_power = 12;

    //     /* Configure the stinker partner */
    //     pkt.datarate = DR_LORA_SF7;
    //     buffer_sf[2] = DR_LORA_SF7;
    //     send(new_socket, buffer_sf, sizeof(buffer_sf), 0);
    //     pkt.rf_power = jammer_power;
    //     buffer_tx[2] = desired_power;
    //     send(new_socket, buffer_tx, sizeof(buffer_tx), 0);

    //     /* (Re)set FCnt cause this may be a new test! */
    //     /* Dont wanna reset mid test however! */
    //     fcnt = 1; // Set it to 1! Cant have 0
    //     pkt.payload[6] = fcnt & 0x00FF;
    //     pkt.payload[7] = fcnt >> 8;

    //     MSG_INFO("Packet airtime is %ldms\n", packet_airtime_ms);
    //     MSG_INFO("------- TX Power Difference is %ddB (Radio 0 (jammer): %ddB Radio 1: %ddB) -------\n",
    //         jammer_power - desired_power, jammer_power, desired_power);

    //     wait_time_ms = (long)(ms_per_minute / packets_per_minute);

    //     while (wait_time_ms > packet_airtime_ms) {
    //         /* Prep new test variables */
    //         transmitted = 0;
    //         run_time = 0;
    //         stinker_time_ms_second = 6000; // Have stinker partner transmit right away
    //         stinker_time_ms_main = stinker_time_ms_second / 2;   // Force the main radio to wait

    //         MSG_INFO("Starting Packets Per Minute (PPM) at %d test\n", packets_per_minute);
    //         MSG_INFO("Packets to send this testing period: %ld\n", packets_per_minute * (test_duration_secs / 60));
    //         MSG_INFO("Delay between messages should be: %lums\n", wait_time_ms);

    //         while ((run_time < test_duration_secs) && (!exit_sig && !quit_sig)) {

    //             /* Get time first */
    //             clock_gettime(CLOCK_MONOTONIC, &start_time);
    //             //start_time = time(NULL);

    //             if (stinker_time_ms_second >= 6000) { // Send message every 6 seconds
    //                 /* Update our counters */
    //                 buffer_fcnt[3] = fcnt_client & 0x00FF;
    //                 buffer_fcnt[4] = fcnt_client >> 8;

    //                 fcnt_client++;
                    
    //                 send(new_socket, buffer_fcnt, sizeof(buffer_fcnt), 0);
    //                 stinker_time_ms_second = 0; /* Reset for next send */
    //             }

    //             /* Lets send that packet */
    //             // Ensure stinker time is adjust from micro to milli
    //             if (stinker_time_ms_main >= wait_time_ms) {
    //                 i = lgw_status(0, 1, &tx_status); // Check radio 0
    //                 if (i == LGW_HAL_ERROR) {
    //                     MSG_ERR("lgw_status failed with code %d\n", tx_status);
    //                 } else {
    //                     if (tx_status == TX_EMITTING) {
    //                         // Do nothing
    //                     } else if (tx_status == TX_FREE) {
    //                         // Update frame counter and send!
    //                         pkt.payload[6] = fcnt & 0x00FF;
    //                         pkt.payload[7] = fcnt >> 8;

    //                         i = lgw_send(&pkt);
    //                         if (i != LGW_HAL_SUCCESS) {
    //                             MSG_ERR("failed to send for some reason\n");
    //                         } else {
    //                             /* Update our counters */
    //                             MSG_INFO("%d\n", pkt.rf_power);
    //                             transmitted++;
    //                             fcnt++;

    //                             stinker_time_ms_main = 0; /* Reset for next send */
    //                         }
    //                     }
    //                 }
    //             }
                
    //             wait_ms(10); // MUST WAIT, otherwise it wont ever call the packet sending

    //             clock_gettime(CLOCK_MONOTONIC, &end_time);
    //             time_diff = (end_time.tv_sec - start_time.tv_sec) * 1e3 + (end_time.tv_nsec - start_time.tv_nsec)/ 1e6;
    //             stinker_time_ms_main += time_diff;
    //             stinker_time_ms_second += time_diff;
    //             run_time += end_time.tv_sec - start_time.tv_sec;

    //             if (exit_sig || quit_sig) {
    //                 interrupt_cleanup(new_socket, server_fd);
    //             }
    //         }

    //         if (exit_sig || quit_sig) {
    //             interrupt_cleanup(new_socket, server_fd);
    //         }

    //         MSG_INFO("Messages sent during period %llu\n", transmitted);
    //         MSG_INFO("Ending Packets Per Minute (PPM) at %d test\n\n", packets_per_minute);

    //         if (please_leave) {
    //             break;
    //         }

    //         packets_per_minute *= scaler;                               // Packets per minute as per scaler
    //         wait_time_ms = (long)(ms_per_minute / packets_per_minute);  // Update the wait time

    //         if (wait_time_ms > packet_airtime_ms)
    //             wait_ms(ms_per_minute);                             // Waiting 1 minute to seperate our times
    //     }

    //     if (please_leave) {
    //         break;
    //     }

    //     wait_ms(packets_per_minute / 2); // Half minute between tests
    // }



    // /* Case to test spreading factor power */
    // for (int j = 7; j < 13; j++) {
    //     sprintf(file_helper, "stinkers_dutchess_sf%d_tx12dbm", j);
    //     log_open(file_helper);
    //     pkt.datarate = j;
    //     packets_per_minute = 10;

    //     /* Configure the stinker partner */
    //     buffer_sf[2] = j;
    //     send(new_socket, buffer_sf, sizeof(buffer_sf), 0);

    //     for (; packets_per_minute <= max_ppm; packets_per_minute *= scaler) {


    //         /* (Re)set FCnt cause this may be a new test! */
    //         fcnt = 1; // Set it to 1! Cant have 0
    //         pkt.payload[6] = fcnt & 0x00FF;
    //         pkt.payload[7] = fcnt >> 8;

    //         /* Prep new test variables */
    //         transmitted = 0;
    //         wait_time_ms = ms_per_minute / packets_per_minute;
    //         run_time = 0;

    //         MSG_INFO("Starting Packets Per Minute (PPM) at %d test\n", packets_per_minute);
    //         MSG_INFO("Total number of packets should be %ld\n", packets_per_minute * (test_duration_secs / 60));
    //         MSG_INFO("Delay between messages should be %lu\n", wait_time_ms);

    //         while ((run_time < test_duration_secs) && (!exit_sig && !quit_sig)) {

    //             /* Get time first */
    //             start_time = time(NULL);
    //             /* Lets send that packet */
    //             i = lgw_status(0, 1, &tx_status); // Check radio 0
    //             if (i == LGW_HAL_ERROR) {
    //                 MSG_ERR("lgw_status failed with code %d\n", tx_status);
    //             } else {
    //                 if (tx_status == TX_EMITTING) {
    //                     /* ask the other device to send */
    //                     //MSG_INFO("Asking buddy to send\n");
    //                     buffer_fcnt[3] = fcnt & 0x00FF;
    //                     buffer_fcnt[4] = fcnt >> 8;
                        
    //                     send(new_socket, buffer_fcnt, sizeof(buffer_fcnt), 0);
                        
    //                     /* Update our counters */
    //                     transmitted++;
    //                     fcnt++;
    //                     /* Update the packet FCnt for next trans */
    //                     pkt.payload[6] = fcnt & 0x00FF;
    //                     pkt.payload[7] = fcnt >> 8;

    //                 } else if (tx_status == TX_FREE) {
    //                     // Send on radio 0
    //                     //MSG_INFO("Sending message\n");
    //                     pkt.rf_chain = 0;
    //                     pkt.freq_hz = 916800000;
    //                     i = lgw_send(&pkt);
    //                     if (i != LGW_HAL_SUCCESS) {
    //                         MSG_ERR("failed to send for some reason\n");
    //                     } else {
    //                         /* Update our counters */
    //                         transmitted++;
    //                         fcnt++;
    //                         /* Update the packet FCnt for next trans */
    //                         pkt.payload[6] = fcnt & 0x00FF;
    //                         pkt.payload[7] = fcnt >> 8;
    //                     }
    //                 }
    //             }
    //            wait_ms(wait_time_ms);
    //             run_time += time(NULL) - start_time;
    //         }

    //         if (exit_sig || quit_sig) {
    //             MSG_ERR("Exit signal heard! Im gone...\n");
    //             send(new_socket, exit_str, strlen(exit_str), 0);
    //             /* close the client socket */ 
    //             close(new_socket);
    //             /* closing the server socket */ 
    //             shutdown(server_fd, SHUT_RDWR);
    //             sniffer_stop();
    //             return -1;
    //         }

    //         MSG_INFO("Messages sent during period %llu\n", transmitted);
    //         MSG_INFO("Ending Packets Per Minute (PPM) at %d test\n", packets_per_minute);
    //         wait_ms(ms_per_minute); // Waiting 1 minute to seperate our times
    //     }
    // }
    

    // /* Case to test transmission power */
    // pkt.datarate = DR_LORA_SF7;
    // for (int j = 1; j < 3; j++) {
    //     switch (j) {
    //         case 0 :
    //             power = 12;
    //             break;
    //         case 1 :
    //             power = 19;
    //             break;
    //         case 2 :
    //             power = 27;
    //             break;
    //         default :
    //             power = 12;
    //     }


    //     sprintf(file_helper, "stinkers_dutchess_sf7_tx%ddbm", power);
    //     log_open(file_helper);
    //     pkt.rf_power = power;
    //     pkt.freq_hz = 916800000;
    //     packets_per_minute = 10;

    //     buffer_tx[2] = power;
    //     send(new_socket, buffer_tx, sizeof(buffer_tx), 0);

    //     for (; packets_per_minute <= max_ppm; packets_per_minute *= scaler) {


    //         /* (Re)set FCnt cause this may be a new test! */
    //         fcnt = 1; // Set it to 1! Cant have 0
    //         pkt.payload[6] = fcnt & 0x00FF;
    //         pkt.payload[7] = fcnt >> 8;

    //         /* Prep new test variables */
    //         transmitted = 0;
    //         wait_time_ms = ms_per_minute / packets_per_minute;
    //         run_time = 0;

    //         MSG_INFO("Starting Packets Per Minute (PPM) at %d test\n", packets_per_minute);
    //         MSG_INFO("Total number of packets should be %ld\n", packets_per_minute * (test_duration_secs / 60));
    //         MSG_INFO("Delay between messages should be %lu\n", wait_time_ms);

    //         while ((run_time < test_duration_secs) && (!exit_sig && !quit_sig)) {
    //             /* Get time first */
    //             start_time = time(NULL);
    //             /* Lets send that packet */
    //             i = lgw_status(0, 1, &tx_status); // Check radio 0
    //             if (i == LGW_HAL_ERROR) {
    //                 MSG_ERR("lgw_status failed with code %d\n", tx_status);
    //             } else {
    //                 if (tx_status == TX_EMITTING) {
    //                     /* ask the other device to send */
    //                     buffer_fcnt[3] = fcnt & 0x00FF;
    //                     buffer_fcnt[4] = fcnt >> 8;
                        
    //                     send(new_socket, buffer_fcnt, sizeof(buffer_fcnt), 0);
                        
    //                     /* Update our counters */
    //                     transmitted++;
    //                     fcnt++;
    //                     /* Update the packet FCnt for next trans */
    //                     pkt.payload[6] = fcnt & 0x00FF;
    //                     pkt.payload[7] = fcnt >> 8;

    //                 } else if (tx_status == TX_FREE) {
    //                     // Send on radio 0
    //                     //MSG_INFO("Sending message\n");
    //                     pkt.rf_chain = 0;
    //                     pkt.freq_hz = 916800000;
    //                     i = lgw_send(&pkt);
    //                     if (i != LGW_HAL_SUCCESS) {
    //                         MSG_ERR("failed to send for some reason\n");
    //                     } else {
    //                         /* Update our counters */
    //                         transmitted++;
    //                         fcnt++;
    //                         /* Update the packet FCnt for next trans */
    //                         pkt.payload[6] = fcnt & 0x00FF;
    //                         pkt.payload[7] = fcnt >> 8;
    //                     }
    //                 }
    //             }

                
    //             wait_ms(wait_time_ms);
    //             run_time += time(NULL) - start_time;
    //         }

    //         if (exit_sig || quit_sig) {
    //             MSG_ERR("Exit signal heard! Im gone...\n");
    //             send(new_socket, exit_str, strlen(exit_str), 0);
    //             /* close the client socket */ 
    //             close(new_socket);
    //             /* closing the server socket */ 
    //             shutdown(server_fd, SHUT_RDWR);
    //             sniffer_stop();
    //             return -1;
    //         }

    //         MSG_INFO("Messages sent during period %llu\n", transmitted);
    //         MSG_INFO("Ending Packets Per Minute (PPM) at %d test\n", packets_per_minute);
    //         wait_ms(ms_per_minute); // Waiting 1 minute to seperate our times
    //     }
    // }

    //experiment_offered_load(256, 2, 300);

        /* Replaced code below with the function above, kept for safety sake */
    // send(new_socket, exit_str, strlen(exit_str), 0);

    // /* close the client socket */ 
    // close(new_socket);

    // /* closing the server socket */ 
    // shutdown(server_fd, SHUT_RDWR);

    // sniffer_stop();
    // MSG_INFO("Successfully exited our packet stinker program\n");

    // return EXIT_SUCCESS;