# LoRaWAN Sniffer and Stinker
ENGG4812 Thesis Project - LoRaWAN Sniffer IoT Project for Network Monitoring

This repository contains the code for the network monitoring 'sniffer' and traffic generating 'stinker'.

For the sake of simplicity and due to a combination of implementation needs and how the C make and library dependencies function for these LoRaWAN cards, the sniffer and stinker have been placed into their own folders for better handling.

# Getting them working

To properly build the Sniffer and Stinker you will need to run the **make** command in 'libtools' then 'libloragw' directories. From here all the necessary files to interface with the LoRa concentrator cards are created. You can then run **make** in the sniffer or stinker (client or server) directories

Always ensure the version of the libtools and libloragw directories match what concentrator card you are using. Change these files to match either the latest semtech versions or use the modified version provided by the card manufacturer (i.e. RAK)

## Sniffer

This directory is built to interface a Raspberry Pi to a RAK2287 via its appropriate PiHat. Build with the above steps and it should run fine, you will either have to reuse or get new authorisation files and endpoint to properly utilise the data uploading functionality of the Sniffer.

## Stinker

In order to increase traffic generation, the Stinker is composed of two radios due to the limitation of only being able to control a single LoRaWAN card per compiled C program. These have been split into the server and client.

Additionally, as this stinker was implemented using PiHat (server) and USB (client) based RAK2287, the libtools and libloragw folders are different for the server and client directories.

They share a basic port connection which allows the "server" program  finer timing control of by telling the "client" when to transmit. The system is quite limited and only allows for control of the FCnt and transmission parameters (SF, TX power), but with some modification could easily send fully premade lgw_pkt_tx_s structs.
