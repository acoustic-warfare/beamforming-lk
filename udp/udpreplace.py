#!/usr/bin/python3

"""
File: udpreplace.py 
Author: Irreq 
Date: 2023-10-20

Description: Script for replacing IP destinations for .pcap files 
"""

from scapy.all import *

# """
# tshark -r in.pcap -w out.pcap -Y "frame.number >= N && frame.number <= M"
# """

import multiprocessing
import argparse

parser = argparse.ArgumentParser(
    prog="Find and replace UDP ip and port",
    description="What the program does",
    epilog="Text at the bottom of help",
)

# parser.add_argument("filename")  # positional argument
parser.add_argument("ip_from")
parser.add_argument("ip_to")
parser.add_argument("port")
parser.add_argument("-c", "--count")  # option that takes a value
parser.add_argument("-v", "--verbose", action="store_true")
args = parser.parse_args()
import os
import glob


def choose_pcap_file(directory):
    # Ensure the directory exists
    if not os.path.exists(directory):
        print("Directory does not exist.")
        return None

    # Use glob to find .pcap files
    pcap_files = glob.glob(os.path.join(directory, "*.pcap"))

    # Check if any .pcap files were found
    if not pcap_files:
        print("No .pcap files found in the directory.")
        return None

    # Display the found .pcap files to the user
    print("Found .pcap files in the directory:")
    for i, file_path in enumerate(pcap_files):
        print(f"{i+1}. {os.path.basename(file_path)}")

    # Ask the user to choose a file
    while True:
        choice = input("Enter the number of the file you want to choose: ")
        try:
            choice = int(choice)
            if choice < 1 or choice > len(pcap_files):
                raise ValueError
            break
        except ValueError:
            print(
                "Invalid choice. Please enter a number between 1 and", len(pcap_files)
            )

    # Return the chosen file path
    return pcap_files[choice - 1]


# Example usage:
directory = "recorded/"
chosen_file = choose_pcap_file(directory)
if chosen_file:
    print("Chosen file:", chosen_file)
else:
    print("No file chosen")
    exit()

args.filename = chosen_file
print("This might take a while (1>minute)...")
packets = rdpcap(args.filename)
print("Completed filereading...")
target = args.ip_from
new = args.ip_to


def find_and_replace(packet):
    if IP in packet:
        packet[IP].dst = new
    return packet


print("Replacing addresses...")
# create a process pool that uses all cpus
with multiprocessing.Pool() as pool:
    # call the function for each item in parallel, get results as tasks complete
    modified_packets = list(pool.imap(find_and_replace, packets))

# Save the modified packets to a new pcap file
output_file = args.filename.replace(".pcap", "_replace.pcap")
output_file = output_file.replace(directory, "converted/")
print(f"Saving file: {output_file}...")
wrpcap(output_file, modified_packets)
print("Done!")
exit()


def modify_destination_ip(packet, new_destination_ip):
    if IP in packet:
        packet[IP].dst = new_destination_ip
    return packet


def modify_pcap_file(input_file, output_file, new_destination_ip):
    packets = rdpcap(input_file)  # Read pcap file
    # print(packets)
    # return

    # Modify each packet's destination IP address
    modified_packets = [
        modify_destination_ip(packet, new_destination_ip) for packet in packets
    ]

    # Save the modified packets to a new pcap file
    wrpcap(output_file, modified_packets)


if __name__ == "__main__":
    input_pcap_file = "out.pcap"
    output_pcap_file = "out_rp.pcap"
    new_destination_ip = "127.0.0.1"

    modify_pcap_file(input_pcap_file, output_pcap_file, new_destination_ip)
