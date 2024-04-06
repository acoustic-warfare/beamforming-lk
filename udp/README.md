# UDP Transfers

You may want to perform offline testing but on real transfers, then you can
replay recorded packet transfers. To listen to transfers in offline mode, it is
good to convert the packet destination IP to one that exists without the
connected interface, i.e. `localhost`. This directory can be used to store and
replay transfers from the FPGA to the PC. We used `Wireshark` (https://www.wireshark.org/) 
for saving transfers to disk.

## Converting IP Destination

Conversion to localhost

The converter requires the package scapy for modifiying `.pcap` destinations.
Create a virtual environment:
```bash
python -m venv .venv
```

Activate it
```bash 
source env/bin/activate
```

Install required packages
```bash 
python -m pip install -r requirements.txt 
```

Perform the conversion
```bash
python udpreplace.py <FILE>.pcap <FROM> <TO> <PORT>
```


## IP-Converting Example 

You want to convert the recording `recorded/recording.pcap` from IP address
`10.0.0.1` to `127.0.0.1` and set the port to `12345` which will save the
resulting file to `converted/recording_converted.pcap` which will look like:

This process is embarrassingly slow, a `10s` recording may take up to `2min` to
convert... 

```bash 
python udpreplace.py recorded/recording.pcap 10.0.0.1 127.0.0.1 12345
```

Deactivate (optional)

```bash 
deactivate
```


## UDP-Replaying

To replay saved packets, you could use the program `udpreplay` 

```bash 
udpreplay -i lo FILE.pcap
```

For example 
```bash 
udpreplay -i lo converted/recording_converted.pcap
```


