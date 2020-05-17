t2scan
======

This is a small channel scan tool which generates DVB-T/T2 channels.conf files.
ATSC is also available, but untested. 
The default output format is for vdr, but other output formats are also supported.

t2scan is based on wirbel's w_scan, which is based on the old "scan" tool from 
linuxtv-dvb-apps-1.1.0. Please see the "Credits" section at the end of this file.

The differences are:

t2scan vs w_scan:
- Simplified scanning: The scan happens in one run and the NIT is only used to get the parameters of the *current* channel. This design decision was made since I noticed that often the NIT data referring to other channels is not reliable for DVB-T/T2, leading to channels not being found or having wrong IDs resulting in a non-working EPG in vdr. The approach should work with most current DVB-T/T2 cards since they can autodetect most parameters of a transponder when tuning. For DVB-S, this approach would result in many channels not being found, therefore I removed DVB-S (and -C) support.
- Added a parameter to determing the DVB-T type for scan ("-t1" for DVB-T only, "-t2" for DVB-T2 only). This makes the scan much faster if the user knows that only one DVB-T type is used in the user's region. The option for the tuning speed ("-t" in w_scan) has been changed to "-S".
- Added a parameter to determine the lowest channel to scan ("-c") and the highest channel to scan ("-C"). This makes the scan much faster if the user knows which channels are used in the user's region.
- Only DVB-T/T2 (ATSC available, but untested), support for DVB-C and DVB-S/S2 has been removed.
- Ability to scan other PLP IDs than only PLP ID 0 for DVB-T2. Use "-p" parameter to specify a comma-separated list of PLP IDs to be scanned (in that order). The special value -1 auto-detects the first PLP. There is a mechanism to avoid auto-detected PLPs from being scanned again and to update the PLP ID of services if the one in the T2 delivery descriptor of the auto-detected PLP was wrong. Default value for "-p" is -1,0,1 for most countries (for Austria and Italy: -1,1,0; for Russia: -1,1,2,3).
- Removed several options, trying to keep this tool very simple.

w_scan vs scan:
- no initial tuning data needed, because scanning without this data is exactly
  what a scan tool like this should do
- it detects automatically which DVB/ATSC card to use
- much more output formats, interfacing to other dtv software.

1 Building t2scan
-----------------

1.1 Obtaining t2scan
--------------------
You can always obtain the latest version from the Github project page
https://github.com/mighty-p/t2scan

Either download one of the releases (https://github.com/mighty-p/t2scan/releases) or clone the latest state: `git clone https://github.com/mighty-p/t2scan`


1.2 Using the autotools tool chain
----------------------------------
Make sure you are in the t2scan directory and issue the usual

```
./configure
make
make install
```

NOTE: For compiling need up-to-date dvb headers with DVB API 5.3 support
are needed. If configure fails complaining about missing or old DVB headers,
run in your kernel source 'make headers_install' 
(needs kernel 2.6.29 or higher).
At your choice you may also manually update this files; they're located
in /usr/include/linux/dvb.

1.3 Using the cmake tool chain
------------------------------
You may also use the new cmake tool chain, but be warned - that tool chain is
new and experimental in t2scan up to now. It's intended to be used as

mkdir build && cd build
cmake ..
make
make install

NOTE: `cmake` allows as `./configure` does, a user defined install prefix. Use `cmake -DCMAKE_INSTALL_PREFIX=<YOUR_INSTALL_PREFIX> ..`. If you don't overwrite CMAKE_INSTALL_PREFIX, w_scan will be installed with prefix=/usr.

2 Usage
-------

### Basic Usage

One of the goals of t2scan is to be as simple to use as possible. Therefore, in most regions it should be sufficient to call t2scan without any parameters and it will find all available TV and radio stations that can be received: `t2scan`

Obviously you can store the result in a file: `t2scan > channels_new.conf`

I don't recommend appending the new channels directly to your channels.conf. t2scan may find duplicate channels or channels you are not interested in. Therefore my recommendation is to copy & paste only those channels that you need.

#### Filtering

To speed up the scan, there are several simple filter options that you can use if you want to scan only certain channels or if you know that (for example) only DVB-T2 (and no DVB-T) is used in your region. Some examples below:

* DVB-T Type (`-t`): 
  * To scan only for DVB-T2 channels: `t2scan -t2`
  * To scan only for DVB-T (but not DVB-T2) channels: `t2scan -t1`
* List of channels (`-l`): 
  * To scan only the channels 21, 24, 27: `t2scan -l21,24,27`
* Minimum and maximum channels (`-c`/`-C`): 
  * To scan only channels 21 to 49: `t2scan -c21 -C49`
* DVB-T2 PLP IDs (`-p`):
  * To scan without PLP ID filter (auto-detects first PLP): `t2scan -p-1`
  * To scan PLP IDs 0, 1 and 2: `t2scan -p0,1,2`

#### Output options

* Exclude encrypted (`-E`): 
  * To exclude encrypted services from output: `t2scan -E`
* Output type (`-o`): 
  * To create channel data for xine (instead of vdr): `t2scan -oxine`
* Mark duplicates (`-d`): 
  * To mark duplicate channels and services in the output: `t2scan -d`
* Type of services (`-s`): 
  * To output only radio services: `t2scan -sr`
  * To include TV, radio, and other services in output: `t2scan -stro`

#### Overview of all basic options

To see all t2scan basic options, use `t2scan -h`. 

### Expert options

There is also a set of so-called expert options to cover some seldom use cases or to help debug in case of problems. Probably the most common of them are the country settings and the option to set a different DVB-T card, as described below. To see all expert options, use `t2scan -H`.

#### Country settings

Normally, t2scan detects your country from your system settings and uses an appropriate channel list (and for DVB-T2 also the appropriate PLP-ID) to scan. However, there can be some edge cases where this does not work well: For example your system may be configured for a different country than the one you are currently in, or you may want to receive channels from a neighbor country and it does not work with the default settings (NOTE: you can still give the default settings a try...). Use the `-Y` parameter to override the country to be used and the `-L` parameter if you want to set the channel list manually.

#### Use different DVB-T card

t2scan automatically checks which DVB hardware is available in the system and selects automatically an appropriate DVB-T(2) adapter to use. If you want to override this selection, you can use the `-a` parameter and provide the number of the adapter. This should really be only needed if you have multiple DVB-T(2) adapters in the system and want t2scan to use one specific of them.


3 Copyright
-----------
t2scan is GPLv2 Software, see included file LICENSE for details.

4 Contacting the author
-----------------------
The author can be reached in the vdr-portal (user "mighty-p"). Also you can report issues
on the Github project page: https://github.com/mighty-p/t2scan

5 Credits
---------
- "wirbel" Winfried Koehler for the original w_scan
- "e9hack" Hartmut Birr for onid-patch (2006-09-01)
- "seaman" giving his his Airstar2 for testing purposes to wirbel
- "Wicky" for testing with Airstar2/Zarlink MT352 DVB-T
- "kilroy" for testing with Airstar2/Zarlink MT352 DVB-T and Avermedia
- "Fabrizio" for testing with Airstar2/Zarlink MT352 DVB-T
- Arturo Martinez <martinez at the server embl dot de> for a huge bunch of tests on DVB-S/DVB-S2
- Rolf Ahrenberg for doing DVB-T/T2 tests and suggestions to improve w_scan
- "GTC" for providing a patch to fix the tuning timeouts code
- Andreas Mikula for reporting that Austria uses PLP ID 1 and for help with debugging
- "HelmutB" for informing me about Austria using PLP ID 1, for testing and advice about PLP IDs.
- "clausmuus" for helping me test why scanning on some devices has not worked reliably
- "highrgb" for testing
- "motze" for testing and for finding out that Italy uses PLP ID 1 for their DVB-T2 test multiplex
