t2scan
======

This is a small channel scan tool which generates DVB-T/T2 channels.conf files.
ATSC is also available, but untested. 
The default output format is for vdr, but other output formats are also supported.

t2scan is based on wirbel's w_scan, which is based on the old "scan" tool from 
linuxtv-dvb-apps-1.1.0. Please see the "Credits" section at the end of this file.

The differences are:

t2scan vs w_scan:
- only DVB-T/T2 (ATSC available, but untested)
- simplified scanning (one-pass, NIT is only used to get the parameters of the *current* channel)
- removed several options, trying to keep this tool very simple

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

Either download one of the releases (https://github.com/mighty-p/t2scan/releases)
- or clone the latest state: git clone https://github.com/mighty-p/t2scan


1.2 Using the autotools tool chain
----------------------------------
Make sure you are in the t2scan directory and issue the usual

./configure
make
make install

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

NOTE: cmake allows as ./configure does, a user defined install prefix.
      Use 'cmake -DCMAKE_INSTALL_PREFIX=<YOUR_INSTALL_PREFIX> ..'. If you don't
      overwrite CMAKE_INSTALL_PREFIX, w_scan will be installed with prefix=/usr.

2 Basic usage
-------------

The simplest use case is to just call the program without parameters:

./t2scan

Obviously you can store the result in a file:

./t2scan > channels_new.conf

I don't recommend appending the new channels directly to your channels.conf. t2scan
may find duplicate channels or channels you are not interested in. Therefore my
recommendation is to copy & paste only those channels that you need.

Per default, t2scan takes a channel list for Germany to scan. This should, however, also be
OK in some other countries in Europe. For me, it also worked in Luxembourg and France.
Use the "-c" parameter to switch to a different channel list, e.g. the one for Great Britain:

./t2scan -c GB

For more sophisticated scan options see ./t2scan -h.

3 Copyright
-----------
t2scan is GPLv2 Software, see included file LICENSE for details.

4 Contacting the author
-----------------------
The author can be reached in the vdr-portal (user "mighty_p"). Also you can report issues
on the Github project page: https://github.com/mighty-p/t2scan

5 Credits
---------
- "wirbel" Winfried Koehler for the original w_scan
- "e9hack" Hartmut Birr for onid-patch (2006-09-01)
- "seaman" giving his his Airstar2 for testing purposes to wirbel
- "Wicky" for testing with Airstar2/Zarlink MT352 DVB-T
- "kilroy" for testing with Airstar2/Zarlink MT352 DVB-T and Avermedia
- "Fabrizio" for testing with Airstar2/Zarlink MT352 DVB-T
-  Arturo Martinez <martinez@embl.de> for a huge bunch of tests on DVB-S/DVB-S2
-  Rolf Ahrenberg for doing DVB-T/T2 tests and suggestions to improve w_scan

