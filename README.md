Generic PixelPusher Server implementation
=========================================

This implements the PixelPusher protocol similar to the
[PixelPusher devices] by Heroic Robotics.

This code attempts to be compatible with a basic subset of that protocol.

Originally implemented
in my [rpi-matrix-pixelpusher] repository, but separately provided here for
clearer API boundaries and simpler re-use.

License
-------
(c) 2013 Henner Zeller <h.zeller@acm.org>.

This is free software, licensed with the
[GNU General Public License, Version 3.0][gpl]

Please read and understand the license to learn which freedoms it provides to
you and users of the code you link it with.

Using this Library
------------------
This is a library, which you can use in your own code. If you are not a
programmer, this is not for you (in that case might more be interested
in solutions such as the [RGB Matrix][rpi-matrix-pixelpusher] or
the [Spixels PixelPusher][spixels-pixelpusher] implementation).

All you have to do, is to include [include/pp-server.h](./include/pp-server.h),
implement your output device, which needs to extend `pp::OutputDevice`, then
call `pp::StartPixelPusherServer()`.

```c++
class MyPixelOutputDevice : public pp::OutputDevice {
   // ... implement necessary methods are required by pp::OutputDevice,
   // such as SetPixel() (see include/pp-serve.h for details).
};

int main() {
   MyPixelOutputDevice pixel_output_device;   // Your implementation.

   pp::PPOptions options;
   options.network_interface = "wlan0";
   if (!pp::RunPixelPusherServer(options, &pixel_output_device))
       return 1;

   // -- Now, the PixelPusherServer is running in the background.
   for (;;) sleep(MAX_INT);  // do whatever else. Or just wait forever.

   pp::ShutdownPixelPusherServer();
   return 0;
}
```

.. then link the library found in lib (Makefile in there, simply
run 'make' in that directory).
Typically, you'd provide the path to the library in your compilation
call like so:

```
g++ my-pp-server.cc -o my-pp-server -Llib -lpixel-push-server
```

As a general suggestion to use it in your own code is to use include this
library as a sub-module in git, e.g. here checking it out in a sub-directory
`pp-server` in your project:


```
     git submodule add https://github.com/hzeller/pixelpusher-server pp-server
```

For an example how to do this, check out the
[RGBMatrix implementation][rpi-matrix-pixelpusher] using this library as a
sub-module and how it is referenced in the Makefile.

#### Network UDP packet size
The `PPOptions::udp_packet_size` parameter specifies the size of the allowed
UDP packets.
Some network switches (and the original PixelPusher hardware) don't like
large packets so the default is a conservative 1460 here.

But if we have a lot of pixels, using the highest number possible is
desirable so ideally we can transmit a full frame-buffer with one packet (use
something like 65507 here). This works well when sending data from an application
on your regular computer e.g. using [processing](http://processing.org/).

Even if the network supports it, sometimes sending devices limit the packet
size (e.g. iOS, 8192 Bytes seems to be the limit of packets to send; important
if you use LED labs softare) so in that case, we have to
use 8192 as `udp_packet_size`.


Controlling Software
--------------------
You can control these for instance with the Processing framework
<http://processing.org/>. The processing framework already has a contrib
library section that allows you to select PixelPusher supporting libs.

Another software supporting the PixelPusher support is
L.E.D. Lab http://www.ledlabs.co/

Artnet / sACN
-------------
If you use the Heroic Robotics [artnet bridge][artnet], you can specify the
artnet-universe and the artnet-channel in the `pp::PPOptions` struct.

[gpl]: https://www.gnu.org/licenses/gpl-3.0.txt
[PixelPusher devices]: http://www.heroicrobotics.com/products/pixelpusher
[rpi-matrix-pixelpusher]: https://github.com/hzeller/rpi-matrix-pixelpusher
[spixels-pixelpusher]: https://github.com/hzeller/spixels-pixelpusher
[artnet]: http://heroicrobotics.boards.net/thread/39/artnet-support-sacn