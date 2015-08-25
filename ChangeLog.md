## Changes between libo3d3xx 0.1.7 and 0.1.8

### Schema-aware FrameGrabber

https://github.com/lovepark/libo3d3xx/issues/4

Per suggestion by @graugans (IFM), the `FrameGrabber` will now apply a known
schema at runtime in the event the configured schema for the active application
is different than what is expected. Additionally, the configured schema gets
restored once the FrameGrabber is destroyed. This allows for compatibility with
other computing environments that may expect the configured schema to be
returned.

### Interoperability with IFM's Vision Assistant

IFM's software provides the means to import/export applications to/from the
camera where the files have an IFM-specific serialization (zip-compressed JSON
as of this writing). `o3d3xx::Camera` now has an `ImportIFMApp` and
`ExportIFMApp` method to enable passing data to/from the camera in a way that
is compatible with Vision Assistant. Additionally, two new command line tools
have been provided (`o3d3xx-ifm-import` and `o3d3xx-ifm-export`) that exploits
the new camera methods to move data to/from files employing the IFM
serialization scheme.

### Initial support for 100K pixel images

Unlocking the 100K pixel support on the O3D303 is not yet (as of this writing)
officially supported by IFM and you should only use this feature at your own
risk. However, if you like to live on the edge, you can play with this feature
now. We have provided an application, `test/data/100k.o3d3xxapp`, that unlocks
this feature into a new application on your camera. You can import this
application using `o3d3xx-ifm-import` and then mark it active using
`o3d3xx-config`.

## The initial release of libo3d3xx was 0.1.7
