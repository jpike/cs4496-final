cs4496-final
============
Jacob Pike and Xo Wang

This project contains code for the final project in CS 4496. The initial code
was provided by the professor and can be downloaded here:

    http://web.me.com/ckarenliu/CS4496/Project4.html


Build
-----
Visual Studio 2010 project and solution files are included. Note that it should
be configured for x64, and the binary libraries for vl and fltk are also x64.


vl
--
Note that the following files in vl are modified:

    include/vl/Mat4.h
    src/Mat4.cpp
    src/LibVLd.cpp

The reason was that profiling data from running our IK solver indicated vl was a
significant bottleneck and required optimization. Noteably, Mat4.cpp contains an
implementation of row-major double-precision 4x4 matrix muliplication in SSE2
intrinsics.
