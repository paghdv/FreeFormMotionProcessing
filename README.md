# FreeFormMotionProcessing
Implementation of the interpolation section of ["Free Form Motion Processing"](http://mgarland.org/files/papers/freeform.pdf) - Kircher &amp; Garland - 2008

Use case example:

We have two 3D frames of the same horse in motion (look at the legs). 
![](samples/horse1.png =300px) ![](samples/horse2.png =300px)

If we simple interpolate these two frames using cartesian coordinates we obtain:
![](samples/cartesian.png =300px)

There is a clear problem of a loos of volume in the legs, plus the motion itself being lost in the interpolation. Instead, if we use the mesh representation and interpolation proposed in "Free Form Motion Processing" we obtain:
![](samples/ffmp.png =300px)
